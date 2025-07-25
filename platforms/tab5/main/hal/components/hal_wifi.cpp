/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "hal/hal_esp32.h"
#include <mooncake_log.h>
#include <vector>
#include <memory>
#include <string.h>
#include <math.h>
#include <bsp/m5stack_tab5.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_http_server.h>
#include "driver/jpeg_encode.h"
#include <inttypes.h>

#define TAG "wifi"

// カメラフレーム共有用
typedef struct {
    uint8_t* frame_buffer;
    uint8_t* resize_buffer;  // リサイズ用バッファ
    size_t frame_size;
    uint32_t width;
    uint32_t height;
    bool frame_ready;
    SemaphoreHandle_t frame_mutex;
    jpeg_encoder_handle_t jpeg_handle;
    uint8_t* jpeg_out_buf;
    size_t jpeg_out_buf_size;
} camera_frame_t;

static camera_frame_t g_camera_frame = {
    .frame_buffer = nullptr,
    .resize_buffer = nullptr,
    .frame_size = 0,
    .width = 0,
    .height = 0,
    .frame_ready = false,
    .frame_mutex = nullptr,
    .jpeg_handle = nullptr,
    .jpeg_out_buf = nullptr,
    .jpeg_out_buf_size = 0
};

#define WIFI_SSID    "M5Tab5-UserDemo-WiFi"
#define WIFI_PASS    ""
#define MAX_STA_CONN 4
#define JPEG_ENC_QUALITY 80
#define PART_BOUNDARY "123456789000000000000987654321"

// サポートされる最大解像度（ESP32P4 JPEGエンコーダーの制限）
// ドキュメントのパフォーマンステーブルに基づく最小安全サイズ
#define MAX_ENCODE_WIDTH  320  // 320x480が最小のサポートサイズ
#define MAX_ENCODE_HEIGHT 240  // アスペクト比を考慮した240
#define MIN_ENCODE_WIDTH  320  // 最小幅（安全な値）
#define MIN_ENCODE_HEIGHT 240  // 最小高さ（安全な値）

// 16の倍数に調整する関数（より安全）
uint32_t align_to_16(uint32_t value) {
    // ESP32P4 JPEG エンコーダーは 320x240 以上を推奨
    if (value < MIN_ENCODE_WIDTH && value < MIN_ENCODE_HEIGHT) {
        value = (value < MIN_ENCODE_WIDTH) ? MIN_ENCODE_WIDTH : MIN_ENCODE_HEIGHT;
    }
    return (value + 15) & ~15;
}

// RGB565画像をリサイズする関数（ニアレストネイバー法）
esp_err_t resize_rgb565_image(uint8_t* src_buf, uint32_t src_width, uint32_t src_height,
                              uint8_t* dst_buf, uint32_t dst_width, uint32_t dst_height) {
    if (!src_buf || !dst_buf || src_width == 0 || src_height == 0 || dst_width == 0 || dst_height == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint16_t* src_pixels = (uint16_t*)src_buf;
    uint16_t* dst_pixels = (uint16_t*)dst_buf;
    
    // スケール係数を計算
    float x_scale = (float)src_width / dst_width;
    float y_scale = (float)src_height / dst_height;
    
    for (uint32_t dst_y = 0; dst_y < dst_height; dst_y++) {
        for (uint32_t dst_x = 0; dst_x < dst_width; dst_x++) {
            // ソース画像での対応する座標を計算
            uint32_t src_x = (uint32_t)(dst_x * x_scale);
            uint32_t src_y = (uint32_t)(dst_y * y_scale);
            
            // 境界チェック
            if (src_x >= src_width) src_x = src_width - 1;
            if (src_y >= src_height) src_y = src_height - 1;
            
            // ピクセルをコピー
            uint32_t src_idx = src_y * src_width + src_x;
            uint32_t dst_idx = dst_y * dst_width + dst_x;
            dst_pixels[dst_idx] = src_pixels[src_idx];
        }
    }
    
    return ESP_OK;
}

// カメラストリーミング用HTTPハンドラ
esp_err_t camera_stream_handler(httpd_req_t* req)
{
    esp_err_t res = ESP_OK;
    uint8_t* jpeg_ptr = nullptr;
    size_t jpeg_size = 0;
    uint32_t jpeg_encoded_size = 0;
    
    ESP_LOGI(TAG, "Camera stream handler started");
    
    const char* stream_content_type = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
    const char* stream_boundary = "\r\n--" PART_BOUNDARY "\r\n";
    const char* stream_part = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
    
    // MJPEG ヘッダー送信
    httpd_resp_set_type(req, stream_content_type);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "30");
    
    // 最初のバウンダリを送信
    if (httpd_resp_send_chunk(req, stream_boundary, strlen(stream_boundary)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send initial boundary");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Headers sent, starting stream loop");
    
    while (true) {
        // フレームが準備できるまで待機
        if (g_camera_frame.frame_mutex && xSemaphoreTake(g_camera_frame.frame_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_LOGD(TAG, "Mutex acquired, checking frame readiness");
            
            if (g_camera_frame.frame_ready && g_camera_frame.frame_buffer && g_camera_frame.jpeg_handle) {
                ESP_LOGD(TAG, "Frame ready: %" PRIu32 "x%" PRIu32 ", size: %zu", g_camera_frame.width, g_camera_frame.height, g_camera_frame.frame_size);
                
                // リサイズが必要かチェック
                uint32_t encode_width = g_camera_frame.width;
                uint32_t encode_height = g_camera_frame.height;
                uint8_t* encode_buffer = g_camera_frame.frame_buffer;
                size_t encode_buffer_size = g_camera_frame.frame_size;
                
                if (encode_width > MAX_ENCODE_WIDTH || encode_height > MAX_ENCODE_HEIGHT) {
                    // 比率を保持してリサイズ
                    float scale = fminf((float)MAX_ENCODE_WIDTH / encode_width, (float)MAX_ENCODE_HEIGHT / encode_height);
                    
                    // 新しいサイズを計算
                    uint32_t new_width = (uint32_t)(encode_width * scale);
                    uint32_t new_height = (uint32_t)(encode_height * scale);
                    
                    // ESP32P4の最小制約を満たすように調整
                    if (new_width < MIN_ENCODE_WIDTH) {
                        // アスペクト比を保持しながら最小サイズに調整
                        scale = (float)MIN_ENCODE_WIDTH / encode_width;
                        new_width = MIN_ENCODE_WIDTH;
                        new_height = (uint32_t)(encode_height * scale);
                    }
                    if (new_height < MIN_ENCODE_HEIGHT) {
                        // アスペクト比を保持しながら最小サイズに調整
                        scale = (float)MIN_ENCODE_HEIGHT / encode_height;
                        new_height = MIN_ENCODE_HEIGHT;
                        new_width = (uint32_t)(encode_width * scale);
                    }
                    
                    encode_width = align_to_16(new_width);
                    encode_height = align_to_16(new_height);
                    
                    // 最大値を超えないように再調整
                    if (encode_width > MAX_ENCODE_WIDTH) {
                        encode_width = MAX_ENCODE_WIDTH;
                        encode_width = (encode_width / 16) * 16; // 16の倍数に丸める
                    }
                    if (encode_height > MAX_ENCODE_HEIGHT) {
                        encode_height = MAX_ENCODE_HEIGHT;
                        encode_height = (encode_height / 16) * 16; // 16の倍数に丸める
                    }
                    
                    encode_buffer_size = encode_width * encode_height * 2; // RGB565
                    
                    // リサイズバッファを確保
                    if (g_camera_frame.resize_buffer == nullptr) {
                        size_t resize_buffer_size = MAX_ENCODE_WIDTH * MAX_ENCODE_HEIGHT * 2;
                        
                        // ESP32P4 JPEG エンコーダー用の適切なメモリアロケーション
                        jpeg_encode_memory_alloc_cfg_t mem_cfg = {
                            .buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER,
                        };
                        size_t allocated_size = 0;
                        g_camera_frame.resize_buffer = (uint8_t*)jpeg_alloc_encoder_mem(resize_buffer_size, &mem_cfg, &allocated_size);
                        if (g_camera_frame.resize_buffer == nullptr) {
                            ESP_LOGE(TAG, "Failed to allocate resize buffer");
                            xSemaphoreGive(g_camera_frame.frame_mutex);
                            continue;
                        }
                        ESP_LOGI(TAG, "Resize buffer allocated: %zu bytes (properly aligned)", allocated_size);
                    }
                    
                    // 画像をリサイズ
                    esp_err_t resize_result = resize_rgb565_image(
                        g_camera_frame.frame_buffer, g_camera_frame.width, g_camera_frame.height,
                        g_camera_frame.resize_buffer, encode_width, encode_height
                    );
                    
                    if (resize_result != ESP_OK) {
                        ESP_LOGE(TAG, "Image resize failed: %s", esp_err_to_name(resize_result));
                        xSemaphoreGive(g_camera_frame.frame_mutex);
                        continue;
                    }
                    
                    encode_buffer = g_camera_frame.resize_buffer;
                    ESP_LOGI(TAG, "Frame resized from %" PRIu32 "x%" PRIu32 " to %" PRIu32 "x%" PRIu32 " (scale=%.3f)", 
                            g_camera_frame.width, g_camera_frame.height, encode_width, encode_height, scale);
                } else {
                    ESP_LOGI(TAG, "Using original frame size: %" PRIu32 "x%" PRIu32, encode_width, encode_height);
                }
                
                // RGB565をJPEGに変換
                jpeg_encode_cfg_t jpeg_cfg = {};
                jpeg_cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB565;
                jpeg_cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV422; // YUV422サンプリングを明示的に指定
                jpeg_cfg.image_quality = JPEG_ENC_QUALITY;
                jpeg_cfg.width = encode_width;
                jpeg_cfg.height = encode_height;
                
                // エンコード前の最終検証
                if (encode_width % 16 != 0 || encode_height % 16 != 0) {
                    ESP_LOGE(TAG, "Invalid resolution: %" PRIu32 "x%" PRIu32 " (not aligned to 16)", encode_width, encode_height);
                    xSemaphoreGive(g_camera_frame.frame_mutex);
                    continue;
                }
                
                if (encode_width < MIN_ENCODE_WIDTH || encode_height < MIN_ENCODE_HEIGHT ||
                    encode_width > MAX_ENCODE_WIDTH || encode_height > MAX_ENCODE_HEIGHT) {
                    ESP_LOGE(TAG, "Resolution out of bounds: %" PRIu32 "x%" PRIu32 " (valid range: %dx%d to %dx%d)", 
                            encode_width, encode_height, MIN_ENCODE_WIDTH, MIN_ENCODE_HEIGHT, 
                            MAX_ENCODE_WIDTH, MAX_ENCODE_HEIGHT);
                    xSemaphoreGive(g_camera_frame.frame_mutex);
                    continue;
                }
                
                ESP_LOGI(TAG, "Starting JPEG encoding: %" PRIu32 "x%" PRIu32 ", quality=%d, buffer_size=%zu, fmt=%d", 
                        encode_width, encode_height, JPEG_ENC_QUALITY, encode_buffer_size, JPEG_ENCODE_IN_FORMAT_RGB565);
                
                res = jpeg_encoder_process(g_camera_frame.jpeg_handle, &jpeg_cfg, 
                                         encode_buffer, encode_buffer_size,
                                         g_camera_frame.jpeg_out_buf, g_camera_frame.jpeg_out_buf_size, 
                                         &jpeg_encoded_size);
                
                if (res == ESP_OK) {
                    jpeg_ptr = g_camera_frame.jpeg_out_buf;
                    jpeg_size = jpeg_encoded_size;
                    ESP_LOGD(TAG, "JPEG encoding successful, size: %zu", jpeg_size);
                } else {
                    ESP_LOGE(TAG, "JPEG encoding failed: %s", esp_err_to_name(res));
                }
                
                xSemaphoreGive(g_camera_frame.frame_mutex);
                
                if (res == ESP_OK && jpeg_ptr && jpeg_size > 0) {
                    // ヘッダー送信
                    char part_buf[128];
                    int hlen = snprintf(part_buf, sizeof(part_buf), stream_part, jpeg_size);
                    if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to send part header");
                        break;
                    }
                    
                    // JPEG データ送信
                    if (httpd_resp_send_chunk(req, (const char*)jpeg_ptr, jpeg_size) != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to send JPEG data");
                        break;
                    }
                    
                    // 次のフレーム用の境界送信
                    if (httpd_resp_send_chunk(req, stream_boundary, strlen(stream_boundary)) != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to send boundary");
                        break;
                    }
                    
                    ESP_LOGD(TAG, "Frame sent successfully");
                } else {
                    ESP_LOGW(TAG, "Skipping frame due to encoding error or invalid data");
                }
            } else {
                ESP_LOGW(TAG, "Frame not ready: ready=%d, buffer=%p, handle=%p", 
                        g_camera_frame.frame_ready, g_camera_frame.frame_buffer, g_camera_frame.jpeg_handle);
                xSemaphoreGive(g_camera_frame.frame_mutex);
            }
        } else {
            ESP_LOGW(TAG, "Failed to acquire mutex or mutex is null");
        }
        
        vTaskDelay(pdMS_TO_TICKS(33)); // 約30fps
    }
    
    ESP_LOGI(TAG, "Camera stream handler ended");
    return res;
}

// HTTP 处理函数
esp_err_t hello_get_handler(httpd_req_t* req)
{
    const char* html_response = R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
            <title>Hello</title>
            <style>
                body {
                    display: flex;
                    flex-direction: column;
                    justify-content: center;
                    align-items: center;
                    height: 100vh;
                    margin: 0;
                    font-family: sans-serif;
                    background-color: #f0f0f0;
                }
                h1 {
                    font-size: 48px;
                    color: #333;
                    margin: 0;
                }
                p {
                    font-size: 18px;
                    color: #666;
                    margin-top: 10px;
                }
            </style>
        </head>
        <body>
            <h1>Hello World</h1>
            <p>From M5Tab5</p>
            <p><a href="/stream">Camera Live Stream</a></p>
        </body>
        </html>
    )rawliteral";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// URI 路由
httpd_uri_t hello_uri = {.uri = "/", .method = HTTP_GET, .handler = hello_get_handler, .user_ctx = nullptr};
httpd_uri_t camera_stream_uri = {.uri = "/stream", .method = HTTP_GET, .handler = camera_stream_handler, .user_ctx = nullptr};

// 启动 Web Server
httpd_handle_t start_webserver()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8; // ハンドラ数を増やす
    httpd_handle_t server = nullptr;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &hello_uri);
        httpd_register_uri_handler(server, &camera_stream_uri);
        
        // フレーム共有用のmutex初期化
        if (g_camera_frame.frame_mutex == NULL) {
            g_camera_frame.frame_mutex = xSemaphoreCreateMutex();
        }
        
        // JPEGエンコーダー初期化
        if (g_camera_frame.jpeg_handle == NULL) {
            ESP_LOGI(TAG, "Initializing JPEG encoder");
            jpeg_encode_engine_cfg_t encode_eng_cfg = {
                .intr_priority = 0,
                .timeout_ms = 40,
            };
            
            esp_err_t ret = jpeg_new_encoder_engine(&encode_eng_cfg, &g_camera_frame.jpeg_handle);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "JPEG encoder init failed: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGI(TAG, "JPEG encoder initialized successfully");
                // JPEG出力バッファ確保 (適切なアロケーション関数を使用)
                g_camera_frame.jpeg_out_buf_size = 64 * 1024; // 64KB
                
                // ESP32P4 JPEG エンコーダー用の適切なメモリアロケーション
                jpeg_encode_memory_alloc_cfg_t mem_cfg = {
                    .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
                };
                size_t allocated_size = 0;
                g_camera_frame.jpeg_out_buf = (uint8_t*)jpeg_alloc_encoder_mem(g_camera_frame.jpeg_out_buf_size, &mem_cfg, &allocated_size);
                if (g_camera_frame.jpeg_out_buf == NULL) {
                    ESP_LOGE(TAG, "JPEG output buffer allocation failed");
                } else {
                    g_camera_frame.jpeg_out_buf_size = allocated_size; // 実際に割り当てられたサイズを使用
                    ESP_LOGI(TAG, "JPEG output buffer allocated: %zu bytes (properly aligned)", g_camera_frame.jpeg_out_buf_size);
                }
            }
        }
    }
    return server;
}

// 初始化 Wi-Fi AP 模式
void wifi_init_softap()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {};
    std::strncpy(reinterpret_cast<char*>(wifi_config.ap.ssid), WIFI_SSID, sizeof(wifi_config.ap.ssid));
    std::strncpy(reinterpret_cast<char*>(wifi_config.ap.password), WIFI_PASS, sizeof(wifi_config.ap.password));
    wifi_config.ap.ssid_len       = std::strlen(WIFI_SSID);
    wifi_config.ap.max_connection = MAX_STA_CONN;
    wifi_config.ap.authmode       = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP started. SSID:%s password:%s", WIFI_SSID, WIFI_PASS);
}

static void wifi_ap_test_task(void* param)
{
    wifi_init_softap();
    start_webserver();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

bool HalEsp32::wifi_init()
{
    mclog::tagInfo(TAG, "wifi init");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xTaskCreate(wifi_ap_test_task, "ap", 4096, nullptr, 5, nullptr);
    return true;
}

void HalEsp32::setExtAntennaEnable(bool enable)
{
    _ext_antenna_enable = enable;
    mclog::tagInfo(TAG, "set ext antenna enable: {}", _ext_antenna_enable);
    bsp_set_ext_antenna_enable(_ext_antenna_enable);
}

bool HalEsp32::getExtAntennaEnable()
{
    return _ext_antenna_enable;
}

void HalEsp32::startWifiAp()
{
    wifi_init();
}

// カメラフレームデータを更新する関数
void updateCameraFrameForStreaming(uint8_t* frameBuffer, uint32_t width, uint32_t height)
{
    if (g_camera_frame.frame_mutex && xSemaphoreTake(g_camera_frame.frame_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_camera_frame.frame_buffer = frameBuffer;
        g_camera_frame.width = width;
        g_camera_frame.height = height;
        g_camera_frame.frame_size = width * height * 2; // RGB565
        g_camera_frame.frame_ready = true;
        
        static uint32_t frame_count = 0;
        frame_count++;
        if (frame_count % 30 == 0) { // 30フレームごとにログ出力
            ESP_LOGI(TAG, "Frame updated #%lu: %" PRIu32 "x%" PRIu32 ", size=%zu bytes, buffer=%p", 
                    frame_count, width, height, g_camera_frame.frame_size, frameBuffer);
        }
        
        xSemaphoreGive(g_camera_frame.frame_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to update camera frame - mutex not available");
    }
}

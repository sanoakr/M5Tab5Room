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

#define TAG "wifi"

// カメラフレーム共有用
typedef struct {
    uint8_t* frame_buffer;
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

// カメラストリーミング用HTTPハンドラ
esp_err_t camera_stream_handler(httpd_req_t* req)
{
    esp_err_t res = ESP_OK;
    uint8_t* jpeg_ptr = nullptr;
    size_t jpeg_size = 0;
    uint32_t jpeg_encoded_size = 0;
    
    const char* stream_content_type = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
    const char* stream_boundary = "\r\n--" PART_BOUNDARY "\r\n";
    const char* stream_part = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
    
    // MJPEG ヘッダー送信
    httpd_resp_set_type(req, stream_content_type);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "30");
    
    while (true) {
        // フレームが準備できるまで待機
        if (g_camera_frame.frame_mutex && xSemaphoreTake(g_camera_frame.frame_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (g_camera_frame.frame_ready && g_camera_frame.frame_buffer && g_camera_frame.jpeg_handle) {
                
                // RGB565をJPEGに変換
                jpeg_encode_cfg_t jpeg_cfg = {};
                jpeg_cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB565;
                jpeg_cfg.image_quality = JPEG_ENC_QUALITY;
                jpeg_cfg.width = g_camera_frame.width;
                jpeg_cfg.height = g_camera_frame.height;
                
                res = jpeg_encoder_process(g_camera_frame.jpeg_handle, &jpeg_cfg, 
                                         g_camera_frame.frame_buffer, g_camera_frame.frame_size,
                                         g_camera_frame.jpeg_out_buf, g_camera_frame.jpeg_out_buf_size, 
                                         &jpeg_encoded_size);
                
                if (res == ESP_OK) {
                    jpeg_ptr = g_camera_frame.jpeg_out_buf;
                    jpeg_size = jpeg_encoded_size;
                }
                
                xSemaphoreGive(g_camera_frame.frame_mutex);
                
                if (res == ESP_OK && jpeg_ptr) {
                    // MJPEG境界送信
                    if (httpd_resp_send_chunk(req, stream_boundary, strlen(stream_boundary)) != ESP_OK) {
                        break;
                    }
                    
                    // ヘッダー送信
                    char part_buf[128];
                    int hlen = snprintf(part_buf, sizeof(part_buf), stream_part, jpeg_size);
                    if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK) {
                        break;
                    }
                    
                    // JPEG データ送信
                    if (httpd_resp_send_chunk(req, (const char*)jpeg_ptr, jpeg_size) != ESP_OK) {
                        break;
                    }
                }
            } else {
                xSemaphoreGive(g_camera_frame.frame_mutex);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(33)); // 約30fps
    }
    
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
            jpeg_encode_engine_cfg_t encode_eng_cfg = {
                .intr_priority = 0,
                .timeout_ms = 40,
            };
            
            esp_err_t ret = jpeg_new_encoder_engine(&encode_eng_cfg, &g_camera_frame.jpeg_handle);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "JPEG encoder init failed: %s", esp_err_to_name(ret));
            } else {
                // JPEG出力バッファ確保 (最大サイズを想定)
                g_camera_frame.jpeg_out_buf_size = 1280 * 720 * 2; // RGB565のサイズと同程度
                g_camera_frame.jpeg_out_buf = (uint8_t*)heap_caps_malloc(g_camera_frame.jpeg_out_buf_size, MALLOC_CAP_DMA);
                if (g_camera_frame.jpeg_out_buf == NULL) {
                    ESP_LOGE(TAG, "JPEG output buffer allocation failed");
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
        xSemaphoreGive(g_camera_frame.frame_mutex);
    }
}

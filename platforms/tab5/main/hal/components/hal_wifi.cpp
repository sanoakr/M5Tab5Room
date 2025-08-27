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
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "sc202cs.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"
#include "linux/videodev2.h"
#include "driver/ppa.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <inttypes.h>
#include <esp_pm.h>
#include <esp_sleep.h>

#define TAG "wifi"

// hal_cameraと同じV4L2カメラ定義
#define EXAMPLE_VIDEO_BUFFER_COUNT 2
#define MEMORY_TYPE                V4L2_MEMORY_MMAP
#define CAM_DEV_PATH               ESP_VIDEO_MIPI_CSI_DEVICE_NAME

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))
#endif

typedef struct cam {
    int fd;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint8_t* buffer[EXAMPLE_VIDEO_BUFFER_COUNT];
} cam_t;

typedef enum {
    EXAMPLE_VIDEO_FMT_RAW8   = V4L2_PIX_FMT_SBGGR8,
    EXAMPLE_VIDEO_FMT_RAW10  = V4L2_PIX_FMT_SBGGR10,
    EXAMPLE_VIDEO_FMT_GREY   = V4L2_PIX_FMT_GREY,
    EXAMPLE_VIDEO_FMT_RGB565 = V4L2_PIX_FMT_RGB565,
    EXAMPLE_VIDEO_FMT_RGB888 = V4L2_PIX_FMT_RGB24,
    EXAMPLE_VIDEO_FMT_YUV422 = V4L2_PIX_FMT_YUV422P,
    EXAMPLE_VIDEO_FMT_YUV420 = V4L2_PIX_FMT_YUV420,
} example_fmt_t;

// ステータス管理用
typedef struct {
    char name[64];
    char main_status[128];
    char sub_status[128];
    uint32_t color;
    SemaphoreHandle_t status_mutex;
} status_data_t;

static status_data_t g_status_data = {
    .name = "さのは（おそらく）",
    .main_status = "不在です",
    .sub_status = "（学外にいます）",
    .color = 0xEF5350, // 赤色
    .status_mutex = nullptr
};

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
    // カメラ自動取得用
    bool auto_capture_enabled;
    int camera_fd;
    uint8_t* auto_frame_buffer;
    size_t auto_frame_buffer_size;
    TaskHandle_t capture_task_handle;
    // カメラ独立管理用
    bool camera_initialized;
    bool camera_running;
    SemaphoreHandle_t camera_control_mutex;
    // 実際のカメラフレーム管理用
    uint8_t* real_camera_buffer;
    uint32_t real_camera_width;
    uint32_t real_camera_height;
    bool real_camera_frame_available;
    uint32_t last_real_frame_time;
    // 独立カメラ制御用
    TaskHandle_t direct_camera_task_handle;
    bool direct_camera_enabled;
    uint8_t* direct_camera_frame_buffer;
    size_t direct_camera_frame_size;
    void* camera_device_handle;  // 実際のカメラデバイスハンドル
    // V4L2カメラシステム用
    cam_t* v4l2_camera;
    bool v4l2_camera_initialized;
    ppa_client_handle_t ppa_handle;
    uint8_t* processed_frame_buffer;
    size_t processed_frame_size;
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
    .jpeg_out_buf_size = 0,
    .auto_capture_enabled = false,
    .camera_fd = -1,
    .auto_frame_buffer = nullptr,
    .auto_frame_buffer_size = 0,
    .capture_task_handle = nullptr,
    .camera_initialized = false,
    .camera_running = false,
    .camera_control_mutex = nullptr,
    .real_camera_buffer = nullptr,
    .real_camera_width = 0,
    .real_camera_height = 0,
    .real_camera_frame_available = false,
    .last_real_frame_time = 0,
    .direct_camera_task_handle = nullptr,
    .direct_camera_enabled = false,
    .direct_camera_frame_buffer = nullptr,
    .direct_camera_frame_size = 0,
    .camera_device_handle = nullptr,
    // V4L2カメラシステム用
    .v4l2_camera = nullptr,
    .v4l2_camera_initialized = false,
    .ppa_handle = nullptr,
    .processed_frame_buffer = nullptr,
    .processed_frame_size = 0
};

#define WIFI_SSID    "M5Tab5-UserDemo-WiFi"
#define WIFI_PASS    ""
#define MAX_STA_CONN 4
#define JPEG_ENC_QUALITY 60  // 電力消費軽減のため品質を下げる
#define PART_BOUNDARY "123456789000000000000987654321"

// サポートされる最大解像度（ESP32P4 JPEGエンコーダーの制限）
// ドキュメントのパフォーマンステーブルに基づく最小安全サイズ
#define MAX_ENCODE_WIDTH  320  // 320x240が最小のサポートサイズ
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

// hal_cameraと同じカメラ開放関数
int wifi_video_open(char* dev, example_fmt_t init_fmt)
{
    struct v4l2_format default_format;
    struct v4l2_capability capability;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    int fd = open(dev, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "Open video failed");
        return -1;
    }

    if (ioctl(fd, VIDIOC_QUERYCAP, &capability)) {
        ESP_LOGE(TAG, "failed to get capability");
        goto exit_0;
    }

    ESP_LOGI(TAG, "version: %d.%d.%d", (uint16_t)(capability.version >> 16), (uint8_t)(capability.version >> 8),
             (uint8_t)capability.version);
    ESP_LOGI(TAG, "driver:  %s", capability.driver);
    ESP_LOGI(TAG, "card:    %s", capability.card);
    ESP_LOGI(TAG, "bus:     %s", capability.bus_info);

    memset(&default_format, 0, sizeof(struct v4l2_format));
    default_format.type = type;
    if (ioctl(fd, VIDIOC_G_FMT, &default_format) != 0) {
        ESP_LOGE(TAG, "failed to get format");
        goto exit_0;
    }

    ESP_LOGI(TAG, "width=%" PRIu32 " height=%" PRIu32, default_format.fmt.pix.width, default_format.fmt.pix.height);

    if (default_format.fmt.pix.pixelformat != init_fmt) {
        struct v4l2_format format = {.type = type,
                                     .fmt  = {.pix = {.width       = default_format.fmt.pix.width,
                                                      .height      = default_format.fmt.pix.height,
                                                      .pixelformat = init_fmt}}};

        if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) {
            ESP_LOGE(TAG, "failed to set format");
            goto exit_0;
        }
    }

    return fd;
exit_0:
    close(fd);
    return -1;
}

// hal_cameraと同じカメラ構造体初期化関数
static esp_err_t wifi_new_cam(int cam_fd, cam_t** ret_wc)
{
    int ret;
    struct v4l2_format format;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_requestbuffers req;
    cam_t* wc;

    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cam_fd, VIDIOC_G_FMT, &format) != 0) {
        ESP_LOGE(TAG, "Failed get fmt");
        return ESP_FAIL;
    }

    wc = (cam_t*)malloc(sizeof(cam_t));
    if (!wc) {
        return ESP_ERR_NO_MEM;
    }

    wc->fd           = cam_fd;
    wc->width        = format.fmt.pix.width;
    wc->height       = format.fmt.pix.height;
    wc->pixel_format = format.fmt.pix.pixelformat;

    memset(&req, 0, sizeof(req));
    req.count  = ARRAY_SIZE(wc->buffer);
    req.type   = type;
    req.memory = MEMORY_TYPE;
    if (ioctl(wc->fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "failed to req buffers");
        ret = ESP_FAIL;
        goto errout;
    }

    for (int i = 0; i < ARRAY_SIZE(wc->buffer); i++) {
        struct v4l2_buffer buf;

        memset(&buf, 0, sizeof(buf));
        buf.type   = type;
        buf.memory = MEMORY_TYPE;
        buf.index  = i;
        if (ioctl(wc->fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "failed to query buffer");
            ret = ESP_FAIL;
            goto errout;
        }

        wc->buffer[i] = (uint8_t*)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, wc->fd, buf.m.offset);
        if (!wc->buffer[i]) {
            ESP_LOGE(TAG, "failed to map buffer");
            ret = ESP_FAIL;
            goto errout;
        }

        if (ioctl(wc->fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "failed to queue frame buffer");
            ret = ESP_FAIL;
            goto errout;
        }
    }

    if (ioctl(wc->fd, VIDIOC_STREAMON, &type)) {
        ESP_LOGE(TAG, "failed to start stream");
        ret = ESP_FAIL;
        goto errout;
    }

    *ret_wc = wc;
    return ESP_OK;

errout:
    free(wc);
    return ret;
}

// カメラ管理関数
bool isCameraInitialized() {
    if (g_camera_frame.camera_control_mutex && 
        xSemaphoreTake(g_camera_frame.camera_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool initialized = g_camera_frame.camera_initialized;
        xSemaphoreGive(g_camera_frame.camera_control_mutex);
        return initialized;
    }
    return false;
}

bool isCameraRunning() {
    if (g_camera_frame.camera_control_mutex && 
        xSemaphoreTake(g_camera_frame.camera_control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool running = g_camera_frame.camera_running;
        xSemaphoreGive(g_camera_frame.camera_control_mutex);
        return running;
    }
    return false;
}

esp_err_t initServerManagedCamera() {
    ESP_LOGI(TAG, "Initializing server-managed camera hardware");
    
    if (g_camera_frame.camera_control_mutex == nullptr) {
        g_camera_frame.camera_control_mutex = xSemaphoreCreateMutex();
        if (g_camera_frame.camera_control_mutex == nullptr) {
            ESP_LOGE(TAG, "Failed to create camera control mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    if (xSemaphoreTake(g_camera_frame.camera_control_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (!g_camera_frame.camera_initialized) {
            // カメラハードウェアを直接初期化
            ESP_LOGI(TAG, "Initializing camera hardware directly for server");
            
            // BSPのカメラオシレータを初期化
            esp_err_t ret = bsp_cam_osc_init();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to initialize camera oscillator: %s", esp_err_to_name(ret));
                xSemaphoreGive(g_camera_frame.camera_control_mutex);
                return ret;
            }
            
            g_camera_frame.camera_initialized = true;
            g_camera_frame.camera_running = true;  // 直接実行状態に
            ESP_LOGI(TAG, "Server-managed camera hardware initialized successfully");
            xSemaphoreGive(g_camera_frame.camera_control_mutex);
            return ESP_OK;
        } else {
            ESP_LOGI(TAG, "Server-managed camera already initialized");
            xSemaphoreGive(g_camera_frame.camera_control_mutex);
            return ESP_OK;
        }
    }
    
    ESP_LOGE(TAG, "Failed to acquire camera control mutex");
    return ESP_ERR_TIMEOUT;
}

esp_err_t startServerManagedCamera() {
    ESP_LOGI(TAG, "Starting server-managed camera");
    
    if (xSemaphoreTake(g_camera_frame.camera_control_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (g_camera_frame.camera_initialized && !g_camera_frame.camera_running) {
            // カメラを開始
            g_camera_frame.camera_running = true;
            ESP_LOGI(TAG, "Server-managed camera started");
        } else if (g_camera_frame.camera_running) {
            ESP_LOGI(TAG, "Server-managed camera already running");
        } else {
            ESP_LOGW(TAG, "Camera not initialized, cannot start");
        }
        xSemaphoreGive(g_camera_frame.camera_control_mutex);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to acquire camera control mutex");
    return ESP_ERR_TIMEOUT;
}

// カメラ自動初期化関数
esp_err_t init_auto_camera() {
    ESP_LOGI(TAG, "Initializing auto camera for continuous streaming");
    
    // 自動フレーム生成を有効にする
    g_camera_frame.auto_capture_enabled = true;
    
    // 独立カメラ制御を有効にする
    g_camera_frame.direct_camera_enabled = true;
    
    ESP_LOGI(TAG, "Auto capture enabled - will provide continuous streaming with direct camera control");
    return ESP_OK;
}

// カメラの独立制御タスク（hal_cameraと同じV4L2方式を使用）
void direct_camera_capture_task(void* param) {
    ESP_LOGI(TAG, "Direct camera capture task started - using V4L2 real camera system");
    
    // ESP32P4のV4L2カメラシステムを初期化（hal_cameraと同じ方法）
    const uint32_t camera_width = 1280;
    const uint32_t camera_height = 720;
    const size_t camera_frame_size = camera_width * camera_height * 2; // RGB565
    
    // フレームバッファを確保
    g_camera_frame.direct_camera_frame_buffer = (uint8_t*)heap_caps_malloc(camera_frame_size, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (g_camera_frame.direct_camera_frame_buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate direct camera frame buffer");
        vTaskDelete(NULL);
        return;
    }
    
    // 処理済みフレームバッファを確保
    g_camera_frame.processed_frame_buffer = (uint8_t*)heap_caps_malloc(camera_frame_size, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (g_camera_frame.processed_frame_buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate processed frame buffer");
        heap_caps_free(g_camera_frame.direct_camera_frame_buffer);
        vTaskDelete(NULL);
        return;
    }
    
    g_camera_frame.direct_camera_frame_size = camera_frame_size;
    g_camera_frame.processed_frame_size = camera_frame_size;
    ESP_LOGI(TAG, "Allocated camera frame buffers: %" PRIu32 "x%" PRIu32 ", size=%d bytes", 
            (uint32_t)camera_width, (uint32_t)camera_height, (int)camera_frame_size);
    
    // ESP32P4 Video systemを初期化（hal_cameraと同じ方法）
    ESP_LOGI(TAG, "Initializing ESP32P4 V4L2 camera system");
    
    uint32_t frame_counter = 0;  // 変数宣言を前に移動
    struct v4l2_buffer buf;
    
    if (!g_camera_frame.v4l2_camera_initialized) {
        // hal_cameraと同じCSI設定を使用
        esp_video_init_csi_config_t csi_config = {
            .sccb_config = {
                .init_sccb  = false,
                .i2c_handle = bsp_i2c_get_handle(),
                .freq       = 400000,
            },
            .reset_pin = -1,
            .pwdn_pin  = -1,
        };
        
        esp_video_init_config_t cam_config = {
            .csi  = &csi_config,
            .dvp  = NULL,
            .jpeg = NULL,
            .isp  = NULL
        };
        
        ESP_LOGI(TAG, "Initializing ESP video system...");
        esp_err_t video_init_result = esp_video_init(&cam_config);
        if (video_init_result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize ESP video system: %s", esp_err_to_name(video_init_result));
            goto cleanup;
        }
        
        ESP_LOGI(TAG, "Opening V4L2 camera device: %s", CAM_DEV_PATH);
        int video_cam_fd = wifi_video_open((char*)CAM_DEV_PATH, EXAMPLE_VIDEO_FMT_RGB565);
        if (video_cam_fd < 0) {
            ESP_LOGW(TAG, "Failed to open V4L2 camera device - falling back to simulation mode");
            // フォールバック：シミュレーションモードで継続
            g_camera_frame.v4l2_camera_initialized = false;
            g_camera_frame.camera_initialized = true;
            g_camera_frame.camera_running = true;
            goto skip_v4l2_init;
        }
        
        ESP_LOGI(TAG, "Initializing camera structure...");
        esp_err_t cam_init_result = wifi_new_cam(video_cam_fd, &g_camera_frame.v4l2_camera);
        if (cam_init_result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize camera: %s", esp_err_to_name(cam_init_result));
            close(video_cam_fd);
            goto cleanup;
        }
        
        // PPA（Picture Processing Accelerator）を初期化
        ppa_client_config_t ppa_config = {
            .oper_type = PPA_OPERATION_SRM,
            .max_pending_trans_num = 1,
            .data_burst_length = PPA_DATA_BURST_LENGTH_128
        };
        esp_err_t ppa_result = ppa_register_client(&ppa_config, &g_camera_frame.ppa_handle);
        if (ppa_result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register PPA client: %s", esp_err_to_name(ppa_result));
            goto cleanup;
        }
        
        g_camera_frame.v4l2_camera_initialized = true;
        g_camera_frame.camera_initialized = true;
        g_camera_frame.camera_running = true;
        
        ESP_LOGI(TAG, "V4L2 camera system initialized successfully - Real camera ready!");
        ESP_LOGI(TAG, "Camera format: %" PRIu32 "x%" PRIu32 ", pixel_format: 0x%" PRIx32, 
                g_camera_frame.v4l2_camera->width, g_camera_frame.v4l2_camera->height, 
                g_camera_frame.v4l2_camera->pixel_format);
    }
    
skip_v4l2_init:
    
    while (g_camera_frame.direct_camera_enabled && g_camera_frame.camera_running) {
        if (g_camera_frame.v4l2_camera_initialized) {
            // V4L2から実際のカメラフレームを取得
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = MEMORY_TYPE;
            
            if (ioctl(g_camera_frame.v4l2_camera->fd, VIDIOC_DQBUF, &buf) != 0) {
                ESP_LOGE(TAG, "Failed to receive video frame from V4L2");
                vTaskDelay(pdMS_TO_TICKS(200)); // 約5fps
                continue;
            }
            
            if (g_camera_frame.frame_mutex && xSemaphoreTake(g_camera_frame.frame_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                uint32_t time_ms = esp_timer_get_time() / 1000;
                
                // PPAを使用してフレームを処理（hal_cameraと同じ方法）
                ppa_srm_oper_config_t srm_config = {};
                srm_config.in.buffer = g_camera_frame.v4l2_camera->buffer[buf.index];
                srm_config.in.pic_w = 1280;
                srm_config.in.pic_h = 720;
                srm_config.in.block_w = 1280;
                srm_config.in.block_h = 720;
                srm_config.in.block_offset_x = 0;
                srm_config.in.block_offset_y = 0;
                srm_config.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
                
                srm_config.out.buffer = g_camera_frame.processed_frame_buffer;
                srm_config.out.buffer_size = g_camera_frame.processed_frame_size;
                srm_config.out.pic_w = 1280;
                srm_config.out.pic_h = 720;
                srm_config.out.block_offset_x = 0;
                srm_config.out.block_offset_y = 0;
                srm_config.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
                
                srm_config.rotation_angle = PPA_SRM_ROTATION_ANGLE_180;
                srm_config.scale_x = 1;
                srm_config.scale_y = 1;
                srm_config.mirror_x = true;
                srm_config.mirror_y = false;
                srm_config.rgb_swap = false;
                srm_config.byte_swap = false;
                srm_config.mode = PPA_TRANS_MODE_BLOCKING;
                
                esp_err_t ppa_result = ppa_do_scale_rotate_mirror(g_camera_frame.ppa_handle, &srm_config);
                if (ppa_result == ESP_OK) {
                    // 実際のカメラフレーム情報を設定
                    g_camera_frame.real_camera_buffer = g_camera_frame.processed_frame_buffer;
                    g_camera_frame.real_camera_width = camera_width;
                    g_camera_frame.real_camera_height = camera_height;
                    g_camera_frame.real_camera_frame_available = true;
                    g_camera_frame.last_real_frame_time = time_ms;
                    
                    // 共通フレームバッファにも設定
                    g_camera_frame.frame_buffer = g_camera_frame.processed_frame_buffer;
                    g_camera_frame.width = camera_width;
                    g_camera_frame.height = camera_height;
                    g_camera_frame.frame_size = camera_frame_size;
                    g_camera_frame.frame_ready = true;
                    
                    frame_counter++;
                    if (frame_counter % 300 == 1) {  // 最初と10秒ごとにログ出力
                        ESP_LOGI(TAG, "Real V4L2 camera frame #%lu: %" PRIu32 "x%" PRIu32 ", size=%d bytes (ACTUAL CAMERA DATA)", 
                                frame_counter, (uint32_t)camera_width, (uint32_t)camera_height, (int)camera_frame_size);
                    }
                } else {
                    ESP_LOGW(TAG, "PPA processing failed: %s", esp_err_to_name(ppa_result));
                }
                
                xSemaphoreGive(g_camera_frame.frame_mutex);
            }
            
            // V4L2バッファを戻す
            if (ioctl(g_camera_frame.v4l2_camera->fd, VIDIOC_QBUF, &buf) != 0) {
                ESP_LOGE(TAG, "Failed to free video frame");
            }
        } else {
            // シミュレーションモード（V4L2が使用できない場合）
            if (g_camera_frame.frame_mutex && xSemaphoreTake(g_camera_frame.frame_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                uint32_t time_ms = esp_timer_get_time() / 1000;
                
                // 高品質カメラシミュレーション
                uint16_t* pixels = (uint16_t*)g_camera_frame.direct_camera_frame_buffer;
                
                for (uint32_t y = 0; y < camera_height; y++) {
                    for (uint32_t x = 0; x < camera_width; x++) {
                        uint8_t r, g, b;
                        
                        // V4L2失敗からのフォールバック表示
                        float wave_x = sinf(((float)x / camera_width) * 3.14159f + (float)time_ms / 1000.0f) * 0.3f + 0.7f;
                        float wave_y = cosf(((float)y / camera_height) * 3.14159f + (float)time_ms / 1500.0f) * 0.3f + 0.7f;
                        
                        r = (uint8_t)((wave_x * x * 31) / camera_width);
                        g = (uint8_t)((wave_y * y * 63) / camera_height);
                        b = (uint8_t)(((wave_x + wave_y) * (x + y) * 31) / (camera_width + camera_height));
                        
                        // 境界チェック
                        r = r > 31 ? 31 : r;
                        g = g > 63 ? 63 : g;
                        b = b > 31 ? 31 : b;
                        
                        // ステータス表示：フォールバックモード
                        if (y >= 50 && y < 150 && x >= 50 && x < 1200) {
                            if ((frame_counter / 100) % 2 == 0) {
                                r = 31; g = 31; b = 0; // 黄色で"FALLBACK MODE"
                            } else {
                                r = 31; g = 15; b = 0; // オレンジ
                            }
                        }
                        
                        pixels[y * camera_width + x] = (r << 11) | (g << 5) | b;
                    }
                }
                
                // シミュレーションフレーム情報を設定
                g_camera_frame.real_camera_buffer = g_camera_frame.direct_camera_frame_buffer;
                g_camera_frame.real_camera_width = camera_width;
                g_camera_frame.real_camera_height = camera_height;
                g_camera_frame.real_camera_frame_available = true;
                g_camera_frame.last_real_frame_time = time_ms;
                
                // 共通フレームバッファにも設定
                g_camera_frame.frame_buffer = g_camera_frame.direct_camera_frame_buffer;
                g_camera_frame.width = camera_width;
                g_camera_frame.height = camera_height;
                g_camera_frame.frame_size = camera_frame_size;
                g_camera_frame.frame_ready = true;
                
                frame_counter++;
                if (frame_counter % 300 == 1) {  // 最初と10秒ごとにログ出力
                    ESP_LOGI(TAG, "Fallback simulation frame #%lu: %" PRIu32 "x%" PRIu32 ", size=%d bytes (V4L2 FALLBACK)", 
                            frame_counter, (uint32_t)camera_width, (uint32_t)camera_height, (int)camera_frame_size);
                }
                
                xSemaphoreGive(g_camera_frame.frame_mutex);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(200)); // 約5fps
    }
    
    ESP_LOGI(TAG, "Direct V4L2 camera capture task ended - total frames: %lu", frame_counter);
    
cleanup:
    // リソースを解放
    if (g_camera_frame.ppa_handle) {
        ppa_unregister_client(g_camera_frame.ppa_handle);
        g_camera_frame.ppa_handle = nullptr;
    }
    
    if (g_camera_frame.v4l2_camera) {
        if (g_camera_frame.v4l2_camera->fd >= 0) {
            close(g_camera_frame.v4l2_camera->fd);
        }
        free(g_camera_frame.v4l2_camera);
        g_camera_frame.v4l2_camera = nullptr;
    }
    
    if (g_camera_frame.direct_camera_frame_buffer) {
        heap_caps_free(g_camera_frame.direct_camera_frame_buffer);
        g_camera_frame.direct_camera_frame_buffer = nullptr;
    }
    
    if (g_camera_frame.processed_frame_buffer) {
        heap_caps_free(g_camera_frame.processed_frame_buffer);
        g_camera_frame.processed_frame_buffer = nullptr;
    }
    
    g_camera_frame.v4l2_camera_initialized = false;
    
    vTaskDelete(NULL);
}

// カメラ自動フレーム取得タスク
void camera_auto_capture_task(void* param) {
    ESP_LOGI(TAG, "Camera auto capture task started - waiting for direct camera frames");
    
    uint32_t frame_counter = 0;
    uint32_t real_frame_count = 0;
    uint32_t wait_count = 0;
    
    while (g_camera_frame.auto_capture_enabled) {
        // フレームミューテックスを取得
        if (g_camera_frame.frame_mutex && xSemaphoreTake(g_camera_frame.frame_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            
            uint32_t current_time = esp_timer_get_time() / 1000;
            
            // 独立カメラまたは実際のカメラフレームが利用可能かチェック（5秒以内の新しいフレーム）
            if (g_camera_frame.real_camera_frame_available && 
                g_camera_frame.real_camera_buffer != nullptr &&
                (current_time - g_camera_frame.last_real_frame_time) < 5000) {
                
                // 独立カメラまたは実際のカメラフレームを使用（既に設定済み）
                // direct_camera_capture_taskが既にフレームを設定している
                real_frame_count++;
                
                if (real_frame_count % 600 == 1) {  // 最初と20秒ごとにログ出力
                    ESP_LOGI(TAG, "Using REAL camera frame #%lu: %" PRIu32 "x%" PRIu32 ", size=%d bytes", 
                            real_frame_count, (uint32_t)g_camera_frame.width, (uint32_t)g_camera_frame.height, (int)g_camera_frame.frame_size);
                }
            } else {
                // カメラフレームを待機中
                wait_count++;
                if (wait_count % 300 == 1) {  // 最初と10秒ごとにログ出力
                    ESP_LOGI(TAG, "Waiting for camera frames... (%lu attempts, last_frame_time=%lu, current=%lu)", 
                            wait_count, g_camera_frame.last_real_frame_time, current_time);
                }
                
                // フレームが利用できない場合は待機（テストパターンは生成しない）
                g_camera_frame.frame_ready = false;
            }
            
            frame_counter++;
            
            // 統計情報を定期的に出力
            if (frame_counter % 1800 == 0) {  // 60秒ごと
                ESP_LOGI(TAG, "Streaming stats: Real camera frames: %lu, Wait attempts: %lu", 
                        real_frame_count, wait_count);
            }
            
            xSemaphoreGive(g_camera_frame.frame_mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(200)); // 約5fps
    }
    
    ESP_LOGI(TAG, "Camera auto capture task ended - final stats: Real camera: %lu, Wait attempts: %lu", 
            real_frame_count, wait_count);
    vTaskDelete(NULL);
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

// カメラストリーミング用HTTPハンドラ（テストパターンのみ）
esp_err_t camera_stream_handler(httpd_req_t* req)
{
    ESP_LOGI(TAG, "Test pattern stream handler started - NO REAL CAMERA");
    
    // 簡単なテストパターンHTMLを返す
    const char* test_pattern_html = R"(
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="UTF-8">
            <title>Camera Disabled</title>
            <style>
                body { 
                    font-family: Arial, sans-serif; 
                    text-align: center; 
                    margin: 50px; 
                    background-color: #f0f0f0; 
                }
                .message {
                    background: #fff;
                    border: 2px solid #333;
                    border-radius: 10px;
                    padding: 30px;
                    margin: 20px auto;
                    max-width: 600px;
                }
                .pattern {
                    width: 320px;
                    height: 240px;
                    background: linear-gradient(45deg, #ff6b6b, #4ecdc4, #45b7d1, #f9ca24);
                    margin: 20px auto;
                    border: 2px solid #333;
                    animation: colorShift 3s ease-in-out infinite alternate;
                }
                @keyframes colorShift {
                    0% { filter: hue-rotate(0deg); }
                    100% { filter: hue-rotate(360deg); }
                }
            </style>
        </head>
        <body>
            <div class="message">
                <h1>📷 カメラストリーミング無効</h1>
                <p>カメラストリーミング機能は無効化されています。</p>
                <p>リアルカメラの代わりにテストパターンを表示中です。</p>
                <div class="pattern"></div>
                <p><small>Camera streaming has been disabled by user request</small></p>
            </div>
        </body>
        </html>
    )";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, test_pattern_html, HTTPD_RESP_USE_STRLEN);
    
    ESP_LOGI(TAG, "Test pattern HTML sent - camera streaming disabled");
    return ESP_OK;
}

// ステータス更新API
esp_err_t status_api_handler(httpd_req_t* req)
{
    if (g_status_data.status_mutex && xSemaphoreTake(g_status_data.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        char json_response[512];
        snprintf(json_response, sizeof(json_response), 
                "{"
                "\"name\":\"%s\","
                "\"main_status\":\"%s\","
                "\"sub_status\":\"%s\","
                "\"color\":\"#%06" PRIX32 "\""
                "}", 
                g_status_data.name, 
                g_status_data.main_status, 
                g_status_data.sub_status,
                g_status_data.color);
        
        xSemaphoreGive(g_status_data.status_mutex);
        
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Status mutex error");
    return ESP_FAIL;
}

// HTTP 处理函数
esp_err_t hello_get_handler(httpd_req_t* req)
{
    const char* html_response = R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="UTF-8">
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <title>M5Stack TAB5 Streaming RoomSign</title>
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
                .stream-container {
                    display: flex;
                    flex-direction: column;
                    align-items: center;
                    justify-content: center;
                    margin-top: 32px;
                    background: #fff;
                    border-radius: 16px;
                    box-shadow: 0 2px 12px rgba(0,0,0,0.08);
                    padding: 24px 32px;
                }
                #camera-stream {
                    max-width: 90vw;
                    max-height: 60vh;
                    border: 2px solid #333;
                    border-radius: 8px;
                    background: #222;
                    display: block;
                    margin-bottom: 16px;
                }
                .toggle-switch {
                    position: relative;
                    display: inline-block;
                    width: 80px;
                    height: 40px;
                    background-color: #ccc;
                    border-radius: 20px;
                    cursor: pointer;
                    transition: background-color 0.3s ease;
                }
                .toggle-switch.active {
                    background-color: #1976d2;
                }
                .toggle-slider {
                    position: absolute;
                    top: 2px;
                    left: 2px;
                    width: 36px;
                    height: 36px;
                    background-color: white;
                    border-radius: 50%;
                    transition: left 0.3s ease;
                    box-shadow: 0 2px 4px rgba(0,0,0,0.2);
                }
                .toggle-switch.active .toggle-slider {
                    left: 42px;
                }
                .toggle-label {
                    margin-bottom: 8px;
                    font-size: 14px;
                    color: #333;
                    text-align: center;
                }
                .status-display {
                    margin-top: 32px;
                    padding: 24px 32px;
                    background: #fff;
                    border-radius: 16px;
                    box-shadow: 0 2px 12px rgba(0,0,0,0.08);
                    text-align: center;
                    max-width: 600px;
                }
                .status-name {
                    font-size: 20px;
                    color: #666;
                    margin-bottom: 16px;
                }
                .status-main {
                    font-size: 48px;
                    font-weight: bold;
                    margin-bottom: 12px;
                    color: #EF5350;
                }
                .status-sub {
                    font-size: 24px;
                    color: #666;
                }
            </style>
            <script>
                function toggleStream() {
                    // カメラストリーミングは無効化されているため何もしない
                    console.log('Camera streaming is disabled');
                }
                
                function updateStatus() {
                    fetch('/api/status')
                        .then(response => response.json())
                        .then(data => {
                            document.getElementById('status-name').textContent = data.name;
                            document.getElementById('status-main').textContent = data.main_status;
                            document.getElementById('status-sub').textContent = data.sub_status;
                            document.getElementById('status-main').style.color = data.color;
                            document.getElementById('status-sub').style.color = data.color;
                        })
                        .catch(error => {
                            console.error('Status update error:', error);
                        });
                }
                
                document.addEventListener('DOMContentLoaded', function() {
                    // カメラストリーミングは無効化されているためトグル機能なし
                    console.log('Camera streaming disabled - no toggle functionality');
                    
                    // Update status every 2 seconds
                    updateStatus();
                    setInterval(updateStatus, 2000);
                });
            </script>
        </head>
        <body>
            <h1>Tab5 RoomSign</h1>
            <p>From M5StackTab5</p>
            <div class="stream-container">
                <div style="width: 320px; height: 240px; border: 2px solid #ff6b6b; border-radius: 8px; background: linear-gradient(45deg, #ff6b6b, #4ecdc4); display: flex; align-items: center; justify-content: center; margin-bottom: 16px;">
                    <div style="text-align: center; color: white; font-weight: bold; text-shadow: 1px 1px 2px rgba(0,0,0,0.7);">
                        📷 CAMERA DISABLED<br>
                        <small>カメラ無効</small>
                    </div>
                </div>
                <div class="toggle-label" style="color: #ff6b6b;">Camera Streaming Disabled</div>
                <div style="font-size: 12px; color: #666; margin-top: 8px;">リアルカメラストリーミングは無効化されています</div>
            </div>
            <div class="status-display">
                <div id="status-name" class="status-name">さのは（おそらく）</div>
                <div id="status-main" class="status-main">不在です</div>
                <div id="status-sub" class="status-sub">（学外にいます）</div>
            </div>
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
httpd_uri_t status_api_uri = {.uri = "/api/status", .method = HTTP_GET, .handler = status_api_handler, .user_ctx = nullptr};

// 启动 Web Server
httpd_handle_t start_webserver()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8; // ハンドラ数を増やす
    httpd_handle_t server = nullptr;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &hello_uri);
        httpd_register_uri_handler(server, &camera_stream_uri);
        httpd_register_uri_handler(server, &status_api_uri);
        
        // ステータス管理用のmutex初期化
        if (g_status_data.status_mutex == NULL) {
            g_status_data.status_mutex = xSemaphoreCreateMutex();
        }
        
        // フレーム共有用のmutex初期化
        if (g_camera_frame.frame_mutex == NULL) {
            g_camera_frame.frame_mutex = xSemaphoreCreateMutex();
        }
        
        // カメラ機能を完全に無効化 - テストパターンのみ使用
        ESP_LOGI(TAG, "Camera initialization SKIPPED - test pattern mode only");
        ESP_LOGI(TAG, "Real camera streaming disabled by user request");
        
        // JPEGエンコーダーも無効化（テストパターンには不要）
        ESP_LOGI(TAG, "JPEG encoder initialization SKIPPED - not needed for test patterns");
        
        // カメラ自動取得タスクも無効化
        ESP_LOGI(TAG, "Camera auto capture and direct capture tasks DISABLED");
        ESP_LOGI(TAG, "Web streaming will show test patterns only");
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
    
    // カメラストリーミング機能を無効化 - テストパターンのみ使用
    ESP_LOGI(TAG, "Camera streaming DISABLED - using test patterns only");
    ESP_LOGI(TAG, "Real camera initialization and streaming are disabled by user request");
    
    start_webserver();
    ESP_LOGI(TAG, "WiFi AP and streaming server started - TEST PATTERN ONLY mode");
    
    ESP_LOGI(TAG, "Camera streaming disabled - web page will show test patterns instead of real camera");

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

    xTaskCreate(wifi_ap_test_task, "ap", 8192, nullptr, 5, nullptr);
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

// カメラ停止を通知する関数（カメラコンポーネントから呼ばれる）
void notifyCameraStop()
{
    if (g_camera_frame.frame_mutex && xSemaphoreTake(g_camera_frame.frame_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        ESP_LOGI(TAG, "Camera stopped notification - switching to fallback test pattern");
        
        // 実際のカメラフレームを無効化
        g_camera_frame.real_camera_frame_available = false;
        g_camera_frame.real_camera_buffer = nullptr;
        g_camera_frame.real_camera_width = 0;
        g_camera_frame.real_camera_height = 0;
        g_camera_frame.last_real_frame_time = 0;
        
        // フォールバックパターンに自動的に切り替わる（camera_auto_capture_taskが処理）
        ESP_LOGI(TAG, "Real camera unavailable, auto-task will use fallback pattern");
        
        xSemaphoreGive(g_camera_frame.frame_mutex);
    }
}

// hai_cameraで使用するカメラ状態チェック関数
bool shouldInitializeCamera() {
    return !isCameraInitialized();
}

bool shouldStartCamera() {
    return isCameraInitialized() && !isCameraRunning();
}

// hai_cameraでの初期化（必要な場合のみ）
esp_err_t initCameraIfNeeded() {
    if (shouldInitializeCamera()) {
        ESP_LOGI(TAG, "hai_camera: Camera not initialized by server, initializing...");
        return initServerManagedCamera();
    } else {
        ESP_LOGI(TAG, "hai_camera: Camera already initialized by server");
        return ESP_OK;
    }
}

// hai_cameraでの開始（必要な場合のみ）
esp_err_t startCameraIfNeeded() {
    if (shouldStartCamera()) {
        ESP_LOGI(TAG, "hai_camera: Camera not running, starting...");
        return startServerManagedCamera();
    } else {
        ESP_LOGI(TAG, "hai_camera: Camera already running");
        return ESP_OK;
    }
}

// hai_cameraが閉じられても何もしない（カメラは停止しない）
void onHaiCameraClosed() {
    ESP_LOGI(TAG, "hai_camera: Application closed, but keeping camera running for streaming");
    // カメラは停止しない - 配信を継続
}

// ステータス更新関数（view.cppから呼ばれる）
void updateRoomSignStatus(const char* main_status, const char* sub_status, uint32_t color) {
    if (g_status_data.status_mutex && xSemaphoreTake(g_status_data.status_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        strncpy(g_status_data.main_status, main_status, sizeof(g_status_data.main_status) - 1);
        g_status_data.main_status[sizeof(g_status_data.main_status) - 1] = '\0';
        
        strncpy(g_status_data.sub_status, sub_status, sizeof(g_status_data.sub_status) - 1);
        g_status_data.sub_status[sizeof(g_status_data.sub_status) - 1] = '\0';
        
        g_status_data.color = color;
        
        xSemaphoreGive(g_status_data.status_mutex);
    ESP_LOGI(TAG, "Room sign status updated: %s - %s (color: #%06" PRIX32 ")", main_status, sub_status, color);
    }
}

// カメラフレームデータを更新する関数（hai_cameraからの呼び出し用）
// 注意: 現在は独立カメラ制御により無効化されています
void updateCameraFrameForStreaming(uint8_t* frameBuffer, uint32_t width, uint32_t height)
{
    // hal_cameraからのフレーム更新を無効化
    // 独立カメラ制御が常にカメラデータを配信するため、
    // hal_cameraからの更新は混乱を避けるために無視します
    
    static uint32_t ignored_count = 0;
    ignored_count++;
    if (ignored_count == 1 || ignored_count % 300 == 0) { // 最初と10秒ごとにログ出力
        ESP_LOGI(TAG, "Ignored hal_camera frame update #%lu: %" PRIu32 "x%" PRIu32 " (using direct camera instead)", 
                ignored_count, width, height);
    }
    
    // フレーム更新は行わない（direct_camera_capture_taskが独立して処理）
}

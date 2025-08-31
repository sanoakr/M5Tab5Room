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
#include "driver/jpeg_types.h"
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
#include <sys/time.h>
#include "shared/shared.h"

#define TAG "wifi"

// Forward declaration for room sign updater used below
extern void updateRoomSignStatus(const char* main_status, const char* sub_status, uint32_t color);

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

#define WIFI_AP_FALLBACK_SSID    "M5Tab5-UserDemo-WiFi"
#define WIFI_AP_FALLBACK_PASS    ""
#define MAX_STA_CONN 4

// ビルド時に生成された Wi-Fi 認証情報
#include "wifi_credentials.h"
#define JPEG_ENC_QUALITY 60  // 電力消費軽減のため品質を下げる
#define PART_BOUNDARY "123456789000000000000987654321"

// JPEGファイル保存用グローバル変数
static uint8_t* stored_jpeg = nullptr;
static size_t stored_jpeg_size = 0;
static size_t stored_jpeg_buffer_size = 0;

// RGB565画像を上下反転する関数
esp_err_t flip_rgb565_image_vertical(uint8_t* image_data, uint32_t width, uint32_t height) {
    if (!image_data || width == 0 || height == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint16_t* pixels = (uint16_t*)image_data;
    uint32_t row_size = width * sizeof(uint16_t);
    
    // 一時バッファを確保して行をスワップ
    uint16_t* temp_row = (uint16_t*)malloc(row_size);
    if (!temp_row) {
        return ESP_ERR_NO_MEM;
    }
    
    // 上半分と下半分の行を交換
    for (uint32_t y = 0; y < height / 2; y++) {
        uint32_t top_row_offset = y * width;
        uint32_t bottom_row_offset = (height - 1 - y) * width;
        
        // 上の行を一時保存
        memcpy(temp_row, &pixels[top_row_offset], row_size);
        
        // 下の行を上に移動
        memcpy(&pixels[top_row_offset], &pixels[bottom_row_offset], row_size);
        
        // 一時保存した上の行を下に移動
        memcpy(&pixels[bottom_row_offset], temp_row, row_size);
    }
    
    free(temp_row);
    return ESP_OK;
}

// 実際のカメラから画像を取得してJPEGにエンコードする関数
esp_err_t capture_camera_frame_to_jpeg(size_t* jpeg_size)
{
    ESP_LOGI(TAG, "Starting camera frame capture");
    
    if (g_camera_frame.camera_fd <= 0) {
        ESP_LOGE(TAG, "Camera not initialized, fd: %d", g_camera_frame.camera_fd);
        return ESP_ERR_INVALID_STATE;
    }
    
    // カメラから最新フレームを取得
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = MEMORY_TYPE;
    
    // デキューバッファ
    if (ioctl(g_camera_frame.camera_fd, VIDIOC_DQBUF, &buf) == -1) {
        ESP_LOGE(TAG, "Failed to dequeue camera buffer");
        return ESP_FAIL;
    }
    
    if (g_camera_frame.v4l2_camera && buf.index < EXAMPLE_VIDEO_BUFFER_COUNT) {
        uint8_t* frame_data = g_camera_frame.v4l2_camera->buffer[buf.index];
        size_t frame_size = buf.bytesused;
        
        ESP_LOGI(TAG, "Camera frame captured, size: %d bytes", (int)frame_size);
        
        // 画像を上下反転（逆さま問題の修正）
        esp_err_t flip_ret = flip_rgb565_image_vertical(frame_data, 
                                                       g_camera_frame.v4l2_camera->width,
                                                       g_camera_frame.v4l2_camera->height);
        if (flip_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to flip image vertically: %s", esp_err_to_name(flip_ret));
        } else {
            ESP_LOGI(TAG, "Image flipped vertically to correct orientation");
        }
        
        // JPEGエンコード
        if (g_camera_frame.jpeg_handle && stored_jpeg != nullptr && stored_jpeg_buffer_size > 0) {
            ESP_LOGI(TAG, "Starting JPEG encoding - buffer: %p, size: %d", stored_jpeg, (int)stored_jpeg_buffer_size);
            
            jpeg_encode_cfg_t encode_cfg = {
                .height = g_camera_frame.v4l2_camera->height,
                .width = g_camera_frame.v4l2_camera->width,
                .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
                .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
                .image_quality = 60
            };
            
            uint32_t output_size = 0;
            esp_err_t ret = jpeg_encoder_process(g_camera_frame.jpeg_handle,
                                               &encode_cfg,
                                               frame_data,
                                               frame_size,
                                               stored_jpeg,
                                               stored_jpeg_buffer_size,
                                               &output_size);
            
            if (ret == ESP_OK && output_size > 0) {
                *jpeg_size = output_size;
                stored_jpeg_size = *jpeg_size;
                ESP_LOGI(TAG, "JPEG encoding successful, size: %d bytes", (int)*jpeg_size);
            } else {
                ESP_LOGE(TAG, "JPEG encoding failed, error: %s", esp_err_to_name(ret));
                *jpeg_size = 0;
                ret = ESP_FAIL;
            }
            
            // バッファを再度キューに戻す
            if (ioctl(g_camera_frame.camera_fd, VIDIOC_QBUF, &buf) == -1) {
                ESP_LOGW(TAG, "Failed to requeue camera buffer");
            }
            
            return ret;
        } else {
            if (g_camera_frame.jpeg_handle == nullptr) {
                ESP_LOGE(TAG, "JPEG encoder not initialized");
            } else if (stored_jpeg == nullptr) {
                ESP_LOGE(TAG, "JPEG encoder memory not allocated");
            } else if (stored_jpeg_buffer_size == 0) {
                ESP_LOGE(TAG, "JPEG encoder buffer size is zero");
            }
            // バッファを再度キューに戻す
            if (ioctl(g_camera_frame.camera_fd, VIDIOC_QBUF, &buf) == -1) {
                ESP_LOGW(TAG, "Failed to requeue camera buffer");
            }
            return ESP_ERR_INVALID_STATE;
        }
    } else {
        ESP_LOGE(TAG, "Invalid camera buffer or index");
        return ESP_FAIL;
    }
}

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
    
    uint32_t frame_counter = 0;  // 変数宣言を前に移動
    struct v4l2_buffer buf;
    
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

// 簡易SVGテストパターンを生成（ブラウザーで確実に表示できる形式）
static const uint8_t* generate_test_image(size_t* image_size)
{
    static uint8_t test_image_data[10240]; // 10KB程度のテストデータ
    
    // SVG形式のテスト画像を生成
    const char* svg_content = 
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
        "<svg width=\"320\" height=\"240\" xmlns=\"http://www.w3.org/2000/svg\">"
        "<defs>"
        "<linearGradient id=\"grad1\" x1=\"0%\" y1=\"0%\" x2=\"100%\" y2=\"100%\">"
        "<stop offset=\"0%\" style=\"stop-color:#ff6b6b;stop-opacity:1\" />"
        "<stop offset=\"50%\" style=\"stop-color:#4ecdc4;stop-opacity:1\" />"
        "<stop offset=\"100%\" style=\"stop-color:#45b7d1;stop-opacity:1\" />"
        "</linearGradient>"
        "</defs>"
        "<rect width=\"100%\" height=\"100%\" fill=\"url(#grad1)\"/>"
        "<circle cx=\"160\" cy=\"120\" r=\"40\" fill=\"white\" opacity=\"0.8\"/>"
        "<text x=\"160\" y=\"130\" font-family=\"Arial, sans-serif\" font-size=\"16\" "
        "fill=\"#333\" text-anchor=\"middle\">📷 Test</text>"
        "<text x=\"160\" y=\"200\" font-family=\"Arial, sans-serif\" font-size=\"12\" "
        "fill=\"white\" text-anchor=\"middle\">M5Stack TAB5 Camera</text>"
        "</svg>";
    
    size_t content_length = strlen(svg_content);
    if (content_length <= sizeof(test_image_data)) {
        memcpy(test_image_data, svg_content, content_length);
        *image_size = content_length;
        return test_image_data;
    }
    
    // エラー時はnullptrを返す
    *image_size = 0;
    return nullptr;
}

// 静止画取得API（実際のカメラからJPEG画像を取得して保存）
esp_err_t camera_capture_handler(httpd_req_t* req)
{
    ESP_LOGI(TAG, "Camera capture request - capturing real camera image");
    
    // カメラから画像を取得
    size_t image_size = 0;
    esp_err_t ret = capture_camera_frame_to_jpeg(&image_size);
    
    if (ret == ESP_OK && image_size > 0) {
        // 成功レスポンスを返す（画像ファイルパスを含む）
        struct timeval tv = {};
        gettimeofday(&tv, NULL);
        int64_t timestamp = (int64_t)tv.tv_sec * 1000LL + (tv.tv_usec / 1000);
        char json_response[256];
        snprintf(json_response, sizeof(json_response), 
                "{"
                "\"success\": true,"
                "\"image_url\": \"/snapshot.jpg\","
                "\"timestamp\": %lld,"
                "\"size\": %d,"
                "\"message\": \"Camera image captured\""
                "}", 
                timestamp, (int)image_size);
        
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
        
        ESP_LOGI(TAG, "Camera image captured, size: %d bytes, stored in global variable", (int)image_size);
        ESP_LOGI(TAG, "Global stored_jpeg_size after capture: %d", (int)stored_jpeg_size);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Camera capture failed, error: %s", esp_err_to_name(ret));
        
        char error_response[] = "{\"success\": false, \"message\": \"Camera capture failed\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, error_response, HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }
}

// JPEG画像配信ハンドラー
esp_err_t image_file_handler(httpd_req_t* req)
{
    ESP_LOGI(TAG, "JPEG image file request received - stored_jpeg_size: %d", (int)stored_jpeg_size);
    
    if (stored_jpeg_size > 0) {
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, (const char*)stored_jpeg, stored_jpeg_size);
        
        ESP_LOGI(TAG, "JPEG image sent, size: %d bytes", (int)stored_jpeg_size);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "No JPEG data available - stored_jpeg_size is 0");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
}

// ステータス更新API（GET: 現在値取得）
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

// 受信した mode に応じてメイン画面相当の処理（ステータス更新）を適用
static void apply_status_mode(const char* mode)
{
    if (!mode) return;

    const char* main_text = "";
    const char* sub_text  = "";
    uint32_t color        = 0xFFFFFF;

    // view.cpp のボタンに合わせたマッピング
    if (strcmp(mode, "present") == 0) {
        main_text = "在室中です";
        sub_text  = "";
        color     = 0x66BB6A; // 緑
    } else if (strcmp(mode, "absent") == 0) {
        main_text = "不在です";
        sub_text  = "（学外にいます）";
        color     = 0xEF5350; // 赤
    } else if (strcmp(mode, "campus") == 0) {
        main_text = "学内にいます";
        sub_text  = "";
        color     = 0xFFB74D; // オレンジ
    } else if (strcmp(mode, "meeting") == 0) {
        main_text = "ミーティング中";
        sub_text  = "（在室しています）";
        color     = 0x42A5F5; // 青
    } else if (strcmp(mode, "online") == 0) {
        main_text = "オンライン中です";
        sub_text  = "（オンライン会議・授業中）";
        color     = 0xAB47BC; // 紫
    } else {
        ESP_LOGW(TAG, "Unknown status mode: %s", mode);
        return;
    }

    // UI層で処理させるため、共有キューに投入（UIが適用後にHAL経由でWebへ反映）
    shared_data::SharedData_t::RoomStatus st{main_text, sub_text, color};
    EnqueueRoomStatus(st);
}

// ステータス更新API（POST: 状態変更）
esp_err_t status_api_post_handler(httpd_req_t* req)
{
    // CORS 許可
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    // JSON もしくは form-urlencoded の簡易パース
    int total_len = req->content_len;
    if (total_len <= 0 || total_len > 512) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid content length");
        return ESP_FAIL;
    }

    char buf[513];
    int received = 0;
    while (received < total_len) {
        int r = httpd_req_recv(req, buf + received, total_len - received);
        if (r <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read body");
            return ESP_FAIL;
        }
        received += r;
    }
    buf[received] = '\0';

    // mode=present あるいは {"mode":"present"} に対応
    const char* mode_value = nullptr;
    if (strstr(buf, "\"mode\"")) {
        // とても簡易な JSON 抽出
        char* p = strstr(buf, "\"mode\"");
        p       = strchr(p, ':');
        if (p) {
            // コロンの後の引用符を探す
            char* q1 = strchr(p, '"');
            if (q1) {
                char* q2 = strchr(q1 + 1, '"');
                if (q2 && (q2 - q1 - 1) < 32) {
                    static char mode_buf[32];
                    int len = q2 - q1 - 1;
                    strncpy(mode_buf, q1 + 1, len);
                    mode_buf[len] = '\0';
                    mode_value     = mode_buf;
                }
            }
        }
    }
    if (!mode_value) {
        // URL エンコード形式 mode=...
        char* m = strstr(buf, "mode=");
        if (m) {
            m += 5;
            static char mode_buf2[32];
            int i = 0;
            while (*m && *m != '&' && i < (int)sizeof(mode_buf2) - 1) {
                mode_buf2[i++] = *m++;
            }
            mode_buf2[i] = '\0';
            mode_value   = mode_buf2;
        }
    }

    if (!mode_value) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing mode");
        return ESP_FAIL;
    }

    apply_status_mode(mode_value);

    // 現在値を返す
    return status_api_handler(req);
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
                    justify-content: flex-start;
                    align-items: center;
                    min-height: 100vh;
                    margin: 0;
                    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
                    background-color: #f7f7f7;
                    padding: 8px 6px;
                }
                h1 {
                    font-size: 20px;
                    color: #333;
                    margin: 6px 0 8px 0;
                }
                p {
                    font-size: 12px;
                    color: #666;
                    margin: 4px 0 8px 0;
                }
                #camera-snapshot {
                    max-width: 92vw;
                    width: 280px;
                    height: 180px;
                    border: 1px solid #333;
                    border-radius: 6px;
                    background: #f0f0f0;
                    display: block;
                    margin-bottom: 8px;
                    object-fit: cover;
                }
                .snapshot-info {
                    font-size: 11px;
                    color: #666;
                    text-align: center;
                    margin-top: 4px;
                }
                .status-display {
                    width: 100%;
                    max-width: 420px;
                    margin-top: 8px;
                    padding: 10px 14px;
                    background: #fff;
                    border-radius: 10px;
                    box-shadow: 0 1px 6px rgba(0,0,0,0.06);
                    text-align: center;
                }
                .status-name {
                    font-size: 14px;
                    color: #666;
                    margin-bottom: 6px;
                }
                .status-main {
                    font-size: 28px;
                    font-weight: bold;
                    margin-bottom: 6px;
                    color: #EF5350;
                }
                .status-sub {
                    font-size: 16px;
                    color: #666;
                }
                .button-bar {
                    margin-top: 10px;
                    display: flex;
                    gap: 6px;
                    flex-wrap: nowrap; /* 一列固定 */
                    justify-content: center;
                    width: 100%;
                    max-width: 420px;
                }
                .btn {
                    border: none;
                    color: #fff;
                    padding: 6px 6px; /* 横パディング縮小 */
                    border-radius: 8px;
                    font-size: 13px;  /* 文字サイズ微縮小 */
                    cursor: pointer;
                    box-sizing: border-box; /* 幅計算にボーダー含める */
                    text-align: center;
                    white-space: nowrap;
                    flex: 0 0 calc((100% - 24px) / 5); /* 5等分（gap 6px * 4 = 24px）*/
                }
                .btn-green { background: #4CAF50; border: 2px solid #2E7D32; }
                .btn-red { background: #F44336; border: 2px solid #C62828; }
                .btn-orange { background: #FF9800; border: 2px solid #E65100; }
                .btn-blue { background: #2196F3; border: 2px solid #1565C0; }
                .btn-purple { background: #9C27B0; border: 2px solid #6A1B9A; }
                .camera-card {
                    display: flex;
                    flex-direction: column;
                    align-items: center;
                    justify-content: center;
                    width: 100%;
                    max-width: 420px;
                    margin-top: 8px;
                    background: #fff;
                    border-radius: 10px;
                    box-shadow: 0 1px 6px rgba(0,0,0,0.06);
                    padding: 10px 14px;
                }
            </style>
            <script>
                function setStatus(mode) {
                    // UI層に適用依頼だけを送る。Web上の表示はポーリングで反映。
                    fetch('/api/status', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify({ mode })
                    }).catch(err => console.error('Set status error:', err));
                }
                function captureSnapshot() {
                    const img = document.getElementById('camera-snapshot');
                    const info = document.getElementById('snapshot-info');
                    
                    info.textContent = 'スナップショット取得中...';
                    
                    fetch('/camera_capture')
                        .then(response => response.json())
                        .then(data => {
                            if (data.success) {
                                // JPEGファイルのURLにタイムスタンプを追加してキャッシュを回避
                                img.src = data.image_url + '?t=' + data.timestamp;
                                const now = new Date(Number(data.timestamp));
                                // JST(+09:00)での表示を明示
                                const jst = now.toLocaleString('ja-JP', {
                                    timeZone: 'Asia/Tokyo',
                                    year: 'numeric',
                                    month: '2-digit',
                                    day: '2-digit',
                                    hour: '2-digit',
                                    minute: '2-digit',
                                    second: '2-digit',
                                    hour12: false,
                                });
                                info.textContent = `スナップショット撮影時刻: ${jst}`;
                            } else {
                                info.textContent = 'スナップショット取得に失敗しました';
                            }
                        })
                        .catch(error => {
                            console.error('Snapshot capture error:', error);
                            info.textContent = 'スナップショット取得エラー';
                        });
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
                    // 初回更新
                    updateStatus();
                    captureSnapshot();
                    // 5秒ごとにステータスとカメラ画像を更新
                    setInterval(function(){
                        updateStatus();
                        captureSnapshot();
                    }, 5000);
                });
            </script>
        </head>
        <body>
            <!-- タイトル -->
            <h1>Tab5 RoomSign</h1>
            <p style="margin-top:-2px;">From M5StackTab5</p>

            <!-- メッセージ -->
            <div class="status-display">
                <div id="status-name" class="status-name">さのは（おそらく）</div>
                <div id="status-main" class="status-main">不在です</div>
                <div id="status-sub" class="status-sub">（学外にいます）</div>
            </div>

            <!-- ボタン -->
            <div class="button-bar">
                <button class="btn btn-green" onclick="setStatus('present')">在室</button>
                <button class="btn btn-red" onclick="setStatus('absent')">不在</button>
                <button class="btn btn-orange" onclick="setStatus('campus')">学内</button>
                <button class="btn btn-blue" onclick="setStatus('meeting')">ミーティング</button>
                <button class="btn btn-purple" onclick="setStatus('online')">オンライン</button>
            </div>

            <!-- カメラ画像（5秒ごとに自動更新） -->
            <div class="camera-card">
                <img id="camera-snapshot" alt="Camera Snapshot" src="" style="background: linear-gradient(45deg, #ff6b6b, #4ecdc4); display: flex; align-items: center; justify-content: center;">
                <div id="snapshot-info" class="snapshot-info">スナップショット自動更新中...</div>
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
httpd_uri_t camera_capture_uri = {.uri = "/camera_capture", .method = HTTP_GET, .handler = camera_capture_handler, .user_ctx = nullptr};
httpd_uri_t image_file_uri = {.uri = "/snapshot.jpg", .method = HTTP_GET, .handler = image_file_handler, .user_ctx = nullptr};
httpd_uri_t status_api_uri = {.uri = "/api/status", .method = HTTP_GET, .handler = status_api_handler, .user_ctx = nullptr};
httpd_uri_t status_api_post_uri = {.uri = "/api/status", .method = HTTP_POST, .handler = status_api_post_handler, .user_ctx = nullptr};

// 启动 Web Server
httpd_handle_t start_webserver()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8; // ハンドラ数を増やす
    httpd_handle_t server = nullptr;

        if (httpd_start(&server, &config) == ESP_OK) {
            httpd_register_uri_handler(server, &hello_uri);
            httpd_register_uri_handler(server, &camera_stream_uri);
            httpd_register_uri_handler(server, &camera_capture_uri);
            httpd_register_uri_handler(server, &image_file_uri);
            httpd_register_uri_handler(server, &status_api_uri);
            httpd_register_uri_handler(server, &status_api_post_uri);
        
        // ステータス管理用のmutex初期化
        if (g_status_data.status_mutex == NULL) {
            g_status_data.status_mutex = xSemaphoreCreateMutex();
        }
        
        // フレーム共有用のmutex初期化
        if (g_camera_frame.frame_mutex == NULL) {
            g_camera_frame.frame_mutex = xSemaphoreCreateMutex();
        }
        
        // カメラ機能を有効化
        ESP_LOGI(TAG, "Camera initialization starting...");
        
        // ESP32P4 Video systemを初期化（hal_cameraと同じ方法）
        ESP_LOGI(TAG, "Initializing ESP32P4 V4L2 camera system");
        
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
                g_camera_frame.v4l2_camera_initialized = false;
                g_camera_frame.camera_initialized = false;
                g_camera_frame.camera_running = false;
            } else {
                ESP_LOGI(TAG, "ESP video system initialized successfully");
                g_camera_frame.v4l2_camera_initialized = true;
            }
        }
        
        // カメラの初期化
        int camera_fd = -1;
        if (g_camera_frame.v4l2_camera_initialized) {
            camera_fd = wifi_video_open(CAM_DEV_PATH, EXAMPLE_VIDEO_FMT_RGB565);
        }
        
        if (camera_fd < 0) {
            ESP_LOGE(TAG, "Failed to open camera");
        } else {
            ESP_LOGI(TAG, "Camera opened successfully, fd: %d", camera_fd);
            g_camera_frame.camera_fd = camera_fd;
            
            // V4L2カメラ構造体を初期化
            ESP_LOGI(TAG, "Initializing camera structure...");
            esp_err_t cam_init_result = wifi_new_cam(camera_fd, &g_camera_frame.v4l2_camera);
            if (cam_init_result != ESP_OK) {
                ESP_LOGE(TAG, "Failed to initialize camera: %s", esp_err_to_name(cam_init_result));
                close(camera_fd);
                g_camera_frame.camera_fd = -1;
                g_camera_frame.v4l2_camera_initialized = false;
            } else {
                // PPA（Picture Processing Accelerator）を初期化
                ppa_client_config_t ppa_config = {
                    .oper_type = PPA_OPERATION_SRM,
                    .max_pending_trans_num = 1,
                    .data_burst_length = PPA_DATA_BURST_LENGTH_128
                };
                esp_err_t ppa_result = ppa_register_client(&ppa_config, &g_camera_frame.ppa_handle);
                if (ppa_result != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to register PPA client: %s", esp_err_to_name(ppa_result));
                }
                
                g_camera_frame.camera_initialized = true;
                g_camera_frame.camera_running = true;
                
                ESP_LOGI(TAG, "V4L2 camera system initialized successfully - Real camera ready!");
                ESP_LOGI(TAG, "Camera format: %" PRIu32 "x%" PRIu32 ", pixel_format: 0x%" PRIx32, 
                        g_camera_frame.v4l2_camera->width, g_camera_frame.v4l2_camera->height, 
                        g_camera_frame.v4l2_camera->pixel_format);
            }
        }
        
        // JPEGエンコーダーの初期化
        jpeg_encoder_handle_t jpeg_handle = NULL;
        jpeg_encode_engine_cfg_t jpeg_engine_cfg = {
            .intr_priority = 0,
            .timeout_ms = 40,
        };
        ESP_ERROR_CHECK(jpeg_new_encoder_engine(&jpeg_engine_cfg, &jpeg_handle));
        g_camera_frame.jpeg_handle = jpeg_handle;
        ESP_LOGI(TAG, "JPEG encoder initialized");
        
        // JPEGエンコード用のアライメントされたバッファを割り当て
        stored_jpeg_buffer_size = 1024 * 1024; // 1MB
        ESP_LOGI(TAG, "Allocating JPEG encoder memory: %d bytes", (int)stored_jpeg_buffer_size);
        
        // jpeg_alloc_encoder_memの正しい使用法
        jpeg_encode_memory_alloc_cfg_t mem_cfg = {
            .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER
        };
        size_t actual_size = 0;
        stored_jpeg = (uint8_t*)jpeg_alloc_encoder_mem(stored_jpeg_buffer_size, &mem_cfg, &actual_size);
        
        if (stored_jpeg == NULL) {
            ESP_LOGE(TAG, "Failed to allocate JPEG encoder memory");
            // フォールバックとして通常のmallocを試す
            stored_jpeg = (uint8_t*)malloc(stored_jpeg_buffer_size);
            if (stored_jpeg == NULL) {
                ESP_LOGE(TAG, "Failed to allocate JPEG encoder memory with malloc as well");
                stored_jpeg_buffer_size = 0;
            } else {
                ESP_LOGW(TAG, "Using malloc for JPEG encoder memory (may have alignment issues)");
            }
        } else {
            stored_jpeg_buffer_size = actual_size;  // 実際に割り当てられたサイズを使用
            ESP_LOGI(TAG, "JPEG encoder memory allocated successfully: %d bytes", (int)stored_jpeg_buffer_size);
        }
        
        // カメラ自動取得タスクを開始
        g_camera_frame.auto_capture_enabled = true;
        xTaskCreate(camera_auto_capture_task, "camera_auto_capture", 4096, NULL, 5, NULL);
        ESP_LOGI(TAG, "Camera auto capture task started");
        
        ESP_LOGI(TAG, "WiFi AP and streaming server started - REAL CAMERA mode");
        ESP_LOGI(TAG, "Camera streaming enabled - web page will show real camera images");
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
    std::strncpy(reinterpret_cast<char*>(wifi_config.ap.ssid), WIFI_AP_FALLBACK_SSID, sizeof(wifi_config.ap.ssid));
    std::strncpy(reinterpret_cast<char*>(wifi_config.ap.password), WIFI_AP_FALLBACK_PASS, sizeof(wifi_config.ap.password));
    wifi_config.ap.ssid_len       = std::strlen(WIFI_AP_FALLBACK_SSID);
    wifi_config.ap.max_connection = MAX_STA_CONN;
    wifi_config.ap.authmode       = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP (fallback) started. SSID:%s password:%s", WIFI_AP_FALLBACK_SSID, WIFI_AP_FALLBACK_PASS);
}

// 既存APへ接続（STA）
static void wifi_init_station(const char* ssid, const char* pass)
{
    esp_err_t err;
    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    wifi_config_t wifi_config = {};
    std::strncpy(reinterpret_cast<char*>(wifi_config.sta.ssid), ssid, sizeof(wifi_config.sta.ssid));
    std::strncpy(reinterpret_cast<char*>(wifi_config.sta.password), pass ? pass : "", sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.sae_pwe_h2e        = WPA3_SAE_PWE_BOTH;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    // ハンドラ: STA_START で connect、GOT_IP でIP表示
    esp_event_handler_instance_t any_id, got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, [](void* arg, esp_event_base_t, int32_t id, void*) {
        if (id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
            ESP_LOGI(TAG, "Wi-Fi STA start, connecting...");
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGW(TAG, "Wi-Fi disconnected, reconnecting...");
            esp_wifi_connect();
        }
    }, nullptr, &any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, [](void*, esp_event_base_t, int32_t, void* data) {
        auto* event = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        // NTP 同期は一旦無効化
    }, nullptr, &got_ip));

    ESP_LOGI(TAG, "Wi-Fi STA connecting to SSID:%s", ssid);
}

static void wifi_connect_task(void* param)
{
    const char* ssid = WIFI_BUILD_SSID;
    const char* pass = WIFI_BUILD_PASS;
    if (ssid && ssid[0] != '\0') {
        wifi_init_station(ssid, pass);
    } else {
        ESP_LOGW(TAG, "Falling back to SoftAP mode (no build-time SSID)");
        wifi_init_softap();
    }

    // Web サーバ開始
    start_webserver();
    ESP_LOGI(TAG, "Web server started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

// JPEGエンコーダーメモリを解放する関数
void cleanup_jpeg_encoder() {
    if (stored_jpeg != nullptr) {
        free(stored_jpeg);
        stored_jpeg = nullptr;
        stored_jpeg_size = 0;
        stored_jpeg_buffer_size = 0;
        ESP_LOGI(TAG, "JPEG encoder memory freed");
    }
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

    xTaskCreate(wifi_connect_task, "wifi", 8192, nullptr, 5, nullptr);
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

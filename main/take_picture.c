#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_heap_caps.h" // For PSRAM (heap_caps_malloc)

// Includes for Wi-Fi and Web Server
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"

// --- Wi-Fi Configuration ---
// Set these to your network credentials
#define WIFI_SSID      "Jordan's Wifi"
#define WIFI_PASS      "Under_Dog-172"
// ----------------------------


// Pin definitions for ESP32-CAM (AI-Thinker module)
#define CAM_PIN_PWDN    32 // Power down pin, enable camera
#define CAM_PIN_RESET   -1 // Software reset (not used)
#define CAM_PIN_XCLK    0  // XCLK output (to sensor)
#define CAM_PIN_SIOD    26 // I2C SDA
#define CAM_PIN_SIOC    27 // I2C SCL
#define CAM_PIN_D7      35 // Y9 data bit
#define CAM_PIN_D6      34 // Y8 data bit
#define CAM_PIN_D5      39 // Y7 data bit
#define CAM_PIN_D4      36 // Y6 data bit
#define CAM_PIN_D3      21 // Y5 data bit
#define CAM_PIN_D2      19 // Y4 data bit
#define CAM_PIN_D1      18 // Y3 data bit
#define CAM_PIN_D0      5  // Y2 data bit
#define CAM_PIN_VSYNC   25 // VSYNC signal
#define CAM_PIN_HREF    23 // HREF signal
#define CAM_PIN_PCLK    22 // Pixel clock signal

// FAST algorithm parameters and buffer sizes
#define FAST_THRESHOLD   40     // Intensity threshold (t) for FAST corner detection
#define FAST_MIN_CONTIG  12     // Require 12 contiguous pixels on circle (FAST-12)
#define MAX_KEYPOINTS    2048   // Maximum number of keypoints to detect/store

// Data structure for a keypoint (similar to OpenCV KeyPoint x,y,score)
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t score;
} Keypoint;

// Static buffers for image processing
static Keypoint keypoints[MAX_KEYPOINTS];
static bool suppressed_arr[MAX_KEYPOINTS];

// Global buffer for grayscale image (will be allocated in PSRAM)
static uint8_t *gray_buf = NULL;
static int img_width = 0;
static int img_height = 0;

static const char *TAG_CAM = "CAM";
static const char *TAG_WIFI = "WIFI";
static const char *TAG_SRV = "HTTP";
static const char *TAG_FAST = "FAST";

// MJPEG streaming boundary
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";


// Comparator for qsort: sort keypoints by score in descending order (higher score first)
static int compare_keypoints_desc(const void *a, const void *b) {
    const Keypoint *ka = (const Keypoint *)a;
    const Keypoint *kb = (const Keypoint *)b;
    if (ka->score < kb->score) return 1;
    if (ka->score > kb->score) return -1;
    return 0;
}

/**
 * @brief Extracts the Y (luminance) channel from a YUV422 frame
 */
static void extract_grayscale(camera_fb_t *fb) {
    if (!gray_buf) return;
    
    size_t len = fb->len;
    size_t gray_index = 0;
    for (size_t i = 0; i < len; i += 2) {
        gray_buf[gray_index++] = fb->buf[i];
    }
}

/**
 * @brief Draws the detected keypoints onto the YUV422 frame buffer
 */
static void draw_keypoints(camera_fb_t *fb, Keypoint *kps, int num_kp) {
    if (!fb || !kps) return;

    // Draw a 3x3 white square for each keypoint
    for (int i = 0; i < num_kp; ++i) {
        int x = kps[i].x;
        int y = kps[i].y;

        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int px = x + dx;
                int py = y + dy;
                // Check bounds
                if (px >= 0 && px < fb->width && py >= 0 && py < fb->height) {
                    // Calculate the index for the Y byte in the YUV422 buffer
                    // YUV422 format: [Y0, U0, Y1, V0], [Y2, U1, Y3, V1], ...
                    // Index of Y for pixel (px, py) is (py * width * 2) + (px * 2)
                    size_t y_index = (py * fb->width + px) * 2;
                    fb->buf[y_index] = 255; // Set Luminance (Y) to max (white)
                }
            }
        }
    }
}


/**
 * @brief Runs FAST-12 detection, qsort, and NMS on the global gray_buf
 * @param out_num_kp Pointer to store the final number of keypoints
 */
static void run_fast_and_nms(int *out_num_kp) {
    uint8_t *image = gray_buf;
    int width = img_width;
    int height = img_height;
    int threshold = FAST_THRESHOLD;
    int num_kp = 0;

    const int8_t offset_x[16] = {  0,  1,  2,  3,  3,  3,  2,  1,  0, -1, -2, -3, -3, -3, -2, -1 };
    const int8_t offset_y[16] = {  3,  3,  2,  1,  0, -1, -2, -3, -3, -3, -2, -1,  0,  1,  2,  3 };

    for (int y = 3; y < height - 3; ++y) {
        for (int x = 3; x < width - 3; ++x) {
            uint8_t I_p = image[y * width + x];
            uint8_t I_p_t_high = (uint8_t)((I_p + threshold) > 255 ? 255 : I_p + threshold);
            uint8_t I_p_t_low  = (uint8_t)((I_p < threshold) ? 0 : I_p - threshold);

            int bright = 0;
            int dark = 0;
            const int sample_idx[4] = {1, 5, 9, 13};
            for (int k = 0; k < 4; ++k) {
                int idx = sample_idx[k];
                uint8_t val = image[(y + offset_y[idx]) * width + (x + offset_x[idx])];
                if (val > I_p_t_high) bright++;
                if (val < I_p_t_low)  dark++;
            }
            if (bright < 3 && dark < 3) {
                continue;
            }

            bool is_corner = false;
            for (int start = 0; start < 16; ++start) {
                if (image[(y + offset_y[start]) * width + (x + offset_x[start])] > I_p_t_high) {
                    int count = 1;
                    for (int k = 1; k < FAST_MIN_CONTIG; ++k) {
                        int idx = (start + k) & 0xF;
                        if (image[(y + offset_y[idx]) * width + (x + offset_x[idx])] > I_p_t_high) count++;
                        else break;
                    }
                    if (count >= FAST_MIN_CONTIG) {
                        is_corner = true;
                        break;
                    }
                }
                if (image[(y + offset_y[start]) * width + (x + offset_x[start])] < I_p_t_low) {
                    int count = 1;
                    for (int k = 1; k < FAST_MIN_CONTIG; ++k) {
                        int idx = (start + k) & 0xF;
                        if (image[(y + offset_y[idx]) * width + (x + offset_x[idx])] < I_p_t_low) count++;
                        else break;
                    }
                    if (count >= FAST_MIN_CONTIG) {
                        is_corner = true;
                        break;
                    }
                }
            }
            if (!is_corner) {
                continue;
            }

            int score = 0;
            for (int i = 0; i < 16; ++i) {
                int neighbor_val = image[(y + offset_y[i]) * width + (x + offset_x[i])];
                int diff = neighbor_val - I_p;
                if (diff < 0) diff = -diff;
                score += diff;
            }

            if (num_kp < MAX_KEYPOINTS) {
                keypoints[num_kp].x = x;
                keypoints[num_kp].y = y;
                keypoints[num_kp].score = (uint16_t)(score & 0xFFFF);
                num_kp++;
            }
        }
    }

    if (num_kp > 1) {
        qsort(keypoints, num_kp, sizeof(Keypoint), compare_keypoints_desc);
    }
    
    memset(suppressed_arr, 0, num_kp * sizeof(bool));
    for (int i = 0; i < num_kp; ++i) {
        if (suppressed_arr[i]) continue;
        for (int j = i + 1; j < num_kp; ++j) {
            if (suppressed_arr[j]) continue;
            int dx = keypoints[j].x - keypoints[i].x;
            int dy = keypoints[j].y - keypoints[i].y;
            if (dx >= -1 && dx <= 1 && dy >= -1 && dy <= 1) {
                suppressed_arr[j] = true;
            }
        }
    }
    
    int new_count = 0;
    for (int i = 0; i < num_kp; ++i) {
        if (!suppressed_arr[i]) {
            keypoints[new_count++] = keypoints[i];
        }
    }
    num_kp = new_count;

    ESP_LOGI(TAG_FAST, "Detected %d keypoints", num_kp);
    *out_num_kp = num_kp;
}

/**
 * @brief Initialize the camera and allocate PSRAM buffer
 */
static esp_err_t camera_init() {
    camera_config_t cam_config;
    cam_config.pin_pwdn  = CAM_PIN_PWDN;
    cam_config.pin_reset = CAM_PIN_RESET;
    cam_config.pin_xclk  = CAM_PIN_XCLK;
    cam_config.pin_sscb_sda = CAM_PIN_SIOD;
    cam_config.pin_sscb_scl = CAM_PIN_SIOC;
    cam_config.pin_d7 = CAM_PIN_D7;
    cam_config.pin_d6 = CAM_PIN_D6;
    cam_config.pin_d5 = CAM_PIN_D5;
    cam_config.pin_d4 = CAM_PIN_D4;
    cam_config.pin_d3 = CAM_PIN_D3;
    cam_config.pin_d2 = CAM_PIN_D2;
    cam_config.pin_d1 = CAM_PIN_D1;
    cam_config.pin_d0 = CAM_PIN_D0;
    cam_config.pin_vsync = CAM_PIN_VSYNC;
    cam_config.pin_href  = CAM_PIN_HREF;
    cam_config.pin_pclk  = CAM_PIN_PCLK;
    cam_config.xclk_freq_hz = 20000000;
    cam_config.ledc_timer   = LEDC_TIMER_0;
    cam_config.ledc_channel = LEDC_CHANNEL_0;
    cam_config.pixel_format = PIXFORMAT_YUV422; // Must use raw format for processing
    cam_config.frame_size   = FRAMESIZE_QVGA;   // 320x240
    cam_config.jpeg_quality = 12; // Irrelevant for YUV, but set low
    cam_config.fb_count     = 2;
    cam_config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&cam_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_CAM, "Camera init failed with error 0x%x", err);
        return err;
    }

    // OV3660 grayscale special effect workaround
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        sensor->set_special_effect(sensor, 2); // 2 = Grayscale effect
        ESP_LOGI(TAG_CAM, "Sensor special effect set to grayscale");
    }

    // Get frame dimensions
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG_CAM, "Camera capture failed on first frame");
        return ESP_FAIL;
    }
    img_width  = fb->width;
    img_height = fb->height;
    size_t img_size = img_width * img_height;
    esp_camera_fb_return(fb); // return the test frame

    // *** MEMORY FIX ***
    // Allocate the grayscale buffer in PSRAM
    gray_buf = heap_caps_malloc(img_size, MALLOC_CAP_SPIRAM);
    if (!gray_buf) {
        ESP_LOGE(TAG_CAM, "Failed to allocate grayscale image buffer in PSRAM");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG_CAM, "Grayscale buffer (%.1f KB) allocated in PSRAM", img_size / 1024.0);
    
    return ESP_OK;
}

/**
 * @brief HTTP handler for the MJPEG stream
 * This is now the main processing loop.
 */
static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t jpg_len = 0;
    uint8_t *jpg_buf = NULL;
    char *part_buf[64];

    // Set HTTP headers for MJPEG stream
    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    while (true) {
        int num_kp = 0;
        
        // 1. Capture a frame
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG_CAM, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        // 2. Extract luminance (Y) channel into grayscale buffer
        extract_grayscale(fb);

        // 3. Run FAST corner detection and NMS on the grayscale buffer
        run_fast_and_nms(&num_kp);

        // 4. Draw the final keypoints onto the YUV frame
        draw_keypoints(fb, keypoints, num_kp);

        // 5. Convert the modified YUV frame to JPEG
        // This is slow (software encoding) and is the main bottleneck!
        res = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
        if (res != ESP_OK) {
            ESP_LOGE(TAG_CAM, "JPEG conversion failed");
            esp_camera_fb_return(fb);
            break;
        }

        // 6. Send the MJPEG frame header
        size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, jpg_len);
        res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        if (res != ESP_OK) {
            break; // Client disconnected
        }

        // 7. Send the JPEG image data
        res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len);
        if (res != ESP_OK) {
            break; // Client disconnected
        }

        // 8. Send the MJPEG frame boundary
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if (res != ESP_OK) {
            break; // Client disconnected
        }

        // 9. Cleanup
        free(jpg_buf);
        jpg_buf = NULL;
        esp_camera_fb_return(fb);

        // Yield to other tasks (optional, but good practice)
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Cleanup if loop breaks
    if (jpg_buf) {
        free(jpg_buf);
    }
    return res;
}

/**
 * @brief Basic HTTP handler for the main page (/)
 */
static esp_err_t index_handler(httpd_req_t *req) {
    char* resp_str = "<html><head><title>ESP32-CAM FAST</title></head>"
                      "<body><h1>ESP32-CAM FAST Detector</h1>"
                      "<p>View the processed stream at /stream</p>"
                      // Embed the stream directly
                      "<img src=\"/stream\" width=\"320\" height=\"240\">"
                      "</body></html>";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/**
 * @brief Starts the web server
 */
static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // URI handler for the index page
    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    // URI handler for the MJPEG stream
    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    ESP_LOGI(TAG_SRV, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &stream_uri);
        return server;
    }

    ESP_LOGI(TAG_SRV, "Error starting server!");
    return NULL;
}

/**
 * @brief Event handler for Wi-Fi events
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG_WIFI, "connect to the AP fail");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_WIFI, "--------------------------------------------------");
        ESP_LOGI(TAG_WIFI, "Wi-Fi Connected!");
        ESP_LOGI(TAG_WIFI, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG_WIFI, "Open http://" IPSTR "/ in your browser", IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG_WIFI, "--------------------------------------------------");
    }
}

/**
 * @brief Initialize Wi-Fi in Station mode
 */
static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG_WIFI, "wifi_init_sta finished. Connecting to %s...", WIFI_SSID);
}

void app_main(void) {
    // Initialize NVS (required for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize camera and PSRAM buffer
    ESP_ERROR_CHECK(camera_init());

    // Initialize Wi-Fi
    wifi_init_sta();

    // Start the web server
    start_webserver();

    // app_main can now exit; the server and its handler run in their own tasks
    ESP_LOGI("MAIN", "Initialization complete. Server is running.");
}
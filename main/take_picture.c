// ESP32-CAM FAST detector with STA->SoftAP fallback (ESP-IDF)
// - Grayscale + FAST-12 + O(W*H) grid NMS + black 3x3 markers
// - Stream tuning via URL: /stream?t=40&q=60  (threshold & JPEG quality)
// - Tries STA ~10s, else SoftAP "ESP32-CAM-XXXXXX" / "12345678"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"          // <-- needed for esp_timer_get_time()

// ======================== CONFIG ========================
#define WIFI_SSID "YOUR_WIFI_SSID"     // 2.4 GHz only
#define WIFI_PASS "YOUR_WIFI_PASSWORD"
// ========================================================

// =================== ESP32-CAM (AI-Thinker) PINS ===================
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26 // SCCB SDA
#define CAM_PIN_SIOC    27 // SCCB SCL
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// ==================== FAST params ====================
#define FAST_MIN_CONTIG   12             // FAST-12
#define MAX_KEYPOINTS     512            // after NMS
static int g_fast_threshold = 40;        // runtime-tunable via URL

typedef struct {
    uint16_t x, y, score;
} Keypoint;

static Keypoint  keypoints[MAX_KEYPOINTS];
static uint16_t *score_grid = NULL;      // PSRAM: w*h grid of scores

static int g_w = 0, g_h = 0;

static const char *TAG = "CAM_HTTP";
static const int WIFI_CONNECTED_BIT = BIT0;
static EventGroupHandle_t s_wifi_event_group;

// ==================== MJPEG bits ====================
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART_HDR     = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ==================== FAST helpers ====================
static inline void draw_keypoints_gray(camera_fb_t *fb, const Keypoint *kps, int n) {
    uint8_t *buf = fb->buf;
    const int w = fb->width, h = fb->height;
    for (int i = 0; i < n; ++i) {
        const int x = kps[i].x, y = kps[i].y;
        for (int dy = -1; dy <= 1; ++dy) {
            const int yy = y + dy; if ((unsigned)yy >= (unsigned)h) continue;
            for (int dx = -1; dx <= 1; ++dx) {
                const int xx = x + dx; if ((unsigned)xx >= (unsigned)w) continue;
                buf[yy * w + xx] = 0; // black dot for visibility
            }
        }
    }
}

// FAST-12 detect into score_grid + 3x3 local-max NMS (O(W*H))
static void fast_detect_and_nms(const uint8_t *img, int w, int h, int *out_count) {
    if (!score_grid) { *out_count = 0; return; }
    memset(score_grid, 0, w * h * sizeof(uint16_t));

    const int t = g_fast_threshold;
    const int8_t ox[16] = { 0,  1,  2,  3,  3,  3,  2,  1,  0, -1, -2, -3, -3, -3, -2, -1 };
    const int8_t oy[16] = { 3,  3,  2,  1,  0, -1, -2, -3, -3, -3, -2, -1,  0,  1,  2,  3 };
    const int qi[4]     = { 1, 5, 9, 13 };

    // Detect
    for (int y = 3; y < h - 3; ++y) {
        const int yw = y * w;
        for (int x = 3; x < w - 3; ++x) {
            const uint8_t p = img[yw + x];
            const int hi = (p + t > 255) ? 255 : (p + t);
            const int lo = (p - t <   0) ?   0 : (p - t);

            // quick test on 1,5,9,13
            int bright = 0, dark = 0;
            for (int k = 0; k < 4; ++k) {
                const int idx = qi[k];
                const uint8_t v = img[(y + oy[idx]) * w + (x + ox[idx])];
                bright += (v > hi);
                dark   += (v < lo);
            }
            if (bright < 3 && dark < 3) continue;

            bool corner = false;
            for (int s = 0; s < 16 && !corner; ++s) {
                if (img[(y + oy[s]) * w + (x + ox[s])] > hi) {
                    int run = 1;
                    for (int k = 1; k < FAST_MIN_CONTIG; ++k) {
                        const int idx = (s + k) & 0xF;
                        if (img[(y + oy[idx]) * w + (x + ox[idx])] > hi) ++run; else break;
                    }
                    if (run >= FAST_MIN_CONTIG) corner = true;
                }
                if (!corner && img[(y + oy[s]) * w + (x + ox[s])] < lo) {
                    int run = 1;
                    for (int k = 1; k < FAST_MIN_CONTIG; ++k) {
                        const int idx = (s + k) & 0xF;
                        if (img[(y + oy[idx]) * w + (x + ox[idx])] < lo) ++run; else break;
                    }
                    if (run >= FAST_MIN_CONTIG) corner = true;
                }
            }
            if (!corner) continue;

            int score = 0;
            for (int i = 0; i < 16; ++i) {
                const int v = img[(y + oy[i]) * w + (x + ox[i])];
                score += (v > p) ? (v - p) : (p - v);
            }
            score_grid[yw + x] = (uint16_t)((score > 0xFFFF) ? 0xFFFF : score);
        }
    }

    // NMS (3x3 local max)
    int keep = 0;
    for (int y = 3; y < h - 3; ++y) {
        const int yw = y * w;
        for (int x = 3; x < w - 3; ++x) {
            const uint16_t s = score_grid[yw + x];
            if (!s) continue;
            if (s > score_grid[yw - w + x - 1] && s > score_grid[yw - w + x] && s > score_grid[yw - w + x + 1] &&
                s > score_grid[yw     + x - 1] &&                                  s > score_grid[yw     + x + 1] &&
                s > score_grid[yw + w + x - 1] && s > score_grid[yw + w + x] && s > score_grid[yw + w + x + 1]) {
                if (keep < MAX_KEYPOINTS) {
                    keypoints[keep++] = (Keypoint){ (uint16_t)x, (uint16_t)y, s };
                }
            }
        }
    }
    *out_count = keep;
}

// ==================== Camera init ====================
static esp_err_t camera_init_gray(void) {
    camera_config_t cfg = {
        .pin_pwdn     = CAM_PIN_PWDN,
        .pin_reset    = CAM_PIN_RESET,
        .pin_xclk     = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7, .pin_d6 = CAM_PIN_D6, .pin_d5 = CAM_PIN_D5, .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3, .pin_d2 = CAM_PIN_D2, .pin_d1 = CAM_PIN_D1, .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC, .pin_href = CAM_PIN_HREF, .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size   = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY
    };

    // (Replace ESP_RETURN_ON_ERROR) -> explicit check:
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed: 0x%x", err);
        return err;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;
    g_w = fb->width; g_h = fb->height;
    esp_camera_fb_return(fb);

    // Allocate score grid in PSRAM
    score_grid = heap_caps_calloc(g_w * g_h, sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    if (!score_grid) return ESP_ERR_NO_MEM;

    // Optional: lock exposure/gain for stability
    // sensor_t *s = esp_camera_sensor_get();
    // if (s) { s->set_ae_ctrl(s, 0); s->set_agc_ctrl(s, 0); /* s->set_aec_value(s, 300); */ }

    ESP_LOGI(TAG, "Cam: %dx%d GRAYSCALE, score grid OK", g_w, g_h);
    return ESP_OK;
}

// ========================= HTTP server =========================
static const char INDEX_HTML[] =
"<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ESP32-CAM FAST</title>"
"<style>body{font-family:system-ui;margin:0;padding:1rem;background:#111;color:#eee}img{max-width:100%}</style>"
"</head><body><h2>ESP32-CAM FAST (STA→AP)</h2>"
"<p>Use /stream?t=40&q=60 to tune threshold and JPEG quality.</p>"
"<img src='/stream' alt='stream'>"
"<p><a href='/jpg' target='_blank'>Open snapshot</a></p></body></html>";

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t jpg_get_handler(httpd_req_t *req) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }

    int n_kp = 0;
    uint8_t *jpg = NULL; size_t jpg_len = 0;

    fast_detect_and_nms(fb->buf, fb->width, fb->height, &n_kp);
    draw_keypoints_gray(fb, keypoints, n_kp);

    if (!frame2jpg(fb, 80, &jpg, &jpg_len)) { // higher quality snapshot
        esp_camera_fb_return(fb); httpd_resp_send_500(req); return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    esp_err_t res = httpd_resp_send(req, (const char*)jpg, jpg_len);

    free(jpg); esp_camera_fb_return(fb);
    return res;
}

static esp_err_t stream_get_handler(httpd_req_t *req) {
    // Parse ?t= and ?q=
    char qstr[64];
    if (httpd_req_get_url_query_str(req, qstr, sizeof(qstr)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(qstr, "t", val, sizeof(val)) == ESP_OK) {
            int t = atoi(val); if (t >= 5 && t <= 120) g_fast_threshold = t;
        }
    }
    int jpeg_q = 60; // default stream JPEG quality
    if (httpd_req_get_url_query_str(req, qstr, sizeof(qstr)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(qstr, "q", val, sizeof(val)) == ESP_OK) {
            int q = atoi(val); if (q >= 5 && q <= 95) jpeg_q = q;
        }
    }

    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");

    camera_fb_t *fb = NULL;
    uint8_t *jpg = NULL; size_t jpg_len = 0;
    char part_buf[64];

    // FPS logger
    uint32_t frames = 0;
    int64_t t0 = esp_timer_get_time();

    while (true) {
        int n_kp = 0;

        fb = esp_camera_fb_get();
        if (!fb) { ESP_LOGE(TAG, "fb_get NULL"); break; }

        fast_detect_and_nms(fb->buf, fb->width, fb->height, &n_kp);
        draw_keypoints_gray(fb, keypoints, n_kp);

        if (!frame2jpg(fb, jpeg_q, &jpg, &jpg_len)) {
            ESP_LOGE(TAG, "frame2jpg failed");
            esp_camera_fb_return(fb); break;
        }

        if (httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)) != ESP_OK) { free(jpg); esp_camera_fb_return(fb); break; }
        int hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART_HDR, (unsigned)jpg_len);
        if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK) { free(jpg); esp_camera_fb_return(fb); break; }
        if (httpd_resp_send_chunk(req, (const char *)jpg, jpg_len) != ESP_OK) { free(jpg); esp_camera_fb_return(fb); break; }

        free(jpg); jpg = NULL; esp_camera_fb_return(fb);

        frames++;
        int64_t dt = esp_timer_get_time() - t0;
        if (dt >= 1000000) {
            ESP_LOGI(TAG, "FPS ~ %.1f  (t=%d q=%d)", (double)frames * 1e6 / dt, g_fast_threshold, jpeg_q);
            frames = 0; t0 = esp_timer_get_time();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (jpg) free(jpg);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t u_root   = { .uri="/",       .method=HTTP_GET, .handler=root_get_handler   };
        httpd_uri_t u_jpg    = { .uri="/jpg",    .method=HTTP_GET, .handler=jpg_get_handler    };
        httpd_uri_t u_stream = { .uri="/stream", .method=HTTP_GET, .handler=stream_get_handler };
        httpd_register_uri_handler(server, &u_root);
        httpd_register_uri_handler(server, &u_jpg);
        httpd_register_uri_handler(server, &u_stream);
        ESP_LOGI(TAG, "HTTP server started");
    }
    return server;
}

// ============================ Wi-Fi (STA + fallback AP) =========================
static void wifi_evt_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "STA disconnected, retrying…");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "STA Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_try_sta(uint32_t wait_ms) {
    s_wifi_event_group = xEventGroupCreate();
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));

    wifi_country_t country = { .cc="US", .schan=1, .nchan=11, .policy=WIFI_COUNTRY_POLICY_AUTO };
    esp_wifi_set_country(&country);
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_evt_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_evt_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t sta_cfg = { 0 };
    strncpy((char*)sta_cfg.sta.ssid, WIFI_SSID, sizeof(sta_cfg.sta.ssid));
    strncpy((char*)sta_cfg.sta.password, WIFI_PASS, sizeof(sta_cfg.sta.password));
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    sta_cfg.sta.pmf_cfg.required   = false;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(wait_ms));
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

static void wifi_start_softap(void) {
    ESP_LOGW(TAG, "Starting SoftAP…");
    esp_wifi_stop();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    static bool ap_created = false;
    if (!ap_created) {
        (void)esp_netif_create_default_wifi_ap();  // create AP netif; ignore handle
        ap_created = true;
    }

    wifi_config_t apcfg = { 0 };
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    char ssid[32]; snprintf(ssid, sizeof(ssid), "ESP32-CAM-%02X%02X%02X", mac[3], mac[4], mac[5]);

    strncpy((char*)apcfg.ap.ssid, ssid, sizeof(apcfg.ap.ssid));
    strncpy((char*)apcfg.ap.password, "12345678", sizeof(apcfg.ap.password));
    apcfg.ap.ssid_len       = strlen((char*)apcfg.ap.ssid);
    apcfg.ap.authmode       = WIFI_AUTH_WPA_WPA2_PSK;   // set WIFI_AUTH_OPEN for no password
    apcfg.ap.channel        = 6;
    apcfg.ap.max_connection = 2;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &apcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGW(TAG, "SoftAP SSID: %s  PASS: 12345678  IP: 192.168.4.1", (char*)apcfg.ap.ssid);
}

// ============================== MAIN ==============================
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta(); // STA netif now; AP netif later if needed

    ESP_ERROR_CHECK(camera_init_gray());

    bool sta_ok = wifi_try_sta(10000);   // ~10 s
    if (!sta_ok) wifi_start_softap();

    start_webserver();

    if (sta_ok) ESP_LOGI(TAG, "Open: http://<router-IP>/");
    else        ESP_LOGI(TAG, "Open: http://192.168.4.1/");
}

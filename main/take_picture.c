// ESP32-CAM (Ai-Thinker) — Minimal FAST-12 + Census(9x9)/Hamming tracker
// EKF-VIO stream only: VIOHDR / VIOOBS* / VIOEND with normalized image coords.
// - Persistent per-track IDs
// - Sigma derived from Hamming cost -> pixel sigma -> normalized sigma_n
// - One VIOOBS per updated track per frame (no duplicates)
// Build: QVGA GRAYSCALE, fb_count=2, PSRAM required. ESP-IDF v4+.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

// ============================ BUILD-TIME SWITCHES ===========================
#define AUTO_EXPO_SWEEP        1   // one-time sweep to lock a good AEC/AGC
#define LOCK_AE_AFTER_SWEEP    1

// ---- EKF intrinsics (placeholder; replace with real calibration) -----------
static float g_fx = 277.0f;     // pixels
static float g_fy = 277.0f;     // pixels
static float g_cx = 159.5f;     // (w-1)/2 for 320
static float g_cy = 119.5f;     // (h-1)/2 for 240

// FAST grid / NMS
#ifndef FAST_STRIDE
#define FAST_STRIDE 2
#endif
#ifndef TILED_NMS
#define TILED_NMS 1
#endif
#ifndef GRID_X
#define GRID_X 8
#endif
#ifndef GRID_Y
#define GRID_Y 6
#endif
#ifndef CELL_MAX
#define CELL_MAX 6
#endif

// ------------------------- AI-Thinker PINS ----------------------------------
#define CAM_PIN_PWDN      32
#define CAM_PIN_RESET     -1
#define CAM_PIN_XCLK       0
#define CAM_PIN_SIOD      26
#define CAM_PIN_SIOC      27
#define CAM_PIN_D7        35
#define CAM_PIN_D6        34
#define CAM_PIN_D5        39
#define CAM_PIN_D4        36
#define CAM_PIN_D3        21
#define CAM_PIN_D2        19
#define CAM_PIN_D1        18
#define CAM_PIN_D0         5
#define CAM_PIN_VSYNC     25
#define CAM_PIN_HREF      23
#define CAM_PIN_PCLK      22

// --------------------------- FAST PARAMS ------------------------------------
#define FAST_MIN_CONTIG   12
#define MAX_KEYPOINTS     512
static int g_fast_threshold = 40;

// Fixed tracker params (minimal)
#define MAX_TRACKS         160
#define TARGET_TRACKS      140
#define RESEED_CAP_PERF      8
#define RESEED_EVERY         2
#define SEED_MIN_SCORE    1200
#define SEED_BORDER          4
#define WIN_RAD              3       // fixed window radius for matching
#define HAM_ABS_MAX         18       // fixed Hamming gate

// ----------------------- TYPES & GLOBALS ------------------------------------
typedef struct { uint16_t x, y, score; } Keypoint;
typedef int (*cmp_fn)(const void*, const void*);

static int cmp_kp_desc(const void* a, const void* b){
    const Keypoint *ka = (const Keypoint*)a, *kb = (const Keypoint*)b;
    return (int)kb->score - (int)ka->score; // higher first
}

static int grid_select(const Keypoint *in, int n, int w, int h,
                       int gx, int gy, int cell_max, Keypoint *out)
{
    const int cw = (w + gx - 1) / gx;
    const int ch = (h + gy - 1) / gy;
    static uint8_t used[GRID_Y][GRID_X];
    memset(used, 0, sizeof(used));
    int kept = 0;
    for (int i = 0; i < n; ++i) {
        int cx = in[i].x / cw;
        int cy = in[i].y / ch;
        if ((unsigned)cx >= (unsigned)gx || (unsigned)cy >= (unsigned)gy) continue;
        if (used[cy][cx] < cell_max) {
            out[kept++] = in[i];
            used[cy][cx]++;
        }
    }
    return kept;
}

static Keypoint   keypoints[MAX_KEYPOINTS];
static uint16_t  *score_grid = NULL;
static uint8_t   *g_prev = NULL;      // previous frame (QVGA)
static int g_w = 0, g_h = 0;
static const char *TAG = "FAST_VIO_MIN";

// ----------------------------- FAST CORE ------------------------------------
static void fast_detect_and_nms(const uint8_t *img, int w, int h, int *out_count)
{
    if (!score_grid) { *out_count = 0; return; }
    memset(score_grid, 0, w*h*sizeof(uint16_t));

    const int base_t = g_fast_threshold;
    const int8_t ox[16] = { 0,1,2,3,3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1 };
    const int8_t oy[16] = { 3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1,0,1,2,3 };
    const int qi[4]     = { 1,5,9,13 };

    for (int y=3; y<h-3; ++y) {
        const int yw = y*w;
        int x0 = 3;
        if (FAST_STRIDE == 2) x0 += (y & 1); // checkerboard
        for (int x = x0; x < w-3; x += FAST_STRIDE) {
            const uint8_t p = img[yw + x];
            int t = base_t;
            const int hi = (p + t > 255) ? 255 : (p + t);
            const int lo = (p - t <   0) ?   0 : (p - t);

            int bright=0, dark=0;
            for (int k=0;k<4;++k) {
                const int idx = qi[k];
                const uint8_t v = img[(y+oy[idx])*w + (x+ox[idx])];
                bright += (v>hi);
                dark   += (v<lo);
            }
            if (bright < 3 && dark < 3) continue;

            bool corner=false;
            for (int sidx=0; sidx<16 && !corner; ++sidx) {
                if (img[(y+oy[sidx])*w + (x+ox[sidx])] > hi) {
                    int run=1; for (int k=1;k<FAST_MIN_CONTIG;++k){
                        const int idx=(sidx+k)&0xF;
                        if (img[(y+oy[idx])*w + (x+ox[idx])] > hi) ++run; else break;
                    }
                    if (run>=FAST_MIN_CONTIG) corner=true;
                }
                if (!corner && img[(y+oy[sidx])*w + (x+ox[sidx])] < lo) {
                    int run=1; for (int k=1;k<FAST_MIN_CONTIG;++k){
                        const int idx=(sidx+k)&0xF;
                        if (img[(y+oy[idx])*w + (x+ox[idx])] < lo) ++run; else break;
                    }
                    if (run>=FAST_MIN_CONTIG) corner=true;
                }
            }
            if (!corner) continue;

            int score=0;
            for (int i=0;i<16;++i){
                const int v = img[(y+oy[i])*w + (x+ox[i])];
                score += (v>p)? (v-p):(p-v);
            }
            score_grid[yw + x] = (uint16_t)((score>0xFFFF)?0xFFFF:score);
        }
    }

    int keep=0;
    for (int y=3;y<h-3;++y){
        const int yw=y*w;
        for (int x=3;x<w-3;++x){
            const uint16_t s = score_grid[yw+x]; if(!s) continue;
            if (s > score_grid[yw-w + x-1] && s > score_grid[yw-w + x] && s > score_grid[yw-w + x+1] &&
                s > score_grid[yw   + x-1] &&                             s > score_grid[yw   + x+1] &&
                s > score_grid[yw+w + x-1] && s > score_grid[yw+w + x] && s > score_grid[yw+w + x+1]) {
                if (keep<MAX_KEYPOINTS)
                    keypoints[keep++] = (Keypoint){(uint16_t)x,(uint16_t)y,s};
            }
        }
    }

    qsort(keypoints, keep, sizeof(Keypoint), (cmp_fn)cmp_kp_desc);

#if TILED_NMS
    static Keypoint tmp[MAX_KEYPOINTS];
    int kept = grid_select(keypoints, keep, g_w, g_h, GRID_X, GRID_Y, CELL_MAX, tmp);
    if (kept > 0) memcpy(keypoints, tmp, kept * sizeof(Keypoint));
    *out_count = kept;
#else
    *out_count = keep;
#endif
}

// ------------------------- CAMERA + SENSOR CTRL -----------------------------
static esp_err_t camera_init_gray(void)
{
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
    ESP_RETURN_ON_ERROR(esp_camera_init(&cfg), TAG, "camera init failed");

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;
    g_w = fb->width; g_h = fb->height;
    esp_camera_fb_return(fb);

    // Tie intrinsics center to current frame size (fx,fy left as configured)
    g_cx = (g_w - 1) * 0.5f;
    g_cy = (g_h - 1) * 0.5f;

    score_grid = heap_caps_calloc(g_w*g_h, sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    if (!score_grid) return ESP_ERR_NO_MEM;

    g_prev = heap_caps_malloc(g_w*g_h, MALLOC_CAP_SPIRAM);
    if (!g_prev) return ESP_ERR_NO_MEM;
    memset(g_prev, 0, g_w*g_h);

    ESP_LOGI(TAG, "Cam: %dx%d GRAYSCALE", g_w, g_h);
    ESP_LOGI(TAG, "Intrinsics (px): fx=%.2f fy=%.2f cx=%.2f cy=%.2f", g_fx, g_fy, g_cx, g_cy);
    return ESP_OK;
}

static void sensor_set_auto(bool ae, bool agc)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;
    if (s->set_exposure_ctrl) s->set_exposure_ctrl(s, ae ? 1 : 0);
    if (s->set_gain_ctrl)     s->set_gain_ctrl(s,     agc ? 1 : 0);
}
static void sensor_set_aec(int aec_value)
{
    sensor_t *s = esp_camera_sensor_get();
    if (s && s->set_aec_value) s->set_aec_value(s, aec_value);
}
static void sensor_set_agc_gain(int raw_gain)
{
    sensor_t *s = esp_camera_sensor_get();
    if (s && s->set_agc_gain) s->set_agc_gain(s, raw_gain);
}

// ----------------------- EXPOSURE/GAIN SWEEP (one-time) --------------------
typedef struct { float avg; float std; int zero; int aec; int agc; } sweep_stat_t;

static void run_frames_eval(int nframes, sweep_stat_t *st)
{
    for (int i=0;i<2;i++){ camera_fb_t* f=esp_camera_fb_get(); if(f) esp_camera_fb_return(f); }
    double sum=0, sum2=0; int zero=0;
    for (int i=0;i<nframes;i++){
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(2)); i--; continue; }
        int n_kp=0;
        fast_detect_and_nms(fb->buf, g_w, g_h, &n_kp);
        sum += n_kp; sum2 += (double)n_kp*n_kp; if (n_kp==0) zero++;
        esp_camera_fb_return(fb);
    }
    st->avg = (float)(sum/nframes);
    const double var = (sum2/nframes) - (st->avg*st->avg);
    st->std = (float)((var>0)?sqrt(var):0);
    st->zero = zero;
}

static void auto_sweep_choose(void)
{
#if AUTO_EXPO_SWEEP
    sensor_set_auto(false, false);

    sweep_stat_t best = {.avg=-1e9f,.std=1e9f,.zero=9999,.aec=0,.agc=0};
    sweep_stat_t cur;

    const int AECV[] = {220,300,380,460};
    const int AGCV[] = {8,12,16,24};

    for (size_t i=0;i<sizeof(AECV)/sizeof(AECV[0]); ++i){
        for (size_t j=0;j<sizeof(AGCV)/sizeof(AGCV[0]); ++j){
            sensor_set_aec(AECV[i]);
            sensor_set_agc_gain(AGCV[j]);
            vTaskDelay(pdMS_TO_TICKS(60));
            cur.aec=AECV[i]; cur.agc=AGCV[j];
            run_frames_eval(8, &cur);

            ESP_LOGI(TAG, "SWEEP aec=%d agc=%d  avg=%.1f std=%.1f zero=%d",
                     cur.aec, cur.agc, cur.avg, cur.std, cur.zero);

            bool better = (cur.avg > best.avg + 0.5f) ||
                          ((fabsf(cur.avg - best.avg) < 0.5f) && (cur.zero < best.zero)) ||
                          ((fabsf(cur.avg - best.avg) < 0.5f) && (cur.zero == best.zero) && (cur.std < best.std));
            if (better) best = cur;
        }
    }
    sensor_set_aec(best.aec);
    sensor_set_agc_gain(best.agc);
    ESP_LOGI(TAG, "LOCK aec=%d agc=%d  (avg corners=%.1f, std=%.1f)",
             best.aec, best.agc, best.avg, best.std);

#if LOCK_AE_AFTER_SWEEP
    sensor_set_auto(false, false);  // lock both
#else
    sensor_set_auto(true, true);
#endif
#endif
}

// ======================= TRACKER: STATE & MATCHING ==========================
static inline int clampi(int v,int lo,int hi){ return v<lo?lo:(v>hi?hi:v); }
static inline float clampf(float v,float lo,float hi){ return v<lo?lo:(v>hi?hi:v); }

typedef struct {
    uint16_t x, y;       // pixel coords
    uint16_t score0;     // FAST score at seed
    int8_t   vx, vy;     // last accepted displacement (motion prior)
    uint8_t  age;        // frames
    uint8_t  alive;      // 0/1
    uint32_t id;         // persistent track ID
    uint32_t desc[3];    // 96-bit Census
    uint8_t  updated;    // updated this frame
    uint8_t  last_cost;  // 0..255 clamped
} Track;

static Track g_tracks[MAX_TRACKS];
static int   g_ntracks = 0;
static uint32_t g_next_track_id = 1;   // persistent increasing IDs

static inline bool in_bounds_seed(int x,int y){
    return (x>=SEED_BORDER && x<g_w-SEED_BORDER && y>=SEED_BORDER && y<g_h-SEED_BORDER);
}
static inline int l2_sq(int x1,int y1,int x2,int y2){ int dx=x1-x2, dy=y1-y2; return dx*dx+dy*dy; }
static bool too_close_to_tracks(int x,int y){
    for (int i=0;i<g_ntracks;i++){
        if (!g_tracks[i].alive) continue;
        if (l2_sq(x,y, g_tracks[i].x, g_tracks[i].y) <= 9) return true;
    }
    return false;
}

static inline int popc32(uint32_t x){ return __builtin_popcount(x); }

static void build_census9x9(const uint8_t* img, int w, int h, int x, int y, uint32_t out[3])
{
    const uint8_t ref = img[y*w + x];
    uint32_t b0=0,b1=0,b2=0;
    int bit=0;
    for (int dy=-4; dy<=4; ++dy){
        for (int dx=-4; dx<=4; ++dx){
            if (dx==0 && dy==0) continue;
            uint8_t v = img[(y+dy)*w + (x+dx)];
            uint32_t bitval = (v > ref);
            if      (bit < 32) b0 |= (bitval << bit);
            else if (bit < 64) b1 |= (bitval << (bit-32));
            else               b2 |= (bitval << (bit-64));
            bit++;
        }
    }
    out[0]=b0; out[1]=b1; out[2]=b2;
}
static inline int ham96(const uint32_t a[3], const uint32_t b[3]){
    return popc32(a[0]^b[0]) + popc32(a[1]^b[1]) + popc32(a[2]^b[2]);
}

static void tracker_reset(void){ memset(g_tracks, 0, sizeof(g_tracks)); g_ntracks = 0; g_next_track_id = 1; }
static void tracker_compact(void){ int w=0; for (int i=0;i<g_ntracks;i++){ if (g_tracks[i].alive){ if (i!=w) g_tracks[w]=g_tracks[i]; w++; } } g_ntracks=w; }

static int tracker_seed_from_fast(const Keypoint* kps, int n, const uint8_t* img,
                                  int max_add, int seed_min_score)
{
    int added=0;
    for (int i=0;i<n && added<max_add; ++i){
        const Keypoint *k = &kps[i];
        if (k->score < seed_min_score) continue;
        if (!in_bounds_seed(k->x, k->y)) continue;
        if (too_close_to_tracks(k->x, k->y)) continue;

        Track t = {0};
        t.x = k->x; t.y = k->y; t.score0 = k->score; t.age = 0; t.alive = 1; t.vx=0; t.vy=0;
        t.id = g_next_track_id++;
        build_census9x9(img, g_w, g_h, t.x, t.y, t.desc);

        int slot = -1; for (int j=0;j<g_ntracks;j++) if (!g_tracks[j].alive){ slot=j; break; }
        if (slot>=0) g_tracks[slot]=t; else { if (g_ntracks < MAX_TRACKS) g_tracks[g_ntracks++] = t; else break; }
        added++;
    }
    return added;
}

// Windowed Hamming match with ratio+margin; fixed window & gate
static bool match_window_hamming(const uint8_t* prev, const uint8_t* curr, const Track* trk,
                                 int *mx, int *my, int *best_cost, int *second_cost)
{
    const int cx = trk->x + trk->vx;
    const int cy = trk->y + trk->vy;
    int best=9999, second=9999, bx=trk->x, by=trk->y;

    for (int dy=-WIN_RAD; dy<=WIN_RAD; ++dy){
        int y = cy + dy; if (y<SEED_BORDER || y>=g_h-SEED_BORDER) continue;
        for (int dx=-WIN_RAD; dx<=WIN_RAD; ++dx){
            int x = cx + dx; if (x<SEED_BORDER || x>=g_w-SEED_BORDER) continue;
            uint32_t dcur[3]; build_census9x9(curr, g_w, g_h, x, y, dcur);
            int cost = ham96(trk->desc, dcur);
            if (cost < best){ second = best; best = cost; bx = x; by = y; }
            else if (cost < second){ second = cost; }
        }
    }

    *mx = bx; *my = by; *best_cost = best; *second_cost = second;
    if (best > HAM_ABS_MAX) return false;
    bool ratio_ok  = (second <= 0) || (best*100 <= 85*second);
    bool margin_ok = ((second - best) >= 1);
    return (ratio_ok || margin_ok);
}

// Update all tracks; mark updated
static int tracker_update(const uint8_t* prev, const uint8_t* curr)
{
    for (int i=0;i<g_ntracks;i++) g_tracks[i].updated = 0;

    for (int i=0;i<g_ntracks;i++){
        Track *t = &g_tracks[i]; if (!t->alive) continue;

        int mx,my,best,second;
        bool ok = match_window_hamming(prev, curr, t, &mx,&my,&best,&second);
        if (!ok){ t->alive = 0; continue; }

        int dx = mx - t->x; int dy = my - t->y;
        t->x = mx; t->y = my; t->age++;
        if (dx < -3) dx = -3; else if (dx > 3) dx = 3;
        if (dy < -3) dy = -3; else if (dy > 3) dy = 3;
        t->vx = (int8_t)dx; t->vy = (int8_t)dy;

        build_census9x9(curr, g_w, g_h, t->x, t->y, t->desc);
        t->updated = 1; t->last_cost = (uint8_t)clampi(best,0,255);
    }

    // Compact and soft cap to ~TARGET_TRACKS alive (simple)
    tracker_compact();
    int alive=0; for (int i=0;i<g_ntracks;i++){ if (g_tracks[i].alive){ if (alive >= TARGET_TRACKS) { g_tracks[i].alive = 0; } else alive++; } }
    tracker_compact();
    return alive;
}

// ============================== EKF STREAM HELPERS ==========================
// Convert Hamming cost -> pixel sigma (tunable) -> normalized sigma_n
static inline float sigma_n_from_cost(uint8_t ham_cost){
    const float a = 0.25f;    // px
    const float b = 0.05f;    // px per Hamming count unit
    float sigma_px = a + b * (float)ham_cost;
    sigma_px = clampf(sigma_px, 0.25f, 5.0f);
    return sigma_px / ((g_fx > 1e-6f) ? g_fx : 1.0f);
}

// ================================ APP MAIN ==================================
void app_main(void)
{
    ESP_ERROR_CHECK(camera_init_gray());
    auto_sweep_choose();          // one-time exposure/gain optimization (kept)

    // Warm prev buffer
    for (int i=0;i<2;i++){
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb){ memcpy(g_prev, fb->buf, g_w*g_h); esp_camera_fb_return(fb); }
    }
    tracker_reset();

    uint32_t frame_idx = 0;

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }

        int64_t t_us = esp_timer_get_time();

        // Reseed periodically up to cap if we have fewer than TARGET_TRACKS
        if ((frame_idx % RESEED_EVERY) == 0){
            int alive_now=0; for (int i=0;i<g_ntracks;i++) if (g_tracks[i].alive) alive_now++;
            if (alive_now < TARGET_TRACKS){
                int n_kp=0; fast_detect_and_nms(fb->buf, g_w, g_h, &n_kp);
                int want = TARGET_TRACKS - alive_now; if (want > RESEED_CAP_PERF) want = RESEED_CAP_PERF;
                if (want > 0 && n_kp > 0){ (void)tracker_seed_from_fast(keypoints, n_kp, fb->buf, want, SEED_MIN_SCORE); }
            }
        }

        int alive_now = tracker_update(g_prev, fb->buf);
        (void)alive_now; // not printed in minimal build

        // --- EKF-VIO per-frame stream (normalized coords) -------------------
        int n_obs = 0; for (int i=0;i<g_ntracks;i++) if (g_tracks[i].alive && g_tracks[i].updated) n_obs++;
        const float ROW_US = 66.7f; // line time estimate (tunable/measure)

        // Header
        printf("VIOHDR,%" PRIu32 ",%lld,%d,%d,%d,%.3f,%.6f,%.6f,%.6f,%.6f\n",
               frame_idx, (long long)t_us, n_obs, g_w, g_h, ROW_US,
               g_fx, g_fy, g_cx, g_cy);

        // Observations: one per updated track
        for (int i=0;i<g_ntracks;i++){
            Track *t = &g_tracks[i];
            if (!t->alive || !t->updated) continue;

            float xn = ( (float)t->x - g_cx ) / g_fx;
            float yn = ( (float)t->y - g_cy ) / g_fy;
            float sigma_n = sigma_n_from_cost(t->last_cost);

            uint32_t flags = 0; // bit0: updated=true
            flags |= 1u;

            printf("VIOOBS,%" PRIu32 ",%.6f,%.6f,%u,%.6f,%" PRIu32 "\n",
                   t->id, xn, yn, (unsigned)t->age, sigma_n, flags);
        }
        printf("VIOEND\n");

        // roll frame
        memcpy(g_prev, fb->buf, g_w*g_h);
        esp_camera_fb_return(fb);
        frame_idx++;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

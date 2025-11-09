// ESP32-CAM VIO front-end (LOG MODE)
// FAST-12 + Census(9x9) tracker → prints normalized observations for EKF-VIO
// Build: QVGA GRAYSCALE, fb_count=2, PSRAM required. ESP-IDF v4+.
//
// What it prints each frame (to UART):
//   VIOHDR,<frame>,<t_us>,<n_obs>,<w>,<h>,<row_us>
//   VIOOBS,<id>,<xn>,<yn>,<age>,<quality>
//   ...
//   VIOEND
//
// After boot, it runs a quick AE/AGC sweep to maximize usable corners,
// LOCKS exposure/gain, warms the previous frame, then begins tracking + logging.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "esp_camera.h"
#include "esp_heap_caps.h"

// ============================== USER KNOBS ==================================
#define USE_BRIEF              0   // 0=Census-9x9, 1=BRIEF-128 (pairs stubbed)
#define FAST_STRIDE            2
#define GRID_X                 8
#define GRID_Y                 6
#define CELL_MAX               6
#define REPORT_MIN_AGE         2   // only send obs with age >= this
#define REPORT_MAX_OBS       120   // throttle printing

#define FAST_MIN_CONTIG       12
#define MAX_KEYPOINTS        512
static int g_fast_threshold = 40;

// Search/matcher/track policy
#define MAX_TRACKS         160
#define SOFT_CAP_ALIVE      80
#define TARGET_TRACKS      140
#define RESEED_CAP_PERF      8
#define RESEED_CAP_LOW       4
#define RESEED_EVERY         2
#define SEED_MIN_SCORE    1200
#define SEED_MIN_LOW      1400
#define SEED_BORDER          4

#define WIN_RAD_DEFAULT      3
#define WIN_RAD_LOW_TEX      4

#define HAM_ABS_MAX         18
#define HAM_MARGIN_MIN       1
#define FB_EVERY             4
#define FB_THR_PX            2

// ============================== TYPES =======================================
typedef struct { uint16_t x, y, score; } Keypoint;
typedef int (*cmp_fn)(const void*, const void*);

// ============================== GLOBALS =====================================
static const char *TAG = "VIO_LOG";

static Keypoint   keypoints[MAX_KEYPOINTS];
static uint16_t  *score_grid = NULL;
static uint8_t   *g_prev = NULL;
static int g_w = 0, g_h = 0;

// ============================== FAST CORE ===================================
static int cmp_kp_desc(const void* a, const void* b){
    const Keypoint *ka=(const Keypoint*)a,*kb=(const Keypoint*)b;
    return (int)kb->score - (int)ka->score;
}

static int grid_select(const Keypoint *in,int n,int w,int h,int gx,int gy,int cell_max,Keypoint *out){
    const int cw=(w+gx-1)/gx, ch=(h+gy-1)/gy;
    static uint8_t used[GRID_Y][GRID_X]; memset(used,0,sizeof(used));
    int kept=0;
    for(int i=0;i<n;i++){
        int cx=in[i].x/cw, cy=in[i].y/ch;
        if ((unsigned)cx>=(unsigned)gx || (unsigned)cy>=(unsigned)gy) continue;
        if (used[cy][cx] < cell_max) { out[kept++]=in[i]; used[cy][cx]++; }
    }
    return kept;
}

static void fast_detect_and_nms(const uint8_t *img,int w,int h,int *out_count){
    if(!score_grid){ *out_count=0; return; }
    memset(score_grid,0,w*h*sizeof(uint16_t));
    const int t0=g_fast_threshold;
    const int8_t ox[16]={0,1,2,3,3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1};
    const int8_t oy[16]={3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1,0,1,2,3};
    const int qi[4]={1,5,9,13};

    for(int y=3;y<h-3;y++){
        const int yw=y*w;
        int x0=3; if (FAST_STRIDE==2) x0+=(y&1);
        for(int x=x0;x<w-3;x+=FAST_STRIDE){
            const uint8_t p=img[yw+x];
            const int hi=(p+t0>255)?255:(p+t0);
            const int lo=(p-t0<0)?0:(p-t0);

            int bright=0,dark=0;
            for(int k=0;k<4;k++){
                const int idx=qi[k];
                const uint8_t v=img[(y+oy[idx])*w + (x+ox[idx])];
                bright += (v>hi); dark += (v<lo);
            }
            if (bright<3 && dark<3) continue;

            bool corner=false;
            for(int s=0;s<16 && !corner;s++){
                if (img[(y+oy[s])*w + (x+ox[s])] > hi){
                    int run=1; for(int k=1;k<FAST_MIN_CONTIG;k++){
                        const int idx=(s+k)&0xF;
                        if (img[(y+oy[idx])*w + (x+ox[idx])] > hi) run++; else break;
                    }
                    if (run>=FAST_MIN_CONTIG) corner=true;
                }
                if (!corner && img[(y+oy[s])*w + (x+ox[s])] < lo){
                    int run=1; for(int k=1;k<FAST_MIN_CONTIG;k++){
                        const int idx=(s+k)&0xF;
                        if (img[(y+oy[idx])*w + (x+ox[idx])] < lo) run++; else break;
                    }
                    if (run>=FAST_MIN_CONTIG) corner=true;
                }
            }
            if (!corner) continue;

            int score=0;
            for(int i=0;i<16;i++){
                const int v=img[(y+oy[i])*w + (x+ox[i])];
                score += (v>p)?(v-p):(p-v);
            }
            score_grid[yw+x]=(uint16_t)((score>0xFFFF)?0xFFFF:score);
        }
    }

    int keep=0;
    for(int y=3;y<h-3;y++){
        const int yw=y*w;
        for(int x=3;x<w-3;x++){
            const uint16_t s=score_grid[yw+x]; if(!s) continue;
            if (s>score_grid[yw-w+x-1] && s>score_grid[yw-w+x] && s>score_grid[yw-w+x+1] &&
                s>score_grid[yw+x-1]                           && s>score_grid[yw+x+1]     &&
                s>score_grid[yw+w+x-1] && s>score_grid[yw+w+x] && s>score_grid[yw+w+x+1]){
                if (keep<MAX_KEYPOINTS) keypoints[keep++] = (Keypoint){(uint16_t)x,(uint16_t)y,s};
            }
        }
    }

    qsort(keypoints, keep, sizeof(Keypoint), (cmp_fn)cmp_kp_desc);
    static Keypoint tmp[MAX_KEYPOINTS];
    int kept=grid_select(keypoints,keep,g_w,g_h,GRID_X,GRID_Y,CELL_MAX,tmp);
    if (kept>0) memcpy(keypoints,tmp,kept*sizeof(Keypoint));
    *out_count=kept;
}

// ============================== CAMERA ======================================
// AI-Thinker pin map
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

static esp_err_t camera_init_gray(void){
    camera_config_t cfg = {
        .pin_pwdn=CAM_PIN_PWDN, .pin_reset=CAM_PIN_RESET, .pin_xclk=CAM_PIN_XCLK,
        .pin_sccb_sda=CAM_PIN_SIOD, .pin_sccb_scl=CAM_PIN_SIOC,
        .pin_d7=CAM_PIN_D7,.pin_d6=CAM_PIN_D6,.pin_d5=CAM_PIN_D5,.pin_d4=CAM_PIN_D4,
        .pin_d3=CAM_PIN_D3,.pin_d2=CAM_PIN_D2,.pin_d1=CAM_PIN_D1,.pin_d0=CAM_PIN_D0,
        .pin_vsync=CAM_PIN_VSYNC,.pin_href=CAM_PIN_HREF,.pin_pclk=CAM_PIN_PCLK,
        .xclk_freq_hz=20000000, .ledc_timer=LEDC_TIMER_0, .ledc_channel=LEDC_CHANNEL_0,
        .pixel_format=PIXFORMAT_GRAYSCALE, .frame_size=FRAMESIZE_QVGA,
        .jpeg_quality=12, .fb_count=2, .fb_location=CAMERA_FB_IN_PSRAM,
        .grab_mode=CAMERA_GRAB_WHEN_EMPTY
    };
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed (%s)", esp_err_to_name(err));
        return err;
    }

    camera_fb_t *fb=esp_camera_fb_get(); if(!fb) return ESP_FAIL;
    g_w=fb->width; g_h=fb->height; esp_camera_fb_return(fb);

    score_grid = heap_caps_calloc(g_w*g_h,sizeof(uint16_t),MALLOC_CAP_SPIRAM);
    g_prev      = heap_caps_malloc(g_w*g_h, MALLOC_CAP_SPIRAM);
    if(!score_grid || !g_prev) return ESP_ERR_NO_MEM;
    memset(g_prev,0,g_w*g_h);

    ESP_LOGI(TAG,"Cam: %dx%d GRAYSCALE", g_w, g_h);
    return ESP_OK;
}

// --- AE/AGC helpers ---
static void sensor_set_auto(bool ae, bool agc){
    sensor_t *s = esp_camera_sensor_get(); if(!s) return;
    if (s->set_exposure_ctrl) s->set_exposure_ctrl(s, ae ? 1 : 0);
    if (s->set_gain_ctrl)     s->set_gain_ctrl(s,     agc ? 1 : 0);
}
static void sensor_set_aec(int aec){
    sensor_t *s = esp_camera_sensor_get(); if (s && s->set_aec_value) s->set_aec_value(s, aec);
}
static void sensor_set_agc(int gain){
    sensor_t *s = esp_camera_sensor_get(); if (s && s->set_agc_gain)  s->set_agc_gain(s, gain);
}

// ====================== INTRINSICS & NORMALIZATION LUT ======================
typedef struct { float fx, fy, cx, cy, k1, k2, p1, p2, k3; } CamModel;
typedef struct { float xn, yn; } RayN;

// TODO: replace with your calibration for the OV3660 at QVGA
static const CamModel K_OV3660_QVGA = {
    .fx=270.0f, .fy=270.0f, .cx=160.0f, .cy=120.0f,
    .k1=-0.05f, .k2=0.001f, .p1=0.0f, .p2=0.0f, .k3=0.0f
};

static RayN *g_norm_lut=NULL;

static void build_norm_lut(const CamModel* K){
    g_norm_lut = heap_caps_malloc(g_w*g_h*sizeof(RayN), MALLOC_CAP_SPIRAM);
    for(int y=0;y<g_h;y++){
        for(int x=0;x<g_w;x++){
            float xu=(x+0.5f-K->cx)/K->fx, yu=(y+0.5f-K->cy)/K->fy;
            float r2=xu*xu+yu*yu;
            float radial=1.f + K->k1*r2 + K->k2*r2*r2 + K->k3*r2*r2*r2;
            float xt=2.f*K->p1*xu*yu + K->p2*(r2 + 2.f*xu*xu);
            float yt=K->p1*(r2 + 2.f*yu*yu) + 2.f*K->p2*xu*yu;
            g_norm_lut[y*g_w+x].xn = xu*radial + xt;
            g_norm_lut[y*g_w+x].yn = yu*radial + yt;
        }
    }
    ESP_LOGI(TAG,"Built normalization LUT");
}

// ============================== TRACKER =====================================
static inline int popc32(uint32_t x){ return __builtin_popcount(x); }

#if !USE_BRIEF
static void build_census9x9(const uint8_t* img,int w,int h,int x,int y,uint32_t out[3]){
    const uint8_t ref = img[y*w + x];
    uint32_t b0=0,b1=0,b2=0; int bit=0;
    for(int dy=-4;dy<=4;dy++){
        for(int dx=-4;dx<=4;dx++){
            if(dx==0 && dy==0) continue;
            uint8_t v = img[(y+dy)*w + (x+dx)];
            uint32_t b=(v>ref);
            if (bit<32) b0|=(b<<bit);
            else if (bit<64) b1|=(b<<(bit-32));
            else b2|=(b<<(bit-64));
            bit++;
        }
    }
    out[0]=b0; out[1]=b1; out[2]=b2;
}
static inline int ham96(const uint32_t a[3],const uint32_t b[3]){
    return popc32(a[0]^b[0])+popc32(a[1]^b[1])+popc32(a[2]^b[2]);
}
#else
// BRIEF pairs are stubbed here for simplicity; Census is recommended on ESP32-CAM.
static void build_brief128(const uint8_t* img,int w,int h,int x,int y,uint32_t out[4]){ memset(out,0,16); }
static inline int ham128(const uint32_t a[4],const uint32_t b[4]){
    return popc32(a[0]^b[0])+popc32(a[1]^b[1])+popc32(a[2]^b[2])+popc32(a[3]^b[3]);
}
#endif

typedef struct {
    uint16_t x, y;
    uint16_t score0;
    int8_t   vx, vy;
    uint8_t  age, alive;
    uint32_t id;             // stable ID
#if USE_BRIEF
    uint32_t desc[4];
#else
    uint32_t desc[3];
#endif
} Track;

static Track g_tracks[MAX_TRACKS];
static int   g_ntracks=0;
static int   g_win_rad=WIN_RAD_DEFAULT;
static uint32_t g_next_id=1;

static inline bool in_bounds_seed(int x,int y){
    return (x>=SEED_BORDER && x<g_w-SEED_BORDER && y>=SEED_BORDER && y<g_h-SEED_BORDER);
}
static inline int l2_sq(int x1,int y1,int x2,int y2){ int dx=x1-x2, dy=y1-y2; return dx*dx+dy*dy; }
static inline int frame_texture_score(const uint8_t* img,int w,int h){
    int sum=0,cnt=0;
    for(int y=4;y<h-4;y+=8){
        const int yw=y*w;
        for(int x=4;x<w-4;x+=8){
            int gx=img[yw+x+1]-img[yw+x-1];
            int gy=img[(y+1)*w+x]-img[(y-1)*w+x];
            int g=(gx<0?-gx:gx)+(gy<0?-gy:gy);
            sum+=g; cnt++;
        }
    }
    return (cnt?sum/cnt:0);
}

static void tracker_reset(void){ memset(g_tracks,0,sizeof(g_tracks)); g_ntracks=0; g_next_id=1; }
static bool too_close_to_tracks(int x,int y){
    for(int i=0;i<g_ntracks;i++){ if (!g_tracks[i].alive) continue; if (l2_sq(x,y,g_tracks[i].x,g_tracks[i].y)<=9) return true; }
    return false;
}

static int tracker_seed_from_fast(const Keypoint* kps,int n,const uint8_t* img,int max_add,int seed_min_score){
    int added=0;
    for(int i=0;i<n && added<max_add;i++){
        const Keypoint *k=&kps[i];
        if (k->score<seed_min_score || !in_bounds_seed(k->x,k->y) || too_close_to_tracks(k->x,k->y)) continue;
        Track t={0};
        t.x=k->x; t.y=k->y; t.score0=k->score; t.vx=t.vy=0; t.age=0; t.alive=1; t.id=g_next_id++;
#if USE_BRIEF
        build_brief128(img,g_w,g_h,t.x,t.y,t.desc);
#else
        build_census9x9(img, g_w, g_h, t.x, t.y, t.desc);

#endif
        // NOTE: fix: use t.desc
        #if !USE_BRIEF
        build_census9x9(img,g_w,g_h,t.x,t.y,t.desc);
        #endif

        int slot=-1; for(int j=0;j<g_ntracks;j++) if(!g_tracks[j].alive){ slot=j; break; }
        if (slot>=0) g_tracks[slot]=t; else { if (g_ntracks<MAX_TRACKS) g_tracks[g_ntracks++]=t; else break; }
        added++;
    }
    return added;
}

static void tracker_compact(void){
    int w=0; for(int i=0;i<g_ntracks;i++){ if (g_tracks[i].alive){ if (i!=w) g_tracks[w]=g_tracks[i]; w++; } }
    g_ntracks=w;
}

static bool match_window_hamming(const uint8_t* prev,const uint8_t* curr,const Track* trk,
                                 int *mx,int *my,int *best_cost,int *second_cost,int ham_gate)
{
    const int cx=trk->x+trk->vx, cy=trk->y+trk->vy;
    int best=9999, second=9999, bx=trk->x, by=trk->y;
    for(int dy=-g_win_rad; dy<=g_win_rad; dy++){
        int y=cy+dy; if (y<SEED_BORDER || y>=g_h-SEED_BORDER) continue;
        for(int dx=-g_win_rad; dx<=g_win_rad; dx++){
            int x=cx+dx; if (x<SEED_BORDER || x>=g_w-SEED_BORDER) continue;
#if USE_BRIEF
            uint32_t dcur[4]; build_brief128(curr,g_w,g_h,x,y,dcur);
            int cost=ham128(trk->desc,dcur);
#else
            uint32_t dcur[3]; build_census9x9(curr,g_w,g_h,x,y,dcur);
            int cost=ham96(trk->desc,dcur);
#endif
            if (cost<best){ second=best; best=cost; bx=x; by=y; }
            else if (cost<second){ second=cost; }
        }
    }
    *mx=bx; *my=by; *best_cost=best; *second_cost=second;
    if (best>ham_gate) return false;
    bool ratio_ok=(second<=0)||(best*100 <= 85*second);
    bool margin_ok=((second-best)>=HAM_MARGIN_MIN);
    return (ratio_ok||margin_ok);
}

static bool fb_check(const uint8_t* prev,const uint8_t* curr,int x_prev,int y_prev,int x_curr,int y_curr,int ham_gate){
#if USE_BRIEF
    uint32_t dcur[4]; build_brief128(curr,g_w,g_h,x_curr,y_curr,dcur);
#else
    uint32_t dcur[3]; build_census9x9(curr,g_w,g_h,x_curr,y_curr,dcur);
#endif
    int best=9999,bx=x_curr,by=y_curr;
    for(int dy=-g_win_rad; dy<=g_win_rad; dy++){
        int y=y_curr+dy; if (y<SEED_BORDER || y>=g_h-SEED_BORDER) continue;
        for(int dx=-g_win_rad; dx<=g_win_rad; dx++){
            int x=x_curr+dx; if (x<SEED_BORDER || x>=g_w-SEED_BORDER) continue;
#if USE_BRIEF
            uint32_t dprev[4]; build_brief128(prev,g_w,g_h,x,y,dprev);
            int cost=ham128(dcur,dprev);
#else
            uint32_t dprev[3]; build_census9x9(prev,g_w,g_h,x,y,dprev);
            int cost=ham96(dcur,dprev);
#endif
            if (cost<best){ best=cost; bx=x; by=y; }
        }
    }
    int dx=bx-x_prev, dy=by-y_prev;
    if (best>ham_gate) return false;
    return (dx*dx + dy*dy) <= (FB_THR_PX*FB_THR_PX);
}

typedef struct { int matched, lost_ham, lost_fb; int ham_sum, best2_sum; } TrkStats;

static int tracker_update(const uint8_t* prev,const uint8_t* curr,TrkStats* ts,int ham_gate){
    memset(ts,0,sizeof(*ts));
    for(int i=0;i<g_ntracks;i++){
        Track *t=&g_tracks[i]; if (!t->alive) continue;
        int mx,my,best,second;
        bool ok=match_window_hamming(prev,curr,t,&mx,&my,&best,&second,ham_gate);
        if (!ok){ t->alive=0; ts->lost_ham++; continue; }
        bool fb_ok=true; if (((t->age+1)%FB_EVERY)==0) fb_ok=fb_check(prev,curr,t->x,t->y,mx,my,ham_gate);
        if (!fb_ok){ t->alive=0; ts->lost_fb++; continue; }
        int dx=mx-t->x, dy=my-t->y;
        t->x=mx; t->y=my; t->age++;
        if (dx<-3) dx=-3; else if (dx>3) dx=3;
        if (dy<-3) dy=-3; else if (dy>3) dy=3;
        t->vx=(int8_t)dx; t->vy=(int8_t)dy;
#if USE_BRIEF
        build_brief128(curr,g_w,g_h,t->x,t->y,t->desc);
#else
        build_census9x9(curr,g_w,g_h,t->x,t->y,t->desc);
#endif
        ts->matched++; ts->ham_sum+=best; if (second>0) ts->best2_sum+=second;
    }
    tracker_compact();
    int alive=0;
    for(int i=0;i<g_ntracks;i++){ if (g_tracks[i].alive){ if (alive>=SOFT_CAP_ALIVE) g_tracks[i].alive=0; else alive++; } }
    tracker_compact();
    return alive;
}

// ============================== REPORTING ===================================
// Quality is a simple heuristic of age; you can refine later if you keep costs.
static inline uint8_t age_to_quality(uint8_t age){
    int qi = (age > 10) ? 200 : (100 + 10 * (int)age);
    if (qi < 0)   qi = 0;
    if (qi > 255) qi = 255;
    return (uint8_t)qi;
}

// Header + normalized ray observations for EKF-VIO
static void log_vio_frame(uint32_t frame_idx, uint64_t t_us, uint16_t row_time_us){
    int n=0; for(int i=0;i<g_ntracks;i++) if (g_tracks[i].alive && g_tracks[i].age>=REPORT_MIN_AGE) n++;
    if (n > REPORT_MAX_OBS) n = REPORT_MAX_OBS;

    printf("VIOHDR,%lu,%llu,%d,%d,%d,%u\n",
           (unsigned long)frame_idx,
           (unsigned long long)t_us,
           n, g_w, g_h,
           (unsigned)row_time_us);

    int printed=0;
    for(int i=0;i<g_ntracks && printed<n;i++){
        Track *t=&g_tracks[i]; if (!t->alive || t->age<REPORT_MIN_AGE) continue;
        RayN r=g_norm_lut[t->y*g_w + t->x];
        uint8_t q = age_to_quality(t->age);
        printf("VIOOBS,%u,%.6f,%.6f,%u,%u\n",
               (unsigned)t->id, r.xn, r.yn, (unsigned)t->age, (unsigned)q);
        printed++;
    }
    printf("VIOEND\n");
}

// ============================== AE/AGC SWEEP ================================
typedef struct { float avg; int zero; int aec; int agc; } sweep_stat_t;

static void run_quick_sweep(void){
    static const int AECV[] = {220, 300, 380, 460};
    static const int AGCV[] = {  8,  12,  16,  24};

    // manual during sweep
    sensor_set_auto(false, false);

    sweep_stat_t best = {.avg=-1e9f,.zero=9999,.aec=AECV[0],.agc=AGCV[0]};

    for (size_t i=0;i<sizeof(AECV)/sizeof(AECV[0]);++i){
        for (size_t j=0;j<sizeof(AGCV)/sizeof(AGCV[0]);++j){
            sensor_set_aec(AECV[i]);
            sensor_set_agc(AGCV[j]);
            vTaskDelay(pdMS_TO_TICKS(60)); // settle

            // Evaluate “corner richness” over a few frames
            const int frames = 8;
            int zero = 0; double sum = 0;
            for (int k=0;k<frames;k++){
                camera_fb_t *fb = esp_camera_fb_get();
                if(!fb){ k--; vTaskDelay(pdMS_TO_TICKS(2)); continue; }
                int n_kp=0; fast_detect_and_nms(fb->buf, g_w, g_h, &n_kp);
                sum += n_kp; if (n_kp==0) zero++;
                esp_camera_fb_return(fb);
            }
            float avg = (float)(sum/frames);
            bool better = (avg > best.avg + 0.5f)
                       || (fabsf(avg - best.avg) < 0.5f && zero < best.zero);
            if (better) best = (sweep_stat_t){avg, zero, AECV[i], AGCV[j]};

            ESP_LOGI(TAG, "SWEEP aec=%d agc=%d  avg=%.1f zero=%d", AECV[i], AGCV[j], avg, zero);
        }
    }

    // Apply winner and LOCK
// end of auto_sweep_choose()
    sensor_set_auto(false, false);          // freeze controllers
    sensor_set_aec(best.aec);
    sensor_set_agc(best.agc);
    ESP_LOGI(TAG, "LOCK aec=%d agc=%d (avg corners=%.1f)",
            best.aec, best.agc, best.avg);

}

// ================================ MAIN ======================================
void app_main(void){
    ESP_ERROR_CHECK(camera_init_gray());
    build_norm_lut(&K_OV3660_QVGA);

    // 1) Run sweep + lock AE/AGC
    run_quick_sweep();

    // 2) Warm previous AFTER sweep (so state reflects locked exposure)
    for (int i=0;i<2;i++){
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb){ memcpy(g_prev, fb->buf, g_w*g_h); esp_camera_fb_return(fb); }
    }

    // 3) Start tracking + VIO logging
    tracker_reset();

    uint32_t frames=0;
    while (true){
        camera_fb_t *fb=esp_camera_fb_get();
        if (!fb){ vTaskDelay(pdMS_TO_TICKS(1)); continue; }

        uint64_t t_us = (uint64_t)esp_timer_get_time();

        // dynamic search policy from a tiny texture score
        int tex = frame_texture_score(fb->buf,g_w,g_h);
        bool lowtex = (tex<12);
        g_win_rad = lowtex? WIN_RAD_LOW_TEX : WIN_RAD_DEFAULT;
        int ham_gate = HAM_ABS_MAX + (lowtex?1:0);

        // reseed policy
        if ((frames % RESEED_EVERY)==0){
            int alive_now=0; for(int i=0;i<g_ntracks;i++) if (g_tracks[i].alive) alive_now++;
            if (alive_now < TARGET_TRACKS){
                int n_kp=-1; fast_detect_and_nms(fb->buf,g_w,g_h,&n_kp);
                int cap = lowtex? RESEED_CAP_LOW : RESEED_CAP_PERF;
                int want = TARGET_TRACKS - alive_now; if (want>cap) want=cap;
                if (want>0 && n_kp>0){
                    int seed_min = lowtex? SEED_MIN_LOW : SEED_MIN_SCORE;
                    (void)tracker_seed_from_fast(keypoints,n_kp,fb->buf,want,seed_min);
                }
            }
        }

        // track
        TrkStats ts; (void)tracker_update(g_prev,fb->buf,&ts,ham_gate);

        // print VIO observations for this frame (row_time_us unknown: 0)
        log_vio_frame(frames, t_us, 0);

        // roll frame
        memcpy(g_prev, fb->buf, g_w*g_h);
        esp_camera_fb_return(fb);
        frames++;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// core0_sensing.c - Hall sensor capture (V1.1 - Config Integrated)
// =============================================================================
// MIGRATED: Nu gebruikt core0_config.h voor alle configureerbare parameters
// 
// V1.0 gedrag behouden als default (feature flags uit)
// V1.1 features beschikbaar via:
//   - CFG_DIR.enabled         : Direction hint uit A↔B pairing
//   - CFG_SENS.use_improved_fit: Verbeterde fit_error berekening
// =============================================================================

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "core0_config.h"
#include "core0_link.h"
#include "core0_filtered.h"

static const char *TAG = "S02-Core0";

// =============================================================================
// LEGACY POOL ENUM (voor interne gebruik)
// =============================================================================

typedef enum {
    LEGACY_POOL_SOUTH   = -1,
    LEGACY_POOL_NEUTRAL =  0,
    LEGACY_POOL_NORTH   =  1
} legacy_pool_t;

// =============================================================================
// V1.1: EXTENDED EDGE INFO (voor direction hint)
// =============================================================================

typedef struct {
    uint32_t t_us;
    pool_t   from_pool;
    pool_t   to_pool;
    bool     valid;
} last_edge_info_t;

// =============================================================================
// SAMPLE HISTORY (voor metrics berekening)
// =============================================================================

#define HISTORY_MAX 16

typedef struct {
    int16_t samples[HISTORY_MAX];
    uint8_t idx;
    uint8_t count;
    float   sum;
    float   sum_sq;
} sample_history_t;

// =============================================================================
// CONTEXT ACCUMULATOR (voor SNR berekening)
// =============================================================================

typedef struct {
    int   pre_count, post_count;
    float pre_sum, post_sum;
    float pre_absdev, post_absdev;
    float pre_ref, post_ref;
    bool  in_post;
} ctx_env_acc_t;

// =============================================================================
// GLOBALS
// =============================================================================

// Baselines
static float baseline_A;
static float baseline_B;
static int   last_raw_A;
static int   last_raw_B;

// Pool state
static legacy_pool_t stable_pool_A = LEGACY_POOL_NEUTRAL;
static legacy_pool_t stable_pool_B = LEGACY_POOL_NEUTRAL;

// Context accumulators
static ctx_env_acc_t ctxA, ctxB;
// Note: samples_since_cross_* reserved voor toekomstige context tracking

// Sample history
static sample_history_t histA = {0};
static sample_history_t histB = {0};

// V1.0: Simpele timestamps voor pair detection
static uint32_t last_edge_A_us = 0;
static uint32_t last_edge_B_us = 0;

// V1.1: Extended edge info (alleen gebruikt als CFG_DIR.enabled)
static last_edge_info_t last_edge_A_ext = {0};
static last_edge_info_t last_edge_B_ext = {0};

// Pool transition timing
static uint32_t last_pool_change_A_us = 0;
static uint32_t last_pool_change_B_us = 0;

// ADC handle
static adc_continuous_handle_t adc_handle;
static uint8_t rxbuf[2048];

// Filter context reference
#define FR_CTX (&g_fr)

// Debug
static int debug_counter = 0;
static int debug_sample_A = 0;
static int debug_sample_B = 0;

// =============================================================================
// HELPER: Timestamp
// =============================================================================

static inline int64_t now_us(void) {
    return esp_timer_get_time();
}

// =============================================================================
// HELPER: Pool mapping
// =============================================================================

static inline pool_t map_legacy_pool(legacy_pool_t lp) {
    if (lp == LEGACY_POOL_NORTH) return POOL_N;
    if (lp == LEGACY_POOL_SOUTH) return POOL_S;
    return POOL_NEU;
}

// =============================================================================
// HELPER: Context reset
// =============================================================================

static void ctx_env_reset(ctx_env_acc_t *a, float ref) {
    memset(a, 0, sizeof(*a));
    a->pre_ref = ref;
    a->post_ref = ref;
}

// =============================================================================
// HELPER: Pool classification (gebruikt CFG_SENS thresholds)
// =============================================================================

static inline legacy_pool_t classify_pool_A(int raw) {
    if (raw < CFG_SENS_A.low_th)  return LEGACY_POOL_SOUTH;
    if (raw > CFG_SENS_A.high_th) return LEGACY_POOL_NORTH;
    return LEGACY_POOL_NEUTRAL;
}

static inline legacy_pool_t classify_pool_B(int raw) {
    if (raw < CFG_SENS_B.low_th)  return LEGACY_POOL_SOUTH;
    if (raw > CFG_SENS_B.high_th) return LEGACY_POOL_NORTH;
    return LEGACY_POOL_NEUTRAL;
}

// =============================================================================
// SAMPLE HISTORY FUNCTIONS
// =============================================================================

static inline void history_push(sample_history_t* h, int16_t val) {
    uint8_t hist_len = CFG_SENS.history_len;
    if (hist_len > HISTORY_MAX) hist_len = HISTORY_MAX;  // Safety clamp
    
    if (h->count == hist_len) {
        int16_t old = h->samples[h->idx];
        h->sum -= old;
        h->sum_sq -= (float)old * old;
    }
    h->samples[h->idx] = val;
    h->sum += val;
    h->sum_sq += (float)val * val;
    h->idx = (h->idx + 1) % hist_len;  // Modulo by actual length, not max
    if (h->count < hist_len) h->count++;
}

static inline float history_noise(const sample_history_t* h) {
    if (h->count < 2) return 1.0f;
    float mean = h->sum / h->count;
    float var = (h->sum_sq / h->count) - (mean * mean);
    if (var < 0) var = 0;
    return sqrtf(var) + 0.1f;
}

static inline float history_monotonicity(const sample_history_t* h, bool rising) {
    if (h->count < 3) return 0.5f;
    
    int consistent = 0;
    int total = 0;
    
    for (int i = 0; i < h->count - 1; i++) {
        int curr_idx = (h->idx - 1 - i + HISTORY_MAX) % HISTORY_MAX;
        int prev_idx = (h->idx - 2 - i + HISTORY_MAX) % HISTORY_MAX;
        int16_t curr = h->samples[curr_idx];
        int16_t prev = h->samples[prev_idx];
        
        if (rising) {
            if (curr >= prev) consistent++;
        } else {
            if (curr <= prev) consistent++;
        }
        total++;
    }
    
    return (total > 0) ? (float)consistent / total : 0.5f;
}

// =============================================================================
// V1.0: FIT ERROR (origineel)
// =============================================================================

static inline float compute_fit_error_v10(float noise) {
    float fit_error = noise * 5.0f;
    if (fit_error > 255.0f) fit_error = 255.0f;
    return fit_error;
}

// =============================================================================
// V1.1: FIT ERROR (verbeterd - lineaire regressie)
// =============================================================================

static float compute_fit_error_v11(const sample_history_t* h) {
    uint8_t n = h->count;
    if (n < 3) return 0.0f;
    
    int32_t sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    
    for (uint8_t i = 0; i < n; i++) {
        uint8_t buf_idx = (h->idx - n + i + HISTORY_MAX) % HISTORY_MAX;
        int16_t y = h->samples[buf_idx];
        int32_t x = (int32_t)i;
        
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }
    
    int32_t denom = (int32_t)n * sum_xx - sum_x * sum_x;
    if (denom == 0) return 0.0f;
    
    float m = (float)((int32_t)n * sum_xy - sum_x * sum_y) / (float)denom;
    float b = (float)(sum_y - (int32_t)(m * (float)sum_x)) / (float)n;
    
    float sum_residual = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t buf_idx = (h->idx - n + i + HISTORY_MAX) % HISTORY_MAX;
        float y_pred = m * (float)i + b;
        sum_residual += fabsf((float)h->samples[buf_idx] - y_pred);
    }
    
    float fit_error = (sum_residual / (float)n) * 2.55f;
    return fminf(fit_error, 255.0f);
}

static float compute_fit_error_v11_fast(const sample_history_t* h) {
    uint8_t n = h->count;
    if (n < 3) return 0.0f;
    
    uint8_t idx_first = (h->idx - n + HISTORY_MAX) % HISTORY_MAX;
    uint8_t idx_mid   = (h->idx - n/2 + HISTORY_MAX) % HISTORY_MAX;
    uint8_t idx_last  = (h->idx - 1 + HISTORY_MAX) % HISTORY_MAX;
    
    int16_t y_first = h->samples[idx_first];
    int16_t y_mid   = h->samples[idx_mid];
    int16_t y_last  = h->samples[idx_last];
    
    float y_mid_expected = (float)(y_first + y_last) / 2.0f;
    float residual = fabsf((float)y_mid - y_mid_expected);
    
    int16_t y_min = (y_first < y_last) ? y_first : y_last;
    int16_t y_max = (y_first > y_last) ? y_first : y_last;
    if (y_mid < y_min || y_mid > y_max) {
        residual *= 2.0f;
    }
    
    return fminf(residual * 5.0f, 255.0f);
}

// =============================================================================
// V1.1: DIRECTION HINT
// =============================================================================

static dirhint_t determine_direction_v11(
    char current_sensor,
    pool_t current_from,
    pool_t current_to,
    const last_edge_info_t* other_edge,
    uint32_t dt_us,
    bool is_paired)
{
    (void)dt_us;
    
    if (!is_paired || !other_edge->valid) {
        return DIR_NONE;
    }
    
    if (CFG_DIR.require_consistent_transition) {
        bool consistent = (current_from == other_edge->from_pool && 
                          current_to == other_edge->to_pool);
        if (!consistent) {
            return DIR_NONE;
        }
    }
    
    bool A_first = (current_sensor != 'A');
    
    if (CFG_DIR.cw_is_A_first) {
        return A_first ? DIR_CW : DIR_CCW;
    } else {
        return A_first ? DIR_CCW : DIR_CW;
    }
}

// =============================================================================
// EMIT TO FILTER (unified V1.0/V1.1)
// =============================================================================

static void emit_to_filter(
    char sensor_char,
    legacy_pool_t from_pool,
    legacy_pool_t to_pool,
    float dvdt,
    float mono,
    float snr,
    float fit_error,
    uint32_t t_us,
    uint32_t other_edge_us,
    const last_edge_info_t* other_edge_ext,
    edgekind_t edge_kind)
{
    // Pair detection
    uint32_t dt_other = (t_us > other_edge_us) ? (t_us - other_edge_us) : 0;
    bool is_paired = (dt_other <= CFG_PAIR.window_us);
    
    // Direction hint (V1.0: NONE, V1.1: berekend)
    dirhint_t dir = DIR_NONE;
    
    if (CFG_DIR.enabled && other_edge_ext != NULL) {
        dir = determine_direction_v11(
            sensor_char,
            map_legacy_pool(from_pool),
            map_legacy_pool(to_pool),
            other_edge_ext,
            dt_other,
            is_paired
        );
    }
    
    // Convert to Q-formats
    int32_t dvdt_scaled = (int32_t)(fabsf(dvdt) / 100.0f);
    if (dvdt_scaled > 32767) dvdt_scaled = 32767;
    int16_t dvdt_q15 = (int16_t)dvdt_scaled;
    
    uint8_t mono_q8 = (uint8_t)(fminf(mono, 1.0f) * 255.0f);
    // SNR: clamp naar 0-127 range, dan ×2 voor 0-255
    // (origineel: snr * 2.0f, maar snr was al 0.1-100 range)
    uint8_t snr_q8 = (uint8_t)(fminf(snr * 2.0f, 255.0f));
    // Fit error: schaal 0-100 → 0-255 (originele formule)
    uint8_t fit_q8 = (uint8_t)(fminf(fit_error, 100.0f) * 2.55f);
    
    uint8_t polarity = (to_pool == LEGACY_POOL_NORTH) ? 1 : 0;
    
    // Build candidate
    fr_candidate_t X = {
        .t_abs_us    = t_us,
        .dvdt_q15    = dvdt_q15,
        .mono_q8     = mono_q8,
        .snr_q8      = snr_q8,
        .fit_err_q8  = fit_q8,
        .sensor      = (sensor_char == 'A') ? 0 : 1,
        .polarity    = polarity,
        .from_pool   = map_legacy_pool(from_pool),
        .to_pool     = map_legacy_pool(to_pool),
        .pair_flag   = is_paired ? 1 : 0,
        .dir_hint    = dir,
        .edge_kind   = (uint8_t)edge_kind,
    };
    
    fr_consider(FR_CTX, &X);
    
    // Update V1.0 timestamps
    if (sensor_char == 'A') {
        last_edge_A_us = t_us;
    } else {
        last_edge_B_us = t_us;
    }
    
    // Update V1.1 extended info
    if (CFG_DIR.enabled) {
        last_edge_info_t* my_ext = (sensor_char == 'A') ? &last_edge_A_ext : &last_edge_B_ext;
        my_ext->t_us = t_us;
        my_ext->from_pool = map_legacy_pool(from_pool);
        my_ext->to_pool = map_legacy_pool(to_pool);
        my_ext->valid = true;
    }
}

// =============================================================================
// HANDLE POOL TRANSITION
// =============================================================================

static void handle_pool_transition(
    char sensor,
    legacy_pool_t from_pool,
    legacy_pool_t to_pool,
    int raw_now,
    int raw_prev,
    uint32_t other_edge_us,
    const last_edge_info_t* other_ext,
    sample_history_t *hist,
    float baseline)
{
    // dV/dt
    float dy = (float)(raw_now - raw_prev);
    float dvdt = dy * (float)CFG_HW.per_channel_hz;
    
    // Monotonicity
    bool rising = (to_pool == LEGACY_POOL_NORTH) || 
                  (from_pool == LEGACY_POOL_SOUTH && to_pool == LEGACY_POOL_NEUTRAL);
    float mono = history_monotonicity(hist, rising);
    
    // SNR
    float signal = fabsf((float)raw_now - baseline);
    float noise = history_noise(hist);
    float snr = (noise > 0.1f) ? (signal / noise) : 100.0f;
    snr = fminf(fmaxf(snr, 0.1f), 100.0f);
    
    // Fit error (V1.0 of V1.1)
    float fit_error;
    if (CFG_SENS.use_improved_fit) {
        if (CFG_SENS.use_fast_fit) {
            fit_error = compute_fit_error_v11_fast(hist);
        } else {
            fit_error = compute_fit_error_v11(hist);
        }
    } else {
        fit_error = compute_fit_error_v10(noise);
    }
    
    // Neutral edge filtering
    bool is_neutral_edge = (to_pool == LEGACY_POOL_NEUTRAL || 
                            from_pool == LEGACY_POOL_NEUTRAL);
    
    if (!CFG_SENS.include_neutral_edges && is_neutral_edge) {
        return;
    }
    
    // Emit
    uint32_t t_now = (uint32_t)now_us();
    edgekind_t ek = is_neutral_edge ? EDGE_KIND_POOL_NEUT : EDGE_KIND_POOL_HARD;
    
    emit_to_filter(
        sensor,
        from_pool,
        to_pool,
        dvdt,
        mono,
        snr,
        fit_error,
        t_now,
        other_edge_us,
        other_ext,
        ek
    );
}

// =============================================================================
// ADC SETUP
// =============================================================================

static esp_err_t setup_adc_continuous(void) {
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 4096,
        .conv_frame_size = 1024,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));
    
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = CFG_HW.sample_rate_hz,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    
    static adc_digi_pattern_config_t pattern[2];
    pattern[0].atten     = CFG_HW.adc_atten;
    pattern[0].channel   = CFG_HW.adc_channel_A;
    pattern[0].unit      = ADC_UNIT_1;
    pattern[0].bit_width = CFG_HW.adc_bitwidth;
    
    pattern[1].atten     = CFG_HW.adc_atten;
    pattern[1].channel   = CFG_HW.adc_channel_B;
    pattern[1].unit      = ADC_UNIT_1;
    pattern[1].bit_width = CFG_HW.adc_bitwidth;
    
    dig_cfg.pattern_num = 2;
    dig_cfg.adc_pattern = pattern;
    
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    
    return ESP_OK;
}

// =============================================================================
// WDT SETUP
// =============================================================================

static void setup_wdt(void) {
    const esp_task_wdt_config_t twdt_cfg = {
        .timeout_ms = 3000,
        .idle_core_mask = 0,
        .trigger_panic = false
    };
    esp_err_t err = esp_task_wdt_init(&twdt_cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_cfg));
    }
}

// =============================================================================
// CAPTURE TASK
// =============================================================================

static void capture_task(void *arg) {
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    
    // Initialiseer baselines uit config
    baseline_A = CFG_SENS_A.baseline_init;
    baseline_B = CFG_SENS_B.baseline_init;
    last_raw_A = (int)baseline_A;
    last_raw_B = (int)baseline_B;
    
    // Context windows
    const int pre_win = (int)((CFG_SENS.context_ms_pre * 1e-3f) * CFG_HW.per_channel_hz);
    const int post_win = (int)((CFG_SENS.context_ms_post * 1e-3f) * CFG_HW.per_channel_hz);
    
    ctx_env_reset(&ctxA, baseline_A);
    ctx_env_reset(&ctxB, baseline_B);
    
    uint32_t wdt_counter = 0;
    
    while (1) {
        uint32_t n_read = 0;
        esp_err_t ret = adc_continuous_read(
            adc_handle,
            rxbuf,
            sizeof(rxbuf),
            &n_read,
            100
        );
        
        if (ret == ESP_OK && n_read > 0) {
            for (uint32_t i = 0; i < n_read; i += sizeof(adc_digi_output_data_t)) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)&rxbuf[i];
                int ch = p->type1.channel;
                int raw = p->type1.data;
                
                // =========================================================
                // SENSOR A
                // =========================================================
                if (ch == CFG_HW.adc_channel_A) {
                    // Baseline tracking
                    float alpha = CFG_SENS.baseline_alpha;
                    baseline_A = (1.0f - alpha) * baseline_A + alpha * (float)raw;
                    
                    debug_sample_A = raw;
                    history_push(&histA, (int16_t)raw);
                    
                    // Context accumulation
                    if (!ctxA.in_post) {
                        if (ctxA.pre_count < pre_win) {
                            ctxA.pre_sum += raw;
                            ctxA.pre_count++;
                        }
                    } else {
                        if (ctxA.post_count < post_win) {
                            ctxA.post_sum += raw;
                            ctxA.post_count++;
                        }
                    }
                    
                    float devA = fabsf((float)raw - baseline_A);
                    if (!ctxA.in_post && ctxA.pre_count > 0) ctxA.pre_absdev += devA;
                    if (ctxA.in_post && ctxA.post_count > 0) ctxA.post_absdev += devA;
                    
                    // Pool transition detection
                    legacy_pool_t currA = classify_pool_A(raw);
                    if (currA != stable_pool_A) {
                        uint32_t t_now = (uint32_t)now_us();
                        uint32_t dt = t_now - last_pool_change_A_us;
                        
                        if (dt >= CFG_SENS.pool_min_dt_us) {
                            const last_edge_info_t* other_ext = NULL;
                            if (CFG_DIR.enabled) {
                                other_ext = &last_edge_B_ext;
                            }
                            
                            handle_pool_transition(
                                'A', stable_pool_A, currA,
                                raw, last_raw_A,
                                last_edge_B_us,
                                other_ext,
                                &histA, baseline_A
                            );
                            
                            stable_pool_A = currA;
                            last_pool_change_A_us = t_now;
                        }
                    }
                    
                    last_raw_A = raw;
                }
                // =========================================================
                // SENSOR B
                // =========================================================
                else if (ch == CFG_HW.adc_channel_B) {
                    float alpha = CFG_SENS.baseline_alpha;
                    baseline_B = (1.0f - alpha) * baseline_B + alpha * (float)raw;
                    
                    debug_sample_B = raw;
                    history_push(&histB, (int16_t)raw);
                    
                    if (!ctxB.in_post) {
                        if (ctxB.pre_count < pre_win) {
                            ctxB.pre_sum += raw;
                            ctxB.pre_count++;
                        }
                    } else {
                        if (ctxB.post_count < post_win) {
                            ctxB.post_sum += raw;
                            ctxB.post_count++;
                        }
                    }
                    
                    float devB = fabsf((float)raw - baseline_B);
                    if (!ctxB.in_post && ctxB.pre_count > 0) ctxB.pre_absdev += devB;
                    if (ctxB.in_post && ctxB.post_count > 0) ctxB.post_absdev += devB;
                    
                    legacy_pool_t currB = classify_pool_B(raw);
                    if (currB != stable_pool_B) {
                        uint32_t t_now = (uint32_t)now_us();
                        uint32_t dt = t_now - last_pool_change_B_us;
                        
                        if (dt >= CFG_SENS.pool_min_dt_us) {
                            const last_edge_info_t* other_ext = NULL;
                            if (CFG_DIR.enabled) {
                                other_ext = &last_edge_A_ext;
                            }
                            
                            handle_pool_transition(
                                'B', stable_pool_B, currB,
                                raw, last_raw_B,
                                last_edge_A_us,
                                other_ext,
                                &histB, baseline_B
                            );
                            
                            stable_pool_B = currB;
                            last_pool_change_B_us = t_now;
                        }
                    }
                    
                    last_raw_B = raw;
                }
            }
        }
        
        // WDT feed
        wdt_counter++;
        if (wdt_counter >= 100) {
            esp_task_wdt_reset();
            wdt_counter = 0;
        }
        
        // Debug output
        debug_counter++;
        if (debug_counter >= 1000) {
            if (CFG_DBG.general.heartbeat_interval_ms > 0) {
                ESP_LOGI(TAG, "A=%d base=%.0f | B=%d base=%.0f",
                        debug_sample_A, baseline_A,
                        debug_sample_B, baseline_B);
            }
            debug_counter = 0;
        }
        
        taskYIELD();
    }
}

// =============================================================================
// APP MAIN
// =============================================================================

void app_main(void) {
    // 1) Config EERST initialiseren
    core0_config_init();
    core0_config_print();
    
    if (!core0_config_validate()) {
        ESP_LOGE(TAG, "Invalid config - check settings!");
    }
    
    ESP_LOGI(TAG, "%s v%s (mode: %s)", 
             CORE0_FW_BUILD_TAG, 
             CORE0_FW_VERSION_STR,
             IS_DEBUG_BYPASS ? "DEBUG_BYPASS" : 
             IS_RAW_CALIBRATE ? "RAW_CALIBRATE" :
             IS_SMOKE_TEST ? "SMOKE_TEST" : "PRODUCTION");
    
    // Log V1.1 feature status
    ESP_LOGI(TAG, "V1.1 Features: dir_hint=%s, improved_fit=%s",
             CFG_DIR.enabled ? "ON" : "OFF",
             CFG_SENS.use_improved_fit ? "ON" : "OFF");
    
    // 2) WDT
    setup_wdt();
    
    // 3) Link layer
    core0_link_init();
    
    // 4) Filter layer
    fr_init_default(core0_link_get_tx());
    
    // 5) ADC
    if (setup_adc_continuous() != ESP_OK) {
        ESP_LOGE(TAG, "ADC setup failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    
    // 6) Init pool states
    stable_pool_A = LEGACY_POOL_NEUTRAL;
    stable_pool_B = LEGACY_POOL_NEUTRAL;
    
    ESP_LOGI(TAG, "Starting capture task...");
    
    // 7) Start capture
    xTaskCreatePinnedToCore(
        capture_task,
        "capture",
        8192,
        NULL,
        5,
        NULL,
        0
    );
}

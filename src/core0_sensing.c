// core0_sensing.c - Integrated Hall sensor → Binary UART pipeline
//
// Dit vervangt de JSONL output met het binaire framing protocol.
// De detectie-logica blijft identiek aan core0_sensing_01.c

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

// === NIEUWE INCLUDES VOOR BINARY FRAMING ===
#include "core0_link.h"
#include "core0_filtered.h"

static const char *TAG = "S02-Core0";

// ---- Config ----
#define FR_INCLUDE_NEUTRAL_EDGES    1
#define HISTORY_LEN                 8       // samples voor slope/noise berekening
#define SAMPLE_RATE_HZ              200000
#define CONTEXT_MS_PRE              0.7f
#define CONTEXT_MS_POST             0.7f

#define ADC_UNIT_USE        ADC_UNIT_1
#define ADC_CH_A            ADC_CHANNEL_6   // GPIO34
#define ADC_CH_B            ADC_CHANNEL_7   // GPIO35
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH       ADC_BITWIDTH_12

// ---- Pool classification ----
// Map naar core0_filtered.h pool_t (POOL_NEU=0, POOL_N=1, POOL_S=2)
static inline pool_t map_legacy_pool(int legacy_pool) {
    // Values:    POOL_NEU=0, POOL_N=1, POOL_S=2
    if (legacy_pool == 1)  return POOL_N;   // NORTH
    if (legacy_pool == -1) return POOL_S;   // SOUTH
    return POOL_NEU;                        // NEUTRAL
}

// Legacy pool enum (behouden voor backwards compat)
typedef enum {
    LEGACY_POOL_SOUTH = -1,
    LEGACY_POOL_NEUTRAL = 0,
    LEGACY_POOL_NORTH = 1
} legacy_pool_t;

// ---- Context accumulator ----
typedef struct {
    int   pre_count,  post_count;
    float pre_sum,    post_sum;
    float pre_absdev, post_absdev;
    float pre_ref,    post_ref;
    bool  in_post;
} ctx_env_acc_t;

// ---- Globals ----
static float baseline_A = 2048.0f;
static float baseline_B = 2048.0f;
static int   last_raw_A = 2048;
static int   last_raw_B = 2048;

/*
============================================================
  THRESHOLD SUGGESTIONS (copy to core0_sensing.c)
============================================================

// Baseline A: 2005, range: 1347 - 2905
static int A_low_th  = 1741;   // SOUTH threshold
static int A_high_th = 2364;   // NORTH threshold

// Baseline B: 979, range: 586 - 1493
static int B_low_th  = 821;    // SOUTH threshold
static int B_high_th = 1184;   // NORTH threshold

// Alternative (tighter, more sensitive):
// A_low=1610, A_high=2544
// B_low=743, B_high=1287
============================================================
*/
static int A_low_th  = 1610;    // eerder 1740 (vroeger 1500)
static int A_high_th = 2540;    // eerder 2360 (vroeger 2300)
static int B_low_th  = 740;     // eerder 820  (vroeger 700)
static int B_high_th = 1290;    // eerder 1180 (vroeger 1200)

static legacy_pool_t stable_pool_A = LEGACY_POOL_NEUTRAL;
static legacy_pool_t stable_pool_B = LEGACY_POOL_NEUTRAL;

// Context accumulators
static ctx_env_acc_t ctxA, ctxB;
static int samples_since_cross_A = 0;
static int samples_since_cross_B = 0;

// Pair detection state
static uint32_t last_edge_A_us = 0;
static uint32_t last_edge_B_us = 0;
#define PAIR_WINDOW_US 3000  // 3ms pair window

// Minimale tijd tussen pool-transitions om chatter te voorkomen
#define POOL_MIN_DT_US 200 // 0.2 ms, vrij conservatief

static uint32_t last_pool_change_A_us = 0;
static uint32_t last_pool_change_B_us = 0;

// ADC handle en DMA buffer
static adc_continuous_handle_t adc_handle;
static uint8_t rxbuf[2048];

// Filter context (extern defined in core0_filtered.c)
extern fr_ctx_t g_fr;
#define FR_CTX (&g_fr)

// Debug counters
static int debug_counter = 0;
static int debug_sample_A = 0;
static int debug_sample_B = 0;

// ---- Helpers ----
static inline int64_t now_us(void) {
    return esp_timer_get_time();
}

static void ctx_env_reset(ctx_env_acc_t *a, float ref) {
    memset(a, 0, sizeof(*a));
    a->pre_ref = ref;
    a->post_ref = ref;
}

static inline legacy_pool_t classify_pool_A(int raw) {
    if (raw < A_low_th)  return LEGACY_POOL_SOUTH;
    if (raw > A_high_th) return LEGACY_POOL_NORTH;
    return LEGACY_POOL_NEUTRAL;
}

static inline legacy_pool_t classify_pool_B(int raw) {
    if (raw < B_low_th)  return LEGACY_POOL_SOUTH;
    if (raw > B_high_th) return LEGACY_POOL_NORTH;
    return LEGACY_POOL_NEUTRAL;
}

// ============================================================================
// VERBETERDE Pool flank detectie met ECHTE metrics
// ============================================================================

typedef struct {
    int16_t samples[HISTORY_LEN];
    uint8_t idx;
    uint8_t count;
    float sum;
    float sum_sq;  // voor variance
} sample_history_t;

static sample_history_t histA = {0};
static sample_history_t histB = {0};

// Helper: voeg sample toe aan history
static inline void history_push(sample_history_t* h, int16_t val) {
    // Update running stats voordat we oude waarde overschrijven
    if (h->count == HISTORY_LEN) {
        int16_t old = h->samples[h->idx];
        h->sum -= old;
        h->sum_sq -= (float)old * old;
    }

    h->samples[h->idx] = val;
    h->sum += val;
    h->sum_sq += (float)val * val;

    h->idx = (h->idx + 1) % HISTORY_LEN;
    if (h->count < HISTORY_LEN) h->count++;
}

// Helper: bereken noise (standaarddeviatie) over history
static inline float history_noise(const sample_history_t* h) {
    if (h->count < 2) return 1.0f;
    float mean = h->sum / h->count;
    float var = (h->sum_sq / h->count) - (mean * mean);
    if (var < 0) var = 0;  // numerical safety
    return sqrtf(var) + 0.1f;  // +0.1 om div/0 te voorkomen
}

// Helper: bereken monotonicity over recente samples
// Retourneert 0.0-1.0 waar 1.0 = perfect monotoon stijgend/dalend
static inline float history_monotonicity(const sample_history_t* h, bool rising) {
    if (h->count < 3) return 0.5f;

    int consistent = 0;
    int total = 0;

    // Loop door history (nieuwste eerst)
    for (int i = 0; i < h->count - 1; i++) {
        int curr_idx = (h->idx - 1 - i + HISTORY_LEN) % HISTORY_LEN;
        int prev_idx = (h->idx - 2 - i + HISTORY_LEN) % HISTORY_LEN;

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

// ============================================================================
// NIEUWE FUNCTIE: Emit naar binary framing via fr_consider()
// ============================================================================
static void emit_to_filter(
    char sensor_char,
    legacy_pool_t from_pool,
    legacy_pool_t to_pool,
    float dvdt,
    float mono,
    float snr,
    float fit_error,
    uint32_t t_us,
    uint32_t other_sensor_last_edge_us,
    edgekind_t edge_kind)
{
    // Bepaal of dit een paired event is (andere sensor recent actief)
    uint32_t dt_other = t_us - other_sensor_last_edge_us;
    bool is_paired = (dt_other <= PAIR_WINDOW_US);

    // Convert floats naar Q-formaten
    // dvdt_q15: we schalen dvdt naar een redelijk bereik
    // Origineel: dvdt = dy * SAMPLE_RATE_HZ (kan groot zijn, bv 1e6+)
    // We schalen naar ~0-32767 range voor typische waarden
    int32_t dvdt_scaled = (int32_t)(fabsf(dvdt) / 100.0f);  // Tune deze divisor!
    if (dvdt_scaled > 32767) dvdt_scaled = 32767;
    int16_t dvdt_q15 = (int16_t)dvdt_scaled;

    // mono_q8: 0.0-1.0 → 0-255
    uint8_t mono_q8 = (uint8_t)(fminf(mono, 1.0f) * 255.0f);

    // snr_q8: we clampen SNR naar 0-127 range (als 0.5 dB per LSB)
    // snr van 2.0 = 6dB, snr van 99 = ~40dB
    uint8_t snr_q8 = (uint8_t)(fminf(snr * 2.0f, 255.0f));

    // fit_err_q8: lagere is beter, we schalen 0-100 → 0-255
    uint8_t fit_q8 = (uint8_t)(fminf(fit_error, 100.0f) * 2.55f);

    // Polarity: stijgend (+) = N, dalend (-) = S
    uint8_t polarity = (to_pool == LEGACY_POOL_NORTH) ? 1 : 0;

    // Direction hint (voorlopig none, kan later uit A/B timing)
    dirhint_t dir = DIR_NONE;

    // Build candidate
    fr_candidate_t X = {
        .t_abs_us   = t_us,
        .dvdt_q15   = dvdt_q15,
        .mono_q8    = mono_q8,
        .snr_q8     = snr_q8,
        .fit_err_q8 = fit_q8,
        .sensor     = (sensor_char == 'A') ? 0 : 1,
        .polarity   = polarity,
        .from_pool  = map_legacy_pool(from_pool),
        .to_pool    = map_legacy_pool(to_pool),
        .pair_flag  = is_paired ? 1 : 0,
        .dir_hint   = dir,
        .edge_kind  = (uint8_t)edge_kind,
    };

    // Submit to filter pipeline
    fr_consider(FR_CTX, &X);
}

// ============================================================================
// Zero-cross detectie (AANGEPAST: roept emit_to_filter aan i.p.v. emit_jsonl)
// ============================================================================
static void hook_zero_cross_candidate(
    char sensor,
    int prev_raw,
    int curr_raw,
    float baseline,
    ctx_env_acc_t *ctx,
    bool *has_event,
    uint32_t *other_last_edge_us,
    edgekind_t ek)
{
    // Detecteer sign flip rond baseline
    bool prev_above = (prev_raw > baseline);
    bool curr_above = (curr_raw > baseline);
    if (prev_above == curr_above) { *has_event = false; return; }

    // Lineaire crossing-interpolatie
    float y0 = (float)prev_raw - baseline;
    float y1 = (float)curr_raw - baseline;
    float dy = y1 - y0;

    // dv/dt schatting
    float dvdt = dy * (float)SAMPLE_RATE_HZ;

    // Fit error
    float fit_error = fabsf(y0) + fabsf(y1);

    // Monotonicity
    float mono = (fabsf(dy) / (fabsf(y0) + fabsf(y1) + 1e-6f));

    // Context-SNR via pre/post vensters
    float pre_mad  = ctx->pre_count  ? (ctx->pre_absdev  / ctx->pre_count ) : 1.0f;
    float post_mad = ctx->post_count ? (ctx->post_absdev / ctx->post_count) : 1.0f;
    float signal_pk = fmaxf(fabsf(y0), fabsf(y1));
    float noise_est = 0.5f * (pre_mad + post_mad);
    float snr = (noise_est > 1e-3f) ? (signal_pk / noise_est) : 99.0f;

    // Quality gate (zachter dan voorheen - laat filter beslissen)
    if (mono < 0.10f || snr < 1.5f) {
        *has_event = false;
        return;
    }

    // Bepaal pools
    legacy_pool_t from_pool = (dy > 0) ? LEGACY_POOL_SOUTH : LEGACY_POOL_NORTH;
    legacy_pool_t to_pool   = (dy > 0) ? LEGACY_POOL_NORTH : LEGACY_POOL_SOUTH;

    uint32_t t_now = (uint32_t)now_us();

    // === EMIT NAAR BINARY FILTER ===
    emit_to_filter(
        sensor,
        from_pool,
        to_pool,
        dvdt,
        mono,
        snr,
        fit_error,
        t_now,
        *other_last_edge_us,
        ek
    );

    // Update last edge time voor pair detection
    if (sensor == 'A') {
        last_edge_A_us = t_now;
    } else {
        last_edge_B_us = t_now;
    }

    *has_event = true;

    // Reset context voor de volgende flank
    ctx_env_reset(ctx, baseline);
}

// ============================================================================
// NIEUWE handle_pool_transition met ECHTE metrics
// ============================================================================
static void handle_pool_transition(
    char sensor,
    legacy_pool_t from_pool,
    legacy_pool_t to_pool,
    int raw_now,
    int raw_prev,
    uint32_t *other_last_edge_us,
    sample_history_t *hist,      // NEW: pass history
    float baseline)              // NEW: pass baseline
{
    // --- 1. ECHTE dV/dt berekening ---
    float dy = (float)(raw_now - raw_prev);
    float dvdt = dy * (SAMPLE_RATE_HZ / 2.0f);  // Per-channel rate

    // --- 2. ECHTE Monotonicity ---
    // Bepaal richting: stijgend naar NORTH, dalend naar SOUTH
    bool rising = (to_pool == LEGACY_POOL_NORTH) ||
                  (from_pool == LEGACY_POOL_SOUTH && to_pool == LEGACY_POOL_NEUTRAL);
    float mono = history_monotonicity(hist, rising);

    // --- 3. ECHTE SNR ---
    // Signal = absolute afwijking van baseline
    float signal = fabsf((float)raw_now - baseline);
    // Gebruik noise/variance als maat voor signaal-kwaliteit
    // Hogere variance = meer ruis = slechtere "fit"
    float noise = history_noise(hist);

    float snr = signal / noise;

    // Clamp SNR to reasonable range
    if (snr > 100.0f) snr = 100.0f;
    if (snr < 0.1f) snr = 0.1f;

    // --- 4. VERBETERDE Fit Error ---
    // Normaliseer naar 0-255 range
    // noise van 0-50 ADC counts → fit_err 0-255
    float fit_error = noise * 5.0f;  // schaalfactor tunen naar je ADC range
    if (fit_error > 255.0f) fit_error = 255.0f;

    // ALTERNATIEF: gebruik afwijking van verwachte slope
    // Dit meet hoe "schoon" de transitie is
    /*
    float samples_span = 0;
    if (hist->count >= 2) {
        int newest = (hist->idx - 1 + HISTORY_LEN) % HISTORY_LEN;
        int oldest = (hist->idx - hist->count + HISTORY_LEN) % HISTORY_LEN;
        samples_span = fabsf((float)(hist->samples[newest] - hist->samples[oldest]));
    }
    // Verwachte span bij perfecte lineaire transitie vs werkelijke noise
    float expected_noise = samples_span / (hist->count + 1);
    float actual_noise = noise;
    fit_error = fabsf(actual_noise - expected_noise) * 10.0f;
    if (fit_error > 255.0f) fit_error = 255.0f;
    */

    // --- 5. Check of dit een neutral edge is ---
    bool is_neutral_edge = (to_pool == LEGACY_POOL_NEUTRAL ||
                            from_pool == LEGACY_POOL_NEUTRAL);

#if !FR_INCLUDE_NEUTRAL_EDGES
    if (is_neutral_edge) return;
#endif

    // --- 6. Timestamp en edge kind ---
    uint32_t t_now = (uint32_t)now_us();
    edgekind_t ek = is_neutral_edge ? EDGE_KIND_POOL_NEUT : EDGE_KIND_POOL_HARD;

    // --- 7. Emit naar filter met ECHTE waarden ---
    emit_to_filter(
        sensor,
        from_pool,
        to_pool,
        dvdt,
        mono,       // ECHT: 0.0-1.0
        snr,        // ECHT: signal/noise ratio
        fit_error,  // ECHT: afwijking van lineaire trend
        t_now,
        *other_last_edge_us,
        ek
    );

    // Update last edge time
    if (sensor == 'A') {
        last_edge_A_us = t_now;
    } else {
        last_edge_B_us = t_now;
    }
}

// ============================================================================
// ADC Setup
// ============================================================================
static esp_err_t setup_adc_continuous(void)
{
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 4096,
        .conv_frame_size    = 1024,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    static adc_digi_pattern_config_t pattern[2];
    pattern[0].atten     = ADC_ATTEN;
    pattern[0].channel   = ADC_CH_A;
    pattern[0].unit      = ADC_UNIT_1;
    pattern[0].bit_width = ADC_BIT_WIDTH;

    pattern[1].atten     = ADC_ATTEN;
    pattern[1].channel   = ADC_CH_B;
    pattern[1].unit      = ADC_UNIT_1;
    pattern[1].bit_width = ADC_BIT_WIDTH;

    dig_cfg.pattern_num  = 2;
    dig_cfg.adc_pattern  = pattern;

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    return ESP_OK;
}

// ============================================================================
// WDT Setup
// ============================================================================
static void setup_wdt(void)
{
    const esp_task_wdt_config_t twdt_cfg = {
        .timeout_ms     = 3000,   // 3s timeout
        .idle_core_mask = 0,
        .trigger_panic  = false
    };
    esp_err_t err = esp_task_wdt_init(&twdt_cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_cfg));
    }
}

#include "core0_raw_sampling.c"

// ============================================================================
// Capture Task
// ============================================================================
static void capture_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    const float per_ch_hz = (float)SAMPLE_RATE_HZ / 2.0f;
    const int pre_win  = (int)((CONTEXT_MS_PRE  * 1e-3f) * per_ch_hz);
    const int post_win = (int)((CONTEXT_MS_POST * 1e-3f) * per_ch_hz);

    ctx_env_reset(&ctxA, baseline_A);
    ctx_env_reset(&ctxB, baseline_B);

    uint32_t wdt_counter = 0;

#if RAW_SAMPLE_MODE
        bool fresh_A = false;
        bool fresh_B = false;
        int last_raw_sample_A, last_raw_sample_B;
#endif

    while (1) {
        uint32_t n_read = 0;
        esp_err_t ret = adc_continuous_read(
            adc_handle,
            rxbuf,
            sizeof(rxbuf),
            &n_read,
            100  // Kortere timeout voor responsive WDT
        );

        if (ret == ESP_OK && n_read > 0) {
            for (uint32_t i = 0; i < n_read; i += sizeof(adc_digi_output_data_t)) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)&rxbuf[i];
                int ch  = p->type1.channel;
                int raw = p->type1.data;

                if (ch == ADC_CH_A) {
                    // Baseline tracking
                    baseline_A = 0.999f * baseline_A + 0.001f * (float)raw;

#if RAW_SAMPLE_MODE
                    fresh_A = true;
                    last_raw_sample_A = raw;
                    history_push(&histA, (int16_t)raw);  // Voor stats
#else
                    // === NORMALE EVENT DETECTIE CODE ===
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
                    if (!ctxA.in_post && ctxA.pre_count > 0)  ctxA.pre_absdev  += devA;
                    if ( ctxA.in_post && ctxA.post_count > 0) ctxA.post_absdev += devA;

                    // Pool transition detectie
                    legacy_pool_t currA = classify_pool_A(raw);

                    if (currA != stable_pool_A) {
                        // Tijd-hysteresis: voorkom gechatters rond thresholds
                        uint32_t t_now = (uint32_t)now_us();
                        uint32_t dt = t_now - last_pool_change_A_us;

                        if (dt >= POOL_MIN_DT_US) {
                            // Dit pakt alle echte overgangen: S↔NEU↔N
                            // AANGEPASTE aanroep met history en baseline:
                            handle_pool_transition('A', stable_pool_A, currA,
                                                raw, last_raw_A, &last_edge_B_us,
                                                &histA, baseline_A);  // <-- NIEUW
                            stable_pool_A = currA;
                            last_pool_change_A_us = t_now;
                        }
                    }

                    // Zero-cross detectie
                    bool has = false;
                    hook_zero_cross_candidate('A', last_raw_A, raw, baseline_A,
                                              &ctxA, &has, &last_edge_B_us, EDGE_KIND_ZC);
                    if (has) {
                        ctxA.in_post = true;
                        samples_since_cross_A = 0;
                    } else {
                        if (ctxA.in_post) {
                            samples_since_cross_A++;
                            if (samples_since_cross_A > post_win) {
                                ctxA.in_post = false;
                            }
                        }
                    }
                    last_raw_A = raw;

#endif // !RAW_SAMPLE_MODE
                }
                else if (ch == ADC_CH_B) {
                    baseline_B = 0.999f * baseline_B + 0.001f * (float)raw;

#if RAW_SAMPLE_MODE
                    fresh_B = true;
                    last_raw_sample_B = raw;
                    history_push(&histB, (int16_t)raw);
#else
                    // === NORMALE EVENT DETECTIE CODE ===
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
                    if (!ctxB.in_post && ctxB.pre_count > 0)  ctxB.pre_absdev  += devB;
                    if ( ctxB.in_post && ctxB.post_count > 0) ctxB.post_absdev += devB;

                    legacy_pool_t currB = classify_pool_B(raw);

                    if (currB != stable_pool_B) {
                        uint32_t t_now = (uint32_t)now_us();
                        uint32_t dt = t_now - last_pool_change_B_us;

                        if (dt >= POOL_MIN_DT_US) {
                            // AANGEPASTE aanroep:
                            handle_pool_transition('B', stable_pool_B, currB,
                                                raw, last_raw_B, &last_edge_A_us,
                                                &histB, baseline_B);  // <-- NIEUW
                            stable_pool_B = currB;
                            last_pool_change_B_us = t_now;
                        }
                    }

                    bool has = false;
                    hook_zero_cross_candidate('B', last_raw_B, raw, baseline_B,
                                              &ctxB, &has, &last_edge_A_us, EDGE_KIND_ZC);
                    if (has) {
                        ctxB.in_post = true;
                        samples_since_cross_B = 0;
                    } else {
                        if (ctxB.in_post) {
                            samples_since_cross_B++;
                            if (samples_since_cross_B > post_win) {
                                ctxB.in_post = false;
                            }
                        }
                    }
                    last_raw_B = raw;

#endif // !RAW_SAMPLE_MODE
                }
                // =============================================================
                // RAW SAMPLE LOGGING - NA beide channels verwerkt
                // =============================================================
#if RAW_SAMPLE_MODE
                // Log alleen als we BEIDE channels vers hebben
                if (fresh_A && fresh_B) {
                    raw_sample_capture_loop(last_raw_sample_A, last_raw_sample_B,
                                            baseline_A, baseline_B);
                    fresh_A = false;
                    fresh_B = false;
                }
#endif
            } // end for loop over samples
        } // end if read OK

        // WDT feed elke ~100 iteraties
        wdt_counter++;
        if (wdt_counter >= 100) {
            esp_task_wdt_reset();
            wdt_counter = 0;
        }

#if !RAW_SAMPLE_MODE
        // Debug output elke ~1000 iteraties
        debug_counter++;
        if (debug_counter >= 1000) {
            // Stuur debug info via ESP_LOGI (gaat naar serial maar niet in binair protocol)
            ESP_LOGI(TAG, "A=%d base=%.0f | B=%d base=%.0f",
                     debug_sample_A, baseline_A,
                     debug_sample_B, baseline_B);
            debug_counter = 0;
        }
#endif
        // Minimale yield
        taskYIELD();
    }
}

// ============================================================================
// App Main
// ============================================================================
void app_main(void)
{
    ESP_LOGI(TAG, "S02 Core-0 Binary Framing v1.0");

    // 1) WDT setup
    setup_wdt();

    // 2) Binary link layer init (UART + queue + writer task)
    core0_link_init();

    // 3) Filter layer init
    fr_init_default(core0_link_get_tx());

    // 4) ADC continuous setup
    if (setup_adc_continuous() != ESP_OK) {
        ESP_LOGE(TAG, "ADC setup failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // 5) Init pool states
    stable_pool_A = LEGACY_POOL_NEUTRAL;
    stable_pool_B = LEGACY_POOL_NEUTRAL;

    ESP_LOGI(TAG, "Starting capture task...");

    // 6) Start capture task
    xTaskCreatePinnedToCore(
        capture_task,
        "capture",
        8192,          // Grotere stack voor float math
        NULL,
        5,
        NULL,
        0              // Core 0
    );
}

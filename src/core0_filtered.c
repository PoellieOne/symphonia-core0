// core0_filtered.c - Filter layer (V1.1 - Config Integrated)
// =============================================================================
// MIGRATED: Nu gebruikt core0_config.h voor alle configureerbare parameters
// =============================================================================

#include "core0_filtered.h"
#include "core0_config.h"

#include <string.h>
#include "esp_timer.h"

// =============================================================================
// GLOBAL CONTEXT
// =============================================================================

fr_ctx_t g_fr;

// =============================================================================
// PRIVATE DATA
// =============================================================================

static esp_timer_handle_t s_fr_timer;

// Outbox queue
#define FR_OUTBOX_CAP 64

typedef struct {
    fr_event_t buf[FR_OUTBOX_CAP];
    uint16_t r, w;
} fr_outbox_t;

static fr_outbox_t s_outbox = {0};

// Rate statistics
typedef struct {
    uint32_t emitted;
    uint32_t emitted_weak;
    uint32_t emitted_normal;
    uint32_t emitted_strong;
    uint32_t dropped_weak;
    uint32_t dropped_norm;
    uint32_t dropped_strong;
    uint32_t considered_total;
    uint32_t rejected_quality;
    uint32_t last_report_ms;
} rate_stats_t;

static rate_stats_t g_rate_stats = {0};

// =============================================================================
// HELPERS
// =============================================================================

static inline uint8_t clampu8(int v) { 
    if (v < 0) return 0; 
    if (v > 255) return 255; 
    return (uint8_t)v; 
}

static inline bool outbox_push(const fr_event_t* e) {
    uint16_t n = (s_outbox.w + 1) % FR_OUTBOX_CAP;
    if (n == s_outbox.r) return false;
    s_outbox.buf[s_outbox.w] = *e;
    s_outbox.w = n;
    return true;
}

static inline bool outbox_pop(fr_event_t* e) {
    if (s_outbox.r == s_outbox.w) return false;
    *e = s_outbox.buf[s_outbox.r];
    s_outbox.r = (s_outbox.r + 1) % FR_OUTBOX_CAP;
    return true;
}

// =============================================================================
// TIMER CALLBACK
// =============================================================================

static void fr_timer_cb(void* arg) {
    static uint32_t ms = 0;
    fr_tick_1ms(&g_fr, ms++);
}

static void start_fr_timer_1khz(void) {
    const esp_timer_create_args_t a = { 
        .callback = fr_timer_cb, 
        .name = "fr1khz" 
    };
    ESP_ERROR_CHECK(esp_timer_create(&a, &s_fr_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_fr_timer, 1000));
}

// =============================================================================
// INIT FUNCTIONS
// =============================================================================

void fr_init_default(link_tx_t* tx) {
    // Laad parameters uit core0_config
    fr_params_t P = {
        .dvdt_min_q15   = CFG_FILT.dvdt_min_q15,
        .mono_min_q8    = CFG_FILT.mono_min_q8,
        .snr_min_q8     = CFG_FILT.snr_min_q8,
        .fit_err_max_q8 = CFG_FILT.fit_err_max_q8,
        .w_dvdt         = CFG_FILT.w_dvdt,
        .w_mono         = CFG_FILT.w_mono,
        .w_snr          = CFG_FILT.w_snr,
        .w_fit_pen      = CFG_FILT.w_fit_pen,
        .win_ms         = CFG_FILT.coalesce_win_ms,
        .target_evps    = CFG_FILT.target_evps,
        .util_hi_q      = CFG_FILT.util_hi_threshold,
        .l2_only_ms     = 200,  // Gereserveerd
    };
    
    fr_init(&g_fr, tx, &P);
    start_fr_timer_1khz();
}

void fr_init(fr_ctx_t* C, link_tx_t* ltx, const fr_params_t* P) {
    memset(C, 0, sizeof(*C));
    C->ltx = ltx;
    C->P = *P;
    C->tokens = 0.0f;
    C->tokens_per_ms = (float)P->target_evps / 1000.0f;
    C->last_tick_ms = 0;
}

// =============================================================================
// UTILIZATION FEEDBACK
// =============================================================================

void fr_on_utilization(fr_ctx_t* C, uint8_t util_q) {
    C->utilization_q = util_q;
    if (util_q > C->P.util_hi_q) {
        if (C->P.win_ms < CFG_FILT.coalesce_win_max_ms) {
            C->P.win_ms += 1;
        }
    }
}

// =============================================================================
// QUALITY COMPUTATION
// =============================================================================

static void compute_quality(const fr_params_t* P, const fr_candidate_t* X,
                           uint8_t* qlevel, uint8_t* score_q8)
{
    // Gate checks
    int accept = 1;
    if (X->dvdt_q15 < P->dvdt_min_q15) accept = 0;
    if (X->mono_q8 < P->mono_min_q8) accept = 0;
    if (X->snr_q8 < P->snr_min_q8) accept = 0;
    if (X->fit_err_q8 > P->fit_err_max_q8) accept = 0;
    
    if (!accept) {
        *qlevel = 0;
        *score_q8 = 0;
        return;
    }
    
    // Score berekening
    int score = 0;
    score += (int)P->w_dvdt * (int)clampu8((X->dvdt_q15 * 255) / (P->dvdt_min_q15 * 2 + 1));
    score += (int)P->w_mono * (int)X->mono_q8;
    score += (int)P->w_snr * (int)X->snr_q8;
    score -= (int)P->w_fit_pen * (int)X->fit_err_q8;
    
    if (score < 0) score = 0;
    if (score > 65535) score = 65535;
    *score_q8 = (uint8_t)(score >> 8);
    
    // Quality level classificatie (via config thresholds)
    int strong = (X->dvdt_q15 >= (int)(CFG_FILT.qlevel.dvdt_strong_mult * P->dvdt_min_q15)) &&
                 (X->mono_q8 >= CFG_FILT.qlevel.mono_strong_q8) &&
                 (X->snr_q8 >= (uint8_t)(P->snr_min_q8 + CFG_FILT.qlevel.snr_strong_add));
    
    int normal = (X->dvdt_q15 >= P->dvdt_min_q15) &&
                 (X->mono_q8 >= P->mono_min_q8) &&
                 (X->snr_q8 >= P->snr_min_q8);
    
    *qlevel = strong ? 3 : (normal ? 2 : 1);
}

// =============================================================================
// COALESCING & EMIT
// =============================================================================

static int consider_into_buf(fr_ctx_t* C, int s, const fr_candidate_t* X,
                            uint8_t qlevel, uint8_t score_q8)
{
    uint32_t t0 = C->buf[s].t_start_us;
    
    if (!C->buf[s].active) {
        C->buf[s].active = 1;
        C->buf[s].t_start_us = X->t_abs_us;
        C->buf[s].best_qlevel = qlevel;
        C->buf[s].best_score_q8 = score_q8;
        C->buf[s].best = *X;
        return 0;
    }
    
    uint32_t dt = (X->t_abs_us - t0);
    if (dt <= (uint32_t)(C->P.win_ms * 1000U)) {
        // Binnen window: kies beste
        if (qlevel > C->buf[s].best_qlevel ||
            (qlevel == C->buf[s].best_qlevel && score_q8 >= C->buf[s].best_score_q8)) {
            C->buf[s].best_qlevel = qlevel;
            C->buf[s].best_score_q8 = score_q8;
            C->buf[s].best = *X;
        }
    } else {
        // Window klaar: emit beste
        fr_candidate_t B = C->buf[s].best;
        uint8_t qB = C->buf[s].best_qlevel;
        uint8_t scB = C->buf[s].best_score_q8;
        
        // === BACKPRESSURE ===
        int emit_ok = 1;
        
        if (C->tokens < CFG_FILT.drop_threshold.weak) {
            if (qB == 1) emit_ok = 0;
            else if (qB == 2 && C->tokens < CFG_FILT.drop_threshold.normal) emit_ok = 0;
            else if (qB == 3 && C->tokens < CFG_FILT.drop_threshold.strong) emit_ok = 0;
        }
        
        if (emit_ok) {
            // Token cost
            float cost = CFG_FILT.token_cost.normal;
            if (qB == 3) cost = CFG_FILT.token_cost.strong;
            else if (qB == 1) cost = CFG_FILT.token_cost.weak;
            
            C->tokens -= cost;
            if (C->tokens < CFG_FILT.token_floor) {
                C->tokens = CFG_FILT.token_floor;
            }
            
            g_rate_stats.emitted++;
            if (qB == 3) g_rate_stats.emitted_strong++;
            else if (qB == 2) g_rate_stats.emitted_normal++;
            else if (qB == 1) g_rate_stats.emitted_weak++;
            
            // Emit
            uint16_t dt_us = (uint16_t)(B.t_abs_us - C->t_abs_us_prev);
            C->t_abs_us_prev = B.t_abs_us;
            
            uint8_t f0 = make_flags0(B.pair_flag, qB, B.polarity, B.sensor);
            uint8_t f1 = make_flags1(B.from_pool, B.to_pool, B.dir_hint, B.edge_kind);
            
            emit_event24(C->ltx, dt_us, B.t_abs_us, f0, f1,
                        B.dvdt_q15, B.mono_q8, B.snr_q8, B.fit_err_q8,
                        0, scB, C->seq++);
        } else {
            // Drop statistics
            if (qB == 1) g_rate_stats.dropped_weak++;
            else if (qB == 2) g_rate_stats.dropped_norm++;
            else if (qB == 3) g_rate_stats.dropped_strong++;
        }
        
        // Reset voor nieuw window
        C->buf[s].active = 1;
        C->buf[s].t_start_us = X->t_abs_us;
        C->buf[s].best_qlevel = qlevel;
        C->buf[s].best_score_q8 = score_q8;
        C->buf[s].best = *X;
    }
    
    return 1;
}

// =============================================================================
// MAIN API: CONSIDER
// =============================================================================

int fr_consider(fr_ctx_t* C, const fr_candidate_t* X) {
    g_rate_stats.considered_total++;
    
    uint8_t qlevel = 0, score_q8 = 0;
    compute_quality(&C->P, X, &qlevel, &score_q8);
    
    // Debug bypass mode (via runmode)
    if (IS_DEBUG_BYPASS) {
        static uint32_t last_t = 0;
        uint16_t dt = (X->t_abs_us - last_t) <= 0xFFFF ? (uint16_t)(X->t_abs_us - last_t) : 1;
        last_t = X->t_abs_us;
        
        uint8_t dbg_flags0 = make_flags0(X->pair_flag, qlevel, X->polarity, X->sensor);
        uint8_t dbg_flags1 = make_flags1(X->from_pool, X->to_pool, X->dir_hint, X->edge_kind);
        
        emit_event24(C->ltx, dt, X->t_abs_us,
                    dbg_flags0, dbg_flags1,
                    (uint16_t)(X->dvdt_q15),
                    X->mono_q8, X->snr_q8, X->fit_err_q8,
                    0, score_q8, C->seq++);
        return 0;
    }
    
    // Normal mode: quality gate
    if (qlevel == 0) {
        g_rate_stats.rejected_quality++;
        return 0;
    }
    
    int s = X->sensor ? 1 : 0;
    return consider_into_buf(C, s, X, qlevel, score_q8);
}

// =============================================================================
// TICK (1ms)
// =============================================================================

void fr_tick_1ms(fr_ctx_t* C, uint32_t now_ms) {
    // Token refill
    if (C->last_tick_ms == 0) {
        C->last_tick_ms = now_ms;
        g_rate_stats.last_report_ms = now_ms;
    }
    
    uint32_t dt = now_ms - C->last_tick_ms;
    if (dt) {
        if (dt > 2) dt = 2;  // Cap burst
        C->tokens += C->tokens_per_ms * (float)dt;
        
        float max_tokens = C->tokens_per_ms * CFG_FILT.token_max_ms;
        if (C->tokens > max_tokens) C->tokens = max_tokens;
        
        C->last_tick_ms = now_ms;
    }
    
    // Flush stale windows
    for (int s = 0; s < 2; ++s) {
        if (!C->buf[s].active) continue;
        
        uint32_t age_us = (C->t_abs_us_prev > C->buf[s].t_start_us) ?
                          (C->t_abs_us_prev - C->buf[s].t_start_us) : 0;
        
        if (age_us > (uint32_t)(C->P.win_ms * CFG_FILT.stale_window_mult * 1000U)) {
            fr_candidate_t B = C->buf[s].best;
            uint8_t qB = C->buf[s].best_qlevel;
            uint8_t scB = C->buf[s].best_score_q8;
            
            int emit_ok = (qB >= 2);
            if (emit_ok && C->tokens < CFG_FILT.drop_threshold.normal) {
                if (qB == 2) emit_ok = 0;
            }
            
            if (emit_ok) {
                float cost = (qB == 3) ? CFG_FILT.token_cost.strong : CFG_FILT.token_cost.normal;
                C->tokens -= cost;
                if (C->tokens < 0.0f) C->tokens = 0.0f;
                
                g_rate_stats.emitted++;
                if (qB == 3) g_rate_stats.emitted_strong++;
                else if (qB == 2) g_rate_stats.emitted_normal++;
                else if (qB == 1) g_rate_stats.emitted_weak++;
                
                uint16_t dt_us = (uint16_t)(B.t_abs_us - C->t_abs_us_prev);
                C->t_abs_us_prev = B.t_abs_us;
                
                uint8_t f0 = make_flags0(B.pair_flag, qB, B.polarity, B.sensor);
                uint8_t f1 = make_flags1(B.from_pool, B.to_pool, B.dir_hint, B.edge_kind);
                
                emit_event24(C->ltx, dt_us, B.t_abs_us, f0, f1,
                            B.dvdt_q15, B.mono_q8, B.snr_q8, B.fit_err_q8,
                            0, scB, C->seq++);
            } else {
                if (qB == 2) g_rate_stats.dropped_norm++;
            }
            
            C->buf[s].active = 0;
        }
    }
    
    // Filter stats reporting
    if (now_ms - g_rate_stats.last_report_ms >= CFG_LINK.filter_stats_interval_ms) {
        uint16_t window = (uint16_t)(now_ms - g_rate_stats.last_report_ms);
        uint16_t total_dropped = g_rate_stats.dropped_weak +
                                 g_rate_stats.dropped_norm +
                                 g_rate_stats.dropped_strong;
        
        uint8_t pct_strong = 0, pct_normal = 0, pct_weak = 0;
        if (g_rate_stats.emitted > 0) {
            pct_strong = (uint8_t)((g_rate_stats.emitted_strong * 255) / g_rate_stats.emitted);
            pct_normal = (uint8_t)((g_rate_stats.emitted_normal * 255) / g_rate_stats.emitted);
            pct_weak = (uint8_t)((g_rate_stats.emitted_weak * 255) / g_rate_stats.emitted);
        }
        
        if (CFG_LINK.use_filter_stats) {
            emit_filter_stats(C->ltx,
                window,
                (uint16_t)g_rate_stats.emitted,
                total_dropped,
                g_rate_stats.considered_total,
                g_rate_stats.rejected_quality,
                pct_strong,
                pct_normal,
                pct_weak,
                (uint8_t)g_rate_stats.dropped_strong,
                (uint8_t)g_rate_stats.dropped_norm,
                (uint8_t)g_rate_stats.dropped_weak,
                C->tokens,
                (uint8_t)C->P.win_ms);
        }
        
        // Reset stats
        g_rate_stats.emitted = 0;
        g_rate_stats.emitted_weak = 0;
        g_rate_stats.emitted_normal = 0;
        g_rate_stats.emitted_strong = 0;
        g_rate_stats.dropped_weak = 0;
        g_rate_stats.dropped_norm = 0;
        g_rate_stats.dropped_strong = 0;
        g_rate_stats.considered_total = 0;
        g_rate_stats.rejected_quality = 0;
        g_rate_stats.last_report_ms = now_ms;
    }
}

// =============================================================================
// GLUE/LAB API
// =============================================================================

void fr_inject_sample(fr_ctx_t* ctx, int16_t raw_a, int16_t raw_b, uint32_t t_abs_us) {
    (void)ctx; (void)raw_a; (void)raw_b; (void)t_abs_us;
    // Placeholder voor raw sampling integratie
}

bool fr_poll_event(fr_ctx_t* ctx, fr_event_t* out) {
    (void)ctx;
    return outbox_pop(out);
}

void fr_update_considered_count(fr_ctx_t* C, uint32_t count) {
    (void)C;
    g_rate_stats.considered_total += count;
}

// core0_filtered.h - Filter layer (V1.1 - Config Integrated)
// =============================================================================
// MIGRATED: Nu gebruikt core0_config.h voor alle filter parameters
// =============================================================================

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "core0_link.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// POOL & DIRECTION ENUMS
// =============================================================================

typedef enum { 
    POOL_NEU = 0, 
    POOL_N   = 1, 
    POOL_S   = 2, 
    POOL_UNK = 3 
} pool_t;

typedef enum { 
    DIR_NONE = 0, 
    DIR_CW   = 1, 
    DIR_CCW  = 2 
} dirhint_t;

typedef enum {
    EDGE_KIND_ZC        = 0,  // baseline zero-cross
    EDGE_KIND_POOL_HARD = 1,  // directe N<->S pool flip
    EDGE_KIND_POOL_NEUT = 2   // pool <-> NEUTRAL flank
} edgekind_t;

// =============================================================================
// FILTER PARAMETERS (runtime kopie van config)
// =============================================================================

typedef struct {
    // Drempels
    int16_t  dvdt_min_q15;
    uint8_t  mono_min_q8;
    uint8_t  snr_min_q8;
    uint8_t  fit_err_max_q8;
    
    // Scoring gewichten
    uint8_t  w_dvdt;
    uint8_t  w_mono;
    uint8_t  w_snr;
    uint8_t  w_fit_pen;
    
    // Coalescing
    uint16_t win_ms;
    
    // Backpressure
    uint16_t target_evps;
    uint8_t  util_hi_q;
    uint8_t  l2_only_ms;
} fr_params_t;

// =============================================================================
// CANDIDATE STRUCTURE (input naar filter)
// =============================================================================

typedef struct {
    uint32_t  t_abs_us;
    int16_t   dvdt_q15;
    uint8_t   mono_q8;
    uint8_t   snr_q8;
    uint8_t   fit_err_q8;
    uint8_t   sensor;      // 0=A, 1=B
    uint8_t   polarity;    // 0=−, 1=+
    pool_t    from_pool;
    pool_t    to_pool;
    uint8_t   pair_flag;
    dirhint_t dir_hint;
    uint8_t   edge_kind;
} fr_candidate_t;

// =============================================================================
// FILTER CONTEXT
// =============================================================================

typedef struct fr_ctx {
    link_tx_t*   ltx;
    fr_params_t  P;
    
    // Coalescing buffers per sensor
    struct {
        uint8_t  active;
        uint32_t t_start_us;
        uint8_t  best_qlevel;
        uint8_t  best_score_q8;
        fr_candidate_t best;
    } buf[2];
    
    // Backpressure
    float    tokens;
    float    tokens_per_ms;
    uint32_t last_tick_ms;
    uint8_t  utilization_q;
    
    // Sequence counter
    uint8_t  seq;
    
    // Last emit timestamp
    uint32_t t_abs_us_prev;
} fr_ctx_t;

// =============================================================================
// OUTPUT EVENT (voor polling)
// =============================================================================

typedef struct {
    uint16_t dt_us;
    uint32_t t_abs_us;
    uint16_t dvdt_q15;
    uint8_t  mono_q8;
    uint8_t  snr_q8;
    uint8_t  fit_err_q8;
    uint16_t rpm_hint_q;
    uint8_t  flags0;
    uint8_t  flags1;
} fr_event_t;

// =============================================================================
// GLOBAL CONTEXT
// =============================================================================

extern fr_ctx_t g_fr;

// =============================================================================
// API FUNCTIONS
// =============================================================================

// Initialisatie (laadt parameters uit core0_config)
void fr_init_default(link_tx_t* tx);
void fr_init(fr_ctx_t* C, link_tx_t* ltx, const fr_params_t* P);

// Utilization feedback
void fr_on_utilization(fr_ctx_t* C, uint8_t util_q);

// Hoofd API: bied kandidaat aan
int fr_consider(fr_ctx_t* C, const fr_candidate_t* cand);

// Periodieke tick (1ms)
void fr_tick_1ms(fr_ctx_t* C, uint32_t now_ms);

// Lab/glue API
void fr_inject_sample(fr_ctx_t* ctx, int16_t raw_a, int16_t raw_b, uint32_t t_abs_us);
bool fr_poll_event(fr_ctx_t* ctx, fr_event_t* out);

// Stats update helper
void fr_update_considered_count(fr_ctx_t* C, uint32_t count);

#ifdef __cplusplus
}
#endif

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "core0_link.h"  // voor emit_event24 en flags helpers

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { POOL_NEU=0, POOL_N=1, POOL_S=2, POOL_UNK=3 } pool_t;
typedef enum { DIR_NONE=0, DIR_CW=1, DIR_CCW=2 } dirhint_t;

// Forward decl van jouw filter state
typedef struct fr_state fr_state_t;

typedef struct {
  // drempels
  int16_t  dvdt_min_q15;     // bv. 800
  uint8_t  mono_min_q8;      // bv. 153 (=0.6*255)
  uint8_t  snr_min_q8;       // bv. 24  (~12 dB als 0.5 dB/LSB)
  uint8_t  fit_err_max_q8;   // bv. 40

  // scoring gewichten (0..255; som mag >255 zijn)
  uint8_t  w_dvdt;
  uint8_t  w_mono;
  uint8_t  w_snr;
  uint8_t  w_fit_pen;        // wordt afgetrokken

  // coalescing
  uint16_t win_ms;           // bv. 2..4 ms

  // backpressure
  uint16_t target_evps;      // bv. 400 events/s (A+B)
  uint8_t  util_hi_q;        // bv. 217 ≈ 85% (0..255 → 0..100%)
  uint8_t  l2_only_ms;       // duw zwakke events (qlevel==1) tijdelijk weg
} fr_params_t;

typedef struct {
  // tijd
  uint32_t  t_abs_us;        // absolute µs (rolling)
  // ruwe meetwaarden
  int16_t   dvdt_q15;
  uint8_t   mono_q8, snr_q8, fit_err_q8;
  // labels
  uint8_t   sensor;          // 0=A, 1=B
  uint8_t   polarity;        // 0=−, 1=+
  pool_t    from_pool, to_pool;
  uint8_t   pair_flag;       // 0/1
  dirhint_t dir_hint;        // 0..2
} fr_candidate_t;

typedef struct fr_ctx {
  link_tx_t*    ltx;             // TX-link (naar UART-writer)
  fr_params_t   P;

  // coalescing buffers per sensor
  struct {
    uint8_t      active;         // 0=leeg, 1=actief
    uint32_t     t_start_us;     // vensterstart
    uint8_t      best_qlevel;    // 1..3
    uint8_t      best_score_q8;  // 0..255
    fr_candidate_t best;         // volledige beste kandidaat
  } buf[2];

  // backpressure tokens
  float tokens;                  // token bucket
  float tokens_per_ms;           // target_evps/1000
  uint32_t last_tick_ms;         // voor token refill

  // simple utilization hint (optioneel door writer geüpdatet)
  uint8_t  utilization_q;        // 0..255

  // seq teller
  uint8_t seq;

  /// laatst uitgezonden tijdstip
  uint32_t t_abs_us_prev;
} fr_ctx_t;

// Klein, neutraal eventtype om uit de filter te trekken
typedef struct {
  uint16_t dt_us;
  uint32_t t_abs_us;
  uint16_t dvdt_q15;
  uint8_t  mono_q8, snr_q8, fit_err_q8;
  uint16_t rpm_hint_q;
  uint8_t  flags0, flags1;
} fr_event_t;

extern fr_ctx_t g_fr;

// init & updates
void fr_init(fr_ctx_t* C, link_tx_t* ltx, const fr_params_t* P);
void fr_on_utilization(fr_ctx_t* C, uint8_t util_q); // optioneel: vanuit summary

// hoofd-API: bied kandidaat aan (Filtered-RAW doet rest)
int fr_consider(fr_ctx_t* C, const fr_candidate_t* cand);

// periodieke tick (elke ~1 ms aanroepen): flush vensters en onderhoud tokens
void fr_tick_1ms(fr_ctx_t* C, uint32_t now_ms);

// Glue-API voor lab: raw -> filter en events <- filter
// (implementeer in core0_filtered.c, zie patch 3)
void fr_inject_sample(fr_ctx_t* ctx, int16_t raw_a, int16_t raw_b, uint32_t t_abs_us);
bool fr_poll_event  (fr_ctx_t*  ctx, fr_event_t* out);

#ifdef __cplusplus
}
#endif


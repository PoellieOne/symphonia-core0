#include "core0_filtered.h"
#include <string.h>

fr_ctx_t g_fr;   // ← daadwerkelijke definitie (extern define in core0_filtered.h)

static inline uint8_t clampu8(int v){ if(v<0) return 0; if(v>255) return 255; return (uint8_t)v; }

// Eenvoudige kleine outbox-queue voor lab (ringbuffer)
#ifndef FR_OUTBOX_CAP
#define FR_OUTBOX_CAP 64
#endif

#define DBG_BYPASS_EMIT 0

typedef struct {
  fr_event_t buf[FR_OUTBOX_CAP];
  uint16_t r, w;
} fr_outbox_t;

static fr_outbox_t s_outbox = {0};

// Helper: push/pop
static inline bool outbox_push(const fr_event_t* e){
  uint16_t n = (s_outbox.w + 1) % FR_OUTBOX_CAP;
  if (n == s_outbox.r) return false; // vol
  s_outbox.buf[s_outbox.w] = *e;
  s_outbox.w = n;
  return true;
}
static inline bool outbox_pop(fr_event_t* e){
  if (s_outbox.r == s_outbox.w) return false;
  *e = s_outbox.buf[s_outbox.r];
  s_outbox.r = (s_outbox.r + 1) % FR_OUTBOX_CAP;
  return true;
}

// === GLUE: samples in, events eruit ===
// Deze functie roept jouw interne consider/ingest aan.
// Pas aan naar jouw echte ingest functie/signatuur.
void fr_inject_sample(fr_ctx_t* ctx, int16_t raw_a, int16_t raw_b, uint32_t t_abs_us){
   // voorbeeld: als jij fr_consider(ctx, raw_a, raw_b, t_abs_us) hebt:
   // fr_consider(ctx, raw_a, raw_b, t_abs_us);
   (void)ctx; (void)raw_a; (void)raw_b; (void)t_abs_us;

  // ---- LAB PLACEHOLDER ----
  // Als je ingest-functie nog niet hebt, laat dit tijdelijk leeg
  // (de feeder stuurt dan nog geen echte filter-events).
  (void)ctx; (void)raw_a; (void)raw_b; (void)t_abs_us;
}

// Deze functie wordt door jouw detectielogica aangeroepen wanneer een event wordt gevonden.
// Roep dit in jouw bestaande code op het detectiepunt aan (1 regel).
static inline void fr_emit_from_filter(uint16_t dt_us, uint32_t t_abs_us,
                                       uint8_t flags0, uint8_t flags1,
                                       uint16_t dvdt_q15, uint8_t mono_q8, uint8_t snr_q8,
                                       uint8_t fit_err_q8, uint16_t rpm_hint_q, uint8_t score_q8)
{
  fr_event_t e = {
    .dt_us = dt_us, .t_abs_us = t_abs_us,
    .dvdt_q15 = dvdt_q15, .mono_q8 = mono_q8, .snr_q8 = snr_q8,
    .fit_err_q8 = fit_err_q8, .rpm_hint_q = rpm_hint_q,
    .flags0 = flags0, .flags1 = flags1
  };
  (void)score_q8; // als je score hier niet nodig hebt
  outbox_push(&e);
}

// Door de feeder/pipeline gepolld om events op te halen
bool fr_poll_event(fr_ctx_t* ctx, fr_event_t* out){
  (void)ctx;
  return outbox_pop(out);
}

// --- Quality & score
static void compute_quality(const fr_params_t* P, const fr_candidate_t* X,
                            uint8_t* qlevel, uint8_t* score_q8)
{
  // gate-trio
  int accept = 1;
  if (X->dvdt_q15 < P->dvdt_min_q15) accept = 0;
  if (X->mono_q8   < P->mono_min_q8) accept = 0;
  if (X->snr_q8    < P->snr_min_q8)  accept = 0;
  if (X->fit_err_q8> P->fit_err_max_q8) accept = 0;

  if (!accept){ *qlevel = 0; *score_q8 = 0; return; }

  // eenvoudige score
  int score = 0;
  score += (int)P->w_dvdt    * (int)clampu8((X->dvdt_q15 * 255) / (P->dvdt_min_q15*2 + 1)); // ruw genormeerd
  score += (int)P->w_mono    * (int)X->mono_q8;
  score += (int)P->w_snr     * (int)X->snr_q8;
  score -= (int)P->w_fit_pen * (int)X->fit_err_q8;

  if (score < 0) score = 0;
  if (score > 65535) score = 65535;
  *score_q8 = (uint8_t)(score >> 8); // 16b → 8b

  // qlevel: simpel
  // strong: ruim boven minima → dvdt >= 1.5×, mono >= 0.8*255, snr >= (min+12)
  int strong = (X->dvdt_q15 >= (int)(1.5f * P->dvdt_min_q15)) &&
               (X->mono_q8  >= (uint8_t)(0.8f * 255)) &&
               (X->snr_q8   >= (uint8_t)(P->snr_min_q8 + 24));

  int normal = (X->dvdt_q15 >= P->dvdt_min_q15) &&
               (X->mono_q8  >= P->mono_min_q8) &&
               (X->snr_q8   >= P->snr_min_q8);

  *qlevel = strong ? 3 : (normal ? 2 : 1);
}

// --- Coalescing helper
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
  // binnen venster?
  uint32_t dt = (X->t_abs_us - t0);
  if (dt <= (uint32_t)(C->P.win_ms * 1000U)) {
    // kies beste op qlevel > score > recent
    if (qlevel > C->buf[s].best_qlevel ||
        (qlevel == C->buf[s].best_qlevel && score_q8 >= C->buf[s].best_score_q8))
    {
      C->buf[s].best_qlevel = qlevel;
      C->buf[s].best_score_q8 = score_q8;
      C->buf[s].best = *X;
    }
  } else {
    // venster klaar → emit beste, en start nieuw venster met X
    fr_candidate_t B = C->buf[s].best;
    uint8_t qB = C->buf[s].best_qlevel;
    uint8_t scB = C->buf[s].best_score_q8;

    // backpressure: token bucket
    int emit_ok = 1;
    if (C->tokens < 1.0f) {
      // te druk: laat level-1 vallen
      if (qB <= 1) emit_ok = 0;
    } else {
      C->tokens -= 1.0f;
    }
    // TIJDELIJKE OPLOSSING: Backpressure uitschakelen (voor test)
    // C->tokens is altijd hoog genoeg voor de test.
    //C->tokens = 1000.0f; // Voorkom dat C->tokens < 1.0f is
    //C->tokens -= 1.0f;   // Simuleer verbruik (of verwijder dit)
    // einde tijdelijke oplossing

    if (emit_ok) {
      uint16_t dt_us = (uint16_t)(B.t_abs_us - C->t_abs_us_prev);
      C->t_abs_us_prev = B.t_abs_us;

      uint8_t f0 = make_flags0(B.pair_flag, qB, B.polarity, B.sensor);
      uint8_t f1 = make_flags1(B.from_pool, B.to_pool, B.dir_hint);

      emit_event24(C->ltx, dt_us, B.t_abs_us, f0, f1,
                   B.dvdt_q15, B.mono_q8, B.snr_q8, B.fit_err_q8,
                   /*rpm_hint_q*/ 0, scB, C->seq++);
    }

    // reset/start nieuw
    C->buf[s].active = 1;
    C->buf[s].t_start_us = X->t_abs_us;
    C->buf[s].best_qlevel = qlevel;
    C->buf[s].best_score_q8 = score_q8;
    C->buf[s].best = *X;
  }
  return 1;
}

// --- API
void fr_init(fr_ctx_t* C, link_tx_t* ltx, const fr_params_t* P) {
  memset(C, 0, sizeof(*C));
  C->ltx = ltx;
  C->P = *P;
  C->tokens = 0.0f;
  C->tokens_per_ms = (float)P->target_evps / 1000.0f;
  C->last_tick_ms = 0;
  // buffers leeg
}

void fr_on_utilization(fr_ctx_t* C, uint8_t util_q) {
  C->utilization_q = util_q;
  // optioneel: bij hoge util vergroten coalescing window licht
  if (util_q > C->P.util_hi_q) {
    if (C->P.win_ms < 6) C->P.win_ms += 1;
  }
}

int fr_consider(fr_ctx_t* C, const fr_candidate_t* X) {
  uint8_t qlevel=0, score_q8=0;
  compute_quality(&C->P, X, &qlevel, &score_q8);

  #ifdef DBG_BYPASS_EMIT
    // Emit ALTIJD, ook bij qlevel==0, en encodeer qlevel/score in de payload voor inspectie
    static uint32_t last_t = 0;
    uint16_t dt = (X->t_abs_us - last_t) <= 0xFFFF ? (uint16_t)(X->t_abs_us - last_t) : 1;
    last_t = X->t_abs_us;

    // NB: we mappen qlevel/score grof in de velden zodat je ze in capture kunt zien
    emit_event24(C->ltx, dt, X->t_abs_us,
                /*flags0*/ (uint8_t)(0x40 | (qlevel & 0x07)),   // 3 bits qlevel
                /*flags1*/ (uint8_t)(0x10 | ((score_q8>>5)&0x07)), // top3 bits score
                /*dvdt*/   (uint16_t)(X->dvdt_q15),
                /*mono*/   (uint8_t)(X->mono_q8),
                /*snr*/    (uint8_t)(X->snr_q8),
                /*fit*/    (uint8_t)(X->fit_err_q8),
                /*rpm*/    0x1388,
                /*score*/  score_q8,
                /*seq*/    0);
  #endif
  if (qlevel == 0) return 0; // reject-pad zichtbaar via bypass hierboven

  int s = X->sensor ? 1 : 0;
  return consider_into_buf(C, s, X, qlevel, score_q8);
}

void fr_tick_1ms(fr_ctx_t* C, uint32_t now_ms) {
  // refill tokens
  if (C->last_tick_ms == 0) C->last_tick_ms = now_ms;
  uint32_t dt = now_ms - C->last_tick_ms;
  if (dt) {
    C->tokens += C->tokens_per_ms * (float)dt;
    if (C->tokens > 3.0f * C->tokens_per_ms * 100.0f) // limiet
      C->tokens = 3.0f * C->tokens_per_ms * 100.0f;
    C->last_tick_ms = now_ms;
  }

  // vensters die “te lang open staan” toch flushen
  for (int s=0; s<2; ++s) {
    if (!C->buf[s].active) continue;
    uint32_t age_us = (C->t_abs_us_prev > C->buf[s].t_start_us) ?
                      (C->t_abs_us_prev - C->buf[s].t_start_us) :
                      0;
    if (age_us > (uint32_t)(C->P.win_ms * 2000U)) {
      // forceer emit om niet eindeloos te wachten
      fr_candidate_t B = C->buf[s].best;
      uint8_t qB = C->buf[s].best_qlevel;
      uint8_t scB = C->buf[s].best_score_q8;

      int emit_ok = (qB >= 2); // alleen normal/strong
      if (emit_ok) {
        uint16_t dt_us = (uint16_t)(B.t_abs_us - C->t_abs_us_prev);
        C->t_abs_us_prev = B.t_abs_us;
        uint8_t f0 = make_flags0(B.pair_flag, qB, B.polarity, B.sensor);
        uint8_t f1 = make_flags1(B.from_pool, B.to_pool, B.dir_hint);
        emit_event24(C->ltx, dt_us, B.t_abs_us, f0, f1,
                     B.dvdt_q15, B.mono_q8, B.snr_q8, B.fit_err_q8,
                     0, scB, C->seq++);
      }
      C->buf[s].active = 0;
    }
  }
}


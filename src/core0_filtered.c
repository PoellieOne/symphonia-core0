#include "core0_filtered.h"
#include <string.h>
#include "esp_timer.h"

fr_ctx_t g_fr;

static esp_timer_handle_t s_fr_timer;

static inline uint8_t clampu8(int v){ if(v<0) return 0; if(v>255) return 255; return (uint8_t)v; }

// Outbox queue voor lab
#ifndef FR_OUTBOX_CAP
#define FR_OUTBOX_CAP 64
#endif

#define DBG_BYPASS_EMIT 0
#define USE_FILTER_STATS 1  // 1 = gebruik nieuwe PKT_FILTER_STATS, 0 = legacy

typedef struct {
  fr_event_t buf[FR_OUTBOX_CAP];
  uint16_t r, w;
} fr_outbox_t;

static fr_outbox_t s_outbox = {0};

typedef struct {
  uint32_t emitted;
  uint32_t emitted_weak;
  uint32_t emitted_normal;
  uint32_t emitted_strong;
  uint32_t dropped_weak;
  uint32_t dropped_norm;
  uint32_t dropped_strong;
  uint32_t considered_total;
  uint32_t rejected_quality;   // NEW: rejected by quality gates
  uint32_t last_report_ms;
} rate_stats_t;

static rate_stats_t g_rate_stats = {0};

void fr_update_considered_count(fr_ctx_t* C, uint32_t count) {
  g_rate_stats.considered_total += count;
}

// Helpers

static void fr_timer_cb(void* arg) {
  static uint32_t ms = 0;
  fr_tick_1ms(&g_fr, ms++);
}

static void start_fr_timer_1khz(void) {
    const esp_timer_create_args_t a = { .callback = fr_timer_cb, .name = "fr1khz" };
    ESP_ERROR_CHECK( esp_timer_create(&a, &s_fr_timer) );
    ESP_ERROR_CHECK( esp_timer_start_periodic(s_fr_timer, 1000) );
}

void fr_init_default(link_tx_t* tx)
{
    // Filtered-RAW default parameters (veilig startprofiel; later tunen)
    fr_params_t P = {
        .dvdt_min_q15   = 15,             // drempel dv/dt (fixed-point), was 50
        .mono_min_q8    = 105,            // ≈0.6, was 153
        .snr_min_q8     = 24,             // ≈12 dB (als 0.5 dB/LSB), was 24
        .fit_err_max_q8 = 120,            // was 40

        .w_dvdt=5, .w_mono=4, .w_snr=4, .w_fit_pen=3,

        .win_ms=3,                        // coalescing window 3 ms

        .target_evps=150,                 // events per seconde target
        .util_hi_q=200,                   // ~85% utilization (0..255 → 0..100%)
        .l2_only_ms=200                   // (gereserveerd voor toekomstige “weak drop”)
    };

    fr_init(&g_fr, tx, &P);
    start_fr_timer_1khz();
}

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
   // TODO: Implementeer wanneer raw sampling klaar is
   // Deze functie moet:
   // 1. Edge detection op raw_a en raw_b
   // 2. Quality metrics berekenen (dvdt, mono, snr, fit)
   // 3. fr_consider() aanroepen met candidate

   // HUIDIGE STATUS: Placeholder voor toekomstige real-sensor integratie
   (void)ctx; (void)raw_a; (void)raw_b; (void)t_abs_us;
}

// Deze functie wordt door jouw detectielogica aangeroepen wanneer een event wordt gevonden.
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

    // === GRADUATED BACKPRESSURE ===
    int emit_ok = 1;

    // Check of we genoeg tokens hebben
    if (C->tokens < 1.0f) {
        // Graduele dropping op basis van token debt
        if (qB == 1) {
            // Weak: drop altijd bij tokens < 1
            emit_ok = 0;
        } else if (qB == 2) {
            // Normal: drop bij tokens < 0.5
            if (C->tokens < 0.5f) emit_ok = 0;
        } else if (qB == 3) {
            // Strong: drop alleen bij extreme overload (tokens < -2)
            if (C->tokens < -2.0f) emit_ok = 0;
        }
    }

    // Als we gaan emitteren: verbruik een token
    if (emit_ok) {
      // === GRADUATED TOKEN CONSUMPTION ===
      float cost = 1.0f;
      if (qB == 3) {
        cost = 0.5f;  // Strong events kosten minder
      } else if (qB == 1) {
        cost = 1.5f;  // Weak events kosten meer
      }

      C->tokens -= cost;

      // Harder token limit
      if (C->tokens < -2.0f) C->tokens = -2.0f;

      g_rate_stats.emitted++;

      if (qB == 3) {
        g_rate_stats.emitted_strong++;
      } else if (qB == 2) {
        g_rate_stats.emitted_normal++;
      } else if (qB == 1) {
        g_rate_stats.emitted_weak++;
      }

      // Emit het event
      uint16_t dt_us = (uint16_t)(B.t_abs_us - C->t_abs_us_prev);
      C->t_abs_us_prev = B.t_abs_us;

      uint8_t f0 = make_flags0(B.pair_flag, qB, B.polarity, B.sensor);
      uint8_t f1 = make_flags1(B.from_pool, B.to_pool, B.dir_hint, B.edge_kind);

      emit_event24(C->ltx, dt_us, B.t_abs_us, f0, f1,
                   B.dvdt_q15, B.mono_q8, B.snr_q8, B.fit_err_q8,
                   /*rpm_hint_q*/ 0, scB, C->seq++);
    } else {
        // Tel drops per quality level
        if (qB == 1) {
            g_rate_stats.dropped_weak++;
        } else if (qB == 2) {
            g_rate_stats.dropped_norm++;
        } else if (qB == 3) {
            g_rate_stats.dropped_strong++;
        }
    }
    // ============= EINDE BACKPRESSURE =============

   // reset/start nieuw venster met X
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
  // Tel elke consider call
  g_rate_stats.considered_total++;

  uint8_t qlevel=0, score_q8=0;
  compute_quality(&C->P, X, &qlevel, &score_q8);

  #if DBG_BYPASS_EMIT
    // Emit ALTIJD, ook bij qlevel==0, en encodeer qlevel/score in de payload voor inspectie
    static uint32_t last_t = 0;
    uint16_t dt = (X->t_abs_us - last_t) <= 0xFFFF ? (uint16_t)(X->t_abs_us - last_t) : 1;
    last_t = X->t_abs_us;

    // Correcte flags0 met sensor bit!
    // flags0 format: [7:pair][6:5:qlevel][4:polarity][3:sensor][2:0:reserved]
    uint8_t dbg_flags0 = 0;
    dbg_flags0 |= (0 << 7);                    // pair_flag = 0 voor debug
    dbg_flags0 |= ((qlevel & 0x03) << 5);      // qlevel (2 bits)
    dbg_flags0 |= ((X->polarity & 0x01) << 4); // polarity
    dbg_flags0 |= ((X->sensor & 0x01) << 3);   // SENSOR BIT - was missing!

  // flags1 met edge_kind en pools
  uint8_t dbg_flags1 = make_flags1(X->from_pool, X->to_pool, X->dir_hint, X->edge_kind);

    // NB: we mappen qlevel/score grof in de velden zodat je ze in capture kunt zien
    emit_event24(C->ltx, dt, X->t_abs_us,
                /*flags0*/ dbg_flags0,
                /*flags1*/ dbg_flags1,
                /*dvdt*/   (uint16_t)(X->dvdt_q15),
                /*mono*/   (uint8_t)(X->mono_q8),
                /*snr*/    (uint8_t)(X->snr_q8),
                /*fit*/    (uint8_t)(X->fit_err_q8),
                /*rpm*/    0,
                /*score*/  score_q8,
                /*seq*/    C->seq++);

    return 0;  // <-- Stop hier, geen normale processing
  #endif

  if (qlevel == 0) {
    g_rate_stats.rejected_quality++;
    return 0;
  }

  int s = X->sensor ? 1 : 0;
  return consider_into_buf(C, s, X, qlevel, score_q8);
}

void fr_tick_1ms(fr_ctx_t* C, uint32_t now_ms) {
  // Refill tokens
  if (C->last_tick_ms == 0) {
    C->last_tick_ms = now_ms;
    g_rate_stats.last_report_ms = now_ms;
  }

  uint32_t dt = now_ms - C->last_tick_ms;
  if (dt) {
    // === STRIKTE TOKEN REFILL ===
    // Cap dt op max 2ms om burst accumulation te voorkomen
    if (dt > 2) dt = 2;

    C->tokens += C->tokens_per_ms * (float)dt;

    // === STRAKKER TOKEN LIMIET ===
    // Verlaag max tokens van 100ms naar 50ms
    float max_tokens = C->tokens_per_ms * 50.0f;  // was 100.0f
    if (C->tokens > max_tokens) C->tokens = max_tokens;
    // === EINDE AANPASSING ===

    C->last_tick_ms = now_ms;
  }

  // vensters die "te lang open staan" toch flushen
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

      // Check backpressure ook hier
      int emit_ok = (qB >= 2);  // alleen normal/strong
      if (emit_ok && C->tokens < 1.0f) {
        if (qB == 2) emit_ok = 0;  // drop normal bij te weinig tokens
      }

      if (emit_ok) {
        float cost = (qB == 3) ? 0.5f : 1.0f;
        C->tokens -= cost;
        if (C->tokens < 0.0f) C->tokens = 0.0f;
        g_rate_stats.emitted++;

        // Tel hier OOK quality level
        if (qB == 3) {
            g_rate_stats.emitted_strong++;
        } else if (qB == 2) {
            g_rate_stats.emitted_normal++;
        } else if (qB == 1) {
            g_rate_stats.emitted_weak++;
        }

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

  // === FILTER STATS RAPPORTAGE (elke 1000ms) ===
  if (now_ms - g_rate_stats.last_report_ms >= 1000) {
    uint16_t window = (uint16_t)(now_ms - g_rate_stats.last_report_ms);
    uint16_t total_dropped = g_rate_stats.dropped_weak +
                            g_rate_stats.dropped_norm +
                            g_rate_stats.dropped_strong;

    uint32_t rejected = g_rate_stats.rejected_quality;

    // Bereken quality percentages (0-255 scale)
    uint8_t pct_strong = 0, pct_normal = 0, pct_weak = 0;
    if (g_rate_stats.emitted > 0) {
        pct_strong = (uint8_t)((g_rate_stats.emitted_strong * 255) / g_rate_stats.emitted);
        pct_normal = (uint8_t)((g_rate_stats.emitted_normal * 255) / g_rate_stats.emitted);
        pct_weak   = (uint8_t)((g_rate_stats.emitted_weak   * 255) / g_rate_stats.emitted);
    }

    #if USE_FILTER_STATS
      // === GEBRUIK NIEUWE PKT_FILTER_STATS ===
      emit_filter_stats(C->ltx,
          window,
          (uint16_t)g_rate_stats.emitted,
          total_dropped,
          g_rate_stats.considered_total,
          rejected,
          pct_strong,
          pct_normal,
          pct_weak,
          (uint8_t)g_rate_stats.dropped_strong,
          (uint8_t)g_rate_stats.dropped_norm,
          (uint8_t)g_rate_stats.dropped_weak,
          C->tokens,
          (uint8_t)C->P.win_ms);
    #else
      // === LEGACY: gebruik summary16 met fallback ===
      UBaseType_t queue_free = uxQueueSpacesAvailable(C->ltx->q);
      uint8_t queue_free_pct = (uint8_t)((queue_free * 255) / 1024);
      UBaseType_t queue_waiting = uxQueueMessagesWaiting(C->ltx->q);

      bool ok1 = emit_summary16(C->ltx, window,
                  (uint16_t)g_rate_stats.emitted,
                  total_dropped,
                  pct_strong,
                  pct_normal,
                  queue_free_pct,
                  (uint16_t)queue_waiting);

      if (!ok1) {
          // Nood event24
          emit_event24(C->ltx, 0xFFFF, now_ms, 0xFF, 0xFF,
                      (int16_t)g_rate_stats.emitted,
                      (uint8_t)(total_dropped & 0xFF),
                      pct_normal, queue_free_pct,
                      (uint16_t)queue_waiting, 0, 255);
      }
    #endif

    // Reset alle stats
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

// core0_sensing.c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"

#include "core0_link.h"      // framing + UART-writer + emit_event*
#include "core0_filtered.h"  // Filtered-RAW laag



// === S02 Core0 Lab switches ===
#define USE_SMOKE            0   // 1 = oude rookgenerator (direct emit), 0 = filterpad
#define USE_FILTER_GLUE      1   // 1 = voer synthetische raw in core0_filtered en haal events daaruit
#define FILTER_FEED_HZ     480   // synthetische "flank"-frequentie voor lab (pas aan)

typedef enum { SMOKE_E16, SMOKE_E24, SMOKE_ALT } smoke_mode_t;
static struct {
  smoke_mode_t mode;
  uint32_t     evps;         // events per second (frames/s)
  uint32_t     period_us;    // afgeleid
} g_smoke = { .mode = SMOKE_E24, .evps = 100, .period_us = 10000 }; // default

typedef struct {
  uint32_t evps;
  uint32_t period_us;
  uint8_t  seq;
} feeder_cfg_t;

static feeder_cfg_t g_feeder = { .evps = FILTER_FEED_HZ, .period_us = 1000000u/FILTER_FEED_HZ, .seq = 1 };

// ====== Globals ======
static link_tx_t   g_tx;
//static fr_ctx_t    g_fr;

// Boven in file: zorg dat FR_CTX naar de echte global wijst
extern fr_ctx_t g_fr;     // staat gedefinieerd in core0_filtered.c
//extern link_tx_t g_tx;    // je link context
#define FR_CTX   (&g_fr)

// ====== Tijdhelpers ======
static inline uint32_t now_us(void) {
  return (uint32_t) esp_timer_get_time(); // 64b → 32b roll (~71 min)
}

static esp_timer_handle_t g_fr_timer;

static void fr_timer_cb(void* arg) {
  static uint32_t ms = 0;
  fr_tick_1ms(FR_CTX, ms++);  // houd dit super-klein; geen UART hierbinnen
}

static void start_fr_timer_1khz(void) {
  const esp_timer_create_args_t a = { .callback = fr_timer_cb, .name = "fr1khz" };
  ESP_ERROR_CHECK( esp_timer_create(&a, &g_fr_timer) );
  ESP_ERROR_CHECK( esp_timer_start_periodic(g_fr_timer, 1000) ); // 1000 us = 1 kHz
}

// Simpel synthetisch "raw" signaal (A/B square, 90° faseverschil)
static inline void make_raw_pair(uint32_t t_us, int16_t *ra, int16_t *rb){
  // amplitude & offset puur voor lab; pas aan naar jouw filterverwachting
  const int16_t amp = 512, off = 2048;
  // quad fases op basis van periode
  uint32_t phase = t_us % g_feeder.period_us;
  bool hiA = phase < (g_feeder.period_us/2);
  bool hiB = ((phase + g_feeder.period_us/4) % g_feeder.period_us) < (g_feeder.period_us/2);
  *ra = off + (hiA ? amp : -amp);
  *rb = off + (hiB ? amp : -amp);
}

// C-versie van build_and_consider: biedt een kandidaat aan je filter
static inline int build_and_consider(fr_ctx_t* ctx, int sensor, int prev_sign,
                                         int now_sign, uint32_t t_abs_us, int paired)
{
    fr_candidate_t X;
    memset(&X, 0, sizeof X);
    X.t_abs_us   = t_abs_us;
    X.dvdt_q15   = 12000;   // ruim boven drempels
    X.mono_q8    = 200;
    X.snr_q8     = 80;
    X.fit_err_q8 = 6;

    X.sensor     = (uint8_t)sensor;               // 0=A, 1=B
    X.polarity   = (uint8_t)(now_sign ? 1 : 0);   // + = N, - = S (als dat jouw conventie is)
    X.from_pool  = (uint8_t)(prev_sign ? POOL_S : POOL_N);
    X.to_pool    = (uint8_t)(now_sign  ? POOL_N : POOL_S);
    //X.to_pool    = (uint8_t)(POOL_NEU);
    X.pair_flag  = (uint8_t)(paired ? 1 : 0);
    //X.pair_flag  = 1;
    X.dir_hint   = DIR_NONE;                   // filter mag zelf richting bepalen

    return fr_consider(ctx, &X);
};

// === feeder-task: voert raw in de filter en emitt events vanuit de filter ===
static void filter_feeder_task(void* arg){
  // init:
  uint32_t step_us = g_feeder.period_us / 2;         // ← halve periode
  // FIX 2: Sampling Offset. Start de taak met een halve stap verschuiving om de kritische flanken te vermijden.
  uint32_t next_us = (uint32_t)esp_timer_get_time() + step_us + (step_us / 2); // start op half-fase + kwart-stap
  //uint32_t next_us = (uint32_t)esp_timer_get_time() + step_us;  // start op half-fase

  // FIX 3: Variabele om de verwerkingsvolgorde om te draaien
  static int flip_order = 0;

  // De static variabelen voor edge-detectie blijven hier onveranderd
  static int have_prev_a = 0, have_prev_b = 0;
  static int prev_sign_a = 0, prev_sign_b = 0;
  static uint32_t last_edge_a = 0, last_edge_b = 0;
  static uint32_t considered = 0;

  const uint32_t PAIR_US = 3000;   // 3 ms pair-window (pas aan aan jouw filter)

  for(;;){
    uint32_t now = (uint32_t)esp_timer_get_time();

    // --- 1. Timing Check en Taakuitvoering ---
    if ((int32_t)(now - next_us) >= 0) {

      // Increment next_us met de halve periode (960 Hz sampling)
      next_us += step_us;

      int16_t ra, rb;
      make_raw_pair(now, &ra, &rb);

      #if USE_FILTER_GLUE
        // Centreer en bepaal het teken (sign)
        int a = (int)ra - 2048;
        int b = (int)rb - 2048;
        int sign_a = (a >= 0);
        int sign_b = (b >= 0);

        // Eerste keer initialisatie (blijft buiten de flip-logica)
        if (!have_prev_a) { have_prev_a = 1; prev_sign_a = sign_a; last_edge_a = now; }
        if (!have_prev_b) { have_prev_b = 1; prev_sign_b = sign_b; last_edge_b = now; }

        // === FIX 3: Wissel de verwerkingsvolgorde (Toggle Logic) ===
        int process_a_first = flip_order;
        flip_order = 1 - flip_order; // Wissel voor de volgende lus

        // Hulpfunctie om de logica te vereenvoudigen en duplicatie te voorkomen
        void process_sensor_edge(int sensor_idx, int* prev_sign, uint32_t* last_edge, int other_sign, uint32_t other_last_edge) {
            int current_sign = (sensor_idx == 0) ? sign_a : sign_b;
            if (current_sign != *prev_sign) {
                uint32_t t = now;
                // De 'andere' sensor is 1-sensor_idx, maar voor de paired check
                // hebben we de laatste edge van de ANDERE sensor nodig
                int paired = ((t - other_last_edge) <= PAIR_US);

                // Roep de filter aan (gebruikmakend van de gecorrigeerde X.pair_flag)
                considered = considered + build_and_consider(FR_CTX, sensor_idx, *prev_sign, current_sign, t, paired);

                *prev_sign = current_sign;
                *last_edge = t;
            }
        }

        // Voer de sensoren uit in de wisselende volgorde
        if (process_a_first) {
            // Eerst Sensor A (0), dan Sensor B (1)
            process_sensor_edge(0, &prev_sign_a, &last_edge_a, sign_b, last_edge_b);
            process_sensor_edge(1, &prev_sign_b, &last_edge_b, sign_a, last_edge_a);
        } else {
            // Eerst Sensor B (1), dan Sensor A (0)
            process_sensor_edge(1, &prev_sign_b, &last_edge_b, sign_a, last_edge_a);
            process_sensor_edge(0, &prev_sign_a, &last_edge_a, sign_b, last_edge_b);
        }

      #else
        // Fallback: geen filter-glue beschikbaar -> direct een lab-event (rookachtig)
        emit_event24(&g_tx, 1000, now,
                    make_flags0(false,2,1,0), make_flags1(0,1,1),
                    1400, 250, 90, 7, 0x1388, 180, g_feeder.seq++);
      #endif

      // De emit_summary16 logica
      static uint32_t last_beat = 0;
      // Tel elke consider in build_and_consider: ++considered;
      if (now - last_beat >= 1000000) {
        emit_summary16(&g_tx, 1000, 0, 0, 0, 0, 0, considered*16);
        considered = 0;
        last_beat = now;
      }

      // ↓ VOEG EEN PERIODIEKE YIELD TOE OM DE SCHEDULER TE HELPEN
      static uint32_t yield_counter = 0;
      if (++yield_counter > 50) { // Ongeveer elke 50 * 1041us = 52ms
        taskYIELD();
        yield_counter = 0;
      }

    // --- 2. Wachtlogica ---
    } else {
        // FIX 1: Gebruik de zuivere hardware delay om latentie te elimineren
        uint32_t wait = next_us - now;
        if (wait > 10) {
            esp_rom_delay_us(wait);
        }
    }
  }
}

void start_filter_feeder_task(void){
  // evt. init van de filterstruct (als nog niet gedaan)
  // fr_init(FR_CTX);  // alleen wanneer je dat nog niet elders doet
  xTaskCreate(filter_feeder_task, "filter_feeder", 4096, NULL, 2, NULL);
}

#if USE_SMOKE
static void smoke_set(smoke_mode_t m, uint32_t evps){
  g_smoke.mode = m;
  g_smoke.evps = (evps == 0 ? 1 : evps);
  g_smoke.period_us = 1000000u / g_smoke.evps;
}

// ====== Smoke task (precisie-kadans, geen logs) ======
static void smoke_task(void* arg){
  uint8_t  flip = 0;
  uint8_t  seq  = 1;
  uint32_t next_t = (uint32_t)esp_timer_get_time();

  for(;;){
    uint32_t now = (uint32_t)esp_timer_get_time();
    if ((int32_t)(now - next_t) >= 0) {
      // emit
      if (g_smoke.mode == SMOKE_E16) {
        emit_event16(&g_tx, /*dt_us*/1000, make_flags0(false,2,1,0), make_flags1(0,1,1),
                     /*dvdt*/1200, /*mono*/200, /*snr*/80, /*score*/180, seq++);
      } else if (g_smoke.mode == SMOKE_E24) {
        emit_event24(&g_tx, /*dt_us*/1000, /*t_abs*/now,
                     make_flags0(false,2,1,0), make_flags1(0,1,1),
                     /*dvdt*/1200, /*mono*/200, /*snr*/80, /*fit*/7,
                     /*rpm_hint*/0x1388, /*score*/180, seq++);
      } else { // SMOKE_ALT
        if (!flip) {
          emit_event24(&g_tx, 1000, now, make_flags0(false,2,1,0), make_flags1(0,1,1),
                       1200, 200, 80, 7, 0x1388, 180, seq++);
        } else {
          emit_event16(&g_tx, 1000, make_flags0(false,2,1,0), make_flags1(0,1,1),
                       1200, 200, 80, 180, seq++);
        }
        flip ^= 1;
      }
      next_t += g_smoke.period_us;
    } else {
      // micro-slaapje tot de volgende slot
      uint32_t wait_us = next_t - now;
      if (wait_us > 50) vTaskDelay(pdMS_TO_TICKS(1));
      else esp_rom_delay_us(wait_us);
    }
  }
}
#endif

// ====== INIT ======
void app_main(void)
{
  // 1) Logging en Watchdog settings
  esp_log_level_set("*", ESP_LOG_NONE);   // ← alle logs uit tijdens rooktest
  // Zet de Task Watchdog UIT voor deze debug-sessie
  // esp_task_wdt_deinit();               // hard-off; negeert alle add/reset elders
  // Zet de Task Watchdog gecontroleerd aan
  const esp_task_wdt_config_t twdt_cfg = {
    .timeout_ms    = 2000,   // 2 s
    .idle_core_mask = 0,     // géén idle-taken automatisch aanmelden
    .trigger_panic  = false  // geen panic bij timeout (alleen error/log)
  };
  esp_err_t err = esp_task_wdt_init(&twdt_cfg);
  if (err == ESP_ERR_INVALID_STATE) {
    // al actief; reconfigureer i.p.v. init
    ESP_ERROR_CHECK( esp_task_wdt_reconfigure(&twdt_cfg) );
  }

  // 2) TX-queue + UART-writer
  link_init_tx(&g_tx, /*queue_len*/ 512);   // was 128
  start_uart_writer(&g_tx);  // komt uit core0_link.c

  // 3) Filtered-RAW parameters (veilig startprofiel; later tunen)
  fr_params_t P = {
    .dvdt_min_q15   = 800,            // drempel dv/dt (fixed-point)
    .mono_min_q8    = 153,            // ≈0.6
    .snr_min_q8     = 24,             // ≈12 dB (als 0.5 dB/LSB)
    .fit_err_max_q8 = 40,

    .w_dvdt=6, .w_mono=4, .w_snr=3, .w_fit_pen=2,

    .win_ms=3,                        // coalescing window 3 ms

    .target_evps=400,                 // doel events/s (A+B)
    .util_hi_q=217,                   // ~85% utilization (0..255 → 0..100%)
    .l2_only_ms=200                   // (gereserveerd voor toekomstige “weak drop”)
  };
  fr_init(FR_CTX, &g_tx, &P);

  // 4) Start 1 kHz tick via high-res timer (WDT-proof)
  start_fr_timer_1khz();

  // 5) (tijdelijk) Test; haal weg zodra je echte sensing aanhaakt
  #if USE_SMOKE
    smoke_set(SMOKE_E24, 400);
    xTaskCreate(smoke_task, "smoke", 3072, NULL, 2, NULL);
  #else
    extern void start_filter_feeder_task(void);  // komt zo hieronder
    start_filter_feeder_task();
  #endif
}

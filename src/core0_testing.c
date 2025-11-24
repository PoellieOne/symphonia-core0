// core0_testing.c
// Bevat alle lab/test functionaliteit voor Core-0:
// - smoke generator (oude rookroute)
// - synthetic filter_feeder (USE_FILTER_GLUE)
// Dit bestand wordt *ingeplakt* in core0_sensing.c via #include.

// We gaan ervan uit dat core0_sensing.c al de juiste headers include
// (FreeRTOS, esp_timer, core0_link.h, core0_filtered.h, etc.)
// en dat daar ook g_tx en FR_CTX gedefinieerd zijn.

//
// === S02 Core0 Lab switches ===
//
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

// Build_and_consider: biedt een kandidaat aan je filter
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
    X.pair_flag  = (uint8_t)(paired ? 1 : 0);
    X.dir_hint   = DIR_NONE;                   // filter mag zelf richting bepalen

    return fr_consider(ctx, &X);
};

// Feeder-task: voert raw in de filter en emitt events vanuit de filter
static void filter_feeder_task(void* arg){
  // init:
  uint32_t step_us = g_feeder.period_us / 2;         // ← halve periode
  // Sampling Offset. Start de taak met een halve stap verschuiving om de kritische flanken te vermijden.
  uint32_t next_us = (uint32_t)esp_timer_get_time() + step_us + (step_us / 2); // start op half-fase + kwart-stap
  // Variabele om de verwerkingsvolgorde om te draaien
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

        // Wissel de verwerkingsvolgorde (Toggle Logic)
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
        //link_tx_t* tx = core0_link_get_tx();
        // Fallback: geen filter-glue beschikbaar -> direct een lab-event (rookachtig)
        emit_event24(g_tx, 1000, now,
                    make_flags0(false,2,1,0), make_flags1(0,1,1,0),
                    1400, 250, 90, 7, 0x1388, 180, g_feeder.seq++);
      #endif

      // Voeg een periodieke yield toe om de scheduler te helpen
      static uint32_t yield_counter = 0;
      if (++yield_counter > 50) { // Ongeveer elke 50 * 1041us = 52ms
        taskYIELD();
        yield_counter = 0;
      }

    // --- 2. Wachtlogica ---
    } else {
        // Gebruik de zuivere hardware delay om latentie te elimineren
        uint32_t wait = next_us - now;
        if (wait > 10) {
            esp_rom_delay_us(wait);
        }
    }
  }
}

void start_filter_feeder_task(void){
  // fr_init(FR_CTX);  // alleen wanneer je dat nog niet elders doet
  xTaskCreate(filter_feeder_task, "filter_feeder", 4096, NULL, 1, NULL);
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
      //link_tx_t* tx = core0_link_get_tx();
      if (g_smoke.mode == SMOKE_E16) {
        emit_event16(g_tx, /*dt_us*/1000, make_flags0(false,2,1,0), make_flags1(0,1,1),
                     /*dvdt*/1200, /*mono*/200, /*snr*/80, /*score*/180, seq++);
      } else if (g_smoke.mode == SMOKE_E24) {
        emit_event24(g_tx, /*dt_us*/1000, /*t_abs*/now,
                     make_flags0(false,2,1,0), make_flags1(0,1,1),
                     /*dvdt*/1200, /*mono*/200, /*snr*/80, /*fit*/7,
                     /*rpm_hint*/0x1388, /*score*/180, seq++);
      } else { // SMOKE_ALT
        if (!flip) {
          emit_event24(g_tx, 1000, now, make_flags0(false,2,1,0), make_flags1(0,1,1),
                       1200, 200, 80, 7, 0x1388, 180, seq++);
        } else {
          emit_event16(g_tx, 1000, make_flags0(false,2,1,0), make_flags1(0,1,1),
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

// === Eén entrypoint voor app_main ===
void core0_start_testing(void)
{
#if USE_SMOKE
    smoke_set(SMOKE_E24, 480);
    xTaskCreate(smoke_task, "smoke", 3072, NULL, 2, NULL);
#else
    start_filter_feeder_task();
#endif
}

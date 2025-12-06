// core0_config.h - Centrale configuratie voor S02 Core-0 Firmware
// =============================================================================
// Versie: 1.0.0
// Datum: 2025-01-XX
// 
// Dit bestand bundelt ALLE configureerbare parameters voor de Core-0 firmware.
// Doel: één plek voor tuning, geen verspreide magic numbers meer.
//
// Gebruik:
//   #include "core0_config.h"
//   // Access via: CFG_SENS.sensor_A.low_th, CFG_FILT.dvdt_min_q15, etc.
// =============================================================================

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// RUN MODE SELECTIE
// =============================================================================
// Bepaalt welke code-paden actief zijn. Compile-time keuze voor optimale
// code size, maar config struct blijft beschikbaar voor runtime queries.

typedef enum {
    RUNMODE_PRODUCTION = 0,    // Normaal: sensing → filter → link
    RUNMODE_DEBUG_BYPASS,      // Bypass filter quality gates, emit alles
    RUNMODE_RAW_CALIBRATE,     // Log raw ADC voor threshold calibratie
    RUNMODE_SMOKE_TEST,        // Synthetische events voor link testing
} core0_runmode_t;

// *** SELECTEER HIER DE ACTIEVE MODE ***
#ifndef CORE0_RUNMODE
#define CORE0_RUNMODE  RUNMODE_PRODUCTION
#endif

// Convenience macros voor conditional compilation
#define IS_PRODUCTION     (CORE0_RUNMODE == RUNMODE_PRODUCTION)
#define IS_DEBUG_BYPASS   (CORE0_RUNMODE == RUNMODE_DEBUG_BYPASS)
#define IS_RAW_CALIBRATE  (CORE0_RUNMODE == RUNMODE_RAW_CALIBRATE)
#define IS_SMOKE_TEST     (CORE0_RUNMODE == RUNMODE_SMOKE_TEST)

// =============================================================================
// VERSION INFO
// =============================================================================
#define CORE0_FW_VERSION_MAJOR  1
#define CORE0_FW_VERSION_MINOR  0
#define CORE0_FW_VERSION_PATCH  0
#define CORE0_FW_VERSION_STR    "1.0.0"
#define CORE0_FW_BUILD_TAG      "S02-Core0-BinaryFraming"

// =============================================================================
// HARDWARE CONFIGURATIE
// =============================================================================
// Fysieke pin assignments en hardware-specifieke settings.
// Deze veranderen zelden, maar zijn hier voor duidelijkheid.

typedef struct {
    // ADC configuratie
    uint8_t  adc_unit;             // ADC_UNIT_1 = 0
    uint8_t  adc_channel_A;        // Channel voor sensor A (6 = GPIO34)
    uint8_t  adc_channel_B;        // Channel voor sensor B (7 = GPIO35)
    uint8_t  adc_atten;            // Attenuation (ADC_ATTEN_DB_12 = 3)
    uint8_t  adc_bitwidth;         // Bit width (ADC_BITWIDTH_12 = 12)
    uint32_t sample_rate_hz;       // Totale sample rate (beide channels)
    
    // UART configuratie
    uint8_t  uart_port;            // UART_NUM_0 = 0
    uint8_t  uart_tx_gpio;         // TX pin (1 = GPIO1)
    uint8_t  uart_rx_gpio;         // RX pin (3 = GPIO3)
    uint32_t uart_baud;            // Baud rate
    
    // Derived (berekend bij init)
    uint32_t per_channel_hz;       // sample_rate_hz / 2
    float    us_per_sample;        // 1e6 / per_channel_hz
} hw_config_t;

// Hardware defaults
#define HW_ADC_UNIT_DEFAULT         0       // ADC_UNIT_1
#define HW_ADC_CH_A_DEFAULT         6       // GPIO34
#define HW_ADC_CH_B_DEFAULT         7       // GPIO35
#define HW_ADC_ATTEN_DEFAULT        3       // ADC_ATTEN_DB_12
#define HW_ADC_BITWIDTH_DEFAULT     12
#define HW_SAMPLE_RATE_DEFAULT      200000
#define HW_UART_PORT_DEFAULT        0
#define HW_UART_TX_DEFAULT          1
#define HW_UART_RX_DEFAULT          3
#define HW_UART_BAUD_DEFAULT        115200

// =============================================================================
// SENSING CONFIGURATIE (core0_sensing.c)
// =============================================================================
// Parameters voor ADC sampling, pool classificatie, en event detectie.

// Sensor-specifieke thresholds
typedef struct {
    int16_t low_th;                // Onder dit = SOUTH pool
    int16_t high_th;               // Boven dit = NORTH pool
    float   baseline_init;         // Initiële baseline waarde
    // Calibratie resultaten (gevuld door auto-calibrate)
    int16_t cal_min;               // Gemeten minimum
    int16_t cal_max;               // Gemeten maximum
    float   cal_baseline;          // Gemeten baseline
} sensor_thresholds_t;

typedef struct {
    // Per-sensor thresholds
    sensor_thresholds_t sensor_A;
    sensor_thresholds_t sensor_B;
    
    // Baseline tracking
    float baseline_alpha;          // EMA smoothing factor (0.001 = langzaam)
    
    // Pool transition timing
    uint32_t pool_min_dt_us;       // Minimum tijd tussen pool changes (debounce)
    
    // Pre-filter quality gates (vóór fr_consider)
    float zc_mono_min;             // Minimum monotonicity voor zero-cross
    float zc_snr_min;              // Minimum SNR voor zero-cross
    
    // Context envelope windows
    float context_ms_pre;          // Pre-event context (ms)
    float context_ms_post;         // Post-event context (ms)
    
    // Sample history
    uint8_t history_len;           // Samples voor slope/noise berekening
    
    // Edge detection features
    bool include_neutral_edges;    // Emit POOL↔NEUTRAL transitions?
    bool include_zero_cross;       // Emit zero-cross events? (meestal false)
    
    // Auto-calibration
    bool     auto_calibrate;       // Run calibratie bij boot?
    uint16_t calibrate_duration_ms;// Calibratie duur
    float    calibrate_threshold_pct; // Threshold als % van range (0.6 = 60%)
    
    // V1.1 Feature flags - default UITGESCHAKELD voor V1.0 compatibiliteit
    bool use_improved_fit;         // V1.1: Verbeterde fit_error (DEFAULT: false)
    bool use_fast_fit;             // V1.1: Snelle 3-punt versie (als improved=true)
} sensing_config_t;

// Sensing defaults
#define SENS_BASELINE_ALPHA_DEFAULT     0.001f
#define SENS_POOL_MIN_DT_US_DEFAULT     200
#define SENS_ZC_MONO_MIN_DEFAULT        0.10f
#define SENS_ZC_SNR_MIN_DEFAULT         1.5f
#define SENS_CONTEXT_MS_PRE_DEFAULT     0.7f
#define SENS_CONTEXT_MS_POST_DEFAULT    0.7f
#define SENS_HISTORY_LEN_DEFAULT        8
#define SENS_CALIBRATE_DURATION_DEFAULT 5000
#define SENS_CALIBRATE_PCT_DEFAULT      0.60f

// V1.1 Feature defaults - UITGESCHAKELD voor V1.0 compatibiliteit
#define SENS_USE_IMPROVED_FIT_DEFAULT   false   // V1.0: noise×5
#define SENS_USE_FAST_FIT_DEFAULT       false   // Alleen relevant als improved=true

// Sensor A defaults (gebaseerd op jouw calibratie)
#define SENS_A_LOW_TH_DEFAULT           1610
#define SENS_A_HIGH_TH_DEFAULT          2540
#define SENS_A_BASELINE_DEFAULT         2005.0f

// Sensor B defaults
#define SENS_B_LOW_TH_DEFAULT           740
#define SENS_B_HIGH_TH_DEFAULT          1290
#define SENS_B_BASELINE_DEFAULT         979.0f

// =============================================================================
// FILTER CONFIGURATIE (core0_filtered.c)
// =============================================================================
// Quality gates, scoring, coalescing, en backpressure parameters.

typedef struct {
    // Quality gate thresholds
    int16_t  dvdt_min_q15;         // Minimum |dV/dt| (Q15 format)
    uint8_t  mono_min_q8;          // Minimum monotonicity (0-255)
    uint8_t  snr_min_q8;           // Minimum SNR (0-255, ~0.5dB per LSB)
    uint8_t  fit_err_max_q8;       // Maximum fit error (0-255)
    
    // Scoring weights (voor gewogen score berekening)
    uint8_t  w_dvdt;               // Weight voor dV/dt
    uint8_t  w_mono;               // Weight voor monotonicity
    uint8_t  w_snr;                // Weight voor SNR
    uint8_t  w_fit_pen;            // Penalty weight voor fit error
    
    // Quality level classification
    struct {
        float   dvdt_strong_mult;  // dvdt >= mult * min → strong
        float   mono_strong_q8;    // mono >= dit → strong (0-255 scale)
        uint8_t snr_strong_add;    // snr >= min + add → strong
    } qlevel;
    
    // Coalescing (selecteer beste event binnen window)
    uint16_t coalesce_win_ms;      // Window grootte in ms
    uint16_t coalesce_win_max_ms;  // Maximum na backpressure scaling
    
    // Backpressure / Token bucket
    uint16_t target_evps;          // Target events per second (A+B samen)
    float    token_max_ms;         // Max token accumulation (in ms worth)
    float    token_floor;          // Minimum tokens (negatief = debt toegestaan)
    struct {
        float strong;              // Token cost voor strong events
        float normal;              // Token cost voor normal events
        float weak;                // Token cost voor weak events
    } token_cost;
    struct {
        float strong;              // Drop threshold voor strong
        float normal;              // Drop threshold voor normal
        float weak;                // Drop threshold voor weak
    } drop_threshold;
    
    // Utilization feedback
    uint8_t  util_hi_threshold;    // Boven dit: vergroot coalesce window
    
    // Stale window flush
    uint16_t stale_window_mult;    // Flush als window > win_ms * mult
} filter_config_t;

// Filter defaults
#define FILT_DVDT_MIN_DEFAULT           15
#define FILT_MONO_MIN_DEFAULT           105     // ~0.41
#define FILT_SNR_MIN_DEFAULT            24      // ~12 dB
#define FILT_FIT_ERR_MAX_DEFAULT        120
#define FILT_W_DVDT_DEFAULT             5
#define FILT_W_MONO_DEFAULT             4
#define FILT_W_SNR_DEFAULT              4
#define FILT_W_FIT_PEN_DEFAULT          3
#define FILT_QLEVEL_DVDT_MULT_DEFAULT   1.5f
#define FILT_QLEVEL_MONO_STRONG_DEFAULT 204     // 0.8 * 255
#define FILT_QLEVEL_SNR_ADD_DEFAULT     24
#define FILT_COALESCE_WIN_DEFAULT       3
#define FILT_COALESCE_WIN_MAX_DEFAULT   6
#define FILT_TARGET_EVPS_DEFAULT        150
#define FILT_TOKEN_MAX_MS_DEFAULT       50.0f
#define FILT_TOKEN_FLOOR_DEFAULT        -2.0f
#define FILT_TOKEN_COST_STRONG_DEFAULT  0.5f
#define FILT_TOKEN_COST_NORMAL_DEFAULT  1.0f
#define FILT_TOKEN_COST_WEAK_DEFAULT    1.5f
#define FILT_DROP_TH_STRONG_DEFAULT     -2.0f
#define FILT_DROP_TH_NORMAL_DEFAULT     0.5f
#define FILT_DROP_TH_WEAK_DEFAULT       1.0f
#define FILT_UTIL_HI_DEFAULT            200
#define FILT_STALE_MULT_DEFAULT         2

// =============================================================================
// LINK CONFIGURATIE (core0_link.c)
// =============================================================================
// UART transmission, queue sizing, en statistics reporting.

typedef struct {
    // Queue sizing
    uint16_t tx_queue_len;         // Max frames in TX queue
    uint16_t tx_buffer_size;       // UART TX buffer (0 = blocking mode)
    uint16_t rx_buffer_size;       // UART RX buffer
    
    // Statistics reporting intervals
    uint16_t filter_stats_interval_ms;  // PKT_FILTER_STATS interval
    uint16_t link_stats_interval_ms;    // PKT_LINK_STATS interval
    uint16_t link_stats_offset_ms;      // Offset t.o.v. filter stats (stagger)
    
    // Feature flags
    bool use_filter_stats;         // Gebruik PKT_FILTER_STATS (vs legacy)
    bool use_link_stats;           // Gebruik PKT_LINK_STATS (vs legacy)
    bool send_ascii_ping;          // "BINSTART" bij boot
} link_config_t;

// Link defaults
#define LINK_TX_QUEUE_LEN_DEFAULT       512
#define LINK_TX_BUFFER_SIZE_DEFAULT     8192
#define LINK_RX_BUFFER_SIZE_DEFAULT     256
#define LINK_FILTER_STATS_MS_DEFAULT    1000
#define LINK_LINK_STATS_MS_DEFAULT      1000
#define LINK_STATS_OFFSET_MS_DEFAULT    500

// =============================================================================
// PAIR DETECTION CONFIGURATIE
// =============================================================================
// A↔B pairing voor richting bepaling.

typedef struct {
    // Basis pair window
    uint32_t window_us;            // Max tijd tussen A en B voor pair
    
    // Adaptive window (optioneel)
    bool     adaptive;             // Window aanpassen op basis van event rate?
    uint32_t window_min_us;        // Minimum bij hoge event rate
    uint32_t window_max_us;        // Maximum bij lage event rate
    float    adaptive_mult;        // Window = recent_dt * mult
} pair_config_t;

// Pair defaults
#define PAIR_WINDOW_US_DEFAULT          3000
#define PAIR_WINDOW_MIN_US_DEFAULT      500
#define PAIR_WINDOW_MAX_US_DEFAULT      15000
#define PAIR_ADAPTIVE_MULT_DEFAULT      2.0f

// =============================================================================
// DIRECTION DETECTION CONFIGURATIE
// =============================================================================
// Richting bepaling uit A↔B timing.
// V1.1 FEATURE - default UITGESCHAKELD voor V1.0 compatibiliteit.

typedef struct {
    // Feature toggle
    bool     enabled;              // V1.1: dir_hint berekenen? (DEFAULT: false)
    
    // Geometry (alleen gebruikt als enabled=true)
    int16_t  phi_ab_deg;           // Sensor hoek verschil (100° voor jouw setup)
    
    // Direction mapping
    // Bij CW rotatie: welke sensor ziet magneet eerst?
    // Dit hangt af van fysieke montage.
    bool     cw_is_A_first;        // true = A→B = CW, false = B→A = CW
    
    // Transitie consistentie (V1.1)
    bool     require_consistent_transition;  // Check from/to pool match?
} direction_config_t;

// Direction defaults - V1.0 COMPATIBEL (enabled=false)
#define DIR_ENABLED_DEFAULT             false   // V1.0: uitgeschakeld
#define DIR_PHI_AB_DEG_DEFAULT          100
#define DIR_CW_IS_A_FIRST_DEFAULT       true
#define DIR_REQUIRE_CONSISTENT_DEFAULT  true

// =============================================================================
// DEBUG / CALIBRATION MODE CONFIGURATIE
// =============================================================================
// Mode-specifieke parameters.

typedef struct {
    // RAW_CALIBRATE mode
    struct {
        uint16_t decimation;       // Log elke N samples
        uint16_t duration_ms;      // Calibratie duur
        bool     print_csv_header; // Print CSV header bij start
        bool     print_suggestions;// Print threshold suggestions bij einde
    } raw_cal;
    
    // SMOKE_TEST mode
    struct {
        uint16_t events_per_sec;   // Synthetische event rate
        bool     alternate_e16_e24;// Afwisselen tussen EVENT16 en EVENT24
        bool     include_stats;    // Ook FILTER_STATS en LINK_STATS sturen
    } smoke;
    
    // DEBUG_BYPASS mode
    struct {
        bool     bypass_quality;   // Emit ook qlevel=0 events
        bool     bypass_backpressure; // Negeer token bucket
    } bypass;
    
    // General debug
    struct {
        uint16_t heartbeat_interval_ms; // Debug print interval (0 = uit)
        bool     log_pool_transitions;  // Log elke pool change
        bool     log_rejected_events;   // Log rejected candidates
    } general;
} debug_config_t;

// Debug defaults
#define DBG_RAW_DECIMATION_DEFAULT      100
#define DBG_RAW_DURATION_DEFAULT        5000
#define DBG_SMOKE_EVPS_DEFAULT          100
#define DBG_HEARTBEAT_MS_DEFAULT        1000

// =============================================================================
// MASTER CONFIG STRUCT
// =============================================================================

typedef struct {
    // Active run mode
    core0_runmode_t   runmode;
    
    // Sub-configurations
    hw_config_t       hw;
    sensing_config_t  sensing;
    filter_config_t   filter;
    link_config_t     link;
    pair_config_t     pair;
    direction_config_t direction;
    debug_config_t    debug;
    
    // Runtime state (niet-config, maar handig om hier te hebben)
    struct {
        bool initialized;
        bool calibrated;
        uint32_t boot_time_us;
    } state;
} core0_config_t;

// =============================================================================
// GLOBAL CONFIG INSTANCE
// =============================================================================

// Globale config - geïnitialiseerd met defaults in core0_config.c
extern core0_config_t g_cfg;

// =============================================================================
// ACCESSOR MACROS
// =============================================================================
// Shorthand voor frequente toegang

#define CFG           (g_cfg)
#define CFG_MODE      (g_cfg.runmode)
#define CFG_HW        (g_cfg.hw)
#define CFG_SENS      (g_cfg.sensing)
#define CFG_FILT      (g_cfg.filter)
#define CFG_LINK      (g_cfg.link)
#define CFG_PAIR      (g_cfg.pair)
#define CFG_DIR       (g_cfg.direction)
#define CFG_DBG       (g_cfg.debug)
#define CFG_STATE     (g_cfg.state)

// Sensor-specifieke shortcuts
#define CFG_SENS_A    (g_cfg.sensing.sensor_A)
#define CFG_SENS_B    (g_cfg.sensing.sensor_B)

// =============================================================================
// API FUNCTIONS
// =============================================================================

/**
 * Initialiseer config met defaults.
 * Moet als eerste worden aangeroepen in app_main().
 */
void core0_config_init(void);

/**
 * Print huidige configuratie naar log.
 * Handig voor debugging.
 */
void core0_config_print(void);

/**
 * Valideer configuratie op inconsistenties.
 * Returns: true als valid, false als problemen gevonden.
 */
bool core0_config_validate(void);

/**
 * Update sensing thresholds na auto-calibratie.
 * Wordt aangeroepen door calibratie routine.
 */
void core0_config_update_thresholds(
    int16_t a_min, int16_t a_max, float a_baseline,
    int16_t b_min, int16_t b_max, float b_baseline
);

/**
 * Get config as JSON string (voor toekomstige serial config interface).
 * Buffer moet groot genoeg zijn (~1KB).
 */
int core0_config_to_json(char* buf, size_t buflen);

// =============================================================================
// COMPILE-TIME CHECKS
// =============================================================================

// Verify history buffer size is power of 2 for efficient modulo
_Static_assert((SENS_HISTORY_LEN_DEFAULT & (SENS_HISTORY_LEN_DEFAULT - 1)) == 0,
               "HISTORY_LEN must be power of 2");

// Verify sample rate is reasonable
_Static_assert(HW_SAMPLE_RATE_DEFAULT >= 10000 && HW_SAMPLE_RATE_DEFAULT <= 1000000,
               "Sample rate out of reasonable range");

#ifdef __cplusplus
}
#endif

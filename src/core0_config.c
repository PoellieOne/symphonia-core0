// core0_config.c - Configuratie implementatie voor S02 Core-0 Firmware
// =============================================================================

#include "core0_config.h"
#include <string.h>

#include <stdio.h>

// ESP-IDF includes (alleen nodig voor logging)
#ifdef ESP_PLATFORM
#include "esp_log.h"
#include "esp_timer.h"
static const char* TAG = "CFG";
#define LOG_I(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#define LOG_W(fmt, ...) ESP_LOGW(TAG, fmt, ##__VA_ARGS__)
#define LOG_E(fmt, ...) ESP_LOGE(TAG, fmt, ##__VA_ARGS__)
#else
// Fallback voor non-ESP builds (testing)
#define LOG_I(fmt, ...) printf("[CFG] " fmt "\n", ##__VA_ARGS__)
#define LOG_W(fmt, ...) printf("[CFG WARN] " fmt "\n", ##__VA_ARGS__)
#define LOG_E(fmt, ...) printf("[CFG ERROR] " fmt "\n", ##__VA_ARGS__)
#endif

// =============================================================================
// GLOBAL CONFIG INSTANCE
// =============================================================================

core0_config_t g_cfg = {0};  // Zero-init, filled by core0_config_init()

// =============================================================================
// DEFAULT VALUES (static const voor flash storage)
// =============================================================================

static const core0_config_t CONFIG_DEFAULTS = {
    .runmode = CORE0_RUNMODE,
    
    // Hardware configuratie
    .hw = {
        .adc_unit         = HW_ADC_UNIT_DEFAULT,
        .adc_channel_A    = HW_ADC_CH_A_DEFAULT,
        .adc_channel_B    = HW_ADC_CH_B_DEFAULT,
        .adc_atten        = HW_ADC_ATTEN_DEFAULT,
        .adc_bitwidth     = HW_ADC_BITWIDTH_DEFAULT,
        .sample_rate_hz   = HW_SAMPLE_RATE_DEFAULT,
        .uart_port        = HW_UART_PORT_DEFAULT,
        .uart_tx_gpio     = HW_UART_TX_DEFAULT,
        .uart_rx_gpio     = HW_UART_RX_DEFAULT,
        .uart_baud        = HW_UART_BAUD_DEFAULT,
        // Derived fields filled in init
        .per_channel_hz   = HW_SAMPLE_RATE_DEFAULT / 2,
        .us_per_sample    = 2.0f * 1e6f / HW_SAMPLE_RATE_DEFAULT,
    },
    
    // Sensing configuratie
    .sensing = {
        .sensor_A = {
            .low_th        = SENS_A_LOW_TH_DEFAULT,
            .high_th       = SENS_A_HIGH_TH_DEFAULT,
            .baseline_init = SENS_A_BASELINE_DEFAULT,
            .cal_min       = 0,
            .cal_max       = 4095,
            .cal_baseline  = SENS_A_BASELINE_DEFAULT,
        },
        .sensor_B = {
            .low_th        = SENS_B_LOW_TH_DEFAULT,
            .high_th       = SENS_B_HIGH_TH_DEFAULT,
            .baseline_init = SENS_B_BASELINE_DEFAULT,
            .cal_min       = 0,
            .cal_max       = 4095,
            .cal_baseline  = SENS_B_BASELINE_DEFAULT,
        },
        .baseline_alpha       = SENS_BASELINE_ALPHA_DEFAULT,
        .pool_min_dt_us       = SENS_POOL_MIN_DT_US_DEFAULT,
        .zc_mono_min          = SENS_ZC_MONO_MIN_DEFAULT,
        .zc_snr_min           = SENS_ZC_SNR_MIN_DEFAULT,
        .context_ms_pre       = SENS_CONTEXT_MS_PRE_DEFAULT,
        .context_ms_post      = SENS_CONTEXT_MS_POST_DEFAULT,
        .history_len          = SENS_HISTORY_LEN_DEFAULT,
        .include_neutral_edges = true,
        .include_zero_cross   = false,  // Zero-cross detectie meestal niet nodig
        .auto_calibrate       = false,  // Default uit, expliciet aanzetten
        .calibrate_duration_ms = SENS_CALIBRATE_DURATION_DEFAULT,
        .calibrate_threshold_pct = SENS_CALIBRATE_PCT_DEFAULT,
        // V1.1 Features - DEFAULT UITGESCHAKELD
        .use_improved_fit     = SENS_USE_IMPROVED_FIT_DEFAULT,  // false = V1.0
        .use_fast_fit         = SENS_USE_FAST_FIT_DEFAULT,
    },
    
    // Filter configuratie
    .filter = {
        .dvdt_min_q15     = FILT_DVDT_MIN_DEFAULT,
        .mono_min_q8      = FILT_MONO_MIN_DEFAULT,
        .snr_min_q8       = FILT_SNR_MIN_DEFAULT,
        .fit_err_max_q8   = FILT_FIT_ERR_MAX_DEFAULT,
        .w_dvdt           = FILT_W_DVDT_DEFAULT,
        .w_mono           = FILT_W_MONO_DEFAULT,
        .w_snr            = FILT_W_SNR_DEFAULT,
        .w_fit_pen        = FILT_W_FIT_PEN_DEFAULT,
        .qlevel = {
            .dvdt_strong_mult  = FILT_QLEVEL_DVDT_MULT_DEFAULT,
            .mono_strong_q8    = FILT_QLEVEL_MONO_STRONG_DEFAULT,
            .snr_strong_add    = FILT_QLEVEL_SNR_ADD_DEFAULT,
        },
        .coalesce_win_ms     = FILT_COALESCE_WIN_DEFAULT,
        .coalesce_win_max_ms = FILT_COALESCE_WIN_MAX_DEFAULT,
        .target_evps         = FILT_TARGET_EVPS_DEFAULT,
        .token_max_ms        = FILT_TOKEN_MAX_MS_DEFAULT,
        .token_floor         = FILT_TOKEN_FLOOR_DEFAULT,
        .token_cost = {
            .strong = FILT_TOKEN_COST_STRONG_DEFAULT,
            .normal = FILT_TOKEN_COST_NORMAL_DEFAULT,
            .weak   = FILT_TOKEN_COST_WEAK_DEFAULT,
        },
        .drop_threshold = {
            .strong = FILT_DROP_TH_STRONG_DEFAULT,
            .normal = FILT_DROP_TH_NORMAL_DEFAULT,
            .weak   = FILT_DROP_TH_WEAK_DEFAULT,
        },
        .util_hi_threshold   = FILT_UTIL_HI_DEFAULT,
        .stale_window_mult   = FILT_STALE_MULT_DEFAULT,
    },
    
    // Link configuratie
    .link = {
        .tx_queue_len             = LINK_TX_QUEUE_LEN_DEFAULT,
        .tx_buffer_size           = LINK_TX_BUFFER_SIZE_DEFAULT,
        .rx_buffer_size           = LINK_RX_BUFFER_SIZE_DEFAULT,
        .filter_stats_interval_ms = LINK_FILTER_STATS_MS_DEFAULT,
        .link_stats_interval_ms   = LINK_LINK_STATS_MS_DEFAULT,
        .link_stats_offset_ms     = LINK_STATS_OFFSET_MS_DEFAULT,
        .use_filter_stats         = true,
        .use_link_stats           = true,
        .send_ascii_ping          = false,
    },
    
    // Direction configuratie - V1.0 DEFAULT: UITGESCHAKELD
    .direction = {
        .enabled       = DIR_ENABLED_DEFAULT,  // false = V1.0 (geen dir_hint)
        .phi_ab_deg    = DIR_PHI_AB_DEG_DEFAULT,
        .cw_is_A_first = DIR_CW_IS_A_FIRST_DEFAULT,
        .require_consistent_transition = DIR_REQUIRE_CONSISTENT_DEFAULT,
    },
    
    // Pair detection configuratie
    .pair = {
        .window_us      = PAIR_WINDOW_US_DEFAULT,
        .adaptive       = false,
        .window_min_us  = PAIR_WINDOW_MIN_US_DEFAULT,
        .window_max_us  = PAIR_WINDOW_MAX_US_DEFAULT,
        .adaptive_mult  = PAIR_ADAPTIVE_MULT_DEFAULT,
    },
    
    // Debug configuratie
    .debug = {
        .raw_cal = {
            .decimation         = DBG_RAW_DECIMATION_DEFAULT,
            .duration_ms        = DBG_RAW_DURATION_DEFAULT,
            .print_csv_header   = true,
            .print_suggestions  = true,
        },
        .smoke = {
            .events_per_sec     = DBG_SMOKE_EVPS_DEFAULT,
            .alternate_e16_e24  = false,
            .include_stats      = true,
        },
        .bypass = {
            .bypass_quality     = true,
            .bypass_backpressure = false,
        },
        .general = {
            .heartbeat_interval_ms = DBG_HEARTBEAT_MS_DEFAULT,
            .log_pool_transitions  = false,
            .log_rejected_events   = false,
        },
    },
    
    // Runtime state
    .state = {
        .initialized = false,
        .calibrated  = false,
        .boot_time_us = 0,
    },
};

// =============================================================================
// RUNMODE NAMES (voor logging)
// =============================================================================

static const char* RUNMODE_NAMES[] = {
    [RUNMODE_PRODUCTION]    = "PRODUCTION",
    [RUNMODE_DEBUG_BYPASS]  = "DEBUG_BYPASS",
    [RUNMODE_RAW_CALIBRATE] = "RAW_CALIBRATE",
    [RUNMODE_SMOKE_TEST]    = "SMOKE_TEST",
    [RUNMODE_IMPULSE_TEST]  = "IMPULSE_TEST",
};

// =============================================================================
// INIT FUNCTION
// =============================================================================

void core0_config_init(void)
{
    // Kopieer defaults naar global
    memcpy(&g_cfg, &CONFIG_DEFAULTS, sizeof(core0_config_t));
    
    // Bereken derived fields
    g_cfg.hw.per_channel_hz = g_cfg.hw.sample_rate_hz / 2;
    g_cfg.hw.us_per_sample = 1e6f / (float)g_cfg.hw.per_channel_hz;
    
    // Record boot time
#ifdef ESP_PLATFORM
    g_cfg.state.boot_time_us = (uint32_t)esp_timer_get_time();
#else
    g_cfg.state.boot_time_us = 0;
#endif
    
    g_cfg.state.initialized = true;
    
    LOG_I("CORE0_RUNMODE numeric=%d", (int)CORE0_RUNMODE);
    LOG_I("Config initialized - Mode: %s", RUNMODE_NAMES[g_cfg.runmode]);
    LOG_I("FW Version: %s (%s)", CORE0_FW_VERSION_STR, CORE0_FW_BUILD_TAG);
}

// =============================================================================
// PRINT FUNCTION
// =============================================================================

void core0_config_print(void)
{
    LOG_I("========== CORE0 CONFIGURATION ==========");
    LOG_I("Run Mode: %s", RUNMODE_NAMES[g_cfg.runmode]);
    
    LOG_I("--- Hardware ---");
    LOG_I("  ADC: ch_A=%d, ch_B=%d, rate=%lu Hz",
          g_cfg.hw.adc_channel_A, g_cfg.hw.adc_channel_B,
          (unsigned long)g_cfg.hw.sample_rate_hz);
    LOG_I("  UART: port=%d, baud=%lu",
          g_cfg.hw.uart_port, (unsigned long)g_cfg.hw.uart_baud);
    
    LOG_I("--- Sensing ---");
    LOG_I("  Sensor A: low=%d, high=%d, baseline=%.0f",
          g_cfg.sensing.sensor_A.low_th,
          g_cfg.sensing.sensor_A.high_th,
          g_cfg.sensing.sensor_A.baseline_init);
    LOG_I("  Sensor B: low=%d, high=%d, baseline=%.0f",
          g_cfg.sensing.sensor_B.low_th,
          g_cfg.sensing.sensor_B.high_th,
          g_cfg.sensing.sensor_B.baseline_init);
    LOG_I("  Pool debounce: %lu us", (unsigned long)g_cfg.sensing.pool_min_dt_us);
    LOG_I("  Neutral edges: %s", g_cfg.sensing.include_neutral_edges ? "ON" : "OFF");
    LOG_I("  Auto-calibrate: %s", g_cfg.sensing.auto_calibrate ? "ON" : "OFF");
    
    LOG_I("--- Filter ---");
    LOG_I("  Gates: dvdt>=%d, mono>=%d, snr>=%d, fit<=%d",
          g_cfg.filter.dvdt_min_q15, g_cfg.filter.mono_min_q8,
          g_cfg.filter.snr_min_q8, g_cfg.filter.fit_err_max_q8);
    LOG_I("  Coalesce: %d ms, Target: %d ev/s",
          g_cfg.filter.coalesce_win_ms, g_cfg.filter.target_evps);
    
    LOG_I("--- Pair Detection ---");
    LOG_I("  Window: %lu us, Adaptive: %s",
          (unsigned long)g_cfg.pair.window_us,
          g_cfg.pair.adaptive ? "ON" : "OFF");
    
    LOG_I("--- Direction ---");
    LOG_I("  Enabled: %s, Phi_AB: %d deg, CW=A_first: %s",
          g_cfg.direction.enabled ? "ON" : "OFF",
          g_cfg.direction.phi_ab_deg,
          g_cfg.direction.cw_is_A_first ? "YES" : "NO");
    
    LOG_I("==========================================");
}

// =============================================================================
// VALIDATE FUNCTION
// =============================================================================

bool core0_config_validate(void)
{
    bool valid = true;
    
    // Check thresholds
    if (g_cfg.sensing.sensor_A.low_th >= g_cfg.sensing.sensor_A.high_th) {
        LOG_E("Invalid: A.low_th >= A.high_th");
        valid = false;
    }
    if (g_cfg.sensing.sensor_B.low_th >= g_cfg.sensing.sensor_B.high_th) {
        LOG_E("Invalid: B.low_th >= B.high_th");
        valid = false;
    }
    
    // Check sample rate
    if (g_cfg.hw.sample_rate_hz < 10000 || g_cfg.hw.sample_rate_hz > 1000000) {
        LOG_E("Invalid: sample_rate_hz out of range");
        valid = false;
    }
    
    // Check filter gates
    if (g_cfg.filter.mono_min_q8 > 250) {
        LOG_W("Warning: mono_min_q8 very high (%d), may reject all events",
              g_cfg.filter.mono_min_q8);
    }
    
    // Check target evps vs coalesce window
    float max_theoretical_evps = 1000.0f / g_cfg.filter.coalesce_win_ms * 2;
    if (g_cfg.filter.target_evps > max_theoretical_evps) {
        LOG_W("Warning: target_evps (%d) > theoretical max (%.0f) given coalesce window",
              g_cfg.filter.target_evps, max_theoretical_evps);
    }
    
    // Check pair window vs expected timing
    // At 100 RPM with 12 pole pairs: ~5000 us per sector
    // Pair window should be < sector time
    if (g_cfg.pair.window_us > 10000) {
        LOG_W("Warning: pair_window_us (%lu) very large, may cause false pairs",
              (unsigned long)g_cfg.pair.window_us);
    }
    
    // Check direction config consistency
    if (g_cfg.direction.enabled && g_cfg.direction.phi_ab_deg == 0) {
        LOG_W("Warning: direction enabled but phi_ab_deg=0 (sensors aligned?)");
    }
    
    if (valid) {
        LOG_I("Config validation: PASS");
    } else {
        LOG_E("Config validation: FAIL");
    }
    
    return valid;
}

// =============================================================================
// UPDATE THRESHOLDS (after calibration)
// =============================================================================

void core0_config_update_thresholds(
    int16_t a_min, int16_t a_max, float a_baseline,
    int16_t b_min, int16_t b_max, float b_baseline)
{
    float pct = g_cfg.sensing.calibrate_threshold_pct;
    
    // Store raw calibration results
    g_cfg.sensing.sensor_A.cal_min = a_min;
    g_cfg.sensing.sensor_A.cal_max = a_max;
    g_cfg.sensing.sensor_A.cal_baseline = a_baseline;
    g_cfg.sensing.sensor_B.cal_min = b_min;
    g_cfg.sensing.sensor_B.cal_max = b_max;
    g_cfg.sensing.sensor_B.cal_baseline = b_baseline;
    
    // Calculate thresholds
    float range_A_low  = a_baseline - a_min;
    float range_A_high = a_max - a_baseline;
    float range_B_low  = b_baseline - b_min;
    float range_B_high = b_max - b_baseline;
    
    g_cfg.sensing.sensor_A.low_th  = (int16_t)(a_baseline - pct * range_A_low);
    g_cfg.sensing.sensor_A.high_th = (int16_t)(a_baseline + pct * range_A_high);
    g_cfg.sensing.sensor_A.baseline_init = a_baseline;
    
    g_cfg.sensing.sensor_B.low_th  = (int16_t)(b_baseline - pct * range_B_low);
    g_cfg.sensing.sensor_B.high_th = (int16_t)(b_baseline + pct * range_B_high);
    g_cfg.sensing.sensor_B.baseline_init = b_baseline;
    
    g_cfg.state.calibrated = true;
    
    LOG_I("Thresholds updated from calibration:");
    LOG_I("  A: low=%d, high=%d (range %d-%d, base=%.0f)",
          g_cfg.sensing.sensor_A.low_th, g_cfg.sensing.sensor_A.high_th,
          a_min, a_max, a_baseline);
    LOG_I("  B: low=%d, high=%d (range %d-%d, base=%.0f)",
          g_cfg.sensing.sensor_B.low_th, g_cfg.sensing.sensor_B.high_th,
          b_min, b_max, b_baseline);
}

// =============================================================================
// CONFIG TO JSON (voor toekomstige serial interface)
// =============================================================================

int core0_config_to_json(char* buf, size_t buflen)
{
    int n = snprintf(buf, buflen,
        "{"
        "\"version\":\"%s\","
        "\"runmode\":\"%s\","
        "\"hw\":{"
            "\"sample_rate\":%lu,"
            "\"uart_baud\":%lu"
        "},"
        "\"sensing\":{"
            "\"A\":{\"low\":%d,\"high\":%d,\"base\":%.0f},"
            "\"B\":{\"low\":%d,\"high\":%d,\"base\":%.0f},"
            "\"pool_debounce_us\":%lu,"
            "\"neutral_edges\":%s"
        "},"
        "\"filter\":{"
            "\"dvdt_min\":%d,"
            "\"mono_min\":%d,"
            "\"snr_min\":%d,"
            "\"target_evps\":%d,"
            "\"coalesce_ms\":%d"
        "},"
        "\"pair\":{"
            "\"window_us\":%lu,"
            "\"adaptive\":%s"
        "},"
        "\"direction\":{"
            "\"enabled\":%s,"
            "\"phi_ab_deg\":%d,"
            "\"cw_A_first\":%s"
        "}"
        "}",
        CORE0_FW_VERSION_STR,
        RUNMODE_NAMES[g_cfg.runmode],
        (unsigned long)g_cfg.hw.sample_rate_hz,
        (unsigned long)g_cfg.hw.uart_baud,
        g_cfg.sensing.sensor_A.low_th,
        g_cfg.sensing.sensor_A.high_th,
        g_cfg.sensing.sensor_A.baseline_init,
        g_cfg.sensing.sensor_B.low_th,
        g_cfg.sensing.sensor_B.high_th,
        g_cfg.sensing.sensor_B.baseline_init,
        (unsigned long)g_cfg.sensing.pool_min_dt_us,
        g_cfg.sensing.include_neutral_edges ? "true" : "false",
        g_cfg.filter.dvdt_min_q15,
        g_cfg.filter.mono_min_q8,
        g_cfg.filter.snr_min_q8,
        g_cfg.filter.target_evps,
        g_cfg.filter.coalesce_win_ms,
        (unsigned long)g_cfg.pair.window_us,
        g_cfg.pair.adaptive ? "true" : "false",
        g_cfg.direction.enabled ? "true" : "false",
        g_cfg.direction.phi_ab_deg,
        g_cfg.direction.cw_is_A_first ? "true" : "false"
    );
    
    return n;
}

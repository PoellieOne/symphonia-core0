// ============================================================================
// RAW ADC SAMPLING MODE voor threshold calibratie
// ============================================================================
// Voeg dit toe aan core0_sensing.c of maak een aparte calibratie firmware
//
// Dit logt raw ADC samples naar UART in CSV formaat zodat je:
// 1. Baseline per sensor kunt bepalen
// 2. Min/max bereik tijdens magneet passage kunt meten
// 3. Optimale thresholds kunt berekenen
// ============================================================================

#define RAW_SAMPLE_MODE 0        // Zet op 1 voor calibratie, 0 voor normaal

#if RAW_SAMPLE_MODE

// Configuratie
#define RAW_SAMPLE_DECIMATION 100   // Log elke N samples (200kHz/100 = 2kHz output)
#define RAW_SAMPLE_DURATION_MS 5000 // 5 seconden capture

static uint32_t raw_sample_counter = 0;
static uint32_t raw_sample_start_ms = 0;
static bool raw_sampling_active = true;

// Statistieken tijdens sampling
static struct {
    int32_t sum_A, sum_B;
    int32_t min_A, max_A;
    int32_t min_B, max_B;
    uint32_t count;
} raw_stats = {
    .min_A = 4095, .max_A = 0,
    .min_B = 4095, .max_B = 0,
};

// Log functie - output naar UART als CSV
static void log_raw_sample(uint32_t t_ms, int raw_A, int raw_B, float base_A, float base_B) {
    // CSV formaat: timestamp_ms, raw_A, raw_B, baseline_A, baseline_B
    printf("%lu,%d,%d,%.1f,%.1f\n", t_ms, raw_A, raw_B, base_A, base_B);
}

// Update statistieken
static void update_raw_stats(int raw_A, int raw_B) {
    raw_stats.sum_A += raw_A;
    raw_stats.sum_B += raw_B;
    raw_stats.count++;

    if (raw_A < raw_stats.min_A) raw_stats.min_A = raw_A;
    if (raw_A > raw_stats.max_A) raw_stats.max_A = raw_A;
    if (raw_B < raw_stats.min_B) raw_stats.min_B = raw_B;
    if (raw_B > raw_stats.max_B) raw_stats.max_B = raw_B;
}

// Print finale statistieken en threshold suggesties
static void print_threshold_suggestions(void) {
    if (raw_stats.count == 0) return;

    float mean_A = (float)raw_stats.sum_A / raw_stats.count;
    float mean_B = (float)raw_stats.sum_B / raw_stats.count;

    // Bereken threshold suggesties
    // Low threshold: baseline - 60% van (baseline - min)
    // High threshold: baseline + 60% van (max - baseline)
    float range_A_low = mean_A - raw_stats.min_A;
    float range_A_high = raw_stats.max_A - mean_A;
    float range_B_low = mean_B - raw_stats.min_B;
    float range_B_high = raw_stats.max_B - mean_B;

    int suggested_A_low = (int)(mean_A - 0.6f * range_A_low);
    int suggested_A_high = (int)(mean_A + 0.6f * range_A_high);
    int suggested_B_low = (int)(mean_B - 0.6f * range_B_low);
    int suggested_B_high = (int)(mean_B + 0.6f * range_B_high);

    printf("\n");
    printf("// ============================================================\n");
    printf("// RAW ADC CALIBRATION RESULTS\n");
    printf("// ============================================================\n");
    printf("// Samples: %lu over %d ms\n", raw_stats.count, RAW_SAMPLE_DURATION_MS);
    printf("//\n");
    printf("// SENSOR A (ADC_CH_A / GPIO34):\n");
    printf("//   Range:    %ld - %ld\n", raw_stats.min_A, raw_stats.max_A);
    printf("//   Baseline: %.1f\n", mean_A);
    printf("//   Suggested thresholds:\n");
    printf("static int A_low_th  = %d;  // was 1500\n", suggested_A_low);
    printf("static int A_high_th = %d;  // was 2300\n", suggested_A_high);
    printf("//\n");
    printf("// SENSOR B (ADC_CH_B / GPIO35):\n");
    printf("//   Range:    %ld - %ld\n", raw_stats.min_B, raw_stats.max_B);
    printf("//   Baseline: %.1f\n", mean_B);
    printf("//   Suggested thresholds:\n");
    printf("static int B_low_th  = %d;  // was 700\n", suggested_B_low);
    printf("static int B_high_th = %d;  // was 1200\n", suggested_B_high);
    printf("// ============================================================\n");
    printf("\n");
}

// Aangepaste capture loop voor raw sampling
// Integreer dit in je bestaande capture_task of vervang tijdelijk
static void raw_sample_capture_loop(int raw_A, int raw_B, float baseline_A, float baseline_B) {
    if (!raw_sampling_active) return;

    // Check duration
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (raw_sample_start_ms == 0) {
        raw_sample_start_ms = now_ms;
        printf("t_ms,raw_A,raw_B,baseline_A,baseline_B\n");  // CSV header
    }

    if ((now_ms - raw_sample_start_ms) > RAW_SAMPLE_DURATION_MS) {
        if (raw_sampling_active) {
            raw_sampling_active = false;
            print_threshold_suggestions();
        }
        return;
    }

    // Update stats voor elke sample
    update_raw_stats(raw_A, raw_B);

    // Decimated logging
    raw_sample_counter++;
    if (raw_sample_counter >= RAW_SAMPLE_DECIMATION) {
        raw_sample_counter = 0;
        log_raw_sample(now_ms - raw_sample_start_ms, raw_A, raw_B, baseline_A, baseline_B);
    }
}

#endif // RAW_SAMPLE_MODE

// core0_link.h - Binary framing protocol (V1.1 - Config Integrated)
// =============================================================================
// MIGRATED: Nu gebruikt core0_config.h voor UART en queue parameters
// =============================================================================

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// FRAMING CONSTANTS (niet configurabel - protocol definitie)
// =============================================================================
#define SYNC_BYTE 0xA5
#define VER_1     0x1

// Max frame grootte (veilig)
#define FRAME_MAX_BYTES 64

// =============================================================================
// PACKET TYPES (protocol definitie)
// =============================================================================
typedef enum {
    PKT_EVENT16      = 0x0,  // TYPE=0 - Basis event (11 bytes payload)
    PKT_EVENT24      = 0x1,  // TYPE=1 - Extended event (19 bytes payload)
    PKT_SUMMARY16    = 0x2,  // TYPE=2 - Legacy summary (deprecated)
    PKT_SUMMARY24    = 0x3,  // TYPE=3 - Legacy summary (deprecated)
    PKT_FILTER_STATS = 0x4,  // TYPE=4 - Filter layer statistieken (19 bytes)
    PKT_LINK_STATS   = 0x5,  // TYPE=5 - Transport layer statistieken (19 bytes)
} pkt_type_t;

// =============================================================================
// TX QUEUE STRUCTURE
// =============================================================================
typedef struct {
    QueueHandle_t q;         // Queue van frames
    size_t item_size;        // sizeof(frame_t)
} link_tx_t;

// =============================================================================
// API FUNCTIONS
// =============================================================================

// Initialisatie
void link_init_tx(link_tx_t* tx, size_t queue_len);
bool link_send_frame(link_tx_t* tx, const uint8_t* frame, uint16_t n);
void start_uart_writer(link_tx_t* tx);
void core0_link_init(void);
link_tx_t* core0_link_get_tx(void);

// Helpers
uint16_t crc16_ccitt_false(const uint8_t* data, uint16_t n);
uint8_t mk_typever(pkt_type_t t, uint8_t ver);

// =============================================================================
// EVENT PACKETS
// =============================================================================
bool emit_event16(link_tx_t* tx,
    uint16_t dt_us, uint8_t flags0, uint8_t flags1,
    int16_t dvdt_q15, uint8_t mono_q8, uint8_t snr_q8,
    uint8_t score_q8, uint8_t seq);

bool emit_event24(link_tx_t* tx,
    uint16_t dt_us, uint32_t t_abs_us, uint8_t flags0, uint8_t flags1,
    int16_t dvdt_q15, uint8_t mono_q8, uint8_t snr_q8,
    uint8_t fit_err_q8, uint16_t rpm_hint_q, uint8_t score_q8, uint8_t seq);

// =============================================================================
// LEGACY SUMMARY PACKETS (deprecated - gebruik filter_stats/link_stats)
// =============================================================================
bool emit_summary16(link_tx_t* tx,
    uint16_t window_ms, uint16_t ev_emitted, uint16_t ev_dropped,
    uint8_t lvl_strong, uint8_t lvl_normal, uint8_t lvl_weak,
    uint16_t tx_bytes_in_window);

bool emit_summary24(link_tx_t* tx,
    uint16_t window_ms, uint16_t ev_emitted, uint16_t ev_dropped,
    uint8_t lvl_strong, uint8_t lvl_normal, uint8_t lvl_weak,
    uint8_t t_cross_rms_us_q, uint8_t ab_skew_p95_us_q,
    uint8_t queue_depth_max, uint8_t utilization_q,
    uint8_t rej_lowdvdt_u8, uint8_t rej_nonmono_u8, uint8_t rej_lowsnr_u8);

// =============================================================================
// FILTER LAYER STATISTICS (PKT_FILTER_STATS = 0x4)
// =============================================================================
bool emit_filter_stats(link_tx_t* tx,
    uint16_t window_ms,
    uint16_t events_emitted,
    uint16_t events_dropped,
    uint32_t events_considered,
    uint32_t events_rejected,
    uint8_t pct_strong,
    uint8_t pct_normal,
    uint8_t pct_weak,
    uint8_t drops_strong,
    uint8_t drops_normal,
    uint8_t drops_weak,
    float tokens_current,
    uint8_t coalesce_win_ms);

// =============================================================================
// LINK LAYER STATISTICS (PKT_LINK_STATS = 0x5)
// =============================================================================
bool emit_link_stats(link_tx_t* tx,
    uint16_t window_ms,
    uint16_t frames_sent,
    uint16_t frames_failed,
    uint32_t bytes_sent,
    uint16_t avg_write_us,
    uint8_t uart_blocked_count,
    uint8_t queue_fill_pct,
    uint16_t queue_high_water,
    uint8_t event16_count,
    uint8_t event24_count);

// =============================================================================
// FLAGS HELPERS
// =============================================================================
static inline uint8_t make_flags0(bool pair_flag, uint8_t qlevel, uint8_t polarity, uint8_t sensor) {
    return ((pair_flag ? 1 : 0) << 7)
         | ((qlevel & 0x3) << 5)
         | ((polarity & 0x1) << 4)
         | ((sensor & 0x1) << 3);
}

static inline uint8_t make_flags1(uint8_t from_pool, uint8_t to_pool, uint8_t dir_hint, uint8_t edge_kind) {
    return ((from_pool & 0x3) << 6)
         | ((to_pool & 0x3) << 4)
         | ((dir_hint & 0x3) << 2)
         | ((edge_kind & 0x3) << 0);
}

#ifdef __cplusplus
}
#endif

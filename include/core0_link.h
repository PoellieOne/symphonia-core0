// core0_link.h
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Framing
#define SYNC_BYTE            0xA5
#define VER_1                0x1

// UART config (pas aan op jouw pins/uart#)
#define UART_PORT      UART_NUM_0
#define UART_TX_GPIO   (GPIO_NUM_1)
#define UART_RX_GPIO   (GPIO_NUM_3)
#define UART_BAUD      115200
//#define UART_BAUD      460800  // was 115200

#define TX_BLOCKING   0  // 1 = blocking (tx=0), 0 = buffered (tx=8192)

// Packet TYPE (upper nibble van TYPE|VER)
typedef enum {
  PKT_EVENT16      = 0x0,  // TYPE=0 - Basis event (11 bytes payload)
  PKT_EVENT24      = 0x1,  // TYPE=1 - Extended event (19 bytes payload)
  PKT_SUMMARY16    = 0x2,  // TYPE=2 - Legacy summary (deprecated)
  PKT_SUMMARY24    = 0x3,  // TYPE=3 - Legacy summary (deprecated)
  PKT_FILTER_STATS = 0x4,  // TYPE=4 - Filter layer statistieken (19 bytes)
  PKT_LINK_STATS   = 0x5,  // TYPE=5 - Transport layer statistieken (19 bytes)
} pkt_type_t;

// Max frame grootte (veilig)
#define FRAME_MAX_BYTES      64

// TX queue
typedef struct {
  QueueHandle_t q;     // queue van pointers naar heap-frames of fixed buffers
  size_t item_size;    // sizeof(pointer) of sizeof(fixed buffer)
} link_tx_t;

// API
void   link_init_tx(link_tx_t* tx, size_t queue_len);
bool   link_send_frame(link_tx_t* tx, const uint8_t* frame, uint16_t n);
void   start_uart_writer(link_tx_t* tx);
void   core0_link_init(void);
link_tx_t* core0_link_get_tx(void);

// Helpers: frame builders
uint16_t crc16_ccitt_false(const uint8_t* data, uint16_t n);
uint8_t  mk_typever(pkt_type_t t, uint8_t ver);

// ============================================================================
// EVENT PACKETS: Magnetische detectie events
// ============================================================================

bool emit_event16(link_tx_t* tx,
                  uint16_t dt_us, uint8_t flags0, uint8_t flags1,
                  int16_t dvdt_q15, uint8_t mono_q8, uint8_t snr_q8,
                  uint8_t score_q8, uint8_t seq);

bool emit_event24(link_tx_t* tx,
                  uint16_t dt_us, uint32_t t_abs_us, uint8_t flags0, uint8_t flags1,
                  int16_t dvdt_q15, uint8_t mono_q8, uint8_t snr_q8,
                  uint8_t fit_err_q8, uint16_t rpm_hint_q, uint8_t score_q8, uint8_t seq);

// ============================================================================
// LEGACY SUMMARY PACKETS (deprecated - gebruik filter_stats/link_stats)
// ============================================================================

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

// ============================================================================
// FILTER LAYER STATISTICS (PKT_FILTER_STATS = 0x4)
// ============================================================================
// Rapporteert detectie-performance en quality metrics van de filter laag
//
// Payload layout (19 bytes):
//   [0-1]   window_ms         - Measurement window in milliseconds
//   [2-3]   events_emitted    - Events successfully emitted (passed filter + backpressure)
//   [4-5]   events_dropped    - Events dropped by backpressure (token bucket)
//   [6-7]   events_considered - Total candidates offered to filter (low 16 bits)
//   [8-9]   events_rejected   - Candidates rejected by quality gates
//   [10]    pct_strong        - Percentage strong quality (0-255 = 0-100%)
//   [11]    pct_normal        - Percentage normal quality
//   [12]    pct_weak          - Percentage weak quality
//   [13]    drops_strong      - Count of strong events dropped (backpressure extreme)
//   [14]    drops_normal      - Count of normal events dropped
//   [15]    drops_weak        - Count of weak events dropped
//   [16]    tokens_q8         - Current token bucket level (signed, -128 to +127, scale×10)
//   [17]    coalesce_win_ms   - Current coalescing window size
//   [18]    reserved          - Future use
// ============================================================================

bool emit_filter_stats(link_tx_t* tx,
                       uint16_t window_ms,
                       uint16_t events_emitted,
                       uint16_t events_dropped,
                       uint32_t events_considered,
                       uint32_t events_rejected,
                       uint8_t  pct_strong,
                       uint8_t  pct_normal,
                       uint8_t  pct_weak,
                       uint8_t  drops_strong,
                       uint8_t  drops_normal,
                       uint8_t  drops_weak,
                       float    tokens_current,
                       uint8_t  coalesce_win_ms);

// ============================================================================
// LINK LAYER STATISTICS (PKT_LINK_STATS = 0x5)
// ============================================================================
// Rapporteert transport-performance en UART health metrics
//
// Payload layout (19 bytes):
//   [0-1]   window_ms          - Measurement window in milliseconds
//   [2-3]   frames_sent        - UART frames successfully written
//   [4-5]   frames_failed      - UART write failures
//   [6-9]   bytes_sent         - Total bytes transmitted (32-bit)
//   [10-11] avg_write_us       - Average UART write latency in microseconds
//   [12]    uart_blocked_count - Number of partial writes (blocking detected)
//   [13]    queue_fill_pct     - TX queue fill percentage (0-255 = 0-100%)
//   [14-15] queue_high_water   - Maximum queue depth seen in window
//   [16]    event16_count      - Number of EVENT16 packets sent (low byte)
//   [17]    event24_count      - Number of EVENT24 packets sent (low byte)
//   [18]    reserved           - Future use
// ============================================================================

bool emit_link_stats(link_tx_t* tx,
                     uint16_t window_ms,
                     uint16_t frames_sent,
                     uint16_t frames_failed,
                     uint32_t bytes_sent,
                     uint16_t avg_write_us,
                     uint8_t  uart_blocked_count,
                     uint8_t  queue_fill_pct,
                     uint16_t queue_high_water,
                     uint8_t  event16_count,
                     uint8_t  event24_count);

// ============================================================================
// FLAGS HELPERS
// ============================================================================

static inline uint8_t make_flags0(bool pair_flag, uint8_t qlevel, uint8_t polarity, uint8_t sensor) {
  // qlevel 0..3 ; polarity 0=−,1=+ ; sensor 0=A,1=B
  return  ((pair_flag ? 1 : 0) << 7)
        | ((qlevel & 0x3) << 5)
        | ((polarity & 0x1) << 4)
        | ((sensor   & 0x1) << 3);
}
static inline uint8_t make_flags1(uint8_t from_pool, uint8_t to_pool, uint8_t dir_hint, uint8_t edge_kind) {
  // pools 0..3 ; dir_hint 0..3
  return  ((from_pool & 0x3) << 6)
        | ((to_pool   & 0x3) << 4)
        | ((dir_hint  & 0x3) << 2)
        | ((edge_kind & 0x3) << 0);  // 2 bits voor edge_kind
}

#ifdef __cplusplus
}
#endif

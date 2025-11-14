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

#define TX_BLOCKING   0  // 1 = blocking (tx=0), 0 = buffered (tx=8192)

// Packet TYPE (upper nibble van TYPE|VER)
typedef enum {
  PKT_EVENT16   = 0x0,  // TYPE=0
  PKT_EVENT24   = 0x1,  // TYPE=1
  PKT_SUMMARY16 = 0x2,  // TYPE=2
  PKT_SUMMARY24 = 0x3,  // TYPE=3
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

// Helpers: frame builders
uint16_t crc16_ccitt_false(const uint8_t* data, uint16_t n);
uint8_t  mk_typever(pkt_type_t t, uint8_t ver);

// Specialized emitters (bouwen payload + frame en enqueuen)
bool emit_event16(link_tx_t* tx,
                  uint16_t dt_us, uint8_t flags0, uint8_t flags1,
                  int16_t dvdt_q15, uint8_t mono_q8, uint8_t snr_q8,
                  uint8_t score_q8, uint8_t seq);

bool emit_event24(link_tx_t* tx,
                  uint16_t dt_us, uint32_t t_abs_us, uint8_t flags0, uint8_t flags1,
                  int16_t dvdt_q15, uint8_t mono_q8, uint8_t snr_q8,
                  uint8_t fit_err_q8, uint16_t rpm_hint_q, uint8_t score_q8, uint8_t seq);

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

static inline uint8_t make_flags0(bool pair_flag, uint8_t qlevel, uint8_t polarity, uint8_t sensor) {
  // qlevel 0..3 ; polarity 0=−,1=+ ; sensor 0=A,1=B
  return  ((pair_flag ? 1 : 0) << 7)
        | ((qlevel & 0x3) << 5)
        | ((polarity & 0x1) << 4)
        | ((sensor   & 0x1) << 3);
}
static inline uint8_t make_flags1(uint8_t from_pool, uint8_t to_pool, uint8_t dir_hint) {
  // pools 0..3 ; dir_hint 0..3
  return  ((from_pool & 0x3) << 6)
        | ((to_pool   & 0x3) << 4)
        | ((dir_hint  & 0x3) << 2);
}

#ifdef __cplusplus
}
#endif

// core0_link.c
#pragma message "UART_PORT=UART_NUM_0 verwacht, TX=GPIO1, RX=GPIO3"
#include "core0_link.h"
#include <string.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

typedef struct {
  link_tx_t* ltx;
  volatile uint32_t tx_bytes_window;
  volatile uint16_t emitted_window;
  volatile uint16_t dropped_window;
  uint32_t window_ms;
  TickType_t last_summary_tick;
} uart_ctx_t;

static uart_ctx_t g_uart = {0};

// --- TX queue (we enqueuen complete frame kopieën; voor zero-copy kun je ringbuf nemen)
typedef struct {
  uint16_t len;
  uint8_t  data[FRAME_MAX_BYTES];
} frame_t;

// --- CRC16-CCITT(FALSE): poly 0x1021, init 0xFFFF, refin/refout=false, xorout 0
uint16_t crc16_ccitt_false(const uint8_t* data, uint16_t n) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i=0;i<n;i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b=0;b<8;b++) {
      crc = (crc & 0x8000) ? (crc<<1) ^ 0x1021 : (crc<<1);
    }
  }
  return crc;
}

uint8_t mk_typever(pkt_type_t t, uint8_t ver) {
  return (uint8_t)((((uint8_t)t) << 4) | (ver & 0x0F));
}

void link_init_tx(link_tx_t* tx, size_t queue_len) {
  tx->item_size = sizeof(frame_t);
  tx->q = xQueueCreate(queue_len, tx->item_size);
}

bool link_send_frame(link_tx_t* tx, const uint8_t* frame, uint16_t n) {
  if (!tx || !tx->q || n > FRAME_MAX_BYTES) return false;
  frame_t f = { .len = n };
  memcpy(f.data, frame, n);
  return xQueueSend(tx->q, &f, 0) == pdTRUE;
}

// --- Byte writers LE
static inline void w8(uint8_t* b, uint16_t* o, uint8_t v){ b[(*o)++] = v; }
static inline void w16le(uint8_t* b, uint16_t* o, uint16_t v){ b[(*o)++] = (uint8_t)(v & 0xFF); b[(*o)++] = (uint8_t)(v>>8); }
static inline void w32le(uint8_t* b, uint16_t* o, uint32_t v){ w16le(b,o,(uint16_t)(v & 0xFFFF)); w16le(b,o,(uint16_t)(v>>16)); }

// --- Frame builder (header + payload + CRC)
static bool build_and_send(link_tx_t* tx, pkt_type_t t, const uint8_t* payload, uint8_t plen) {
  uint8_t frame[FRAME_MAX_BYTES];
  uint16_t off = 0;
  w8(frame,&off, SYNC_BYTE);
  w8(frame,&off, mk_typever(t, VER_1));
  w8(frame,&off, plen);
  // CRC over TYPE|VER + LEN + PAYLOAD
  uint8_t hdr_crc_buf[1+1+FRAME_MAX_BYTES]; // small temp
  hdr_crc_buf[0] = mk_typever(t, VER_1);
  hdr_crc_buf[1] = plen;
  memcpy(&hdr_crc_buf[2], payload, plen);
  uint16_t crc = crc16_ccitt_false(hdr_crc_buf, (uint16_t)(2+plen));
  memcpy(&frame[off], payload, plen); off += plen;
  w16le(frame,&off, crc);
  return link_send_frame(tx, frame, off);
}

// --- PACKETS ---

bool emit_event16(link_tx_t* tx,
                  uint16_t dt_us, uint8_t flags0, uint8_t flags1,
                  int16_t dvdt_q15, uint8_t mono_q8, uint8_t snr_q8,
                  uint8_t score_q8, uint8_t seq)
{
  uint8_t p[11]; uint16_t o=0;
  w16le(p,&o, dt_us);
  w8(p,&o, flags0);
  w8(p,&o, flags1);
  w16le(p,&o, (uint16_t)dvdt_q15);
  w8(p,&o, mono_q8);
  w8(p,&o, snr_q8);
  w8(p,&o, score_q8);
  w8(p,&o, seq);
  w8(p,&o, 0); // reserved
  return build_and_send(tx, PKT_EVENT16, p, 11);
}

bool emit_event24(link_tx_t* tx,
                  uint16_t dt_us, uint32_t t_abs_us, uint8_t flags0, uint8_t flags1,
                  int16_t dvdt_q15, uint8_t mono_q8, uint8_t snr_q8,
                  uint8_t fit_err_q8, uint16_t rpm_hint_q, uint8_t score_q8, uint8_t seq)
{
  uint8_t p[19]; uint16_t o=0;
  w16le(p,&o, dt_us);
  w32le(p,&o, t_abs_us);
  w8(p,&o, flags0);
  w8(p,&o, flags1);
  w16le(p,&o, (uint16_t)dvdt_q15);
  w8(p,&o, mono_q8);
  w8(p,&o, snr_q8);
  w8(p,&o, fit_err_q8);
  w16le(p,&o, rpm_hint_q);
  w8(p,&o, score_q8);
  w8(p,&o, seq);
  w16le(p,&o, 0); // reserved 2B
  return build_and_send(tx, PKT_EVENT24, p, 19);
}

bool emit_summary16(link_tx_t* tx,
                    uint16_t window_ms, uint16_t ev_emitted, uint16_t ev_dropped,
                    uint8_t lvl_strong, uint8_t lvl_normal, uint8_t lvl_weak,
                    uint16_t tx_bytes_in_window)
{
  uint8_t p[11]; uint16_t o=0;
  w16le(p,&o, window_ms);
  w16le(p,&o, ev_emitted);
  w16le(p,&o, ev_dropped);
  w8(p,&o,  lvl_strong);
  w8(p,&o,  lvl_normal);
  w8(p,&o,  lvl_weak);
  w16le(p,&o, tx_bytes_in_window);
  return build_and_send(tx, PKT_SUMMARY16, p, 11);
}

bool emit_summary24(link_tx_t* tx,
                    uint16_t window_ms, uint16_t ev_emitted, uint16_t ev_dropped,
                    uint8_t lvl_strong, uint8_t lvl_normal, uint8_t lvl_weak,
                    uint8_t t_cross_rms_us_q, uint8_t ab_skew_p95_us_q,
                    uint8_t queue_depth_max, uint8_t utilization_q,
                    uint8_t rej_lowdvdt_u8, uint8_t rej_nonmono_u8, uint8_t rej_lowsnr_u8)
{
  uint8_t p[19]; uint16_t o=0;
  w16le(p,&o, window_ms);
  w16le(p,&o, ev_emitted);
  w16le(p,&o, ev_dropped);
  w8(p,&o,  lvl_strong);
  w8(p,&o,  lvl_normal);
  w8(p,&o,  lvl_weak);
  w8(p,&o,  t_cross_rms_us_q);
  w8(p,&o,  ab_skew_p95_us_q);
  w8(p,&o,  queue_depth_max);
  w8(p,&o,  utilization_q);
  w8(p,&o,  rej_lowdvdt_u8);
  w8(p,&o,  rej_nonmono_u8);
  w8(p,&o,  rej_lowsnr_u8);
  w8(p,&o,  0); w8(p,&o,0); w8(p,&o,0); // reserved 3B
  return build_and_send(tx, PKT_SUMMARY24, p, 19);
}

static void uart_ascii_ping(void){
  const char *msg = "BINSTART\r\n";
  uart_write_bytes(UART_PORT, msg, strlen(msg));
  // belangrijk in buffered TX: even wachten tot de bytes echt zijn uitgezonden
  uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(50));
  uart_write_bytes(UART_PORT, msg, strlen(msg));
  uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(50));
}

static void uart_writer_task(void* arg) {
  //esp_task_wdt_add(NULL);
  ESP_ERROR_CHECK( esp_task_wdt_add(NULL) );   // alleen writer aanmelden
  const TickType_t TQ = pdMS_TO_TICKS(10);          // queue timeout
  const TickType_t FEED_EVERY = pdMS_TO_TICKS(100); // elke ~100 ms WDT resetten
  TickType_t last_feed = xTaskGetTickCount();

  g_uart.window_ms = 500; // summary elke 500 ms; was 100
  g_uart.last_summary_tick = xTaskGetTickCount();

  // init UART
  uart_config_t cfg = {
      .baud_rate = UART_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };
  uart_param_config(UART_PORT, &cfg);
  uart_set_pin(UART_PORT, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // In buffered modus TX buffer groot, geen RX nodig
  const int RXBUF = 256;      // klein maar ≠ 0
  const int TXBUF = 8192;     // of groter
  #if TX_BLOCKING
    esp_err_t err = uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0);
  #else
    esp_err_t err = uart_driver_install(UART_PORT, RXBUF, TXBUF, 0, NULL, 0);
  #endif
  ESP_ERROR_CHECK(err);

  // eenvoudige window-timer
  const TickType_t ticks_per_window = pdMS_TO_TICKS(g_uart.window_ms);

  // leesbaar in idf.py monitor
  uart_ascii_ping();

  for (;;) {
    // 1) Probeer een frame te pakken (block kort)
    frame_t f;
    if (xQueueReceive(g_uart.ltx->q, &f, TQ) == pdTRUE) {
      int n = uart_write_bytes(UART_PORT, (const char*)f.data, f.len);
      if (n > 0) {
        g_uart.tx_bytes_window += (uint32_t)n;
        g_uart.emitted_window  += 1;
      } else {
        g_uart.dropped_window  += 1;
      }
    }

    // 2) Periodieke Summary16 uitsturen
    TickType_t now = xTaskGetTickCount();

    if ((now - g_uart.last_summary_tick) >= ticks_per_window) {
      // heel eenvoudige levels: we hebben ze nog niet → 0
      emit_summary16(g_uart.ltx,
                     (uint16_t)g_uart.window_ms,
                     g_uart.emitted_window,
                     g_uart.dropped_window,
                     /*lvl_strong*/0, /*lvl_normal*/0, /*lvl_weak*/0,
                     (uint16_t)(g_uart.tx_bytes_window > 65535 ? 65535 : g_uart.tx_bytes_window));
      g_uart.tx_bytes_window = 0;
      g_uart.emitted_window  = 0;
      g_uart.dropped_window  = 0;
      g_uart.last_summary_tick = now;
    }

    // 3) Feed de WDT (of haal de task uit de WDT als je die niet wilt gebruiken)
    if ((now - last_feed) >= FEED_EVERY) {
      ESP_ERROR_CHECK( esp_task_wdt_reset() );
      last_feed = now;
    }
  }
}

// Call once at startup (bijv. in app_main)
void start_uart_writer(link_tx_t* tx) {
  g_uart.ltx = tx;
  xTaskCreatePinnedToCore(uart_writer_task, "uart_writer", 4096, NULL, 5, NULL, tskNO_AFFINITY);
}

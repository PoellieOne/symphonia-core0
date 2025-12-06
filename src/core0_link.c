// core0_link.c - Binary framing protocol (V1.1 - Config Integrated)
// =============================================================================
// MIGRATED: Nu gebruikt core0_config.h voor alle configureerbare parameters
// =============================================================================

#include "core0_link.h"
#include "core0_config.h"

#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

// =============================================================================
// PRIVATE TYPES
// =============================================================================

typedef struct {
    uint16_t len;
    uint8_t data[FRAME_MAX_BYTES];
} frame_t;

typedef struct {
    link_tx_t* ltx;
    volatile uint32_t tx_bytes_window;
    volatile uint16_t frames_sent_window;
    volatile uint16_t frames_failed_window;
    volatile uint32_t uart_write_total_us;
    volatile uint32_t uart_write_calls;
    volatile uint32_t uart_write_blocked;
    volatile uint16_t queue_high_water;
    uint32_t window_ms;
} uart_ctx_t;

// =============================================================================
// PRIVATE DATA
// =============================================================================

static link_tx_t g_tx;
static uart_ctx_t g_uart = {0};

static struct {
    uint32_t event16_sent;
    uint32_t event24_sent;
    uint32_t summary16_sent;
    uint32_t summary24_sent;
    uint32_t filter_stats_sent;
    uint32_t link_stats_sent;
} g_frame_stats = {0};

// =============================================================================
// CRC16-CCITT(FALSE)
// =============================================================================

uint16_t crc16_ccitt_false(const uint8_t* data, uint16_t n) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < n; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

uint8_t mk_typever(pkt_type_t t, uint8_t ver) {
    return (uint8_t)((((uint8_t)t) << 4) | (ver & 0x0F));
}

// =============================================================================
// TX QUEUE FUNCTIONS
// =============================================================================

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

// =============================================================================
// BYTE WRITERS (Little Endian)
// =============================================================================

static inline void w8(uint8_t* b, uint16_t* o, uint8_t v) { 
    b[(*o)++] = v; 
}

static inline void w16le(uint8_t* b, uint16_t* o, uint16_t v) { 
    b[(*o)++] = (uint8_t)(v & 0xFF); 
    b[(*o)++] = (uint8_t)(v >> 8); 
}

static inline void w32le(uint8_t* b, uint16_t* o, uint32_t v) { 
    w16le(b, o, (uint16_t)(v & 0xFFFF)); 
    w16le(b, o, (uint16_t)(v >> 16)); 
}

// =============================================================================
// FRAME BUILDER
// =============================================================================

static bool build_and_send(link_tx_t* tx, pkt_type_t t, const uint8_t* payload, uint8_t plen) {
    uint8_t frame[FRAME_MAX_BYTES];
    uint16_t off = 0;
    
    w8(frame, &off, SYNC_BYTE);
    w8(frame, &off, mk_typever(t, VER_1));
    w8(frame, &off, plen);
    
    // CRC over TYPE|VER + LEN + PAYLOAD
    uint8_t hdr_crc_buf[1 + 1 + FRAME_MAX_BYTES];
    hdr_crc_buf[0] = mk_typever(t, VER_1);
    hdr_crc_buf[1] = plen;
    memcpy(&hdr_crc_buf[2], payload, plen);
    uint16_t crc = crc16_ccitt_false(hdr_crc_buf, (uint16_t)(2 + plen));
    
    memcpy(&frame[off], payload, plen);
    off += plen;
    w16le(frame, &off, crc);
    
    return link_send_frame(tx, frame, off);
}

// =============================================================================
// EVENT PACKETS
// =============================================================================

bool emit_event16(link_tx_t* tx,
    uint16_t dt_us, uint8_t flags0, uint8_t flags1,
    int16_t dvdt_q15, uint8_t mono_q8, uint8_t snr_q8,
    uint8_t score_q8, uint8_t seq)
{
    uint8_t p[11]; uint16_t o = 0;
    w16le(p, &o, dt_us);
    w8(p, &o, flags0);
    w8(p, &o, flags1);
    w16le(p, &o, (uint16_t)dvdt_q15);
    w8(p, &o, mono_q8);
    w8(p, &o, snr_q8);
    w8(p, &o, score_q8);
    w8(p, &o, seq);
    w8(p, &o, 0);  // reserved
    bool ok = build_and_send(tx, PKT_EVENT16, p, 11);
    if (ok) g_frame_stats.event16_sent++;
    return ok;
}

bool emit_event24(link_tx_t* tx,
    uint16_t dt_us, uint32_t t_abs_us, uint8_t flags0, uint8_t flags1,
    int16_t dvdt_q15, uint8_t mono_q8, uint8_t snr_q8,
    uint8_t fit_err_q8, uint16_t rpm_hint_q, uint8_t score_q8, uint8_t seq)
{
    uint8_t p[19]; uint16_t o = 0;
    w16le(p, &o, dt_us);
    w32le(p, &o, t_abs_us);
    w8(p, &o, flags0);
    w8(p, &o, flags1);
    w16le(p, &o, (uint16_t)dvdt_q15);
    w8(p, &o, mono_q8);
    w8(p, &o, snr_q8);
    w8(p, &o, fit_err_q8);
    w16le(p, &o, rpm_hint_q);
    w8(p, &o, score_q8);
    w8(p, &o, seq);
    w16le(p, &o, 0);  // reserved 2B
    bool ok = build_and_send(tx, PKT_EVENT24, p, 19);
    if (ok) g_frame_stats.event24_sent++;
    return ok;
}

// =============================================================================
// LEGACY SUMMARY PACKETS
// =============================================================================

bool emit_summary16(link_tx_t* tx,
    uint16_t window_ms, uint16_t ev_emitted, uint16_t ev_dropped,
    uint8_t lvl_strong, uint8_t lvl_normal, uint8_t lvl_weak,
    uint16_t tx_bytes_in_window)
{
    uint8_t p[11]; uint16_t o = 0;
    w16le(p, &o, window_ms);
    w16le(p, &o, ev_emitted);
    w16le(p, &o, ev_dropped);
    w8(p, &o, lvl_strong);
    w8(p, &o, lvl_normal);
    w8(p, &o, lvl_weak);
    w16le(p, &o, tx_bytes_in_window);
    bool ok = build_and_send(tx, PKT_SUMMARY16, p, 11);
    if (ok) g_frame_stats.summary16_sent++;
    return ok;
}

bool emit_summary24(link_tx_t* tx,
    uint16_t window_ms, uint16_t ev_emitted, uint16_t ev_dropped,
    uint8_t lvl_strong, uint8_t lvl_normal, uint8_t lvl_weak,
    uint8_t t_cross_rms_us_q, uint8_t ab_skew_p95_us_q,
    uint8_t queue_depth_max, uint8_t utilization_q,
    uint8_t rej_lowdvdt_u8, uint8_t rej_nonmono_u8, uint8_t rej_lowsnr_u8)
{
    uint8_t p[19]; uint16_t o = 0;
    w16le(p, &o, window_ms);
    w16le(p, &o, ev_emitted);
    w16le(p, &o, ev_dropped);
    w8(p, &o, lvl_strong);
    w8(p, &o, lvl_normal);
    w8(p, &o, lvl_weak);
    w8(p, &o, t_cross_rms_us_q);
    w8(p, &o, ab_skew_p95_us_q);
    w8(p, &o, queue_depth_max);
    w8(p, &o, utilization_q);
    w8(p, &o, rej_lowdvdt_u8);
    w8(p, &o, rej_nonmono_u8);
    w8(p, &o, rej_lowsnr_u8);
    w8(p, &o, 0); w8(p, &o, 0); w8(p, &o, 0);  // reserved 3B
    bool ok = build_and_send(tx, PKT_SUMMARY24, p, 19);
    if (ok) g_frame_stats.summary24_sent++;
    return ok;
}

// =============================================================================
// FILTER STATS
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
    uint8_t coalesce_win_ms)
{
    uint8_t p[19]; uint16_t o = 0;
    w16le(p, &o, window_ms);
    w16le(p, &o, events_emitted);
    w16le(p, &o, events_dropped);
    w16le(p, &o, (uint16_t)(events_considered & 0xFFFF));
    w16le(p, &o, (uint16_t)(events_rejected & 0xFFFF));
    w8(p, &o, pct_strong);
    w8(p, &o, pct_normal);
    w8(p, &o, pct_weak);
    w8(p, &o, drops_strong);
    w8(p, &o, drops_normal);
    w8(p, &o, drops_weak);
    
    // Encode tokens als signed int8 (×10)
    int tokens_scaled = (int)(tokens_current * 10.0f);
    if (tokens_scaled < -128) tokens_scaled = -128;
    if (tokens_scaled > 127) tokens_scaled = 127;
    w8(p, &o, (uint8_t)tokens_scaled);
    
    w8(p, &o, coalesce_win_ms);
    w8(p, &o, 0);  // reserved
    bool ok = build_and_send(tx, PKT_FILTER_STATS, p, 19);
    if (ok) g_frame_stats.filter_stats_sent++;
    return ok;
}

// =============================================================================
// LINK STATS
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
    uint8_t event24_count)
{
    uint8_t p[19]; uint16_t o = 0;
    w16le(p, &o, window_ms);
    w16le(p, &o, frames_sent);
    w16le(p, &o, frames_failed);
    w32le(p, &o, bytes_sent);
    w16le(p, &o, avg_write_us);
    w8(p, &o, uart_blocked_count);
    w8(p, &o, queue_fill_pct);
    w16le(p, &o, queue_high_water);
    w8(p, &o, event16_count);
    w8(p, &o, event24_count);
    w8(p, &o, 0);  // reserved
    bool ok = build_and_send(tx, PKT_LINK_STATS, p, 19);
    if (ok) g_frame_stats.link_stats_sent++;
    return ok;
}

// =============================================================================
// UART ASCII PING
// =============================================================================

static void uart_ascii_ping(void) {
    if (!CFG_LINK.send_ascii_ping) return;
    
    const char *msg = "BINSTART\r\n";
    uart_write_bytes(CFG_HW.uart_port, msg, strlen(msg));
    uart_wait_tx_done(CFG_HW.uart_port, pdMS_TO_TICKS(50));
    uart_write_bytes(CFG_HW.uart_port, msg, strlen(msg));
    uart_wait_tx_done(CFG_HW.uart_port, pdMS_TO_TICKS(50));
}

// =============================================================================
// UART WRITER TASK
// =============================================================================

static void uart_writer_task(void* arg) {
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    
    const TickType_t TQ = pdMS_TO_TICKS(10);
    const TickType_t FEED_EVERY = pdMS_TO_TICKS(100);
    TickType_t last_feed = xTaskGetTickCount();
    
    g_uart.window_ms = CFG_LINK.link_stats_interval_ms;
    
    // UART configuratie via CFG_HW
    uart_config_t cfg = {
        .baud_rate = CFG_HW.uart_baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(CFG_HW.uart_port, &cfg);
    uart_set_pin(CFG_HW.uart_port, CFG_HW.uart_tx_gpio, CFG_HW.uart_rx_gpio, 
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // Buffer sizes via CFG_LINK
    esp_err_t err = uart_driver_install(
        CFG_HW.uart_port,
        CFG_LINK.rx_buffer_size,
        CFG_LINK.tx_buffer_size,
        0, NULL, 0
    );
    ESP_ERROR_CHECK(err);
    
    // Link stats timing met offset
    uint32_t last_stats_ms = (uint32_t)esp_timer_get_time() / 1000 + CFG_LINK.link_stats_offset_ms;
    
    uart_ascii_ping();
    
    for (;;) {
        // Track queue depth
        UBaseType_t depth_before = uxQueueMessagesWaiting(g_uart.ltx->q);
        if (depth_before > g_uart.queue_high_water) {
            g_uart.queue_high_water = (uint16_t)depth_before;
        }
        
        // Receive frame from queue
        frame_t f;
        if (xQueueReceive(g_uart.ltx->q, &f, TQ) == pdTRUE) {
            uint32_t start_us = (uint32_t)esp_timer_get_time();
            int n = uart_write_bytes(CFG_HW.uart_port, (const char*)f.data, f.len);
            uint32_t end_us = (uint32_t)esp_timer_get_time();
            uint32_t duration_us = end_us - start_us;
            
            g_uart.uart_write_total_us += duration_us;
            g_uart.uart_write_calls++;
            
            if (n > 0) {
                g_uart.tx_bytes_window += (uint32_t)n;
                g_uart.frames_sent_window++;
                if (n < f.len) {
                    g_uart.uart_write_blocked++;
                }
            } else {
                g_uart.frames_failed_window++;
            }
        }
        
        // Link stats reporting
        uint32_t now_ms = (uint32_t)esp_timer_get_time() / 1000;
        uint32_t elapsed_ms = now_ms - last_stats_ms;
        
        if (elapsed_ms >= CFG_LINK.link_stats_interval_ms) {
            uint16_t avg_write_us = 0;
            if (g_uart.uart_write_calls > 0) {
                avg_write_us = (uint16_t)(g_uart.uart_write_total_us / g_uart.uart_write_calls);
            }
            
            UBaseType_t queue_waiting = uxQueueMessagesWaiting(g_uart.ltx->q);
            uint8_t queue_fill_pct = (uint8_t)((queue_waiting * 255) / CFG_LINK.tx_queue_len);
            
            if (CFG_LINK.use_link_stats) {
                emit_link_stats(g_uart.ltx,
                    (uint16_t)elapsed_ms,
                    g_uart.frames_sent_window,
                    g_uart.frames_failed_window,
                    g_uart.tx_bytes_window,
                    avg_write_us,
                    (uint8_t)g_uart.uart_write_blocked,
                    queue_fill_pct,
                    g_uart.queue_high_water,
                    (uint8_t)(g_frame_stats.event16_sent & 0xFF),
                    (uint8_t)(g_frame_stats.event24_sent & 0xFF));
            }
            
            // Reset counters
            g_uart.tx_bytes_window = 0;
            g_uart.frames_sent_window = 0;
            g_uart.frames_failed_window = 0;
            g_uart.uart_write_total_us = 0;
            g_uart.uart_write_calls = 0;
            g_uart.uart_write_blocked = 0;
            g_uart.queue_high_water = 0;
            g_frame_stats.event16_sent = 0;
            g_frame_stats.event24_sent = 0;
            
            last_stats_ms = now_ms;
        }
        
        // WDT feed
        TickType_t now_tick = xTaskGetTickCount();
        if ((now_tick - last_feed) >= FEED_EVERY) {
            ESP_ERROR_CHECK(esp_task_wdt_reset());
            last_feed = now_tick;
        }
    }
}

// =============================================================================
// PUBLIC INIT FUNCTIONS
// =============================================================================

void start_uart_writer(link_tx_t* tx) {
    g_uart.ltx = tx;
    xTaskCreatePinnedToCore(uart_writer_task, "uart_writer", 4096, NULL, 5, NULL, tskNO_AFFINITY);
}

void core0_link_init(void) {
    link_init_tx(&g_tx, CFG_LINK.tx_queue_len);
    start_uart_writer(&g_tx);
}

link_tx_t* core0_link_get_tx(void) { 
    return &g_tx; 
}

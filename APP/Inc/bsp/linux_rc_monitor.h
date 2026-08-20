/*
 * linux_rc_monitor.h -- Passive USART6 monitor for the S7.6 Linux virtual-RC frame.
 *
 * The monitor validates and exposes received 44-byte frames for UART4 diagnostics.
 * It never publishes RC input and cannot affect arming, setpoints, or motor outputs.
 */
#ifndef LINUX_RC_MONITOR_H
#define LINUX_RC_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t received_bytes;
    uint32_t complete_frames;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t format_errors;
    uint32_t uart_errors;
    uint32_t last_heartbeat;
    uint32_t last_source_sequence;
    int16_t last_channels[5];
    uint8_t last_gesture_id;
    uint8_t last_confidence_percent;
    bool last_frame_valid;
} linux_rc_monitor_diagnostics_t;

void linux_rc_monitor_init(void);
void linux_rc_monitor_uart_rx_byte(uint8_t byte);
void linux_rc_monitor_uart_error(void);
void linux_rc_monitor_get_diagnostics(linux_rc_monitor_diagnostics_t *diagnostics);

#ifdef __cplusplus
}
#endif

#endif /* LINUX_RC_MONITOR_H */

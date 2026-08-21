/*
 * linux_rc_monitor.h -- Validated USART6 receiver for S7.6/S7.7 virtual RC.
 *
 * The ISR parser validates framing, CRC, bounds, timestamps and progression. It
 * exposes an atomic candidate snapshot to RcTask; only the S7.7 arbiter may turn
 * that candidate into effective RC input.
 */
#ifndef LINUX_RC_MONITOR_H
#define LINUX_RC_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#include "algorithms/rc_source_arbiter.h"

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
    uint32_t sequence_errors;
    uint32_t timestamp_errors;
    uint32_t session_reset_count;
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
void linux_rc_monitor_get_candidate(rc_virtual_candidate_t *candidate);

#ifdef __cplusplus
}
#endif

#endif /* LINUX_RC_MONITOR_H */

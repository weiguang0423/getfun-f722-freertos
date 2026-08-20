/*
 * linux_rc_monitor.c -- Passive parser for S7.6 virtual-RC frames received on USART6.
 *
 * Data flow: USART6 RX interrupt -> byte synchronizer -> 44-byte frame validation ->
 * diagnostic snapshot. This module intentionally has no dependency on rc_input or flight
 * control, so observing Linux traffic cannot enable or command the aircraft.
 */
#include "bsp/linux_rc_monitor.h"

#include <string.h>

#include "stm32f7xx.h"

#define LINUX_RC_FRAME_SIZE 44U
#define LINUX_RC_CRC_OFFSET 42U
#define LINUX_RC_FLAG_VALID 0x01U
#define LINUX_RC_GESTURE_MAX 4U
#define LINUX_RC_CHANNEL_LIMIT 300
#define LINUX_RC_THROTTLE_LIMIT 250
#define LINUX_RC_AUX_LIMIT 1000

typedef struct
{
    linux_rc_monitor_diagnostics_t diagnostics;
    uint8_t frame[LINUX_RC_FRAME_SIZE];
    uint8_t frame_length;
} linux_rc_monitor_state_t;

static linux_rc_monitor_state_t monitor;

static uint16_t get_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t get_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static uint16_t crc16_ccitt_false(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t index;

    for (index = 0U; index < length; ++index) {
        uint32_t bit;

        crc ^= (uint16_t)data[index] << 8U;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U ? (uint16_t)((crc << 1U) ^ 0x1021U)
                                         : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

static bool channels_in_range(const uint8_t *frame)
{
    const int16_t roll = (int16_t)get_u16_le(&frame[32]);
    const int16_t pitch = (int16_t)get_u16_le(&frame[34]);
    const int16_t yaw = (int16_t)get_u16_le(&frame[36]);
    const int16_t throttle = (int16_t)get_u16_le(&frame[38]);
    const int16_t aux = (int16_t)get_u16_le(&frame[40]);

    return (roll >= -LINUX_RC_CHANNEL_LIMIT) && (roll <= LINUX_RC_CHANNEL_LIMIT) &&
           (pitch >= -LINUX_RC_CHANNEL_LIMIT) && (pitch <= LINUX_RC_CHANNEL_LIMIT) &&
           (yaw >= -LINUX_RC_CHANNEL_LIMIT) && (yaw <= LINUX_RC_CHANNEL_LIMIT) &&
           (throttle >= 0) && (throttle <= LINUX_RC_THROTTLE_LIMIT) &&
           (aux >= 0) && (aux <= LINUX_RC_AUX_LIMIT);
}

static void validate_complete_frame(void)
{
    linux_rc_monitor_diagnostics_t *const diagnostics = &monitor.diagnostics;
    const uint8_t *const frame = monitor.frame;
    const bool frame_valid = (frame[4] & LINUX_RC_FLAG_VALID) != 0U;
    const uint16_t expected_crc = get_u16_le(&frame[LINUX_RC_CRC_OFFSET]);

    diagnostics->complete_frames++;
    if (expected_crc != crc16_ccitt_false(frame, LINUX_RC_CRC_OFFSET)) {
        diagnostics->crc_errors++;
        return;
    }
    if ((frame[2] != 1U) || (frame[3] != LINUX_RC_FRAME_SIZE) ||
        ((frame[4] & ~LINUX_RC_FLAG_VALID) != 0U) ||
        (frame[5] > LINUX_RC_GESTURE_MAX) || (frame[6] > 100U) ||
        (frame[7] != 0U) || !channels_in_range(frame)) {
        diagnostics->format_errors++;
        return;
    }
    if (frame_valid && ((frame[5] == 0U) || (frame[5] == 2U))) {
        diagnostics->format_errors++;
        return;
    }
    if (!frame_valid && ((get_u16_le(&frame[32]) != 0U) ||
                         (get_u16_le(&frame[34]) != 0U) ||
                         (get_u16_le(&frame[36]) != 0U) ||
                         (get_u16_le(&frame[38]) != 0U) ||
                         (get_u16_le(&frame[40]) != 0U))) {
        diagnostics->format_errors++;
        return;
    }

    diagnostics->valid_frames++;
    diagnostics->last_frame_valid = frame_valid;
    diagnostics->last_gesture_id = frame[5];
    diagnostics->last_confidence_percent = frame[6];
    diagnostics->last_source_sequence = get_u32_le(&frame[8]);
    diagnostics->last_heartbeat = get_u32_le(&frame[12]);
    diagnostics->last_channels[0] = (int16_t)get_u16_le(&frame[32]);
    diagnostics->last_channels[1] = (int16_t)get_u16_le(&frame[34]);
    diagnostics->last_channels[2] = (int16_t)get_u16_le(&frame[36]);
    diagnostics->last_channels[3] = (int16_t)get_u16_le(&frame[38]);
    diagnostics->last_channels[4] = (int16_t)get_u16_le(&frame[40]);
}

void linux_rc_monitor_init(void)
{
    memset(&monitor, 0, sizeof(monitor));
}

void linux_rc_monitor_uart_rx_byte(uint8_t byte)
{
    monitor.diagnostics.received_bytes++;

    if (monitor.frame_length == 0U) {
        if (byte == 'G') {
            monitor.frame[monitor.frame_length++] = byte;
        }
        return;
    }
    if (monitor.frame_length == 1U) {
        if (byte == 'R') {
            monitor.frame[monitor.frame_length++] = byte;
        } else {
            monitor.frame_length = byte == 'G' ? 1U : 0U;
            if (monitor.frame_length != 0U) {
                monitor.frame[0] = byte;
            }
        }
        return;
    }

    monitor.frame[monitor.frame_length++] = byte;
    if (monitor.frame_length == LINUX_RC_FRAME_SIZE) {
        validate_complete_frame();
        monitor.frame_length = 0U;
    }
}

void linux_rc_monitor_uart_error(void)
{
    monitor.diagnostics.uart_errors++;
    monitor.frame_length = 0U;
}

void linux_rc_monitor_get_diagnostics(linux_rc_monitor_diagnostics_t *diagnostics)
{
    uint32_t primask;

    if (diagnostics == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *diagnostics = monitor.diagnostics;
    if (primask == 0U) {
        __enable_irq();
    }
}

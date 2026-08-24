/*
 * linux_rc_monitor.c -- Validating parser for S7.6 virtual-RC frames on USART6.
 *
 * Data flow: USART6 RX interrupt -> byte synchronizer -> 44-byte frame validation ->
 * candidate snapshot. RcTask remains the sole owner of source arbitration, so
 * observing Linux traffic alone cannot enable or command the aircraft.
 */
#include "bsp/linux_rc_monitor.h"

#include <string.h>

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#define LINUX_RC_FRAME_SIZE 44U
#define LINUX_RC_CRC_OFFSET 42U
#define LINUX_RC_FLAG_VALID 0x01U
#define LINUX_RC_GESTURE_MAX 4U
#define LINUX_RC_CHANNEL_LIMIT 300
#define LINUX_RC_THROTTLE_LIMIT 250
#define LINUX_RC_AUX_LIMIT 1000
#define LINUX_RC_LINK_TIMEOUT_MS 150U
#define LINUX_RC_SOURCE_TIMEOUT_US 250000ULL

typedef struct
{
    linux_rc_monitor_diagnostics_t diagnostics;
    rc_virtual_candidate_t candidate;
    uint8_t frame[LINUX_RC_FRAME_SIZE];
    uint8_t frame_length;
    bool progression_initialized;
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

static uint64_t get_u64_le(const uint8_t *data)
{
    return (uint64_t)get_u32_le(data) |
           ((uint64_t)get_u32_le(data + 4U) << 32U);
}

static bool sequence_is_newer(uint32_t value, uint32_t previous)
{
    return (int32_t)(value - previous) > 0;
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

static bool channels_match_gesture(const uint8_t *frame)
{
    const int16_t roll = (int16_t)get_u16_le(&frame[32]);
    const int16_t pitch = (int16_t)get_u16_le(&frame[34]);
    const int16_t yaw = (int16_t)get_u16_le(&frame[36]);
    const int16_t throttle = (int16_t)get_u16_le(&frame[38]);
    const int16_t aux = (int16_t)get_u16_le(&frame[40]);

    if ((yaw != 0) || (throttle != 0) || (aux != 0)) {
        return false;
    }
    if (frame[5] == 0U) {
        return (roll == 0) && (pitch == 0) && (frame[6] == 0U);
    }
    if (frame[5] == 1U) {
        return (roll == 0) && (pitch >= -LINUX_RC_CHANNEL_LIMIT) &&
               (pitch <= 0);
    }
    if (frame[5] == 3U) {
        return (roll == 0) && (pitch >= 0) &&
               (pitch <= LINUX_RC_CHANNEL_LIMIT);
    }
    if (frame[5] == 4U) {
        return (roll >= -LINUX_RC_CHANNEL_LIMIT) && (roll <= 0) &&
               (pitch == 0);
    }
    return false;
}

static void validate_complete_frame(void)
{
    linux_rc_monitor_diagnostics_t *const diagnostics = &monitor.diagnostics;
    const uint8_t *const frame = monitor.frame;
    const bool frame_valid = (frame[4] & LINUX_RC_FLAG_VALID) != 0U;
    const uint16_t expected_crc = get_u16_le(&frame[LINUX_RC_CRC_OFFSET]);
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t source_sequence = get_u32_le(&frame[8]);
    const uint32_t heartbeat = get_u32_le(&frame[12]);
    const uint64_t source_timestamp_us = get_u64_le(&frame[16]);
    const uint64_t send_timestamp_us = get_u64_le(&frame[24]);
    bool new_session = false;

    diagnostics->complete_frames++;
    monitor.candidate.valid = false;
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
    if (frame_valid && ((frame[5] == 2U) ||
                        ((frame[5] != 0U) && (frame[6] < 75U)) ||
                        !channels_match_gesture(frame))) {
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

    if ((send_timestamp_us < source_timestamp_us) ||
        (frame_valid && ((source_timestamp_us == 0U) ||
                         ((send_timestamp_us - source_timestamp_us) >
                          LINUX_RC_SOURCE_TIMEOUT_US)))) {
        diagnostics->timestamp_errors++;
        return;
    }

    if (monitor.progression_initialized &&
        !sequence_is_newer(heartbeat, diagnostics->last_heartbeat)) {
        if ((uint32_t)(now_ms - monitor.candidate.received_ms) <
            LINUX_RC_LINK_TIMEOUT_MS) {
            diagnostics->sequence_errors++;
            return;
        }
        new_session = true;
    }
    if (frame_valid && monitor.progression_initialized && !new_session &&
        !sequence_is_newer(source_sequence,
                           diagnostics->last_source_sequence)) {
        diagnostics->sequence_errors++;
        return;
    }

    diagnostics->valid_frames++;
    diagnostics->last_frame_valid = frame_valid;
    diagnostics->last_gesture_id = frame[5];
    diagnostics->last_confidence_percent = frame[6];
    diagnostics->last_source_sequence = source_sequence;
    diagnostics->last_heartbeat = heartbeat;
    diagnostics->last_channels[0] = (int16_t)get_u16_le(&frame[32]);
    diagnostics->last_channels[1] = (int16_t)get_u16_le(&frame[34]);
    diagnostics->last_channels[2] = (int16_t)get_u16_le(&frame[36]);
    diagnostics->last_channels[3] = (int16_t)get_u16_le(&frame[38]);
    diagnostics->last_channels[4] = (int16_t)get_u16_le(&frame[40]);
    if (new_session) {
        diagnostics->session_reset_count++;
    }
    monitor.progression_initialized = true;
    monitor.candidate.valid = frame_valid;
    monitor.candidate.received_ms = now_ms;
    monitor.candidate.source_sequence = source_sequence;
    monitor.candidate.heartbeat = heartbeat;
    monitor.candidate.session_generation =
        diagnostics->session_reset_count;
    memcpy(monitor.candidate.channel, diagnostics->last_channels,
           sizeof(monitor.candidate.channel));
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
    monitor.candidate.valid = false;
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

void linux_rc_monitor_get_candidate(rc_virtual_candidate_t *candidate)
{
    uint32_t primask;

    if (candidate == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *candidate = monitor.candidate;
    if (primask == 0U) {
        __enable_irq();
    }
}

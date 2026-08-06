#include "algorithms/rc_input.h"

#include <stddef.h>

/* Betaflight internal order is Roll, Pitch, Yaw, Throttle, AUX1... */
const uint8_t
    rc_input_aetr_map[RC_INPUT_MAPPABLE_CHANNEL_COUNT] = {
        0U, 1U, 3U, 2U, 4U, 5U, 6U, 7U
    };

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t then_ms)
{
    return now_ms - then_ms;
}

void rc_input_map_aetr(
    const uint16_t raw_channel_us[RC_INPUT_CHANNEL_COUNT],
    uint16_t mapped_channel_us[RC_INPUT_CHANNEL_COUNT])
{
    uint8_t channel;

    if ((raw_channel_us == NULL) || (mapped_channel_us == NULL)) {
        return;
    }

    for (channel = 0U;
         channel < RC_INPUT_MAPPABLE_CHANNEL_COUNT;
         ++channel) {
        mapped_channel_us[channel] =
            raw_channel_us[rc_input_aetr_map[channel]];
    }
    for (channel = RC_INPUT_MAPPABLE_CHANNEL_COUNT;
         channel < RC_INPUT_CHANNEL_COUNT;
         ++channel) {
        mapped_channel_us[channel] = raw_channel_us[channel];
    }
}

void rc_input_set_safe_channels(
    uint16_t channel_us[RC_INPUT_CHANNEL_COUNT])
{
    uint8_t channel;

    if (channel_us == NULL) {
        return;
    }

    for (channel = 0U; channel < RC_INPUT_CHANNEL_COUNT; ++channel) {
        channel_us[channel] = 1000U;
    }
    channel_us[0] = 1500U;
    channel_us[1] = 1500U;
    channel_us[2] = 1500U;
}

void rc_input_failsafe_init(rc_input_failsafe_t *state)
{
    if (state == NULL) {
        return;
    }

    state->phase = RC_INPUT_PHASE_WAITING_FOR_SIGNAL;
    state->frame_received = false;
    state->failsafe_active = true;
    state->last_frame_ms = 0U;
    state->recovery_started_ms = 0U;
    state->last_failsafe_ms = 0U;
    state->failsafe_count = 0U;
    state->recovery_count = 0U;
    state->recovery_frame_count = 0U;
}

void rc_input_failsafe_on_frame(rc_input_failsafe_t *state,
                                uint32_t now_ms)
{
    if (state == NULL) {
        return;
    }

    state->frame_received = true;
    state->last_frame_ms = now_ms;

    if (state->phase == RC_INPUT_PHASE_HEALTHY) {
        return;
    }

    if (state->phase != RC_INPUT_PHASE_RECOVERING) {
        state->phase = RC_INPUT_PHASE_RECOVERING;
        state->recovery_started_ms = now_ms;
        state->recovery_frame_count = 1U;
    } else if (state->recovery_frame_count < UINT16_MAX) {
        state->recovery_frame_count++;
    }

    if ((state->recovery_frame_count >=
         RC_INPUT_RECOVERY_MIN_FRAMES) &&
        (elapsed_ms(now_ms, state->recovery_started_ms) >=
         RC_INPUT_RECOVERY_MS)) {
        state->phase = RC_INPUT_PHASE_HEALTHY;
        state->failsafe_active = false;
        state->recovery_count++;
    }
}

void rc_input_failsafe_update(rc_input_failsafe_t *state,
                              uint32_t now_ms)
{
    if ((state == NULL) || !state->frame_received) {
        return;
    }

    if (elapsed_ms(now_ms, state->last_frame_ms) <
        RC_INPUT_TIMEOUT_MS) {
        return;
    }

    if (state->phase == RC_INPUT_PHASE_HEALTHY) {
        state->failsafe_count++;
        state->last_failsafe_ms = now_ms;
    }
    state->phase = RC_INPUT_PHASE_LOST;
    state->failsafe_active = true;
    state->recovery_frame_count = 0U;
}

bool rc_input_control_valid(const rc_input_failsafe_t *state)
{
    return (state != NULL) &&
           (state->phase == RC_INPUT_PHASE_HEALTHY) &&
           !state->failsafe_active;
}

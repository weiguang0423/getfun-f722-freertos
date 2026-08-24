/*
 * rc_source_arbiter.c - S7.7 fail-closed RC source selection and slew logic.
 *
 * Data flow: physical mapped RC + validated Linux candidate + flight gates ->
 * effective mapped RC. Entry requires ARM, AUX3 authorization, neutral sticks,
 * a fresh candidate and a prior AUX3-low handshake. Exit is immediate to the
 * physical source and cannot automatically reverse after link recovery.
 */
#include "algorithms/rc_source_arbiter.h"

#include <stddef.h>
#include <string.h>

#define RC_SOURCE_AXIS_MID_US 1500U
#define RC_SOURCE_AXIS_RATE_PER_S 600U
#define RC_SOURCE_MAX_SLEW_DT_MS 100U

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t then_ms)
{
    return now_ms - then_ms;
}

static uint16_t absolute_delta(uint16_t value, uint16_t center)
{
    return value >= center ? (uint16_t)(value - center)
                           : (uint16_t)(center - value);
}

static bool authorization_is_active(
    const uint16_t physical[RC_INPUT_CHANNEL_COUNT])
{
    return physical[RC_SOURCE_AUTH_CHANNEL] >= RC_SOURCE_AUTH_MIN_US;
}

static bool takeover_is_requested(
    const uint16_t physical[RC_INPUT_CHANNEL_COUNT])
{
    return (absolute_delta(physical[0], RC_SOURCE_AXIS_MID_US) >
            RC_SOURCE_TAKEOVER_AXIS_DELTA_US) ||
           (absolute_delta(physical[1], RC_SOURCE_AXIS_MID_US) >
            RC_SOURCE_TAKEOVER_AXIS_DELTA_US) ||
           (absolute_delta(physical[2], RC_SOURCE_AXIS_MID_US) >
            RC_SOURCE_TAKEOVER_AXIS_DELTA_US);
}

static bool candidate_is_fresh(const rc_virtual_candidate_t *candidate,
                               uint32_t now_ms)
{
    return (candidate != NULL) && candidate->valid &&
           (elapsed_ms(now_ms, candidate->received_ms) <
            RC_SOURCE_VIRTUAL_TIMEOUT_MS);
}

static uint16_t virtual_target_us(int16_t value)
{
    const int32_t target = (int32_t)RC_SOURCE_AXIS_MID_US + value;
    return (uint16_t)target;
}

static uint16_t slew_toward(uint16_t current, uint16_t target,
                            uint32_t rate_per_s, uint32_t dt_ms,
                            uint16_t *remainder)
{
    uint32_t maximum_step;
    uint32_t scaled_step;

    if (dt_ms > RC_SOURCE_MAX_SLEW_DT_MS) {
        dt_ms = RC_SOURCE_MAX_SLEW_DT_MS;
    }
    if (current == target) {
        *remainder = 0U;
        return current;
    }
    scaled_step = rate_per_s * dt_ms + *remainder;
    maximum_step = scaled_step / 1000U;
    *remainder = (uint16_t)(scaled_step % 1000U);
    if (maximum_step == 0U) {
        return current;
    }
    if (current < target) {
        const uint32_t delta = (uint32_t)target - current;
        return delta <= maximum_step ? target
                                     : (uint16_t)(current + maximum_step);
    }
    if (current > target) {
        const uint32_t delta = (uint32_t)current - target;
        return delta <= maximum_step ? target
                                     : (uint16_t)(current - maximum_step);
    }
    return current;
}

static void exit_virtual(rc_source_arbiter_t *state,
                         rc_source_exit_reason_t reason,
                         uint32_t now_ms)
{
    state->active_source = RC_SOURCE_PHYSICAL;
    state->last_exit_reason = reason;
    state->authorization_seen_low = false;
    state->last_transition_ms = now_ms;
    ++state->exit_count;
}

void rc_source_arbiter_init(rc_source_arbiter_t *state,
                            uint32_t now_ms)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->active_source = RC_SOURCE_PHYSICAL;
    state->last_update_ms = now_ms;
    state->last_transition_ms = now_ms;
}

void rc_source_arbiter_update(
    rc_source_arbiter_t *state,
    const uint16_t physical[RC_INPUT_CHANNEL_COUNT],
    bool physical_valid,
    bool aircraft_armed,
    uint32_t arming_inhibit_flags,
    bool authorization_channel_available,
    const rc_virtual_candidate_t *candidate,
    uint32_t now_ms,
    uint16_t output[RC_INPUT_CHANNEL_COUNT])
{
    bool authorized;
    bool takeover;
    bool candidate_fresh;
    uint32_t dt_ms;
    uint32_t channel;

    if ((state == NULL) || (physical == NULL) || (output == NULL)) {
        return;
    }

    memcpy(output, physical, sizeof(uint16_t) * RC_INPUT_CHANNEL_COUNT);
    authorized = authorization_is_active(physical);
    takeover = takeover_is_requested(physical);
    candidate_fresh = candidate_is_fresh(candidate, now_ms);
    state->authorization_active = authorized;

    if (physical_valid && !authorized) {
        state->authorization_seen_low = true;
    }

    if (state->active_source == RC_SOURCE_VIRTUAL) {
        if (!physical_valid) {
            exit_virtual(state, RC_SOURCE_EXIT_PHYSICAL_INVALID, now_ms);
        } else if (!authorization_channel_available) {
            exit_virtual(state, RC_SOURCE_EXIT_CONFIGURATION_INVALID, now_ms);
        } else if (!aircraft_armed) {
            exit_virtual(state, RC_SOURCE_EXIT_DISARMED, now_ms);
        } else if (arming_inhibit_flags != 0U) {
            exit_virtual(state, RC_SOURCE_EXIT_FLIGHT_INHIBIT, now_ms);
        } else if (!authorized) {
            exit_virtual(state, RC_SOURCE_EXIT_AUTH_REVOKED, now_ms);
        } else if (takeover) {
            exit_virtual(state, RC_SOURCE_EXIT_PHYSICAL_TAKEOVER, now_ms);
        } else if ((candidate == NULL) || !candidate->valid) {
            exit_virtual(state, RC_SOURCE_EXIT_VIRTUAL_INVALID, now_ms);
        } else if (candidate->session_generation !=
                   state->active_session_generation) {
            exit_virtual(state, RC_SOURCE_EXIT_VIRTUAL_RESTART, now_ms);
        } else if (!candidate_fresh) {
            exit_virtual(state, RC_SOURCE_EXIT_VIRTUAL_TIMEOUT, now_ms);
        }
    } else if (physical_valid && authorization_channel_available &&
               aircraft_armed &&
               (arming_inhibit_flags == 0U) && authorized &&
               state->authorization_seen_low && !takeover && candidate_fresh) {
        state->active_source = RC_SOURCE_VIRTUAL;
        state->authorization_seen_low = false;
        state->last_exit_reason = RC_SOURCE_EXIT_NONE;
        state->last_transition_ms = now_ms;
        state->active_session_generation = candidate->session_generation;
        ++state->activation_count;
        memcpy(state->virtual_channel_us, physical,
               sizeof(state->virtual_channel_us));
        memset(state->slew_remainder, 0,
               sizeof(state->slew_remainder));
    }

    dt_ms = elapsed_ms(now_ms, state->last_update_ms);
    state->last_update_ms = now_ms;
    if (state->active_source != RC_SOURCE_VIRTUAL) {
        return;
    }

    /* output[] already contains the current physical snapshot. Linux only
     * replaces Roll/Pitch/Yaw; physical Throttle and every AUX stay live. */
    for (channel = 0U; channel < 3U; ++channel) {
        const uint16_t target = virtual_target_us(candidate->channel[channel]);
        state->virtual_channel_us[channel] =
            slew_toward(state->virtual_channel_us[channel], target,
                        RC_SOURCE_AXIS_RATE_PER_S, dt_ms,
                        &state->slew_remainder[channel]);
        output[channel] = state->virtual_channel_us[channel];
    }
}

/*
 * verify_rc_source_arbiter.c - Host regression for the actual S7.7 C arbiter.
 *
 * The vectors cover startup, explicit authorization, AUX ownership, bounded
 * entry slew, every fail-closed exit, timeout wraparound and mandatory AUX3
 * low/high reauthorization. Compile together with rc_source_arbiter.c.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "algorithms/rc_source_arbiter.h"

static void physical_defaults(uint16_t channel[RC_INPUT_CHANNEL_COUNT])
{
    unsigned int index;

    for (index = 0U; index < RC_INPUT_CHANNEL_COUNT; ++index) {
        channel[index] = 1000U;
    }
    channel[0] = 1500U;
    channel[1] = 1500U;
    channel[2] = 1500U;
}

static rc_virtual_candidate_t valid_candidate(uint32_t now_ms)
{
    rc_virtual_candidate_t candidate;

    memset(&candidate, 0, sizeof(candidate));
    candidate.valid = true;
    candidate.received_ms = now_ms;
    candidate.source_sequence = 10U;
    candidate.heartbeat = 20U;
    candidate.channel[1] = -300;
    candidate.channel[2] = 300;
    return candidate;
}

static void authorize_low_then_high(rc_source_arbiter_t *state,
                                    uint16_t physical[RC_INPUT_CHANNEL_COUNT],
                                    rc_virtual_candidate_t *candidate,
                                    uint32_t *now_ms,
                                    uint16_t output[RC_INPUT_CHANNEL_COUNT])
{
    physical[RC_SOURCE_AUTH_CHANNEL] = 1000U;
    rc_source_arbiter_update(state, physical, true, true, 0U, true, candidate,
                             (*now_ms)++, output);
    physical[RC_SOURCE_AUTH_CHANNEL] = 1800U;
    candidate->received_ms = *now_ms;
    rc_source_arbiter_update(state, physical, true, true, 0U, true, candidate,
                             (*now_ms)++, output);
    assert(state->active_source == RC_SOURCE_VIRTUAL);
}

int main(void)
{
    rc_source_arbiter_t state;
    rc_virtual_candidate_t candidate;
    uint16_t physical[RC_INPUT_CHANNEL_COUNT];
    uint16_t output[RC_INPUT_CHANNEL_COUNT];
    uint32_t now_ms = 1000U;

    physical_defaults(physical);
    candidate = valid_candidate(now_ms);
    physical[RC_SOURCE_AUTH_CHANNEL] = 1800U;
    rc_source_arbiter_init(&state, now_ms);

    physical[RC_SOURCE_AUTH_CHANNEL] = 1000U;
    rc_source_arbiter_update(&state, physical, false, false, 0U, true, &candidate,
                             now_ms++, output);
    physical[RC_SOURCE_AUTH_CHANNEL] = 1800U;
    rc_source_arbiter_update(&state, physical, true, true, 0U, true, &candidate,
                             now_ms++, output);
    assert(state.active_source == RC_SOURCE_PHYSICAL);
    assert(output[RC_SOURCE_AUTH_CHANNEL] == 1800U);

    physical[RC_SOURCE_AUTH_CHANNEL] = 1000U;
    rc_source_arbiter_update(&state, physical, true, false, 0U, true, &candidate,
                             now_ms++, output);
    physical[RC_SOURCE_AUTH_CHANNEL] = 1800U;
    candidate.received_ms = now_ms;
    rc_source_arbiter_update(&state, physical, true, false, 0U, true, &candidate,
                             now_ms++, output);
    assert(state.active_source == RC_SOURCE_PHYSICAL);

    candidate.received_ms = now_ms;
    rc_source_arbiter_update(&state, physical, true, true, 0U, true, &candidate,
                             now_ms++, output);
    assert(state.active_source == RC_SOURCE_VIRTUAL);
    assert(state.activation_count == 1U);
    assert(output[4] == physical[4] && output[5] == physical[5] &&
           output[6] == physical[6]);
    assert(output[1] == 1500U);
    candidate.received_ms = now_ms;
    rc_source_arbiter_update(&state, physical, true, true, 0U, true, &candidate,
                             now_ms + 19U, output);
    now_ms += 20U;
    assert(output[1] == 1488U);

    candidate.valid = false;
    rc_source_arbiter_update(&state, physical, true, true, 0U, true, &candidate,
                             now_ms++, output);
    assert(state.active_source == RC_SOURCE_PHYSICAL);
    assert(state.last_exit_reason == RC_SOURCE_EXIT_VIRTUAL_INVALID);
    assert(output[1] == physical[1] && output[3] == physical[3]);

    candidate = valid_candidate(now_ms);
    rc_source_arbiter_update(&state, physical, true, true, 0U, true, &candidate,
                             now_ms++, output);
    assert(state.active_source == RC_SOURCE_PHYSICAL);
    authorize_low_then_high(&state, physical, &candidate, &now_ms, output);

    physical[0] = 1651U;
    rc_source_arbiter_update(&state, physical, true, true, 0U, true, &candidate,
                             now_ms++, output);
    assert(state.last_exit_reason == RC_SOURCE_EXIT_PHYSICAL_TAKEOVER);
    assert(output[0] == 1651U);
    physical[0] = 1500U;

    authorize_low_then_high(&state, physical, &candidate, &now_ms, output);
    candidate.session_generation++;
    candidate.received_ms = now_ms;
    rc_source_arbiter_update(&state, physical, true, true, 0U, true, &candidate,
                             now_ms++, output);
    assert(state.last_exit_reason == RC_SOURCE_EXIT_VIRTUAL_RESTART);

    authorize_low_then_high(&state, physical, &candidate, &now_ms, output);
    candidate.received_ms = now_ms - RC_SOURCE_VIRTUAL_TIMEOUT_MS;
    rc_source_arbiter_update(&state, physical, true, true, 0U, true, &candidate,
                             now_ms++, output);
    assert(state.last_exit_reason == RC_SOURCE_EXIT_VIRTUAL_TIMEOUT);

    authorize_low_then_high(&state, physical, &candidate, &now_ms, output);
    rc_source_arbiter_update(&state, physical, true, true, 1U, true, &candidate,
                             now_ms++, output);
    assert(state.last_exit_reason == RC_SOURCE_EXIT_FLIGHT_INHIBIT);

    authorize_low_then_high(&state, physical, &candidate, &now_ms, output);
    rc_source_arbiter_update(&state, physical, true, false, 0U, true, &candidate,
                             now_ms++, output);
    assert(state.last_exit_reason == RC_SOURCE_EXIT_DISARMED);

    authorize_low_then_high(&state, physical, &candidate, &now_ms, output);
    rc_source_arbiter_update(&state, physical, true, true, 0U, false, &candidate,
                             now_ms++, output);
    assert(state.last_exit_reason ==
           RC_SOURCE_EXIT_CONFIGURATION_INVALID);

    authorize_low_then_high(&state, physical, &candidate, &now_ms, output);
    rc_source_arbiter_update(&state, physical, false, true, 0U, true, &candidate,
                             now_ms++, output);
    assert(state.last_exit_reason == RC_SOURCE_EXIT_PHYSICAL_INVALID);

    rc_source_arbiter_init(&state, 0xfffffff0U);
    physical[RC_SOURCE_AUTH_CHANNEL] = 1000U;
    candidate = valid_candidate(0xfffffff0U);
    rc_source_arbiter_update(&state, physical, true, true, 0U, true, &candidate,
                             0xfffffff0U, output);
    physical[RC_SOURCE_AUTH_CHANNEL] = 1800U;
    rc_source_arbiter_update(&state, physical, true, true, 0U, true, &candidate,
                             0xfffffff1U, output);
    assert(state.active_source == RC_SOURCE_VIRTUAL);
    rc_source_arbiter_update(&state, physical, true, true, 0U, true, &candidate,
                             0x00000085U, output);
    assert(state.active_source == RC_SOURCE_VIRTUAL);
    rc_source_arbiter_update(&state, physical, true, true, 0U, true, &candidate,
                             0x00000086U, output);
    assert(state.last_exit_reason == RC_SOURCE_EXIT_VIRTUAL_TIMEOUT);

    puts("S7.7 rc_source_arbiter: PASS (startup/auth/AUX/slew/exit/timeout/wrap)");
    return 0;
}

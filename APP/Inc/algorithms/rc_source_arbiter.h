/*
 * rc_source_arbiter.h - Pure S7.7 physical/virtual RC source arbiter.
 *
 * The arbiter consumes an already validated Linux candidate plus the physical
 * AETR snapshot and emits the only channel snapshot allowed into rc_setpoint.
 * Physical throttle and AUX channels always retain ownership; Linux can only
 * replace Roll/Pitch/Yaw after authorization.
 * Any exit latches reauthorization until AUX3 is observed low before a new rise.
 */
#ifndef RC_SOURCE_ARBITER_H
#define RC_SOURCE_ARBITER_H

#include <stdbool.h>
#include <stdint.h>

#include "algorithms/rc_input.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC_SOURCE_AUTH_CHANNEL 6U
#define RC_SOURCE_AUTH_MIN_US 1700U
#define RC_SOURCE_TAKEOVER_AXIS_DELTA_US 150U
#define RC_SOURCE_VIRTUAL_TIMEOUT_MS 150U

typedef enum
{
    RC_SOURCE_PHYSICAL = 0,
    RC_SOURCE_VIRTUAL
} rc_source_t;

typedef enum
{
    RC_SOURCE_EXIT_NONE = 0,
    RC_SOURCE_EXIT_AUTH_REVOKED,
    RC_SOURCE_EXIT_PHYSICAL_TAKEOVER,
    RC_SOURCE_EXIT_VIRTUAL_INVALID,
    RC_SOURCE_EXIT_VIRTUAL_TIMEOUT,
    RC_SOURCE_EXIT_VIRTUAL_RESTART,
    RC_SOURCE_EXIT_FLIGHT_INHIBIT,
    RC_SOURCE_EXIT_DISARMED,
    RC_SOURCE_EXIT_PHYSICAL_INVALID,
    RC_SOURCE_EXIT_CONFIGURATION_INVALID
} rc_source_exit_reason_t;

typedef struct
{
    bool valid;
    uint32_t received_ms;
    uint32_t source_sequence;
    uint32_t heartbeat;
    uint32_t session_generation;
    int16_t channel[5];
} rc_virtual_candidate_t;

typedef struct
{
    rc_source_t active_source;
    rc_source_exit_reason_t last_exit_reason;
    bool authorization_seen_low;
    bool authorization_active;
    uint32_t activation_count;
    uint32_t exit_count;
    uint32_t last_transition_ms;
    uint32_t last_update_ms;
    uint32_t active_session_generation;
    uint16_t virtual_channel_us[3];
    uint16_t slew_remainder[3];
} rc_source_arbiter_t;

void rc_source_arbiter_init(rc_source_arbiter_t *state,
                            uint32_t now_ms);
void rc_source_arbiter_update(
    rc_source_arbiter_t *state,
    const uint16_t physical_channel_us[RC_INPUT_CHANNEL_COUNT],
    bool physical_valid,
    bool aircraft_armed,
    uint32_t arming_inhibit_flags,
    bool authorization_channel_available,
    const rc_virtual_candidate_t *candidate,
    uint32_t now_ms,
    uint16_t output_channel_us[RC_INPUT_CHANNEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* RC_SOURCE_ARBITER_H */

/* Pure S4.4 RC normalization, Actual Rates and AUX mode selection. */
#ifndef RC_SETPOINT_H
#define RC_SETPOINT_H

#include <stdbool.h>
#include <stdint.h>

#include "algorithms/rc_input.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC_SETPOINT_AXIS_COUNT 3U
#define RC_SETPOINT_RATE_TYPE_ACTUAL 3U
#define RC_SETPOINT_ACTUAL_RATE_MAX 200U

typedef enum
{
    RC_SETPOINT_MODE_RATE = 0,
    RC_SETPOINT_MODE_ANGLE
} rc_setpoint_mode_t;

typedef struct
{
    uint16_t input_min_us;
    uint16_t input_mid_us;
    uint16_t input_max_us;
    uint16_t throttle_min_check_us;
    uint8_t deadband_us;
    uint8_t yaw_deadband_us;
    uint8_t actual_center_sensitivity[RC_SETPOINT_AXIS_COUNT];
    uint8_t actual_max_rate[RC_SETPOINT_AXIS_COUNT];
    uint8_t expo_percent[RC_SETPOINT_AXIS_COUNT];
    uint8_t arm_aux_channel;
    uint8_t angle_aux_channel;
    uint16_t arm_range_min_us;
    uint16_t arm_range_max_us;
    uint16_t angle_range_min_us;
    uint16_t angle_range_max_us;
} rc_setpoint_profile_t;

typedef struct
{
    bool valid;
    bool arm_requested;
    rc_setpoint_mode_t mode;
    float normalized_stick[RC_SETPOINT_AXIS_COUNT];
    float throttle;
    float rate_dps[RC_SETPOINT_AXIS_COUNT];
} rc_setpoint_output_t;

const rc_setpoint_profile_t *rc_setpoint_default_profile(void);
bool rc_setpoint_profile_is_valid(const rc_setpoint_profile_t *profile);
bool rc_setpoint_compute(
    const rc_setpoint_profile_t *profile,
    const uint16_t mapped_channel_us[RC_INPUT_CHANNEL_COUNT],
    rc_setpoint_output_t *output);

#ifdef __cplusplus
}
#endif

#endif

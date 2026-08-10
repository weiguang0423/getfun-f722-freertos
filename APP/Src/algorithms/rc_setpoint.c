#include "algorithms/rc_setpoint.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static const rc_setpoint_profile_t default_profile = {
    .input_min_us = 1000U,
    .input_mid_us = 1500U,
    .input_max_us = 2000U,
    .deadband_us = 5U,
    .yaw_deadband_us = 5U,
    .actual_center_sensitivity = {7U, 7U, 7U},
    .actual_max_rate = {67U, 67U, 67U},
    .expo_percent = {0U, 0U, 0U},
    .arm_aux_channel = 4U,
    .angle_aux_channel = 5U,
    .arm_range_min_us = 1700U,
    .arm_range_max_us = 2000U,
    .angle_range_min_us = 1700U,
    .angle_range_max_us = 2000U,
};

static uint16_t clamp_channel(uint16_t value,
                              uint16_t minimum,
                              uint16_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float normalize_axis(uint16_t channel_us,
                            const rc_setpoint_profile_t *profile,
                            uint8_t deadband_us)
{
    const int32_t delta =
        (int32_t)clamp_channel(channel_us,
                               profile->input_min_us,
                               profile->input_max_us) -
        (int32_t)profile->input_mid_us;
    const uint32_t magnitude =
        delta < 0 ? (uint32_t)(-delta) : (uint32_t)delta;
    uint32_t span;
    float normalized;

    if (magnitude <= deadband_us) {
        return 0.0f;
    }

    span = delta < 0
               ? (uint32_t)(profile->input_mid_us -
                            profile->input_min_us - deadband_us)
               : (uint32_t)(profile->input_max_us -
                            profile->input_mid_us - deadband_us);
    normalized = (float)(magnitude - deadband_us) / (float)span;
    if (normalized > 1.0f) {
        normalized = 1.0f;
    }
    return delta < 0 ? -normalized : normalized;
}

static float actual_rate_dps(float normalized,
                             uint8_t center_sensitivity,
                             uint8_t maximum_rate,
                             uint8_t expo_percent)
{
    const float magnitude = fabsf(normalized);
    const float squared = normalized * normalized;
    const float fifth = squared * squared * normalized;
    const float expo = (float)expo_percent / 100.0f;
    const float curve =
        magnitude * (fifth * expo + normalized * (1.0f - expo));
    const float center_dps = (float)center_sensitivity * 10.0f;
    const float maximum_dps = (float)maximum_rate * 10.0f;

    return normalized * center_dps +
           (maximum_dps - center_dps) * curve;
}

static bool range_is_valid(const rc_setpoint_profile_t *profile,
                           uint16_t minimum,
                           uint16_t maximum)
{
    return (minimum <= maximum) &&
           (minimum >= profile->input_min_us) &&
           (maximum <= profile->input_max_us);
}

static bool range_is_active(uint16_t value,
                            uint16_t minimum,
                            uint16_t maximum)
{
    return (value >= minimum) && (value <= maximum);
}

const rc_setpoint_profile_t *rc_setpoint_default_profile(void)
{
    return &default_profile;
}

bool rc_setpoint_profile_is_valid(const rc_setpoint_profile_t *profile)
{
    uint32_t axis;

    if ((profile == NULL) ||
        (profile->input_min_us >= profile->input_mid_us) ||
        (profile->input_mid_us >= profile->input_max_us) ||
        ((uint32_t)profile->deadband_us >=
         (uint32_t)(profile->input_mid_us - profile->input_min_us)) ||
        ((uint32_t)profile->deadband_us >=
         (uint32_t)(profile->input_max_us - profile->input_mid_us)) ||
        ((uint32_t)profile->yaw_deadband_us >=
         (uint32_t)(profile->input_mid_us - profile->input_min_us)) ||
        ((uint32_t)profile->yaw_deadband_us >=
         (uint32_t)(profile->input_max_us - profile->input_mid_us)) ||
        (profile->arm_aux_channel < 4U) ||
        (profile->arm_aux_channel >= RC_INPUT_CHANNEL_COUNT) ||
        (profile->angle_aux_channel < 4U) ||
        (profile->angle_aux_channel >= RC_INPUT_CHANNEL_COUNT) ||
        (profile->arm_aux_channel == profile->angle_aux_channel) ||
        !range_is_valid(profile,
                        profile->arm_range_min_us,
                        profile->arm_range_max_us) ||
        !range_is_valid(profile,
                        profile->angle_range_min_us,
                        profile->angle_range_max_us)) {
        return false;
    }

    for (axis = 0U; axis < RC_SETPOINT_AXIS_COUNT; ++axis) {
        if ((profile->actual_center_sensitivity[axis] == 0U) ||
            (profile->actual_center_sensitivity[axis] >
             profile->actual_max_rate[axis]) ||
            (profile->actual_max_rate[axis] >
             RC_SETPOINT_ACTUAL_RATE_MAX) ||
            (profile->expo_percent[axis] > 100U)) {
            return false;
        }
    }
    return true;
}

bool rc_setpoint_compute(
    const rc_setpoint_profile_t *profile,
    const uint16_t mapped_channel_us[RC_INPUT_CHANNEL_COUNT],
    rc_setpoint_output_t *output)
{
    uint32_t axis;

    if (output == NULL) {
        return false;
    }
    memset(output, 0, sizeof(*output));
    if ((mapped_channel_us == NULL) ||
        !rc_setpoint_profile_is_valid(profile)) {
        return false;
    }

    for (axis = 0U; axis < RC_SETPOINT_AXIS_COUNT; ++axis) {
        const uint8_t deadband =
            axis == 2U ? profile->yaw_deadband_us : profile->deadband_us;

        output->normalized_stick[axis] =
            normalize_axis(mapped_channel_us[axis], profile, deadband);
        output->rate_dps[axis] =
            actual_rate_dps(output->normalized_stick[axis],
                            profile->actual_center_sensitivity[axis],
                            profile->actual_max_rate[axis],
                            profile->expo_percent[axis]);
    }

    output->throttle =
        (float)(clamp_channel(mapped_channel_us[3],
                              profile->input_min_us,
                              profile->input_max_us) -
                profile->input_min_us) /
        (float)(profile->input_max_us - profile->input_min_us);
    output->arm_requested =
        range_is_active(clamp_channel(
                            mapped_channel_us[profile->arm_aux_channel],
                            profile->input_min_us,
                            profile->input_max_us),
                        profile->arm_range_min_us,
                        profile->arm_range_max_us);
    output->mode =
        range_is_active(clamp_channel(
                            mapped_channel_us[profile->angle_aux_channel],
                            profile->input_min_us,
                            profile->input_max_us),
                        profile->angle_range_min_us,
                        profile->angle_range_max_us)
            ? RC_SETPOINT_MODE_ANGLE
            : RC_SETPOINT_MODE_RATE;
    output->valid = true;
    return true;
}

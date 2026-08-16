/*
 * angle_outer_loop.c - Bounded S4.8 Angle outer-loop implementation.
 *
 * Roll/Pitch target = normalized stick * angle limit. Angle error multiplied
 * by Angle P becomes a bounded Rate PID setpoint; Yaw remains in Rate mode.
 * Invalid profiles or non-finite/out-of-range inputs fail closed with a zeroed
 * output so FlightTask can use its existing CONTROL_INVALID path.
 */
#include "algorithms/angle_outer_loop.h"

#include <math.h>
#include <string.h>

static const angle_outer_loop_profile_t default_profile = {
    .angle_limit_deg = 60.0f,
    .angle_p = 5.0f,
    .rate_limit_dps = 670.0f,
};

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

const angle_outer_loop_profile_t *angle_outer_loop_default_profile(void)
{
    return &default_profile;
}

bool angle_outer_loop_profile_is_valid(
    const angle_outer_loop_profile_t *profile)
{
    return (profile != NULL) &&
           isfinite(profile->angle_limit_deg) &&
           isfinite(profile->angle_p) &&
           isfinite(profile->rate_limit_dps) &&
           (profile->angle_limit_deg >= 10.0f) &&
           (profile->angle_limit_deg <= 80.0f) &&
           (profile->angle_p >= 0.1f) &&
           (profile->angle_p <= 20.0f) &&
           (profile->rate_limit_dps >= 50.0f) &&
           (profile->rate_limit_dps <= 2000.0f);
}

bool angle_outer_loop_compute(
    const angle_outer_loop_profile_t *profile,
    const float normalized_stick[ANGLE_OUTER_LOOP_AXIS_COUNT],
    float yaw_rate_dps,
    const float attitude_deg[ANGLE_OUTER_LOOP_LEVEL_AXIS_COUNT],
    angle_outer_loop_output_t *output)
{
    uint32_t axis;

    if (output == NULL) {
        return false;
    }
    memset(output, 0, sizeof(*output));
    if (!angle_outer_loop_profile_is_valid(profile) ||
        (normalized_stick == NULL) || (attitude_deg == NULL) ||
        !isfinite(yaw_rate_dps) ||
        (fabsf(yaw_rate_dps) > 2000.0f)) {
        return false;
    }

    for (axis = 0U; axis < ANGLE_OUTER_LOOP_AXIS_COUNT; ++axis) {
        if (!isfinite(normalized_stick[axis]) ||
            (normalized_stick[axis] < -1.0f) ||
            (normalized_stick[axis] > 1.0f)) {
            return false;
        }
    }
    if (!isfinite(attitude_deg[0]) || !isfinite(attitude_deg[1]) ||
        (fabsf(attitude_deg[0]) > 180.0f) ||
        (fabsf(attitude_deg[1]) > 90.0f)) {
        return false;
    }

    for (axis = 0U; axis < ANGLE_OUTER_LOOP_LEVEL_AXIS_COUNT; ++axis) {
        output->target_angle_deg[axis] =
            normalized_stick[axis] * profile->angle_limit_deg;
        output->current_angle_deg[axis] = attitude_deg[axis];
        output->error_angle_deg[axis] =
            output->target_angle_deg[axis] - attitude_deg[axis];
        output->target_rate_dps[axis] = clampf(
            output->error_angle_deg[axis] * profile->angle_p,
            -profile->rate_limit_dps,
            profile->rate_limit_dps);
    }
    output->target_rate_dps[2] = yaw_rate_dps;
    output->valid = true;
    return true;
}

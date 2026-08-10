#include "algorithms/rate_pid.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static const rate_pid_profile_t default_profile = {
    .kp = {0.10f, 0.10f, 0.12f},
    .ki = {0.25f, 0.25f, 0.25f},
    .kd = {0.001f, 0.001f, 0.0f},
    .integral_limit = 0.20f,
    .output_limit = 0.50f,
    .minimum_dt_s = 0.0005f,
    .maximum_dt_s = 0.002f,
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

const rate_pid_profile_t *rate_pid_default_profile(void)
{
    return &default_profile;
}

bool rate_pid_profile_is_valid(const rate_pid_profile_t *profile)
{
    uint32_t axis;

    if ((profile == NULL) ||
        !isfinite(profile->integral_limit) ||
        !isfinite(profile->output_limit) ||
        !isfinite(profile->minimum_dt_s) ||
        !isfinite(profile->maximum_dt_s) ||
        (profile->integral_limit <= 0.0f) ||
        (profile->output_limit <= 0.0f) ||
        (profile->integral_limit > profile->output_limit) ||
        (profile->minimum_dt_s <= 0.0f) ||
        (profile->minimum_dt_s > profile->maximum_dt_s)) {
        return false;
    }

    for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
        if (!isfinite(profile->kp[axis]) ||
            !isfinite(profile->ki[axis]) ||
            !isfinite(profile->kd[axis]) ||
            (profile->kp[axis] < 0.0f) ||
            (profile->ki[axis] < 0.0f) ||
            (profile->kd[axis] < 0.0f)) {
            return false;
        }
    }
    return true;
}

void rate_pid_reset(rate_pid_state_t *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

bool rate_pid_update(
    const rate_pid_profile_t *profile,
    rate_pid_state_t *state,
    const float setpoint_rad_s[RATE_PID_AXIS_COUNT],
    const float measurement_rad_s[RATE_PID_AXIS_COUNT],
    float dt_s,
    bool integrator_enabled,
    rate_pid_output_t *output)
{
    uint32_t axis;

    if (output == NULL) {
        return false;
    }
    memset(output, 0, sizeof(*output));
    if ((state == NULL) || (setpoint_rad_s == NULL) ||
        (measurement_rad_s == NULL) ||
        !rate_pid_profile_is_valid(profile) || !isfinite(dt_s) ||
        (dt_s < profile->minimum_dt_s) ||
        (dt_s > profile->maximum_dt_s)) {
        rate_pid_reset(state);
        return false;
    }

    for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
        float candidate_integral;
        float unclamped;

        if (!isfinite(setpoint_rad_s[axis]) ||
            !isfinite(measurement_rad_s[axis])) {
            rate_pid_reset(state);
            memset(output, 0, sizeof(*output));
            return false;
        }

        output->setpoint_rad_s[axis] = setpoint_rad_s[axis];
        output->measurement_rad_s[axis] = measurement_rad_s[axis];
        output->error_rad_s[axis] =
            setpoint_rad_s[axis] - measurement_rad_s[axis];
        output->p[axis] =
            profile->kp[axis] * output->error_rad_s[axis];
        output->d[axis] = state->measurement_initialized
                              ? -profile->kd[axis] *
                                    (measurement_rad_s[axis] -
                                     state->previous_measurement_rad_s[axis]) /
                                    dt_s
                              : 0.0f;

        candidate_integral = integrator_enabled
                                 ? clampf(
                                       state->integral[axis] +
                                           profile->ki[axis] *
                                               output->error_rad_s[axis] * dt_s,
                                       -profile->integral_limit,
                                       profile->integral_limit)
                                 : 0.0f;
        unclamped = output->p[axis] + candidate_integral +
                    output->d[axis];
        if (((unclamped > profile->output_limit) &&
             (output->error_rad_s[axis] > 0.0f)) ||
            ((unclamped < -profile->output_limit) &&
             (output->error_rad_s[axis] < 0.0f))) {
            candidate_integral = integrator_enabled
                                     ? state->integral[axis]
                                     : 0.0f;
            unclamped = output->p[axis] + candidate_integral +
                        output->d[axis];
        }

        state->integral[axis] = candidate_integral;
        state->previous_measurement_rad_s[axis] =
            measurement_rad_s[axis];
        output->i[axis] = candidate_integral;
        output->correction[axis] =
            clampf(unclamped,
                   -profile->output_limit,
                   profile->output_limit);
        if (output->correction[axis] != unclamped) {
            output->saturated_mask |= (uint8_t)(1U << axis);
        }
    }

    state->measurement_initialized = true;
    output->valid = true;
    return true;
}

#include "algorithms/quad_x_mixer.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static const float quad_x_matrix[QUAD_X_MIXER_MOTOR_COUNT]
                                [QUAD_X_MIXER_AXIS_COUNT] = {
    [QUAD_X_MOTOR_REAR_RIGHT] = {-1.0f, 1.0f, -1.0f},
    [QUAD_X_MOTOR_FRONT_RIGHT] = {-1.0f, -1.0f, 1.0f},
    [QUAD_X_MOTOR_REAR_LEFT] = {1.0f, 1.0f, 1.0f},
    [QUAD_X_MOTOR_FRONT_LEFT] = {1.0f, -1.0f, -1.0f},
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

bool quad_x_mixer_compute(
    float throttle,
    const float correction[QUAD_X_MIXER_AXIS_COUNT],
    quad_x_mixer_output_t *output)
{
    float motor_correction[QUAD_X_MIXER_MOTOR_COUNT];
    float minimum;
    float maximum;
    float range;
    uint32_t motor;
    uint32_t axis;

    if (output == NULL) {
        return false;
    }
    memset(output, 0, sizeof(*output));
    if ((correction == NULL) || !isfinite(throttle) ||
        (throttle < 0.0f) || (throttle > 1.0f)) {
        return false;
    }
    for (axis = 0U; axis < QUAD_X_MIXER_AXIS_COUNT; ++axis) {
        if (!isfinite(correction[axis])) {
            return false;
        }
        output->correction[axis] = correction[axis];
    }

    for (motor = 0U; motor < QUAD_X_MIXER_MOTOR_COUNT; ++motor) {
        motor_correction[motor] = 0.0f;
        for (axis = 0U; axis < QUAD_X_MIXER_AXIS_COUNT; ++axis) {
            motor_correction[motor] +=
                quad_x_matrix[motor][axis] * correction[axis];
        }
    }

    minimum = motor_correction[0];
    maximum = motor_correction[0];
    for (motor = 1U; motor < QUAD_X_MIXER_MOTOR_COUNT; ++motor) {
        if (motor_correction[motor] < minimum) {
            minimum = motor_correction[motor];
        }
        if (motor_correction[motor] > maximum) {
            maximum = motor_correction[motor];
        }
    }

    range = maximum - minimum;
    output->correction_scale = range > 1.0f ? 1.0f / range : 1.0f;
    minimum *= output->correction_scale;
    maximum *= output->correction_scale;
    output->requested_throttle = throttle;
    output->applied_throttle =
        clampf(throttle, -minimum, 1.0f - maximum);
    output->saturated =
        (output->correction_scale < 1.0f) ||
        (output->applied_throttle != throttle);

    for (motor = 0U; motor < QUAD_X_MIXER_MOTOR_COUNT; ++motor) {
        output->motor[motor] =
            clampf(output->applied_throttle +
                       motor_correction[motor] *
                           output->correction_scale,
                   0.0f,
                   1.0f);
    }
    output->valid = true;
    return true;
}

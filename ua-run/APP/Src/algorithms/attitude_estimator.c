/*
 * attitude_estimator.c - Six-axis Mahony quaternion implementation.
 *
 * The feedback error is measured_down x estimated_down. Proportional feedback
 * corrects roll/pitch continuously while acceleration magnitude is credible.
 * Integral feedback is accumulated only below the spin-rate gate and is
 * clamped per axis. With no magnetometer, yaw remains gyro-integrated.
 */
#include "algorithms/attitude_estimator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RADIANS_TO_DEGREES_F 57.29577951308232088f
#define QUATERNION_MIN_NORM_SQUARED 1.0e-12f

static void increment_saturating(uint32_t *counter)
{
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

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

static bool vectors_are_finite(
    const float acceleration_m_s2[ATTITUDE_ESTIMATOR_AXIS_COUNT],
    const float angular_rate_rad_s[ATTITUDE_ESTIMATOR_AXIS_COUNT])
{
    uint32_t axis;

    for (axis = 0U; axis < ATTITUDE_ESTIMATOR_AXIS_COUNT; ++axis) {
        if (!isfinite(acceleration_m_s2[axis]) ||
            !isfinite(angular_rate_rad_s[axis])) {
            return false;
        }
    }
    return true;
}

static bool normalize_measured_down(
    const float acceleration_m_s2[ATTITUDE_ESTIMATOR_AXIS_COUNT],
    float measured_down[ATTITUDE_ESTIMATOR_AXIS_COUNT])
{
    const float minimum =
        ATTITUDE_ESTIMATOR_MIN_ACCEL_G *
        ATTITUDE_ESTIMATOR_STANDARD_GRAVITY_M_S2;
    const float maximum =
        ATTITUDE_ESTIMATOR_MAX_ACCEL_G *
        ATTITUDE_ESTIMATOR_STANDARD_GRAVITY_M_S2;
    const float magnitude_squared =
        acceleration_m_s2[0] * acceleration_m_s2[0] +
        acceleration_m_s2[1] * acceleration_m_s2[1] +
        acceleration_m_s2[2] * acceleration_m_s2[2];
    float reciprocal_magnitude;

    if (!isfinite(magnitude_squared) ||
        (magnitude_squared < (minimum * minimum)) ||
        (magnitude_squared > (maximum * maximum))) {
        return false;
    }

    reciprocal_magnitude = 1.0f / sqrtf(magnitude_squared);
    if (!isfinite(reciprocal_magnitude) ||
        (reciprocal_magnitude <= 0.0f)) {
        return false;
    }

    measured_down[0] =
        -acceleration_m_s2[0] * reciprocal_magnitude;
    measured_down[1] =
        -acceleration_m_s2[1] * reciprocal_magnitude;
    measured_down[2] =
        -acceleration_m_s2[2] * reciprocal_magnitude;
    return isfinite(measured_down[0]) &&
           isfinite(measured_down[1]) &&
           isfinite(measured_down[2]);
}

static bool normalize_quaternion(attitude_estimator_t *estimator)
{
    const float norm_squared =
        estimator->quaternion[0] * estimator->quaternion[0] +
        estimator->quaternion[1] * estimator->quaternion[1] +
        estimator->quaternion[2] * estimator->quaternion[2] +
        estimator->quaternion[3] * estimator->quaternion[3];
    float reciprocal_norm;
    uint32_t element;

    if (!isfinite(norm_squared) ||
        (norm_squared < QUATERNION_MIN_NORM_SQUARED)) {
        return false;
    }

    reciprocal_norm = 1.0f / sqrtf(norm_squared);
    if (!isfinite(reciprocal_norm) || (reciprocal_norm <= 0.0f)) {
        return false;
    }

    for (element = 0U;
         element < ATTITUDE_ESTIMATOR_QUATERNION_COUNT;
         ++element) {
        estimator->quaternion[element] *= reciprocal_norm;
        if (!isfinite(estimator->quaternion[element])) {
            return false;
        }
    }
    return true;
}

static bool update_euler(attitude_estimator_t *estimator)
{
    const float w = estimator->quaternion[0];
    const float x = estimator->quaternion[1];
    const float y = estimator->quaternion[2];
    const float z = estimator->quaternion[3];
    const float pitch_sine =
        clampf(2.0f * ((w * y) - (z * x)), -1.0f, 1.0f);
    float yaw_deg;

    estimator->roll_deg =
        atan2f(2.0f * ((w * x) + (y * z)),
               1.0f - 2.0f * ((x * x) + (y * y))) *
        RADIANS_TO_DEGREES_F;
    estimator->pitch_deg =
        asinf(pitch_sine) * RADIANS_TO_DEGREES_F;
    yaw_deg =
        atan2f(2.0f * ((w * z) + (x * y)),
               1.0f - 2.0f * ((y * y) + (z * z))) *
        RADIANS_TO_DEGREES_F;

    if (yaw_deg < 0.0f) {
        yaw_deg += 360.0f;
    }
    if (yaw_deg >= 360.0f) {
        yaw_deg -= 360.0f;
    }
    estimator->yaw_deg = yaw_deg;

    return isfinite(estimator->roll_deg) &&
           isfinite(estimator->pitch_deg) &&
           isfinite(estimator->yaw_deg);
}

static bool seed_from_gravity(
    attitude_estimator_t *estimator,
    const float measured_down[ATTITUDE_ESTIMATOR_AXIS_COUNT])
{
    const float roll =
        atan2f(measured_down[1], measured_down[2]);
    const float horizontal =
        sqrtf(measured_down[1] * measured_down[1] +
              measured_down[2] * measured_down[2]);
    const float pitch =
        atan2f(-measured_down[0], horizontal);
    const float half_roll = 0.5f * roll;
    const float half_pitch = 0.5f * pitch;
    const float cosine_roll = cosf(half_roll);
    const float sine_roll = sinf(half_roll);
    const float cosine_pitch = cosf(half_pitch);
    const float sine_pitch = sinf(half_pitch);

    estimator->quaternion[0] = cosine_roll * cosine_pitch;
    estimator->quaternion[1] = sine_roll * cosine_pitch;
    estimator->quaternion[2] = cosine_roll * sine_pitch;
    estimator->quaternion[3] = -sine_roll * sine_pitch;
    memset(estimator->integral_feedback_rad_s,
           0,
           sizeof(estimator->integral_feedback_rad_s));

    if (!normalize_quaternion(estimator) ||
        !update_euler(estimator)) {
        return false;
    }

    estimator->initialized = true;
    estimator->ready = true;
    return true;
}

void attitude_estimator_initialize(attitude_estimator_t *estimator)
{
    if (estimator == NULL) {
        return;
    }

    memset(estimator, 0, sizeof(*estimator));
    estimator->quaternion[0] = 1.0f;
}

void attitude_estimator_reset(attitude_estimator_t *estimator)
{
    if (estimator == NULL) {
        return;
    }

    estimator->initialized = false;
    estimator->ready = false;
    estimator->quaternion[0] = 1.0f;
    estimator->quaternion[1] = 0.0f;
    estimator->quaternion[2] = 0.0f;
    estimator->quaternion[3] = 0.0f;
    memset(estimator->integral_feedback_rad_s,
           0,
           sizeof(estimator->integral_feedback_rad_s));
    estimator->roll_deg = 0.0f;
    estimator->pitch_deg = 0.0f;
    estimator->yaw_deg = 0.0f;
    increment_saturating(&estimator->reset_count);
}

bool attitude_estimator_update(
    attitude_estimator_t *estimator,
    const float acceleration_m_s2[ATTITUDE_ESTIMATOR_AXIS_COUNT],
    const float angular_rate_rad_s[ATTITUDE_ESTIMATOR_AXIS_COUNT],
    float dt_s)
{
    float measured_down[ATTITUDE_ESTIMATOR_AXIS_COUNT];
    bool acceleration_valid;
    float corrected_rate[ATTITUDE_ESTIMATOR_AXIS_COUNT];
    float previous_quaternion[ATTITUDE_ESTIMATOR_QUATERNION_COUNT];
    uint32_t axis;

    if (estimator == NULL) {
        return false;
    }
    if ((acceleration_m_s2 == NULL) ||
        (angular_rate_rad_s == NULL) ||
        !vectors_are_finite(acceleration_m_s2,
                            angular_rate_rad_s) ||
        !isfinite(dt_s) ||
        (dt_s < ATTITUDE_ESTIMATOR_MIN_DT_S) ||
        (dt_s > ATTITUDE_ESTIMATOR_MAX_DT_S)) {
        increment_saturating(&estimator->invalid_input_count);
        attitude_estimator_reset(estimator);
        return false;
    }

    acceleration_valid =
        normalize_measured_down(acceleration_m_s2, measured_down);
    if (!acceleration_valid) {
        increment_saturating(&estimator->accel_rejection_count);
    }

    if (!estimator->initialized) {
        if (!acceleration_valid ||
            !seed_from_gravity(estimator, measured_down)) {
            estimator->ready = false;
            return false;
        }
        increment_saturating(&estimator->update_count);
        return true;
    }

    memcpy(corrected_rate,
           angular_rate_rad_s,
           sizeof(corrected_rate));
    if (acceleration_valid) {
        const float w = estimator->quaternion[0];
        const float x = estimator->quaternion[1];
        const float y = estimator->quaternion[2];
        const float z = estimator->quaternion[3];
        const float estimated_down[ATTITUDE_ESTIMATOR_AXIS_COUNT] = {
            2.0f * ((x * z) - (w * y)),
            2.0f * ((w * x) + (y * z)),
            (w * w) - (x * x) - (y * y) + (z * z)};
        const float error[ATTITUDE_ESTIMATOR_AXIS_COUNT] = {
            measured_down[1] * estimated_down[2] -
                measured_down[2] * estimated_down[1],
            measured_down[2] * estimated_down[0] -
                measured_down[0] * estimated_down[2],
            measured_down[0] * estimated_down[1] -
                measured_down[1] * estimated_down[0]};
        const float spin_rate_squared =
            angular_rate_rad_s[0] * angular_rate_rad_s[0] +
            angular_rate_rad_s[1] * angular_rate_rad_s[1] +
            angular_rate_rad_s[2] * angular_rate_rad_s[2];
        const float spin_limit_squared =
            ATTITUDE_ESTIMATOR_INTEGRAL_SPIN_LIMIT_RAD_S *
            ATTITUDE_ESTIMATOR_INTEGRAL_SPIN_LIMIT_RAD_S;

        for (axis = 0U;
             axis < ATTITUDE_ESTIMATOR_AXIS_COUNT;
             ++axis) {
            if (spin_rate_squared < spin_limit_squared) {
                estimator->integral_feedback_rad_s[axis] =
                    clampf(
                        estimator->integral_feedback_rad_s[axis] +
                            ATTITUDE_ESTIMATOR_KI *
                                error[axis] * dt_s,
                        -ATTITUDE_ESTIMATOR_INTEGRAL_LIMIT_RAD_S,
                        ATTITUDE_ESTIMATOR_INTEGRAL_LIMIT_RAD_S);
            }
            corrected_rate[axis] +=
                ATTITUDE_ESTIMATOR_KP * error[axis] +
                estimator->integral_feedback_rad_s[axis];
        }
    } else {
        increment_saturating(&estimator->gyro_only_update_count);
    }

    memcpy(previous_quaternion,
           estimator->quaternion,
           sizeof(previous_quaternion));
    estimator->quaternion[0] +=
        0.5f * dt_s *
        (-previous_quaternion[1] * corrected_rate[0] -
         previous_quaternion[2] * corrected_rate[1] -
         previous_quaternion[3] * corrected_rate[2]);
    estimator->quaternion[1] +=
        0.5f * dt_s *
        (previous_quaternion[0] * corrected_rate[0] +
         previous_quaternion[2] * corrected_rate[2] -
         previous_quaternion[3] * corrected_rate[1]);
    estimator->quaternion[2] +=
        0.5f * dt_s *
        (previous_quaternion[0] * corrected_rate[1] -
         previous_quaternion[1] * corrected_rate[2] +
         previous_quaternion[3] * corrected_rate[0]);
    estimator->quaternion[3] +=
        0.5f * dt_s *
        (previous_quaternion[0] * corrected_rate[2] +
         previous_quaternion[1] * corrected_rate[1] -
         previous_quaternion[2] * corrected_rate[0]);

    if (!normalize_quaternion(estimator) ||
        !update_euler(estimator)) {
        increment_saturating(&estimator->invalid_input_count);
        attitude_estimator_reset(estimator);
        return false;
    }

    estimator->ready = true;
    increment_saturating(&estimator->update_count);
    return true;
}

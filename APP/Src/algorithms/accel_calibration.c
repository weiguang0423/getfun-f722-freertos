/*
 * accel_calibration.c - Horizontal single-face calibration implementation.
 *
 * Each candidate sample must be finite, remain below 5 deg/s, have 0.8g to
 * 1.2g total magnitude, and have no more than 0.12g horizontal magnitude.
 * After 250 warmup samples, 2000 samples are accumulated with Welford mean and
 * variance. Final horizontal mean, Z magnitude, per-axis standard deviation
 * and resulting bias are checked before exposing a candidate.
 *
 * Bias is computed against +g or -g according to mean Z sign, so either level
 * face yields the same sensor offset. The old committed bias remains active
 * during recalibration and is replaced only by mark_persisted().
 */
#include "algorithms/accel_calibration.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define STANDARD_GRAVITY_M_S2 9.80665f
#define ACCEL_CAL_MAX_RATE_RAD_S 0.0872664626f
#define ACCEL_CAL_MIN_MAGNITUDE_M_S2 (0.8f * STANDARD_GRAVITY_M_S2)
#define ACCEL_CAL_MAX_MAGNITUDE_M_S2 (1.2f * STANDARD_GRAVITY_M_S2)
#define ACCEL_CAL_MAX_INSTANT_HORIZONTAL_M_S2 \
    (0.12f * STANDARD_GRAVITY_M_S2)
#define ACCEL_CAL_MAX_MEAN_HORIZONTAL_M_S2 \
    (0.08f * STANDARD_GRAVITY_M_S2)
#define ACCEL_CAL_MIN_MEAN_Z_M_S2 (0.9f * STANDARD_GRAVITY_M_S2)
#define ACCEL_CAL_MAX_MEAN_Z_M_S2 (1.1f * STANDARD_GRAVITY_M_S2)
#define ACCEL_CAL_MAX_STDDEV_M_S2 (0.03f * STANDARD_GRAVITY_M_S2)
#define ACCEL_CAL_MAX_BIAS_M_S2 (0.2f * STANDARD_GRAVITY_M_S2)
#define SQUARE(value) ((value) * (value))

typedef enum
{
    SAMPLE_ACCEPTED = 0,
    SAMPLE_REJECT_MOTION,
    SAMPLE_REJECT_LEVEL,
    SAMPLE_REJECT_INVALID
} sample_result_t;

_Static_assert(ACCEL_CALIBRATION_REQUIRED_SAMPLES > 1U,
               "accel calibration variance needs two samples");
_Static_assert(ACCEL_CALIBRATION_REQUIRED_SAMPLES <= UINT16_MAX,
               "accel calibration counter is too small");

static void increment_saturating(uint32_t *counter)
{
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

static void clear_window(accel_calibration_t *calibration)
{
    calibration->warmup_sample_count = 0U;
    calibration->stable_sample_count = 0U;
    memset(calibration->candidate_bias_m_s2,
           0,
           sizeof(calibration->candidate_bias_m_s2));
    memset(calibration->mean_m_s2,
           0,
           sizeof(calibration->mean_m_s2));
    memset(calibration->m2_m2_s4,
           0,
           sizeof(calibration->m2_m2_s4));
}

static sample_result_t classify_sample(
    const float acceleration_m_s2[ACCEL_CALIBRATION_AXIS_COUNT],
    const float angular_rate_rad_s[ACCEL_CALIBRATION_AXIS_COUNT])
{
    float magnitude_squared = 0.0f;
    uint32_t axis;

    for (axis = 0U; axis < ACCEL_CALIBRATION_AXIS_COUNT; ++axis) {
        if (!isfinite(acceleration_m_s2[axis]) ||
            !isfinite(angular_rate_rad_s[axis])) {
            return SAMPLE_REJECT_INVALID;
        }
        if (fabsf(angular_rate_rad_s[axis]) >
            ACCEL_CAL_MAX_RATE_RAD_S) {
            return SAMPLE_REJECT_MOTION;
        }
        magnitude_squared +=
            acceleration_m_s2[axis] * acceleration_m_s2[axis];
    }

    if ((magnitude_squared <
         SQUARE(ACCEL_CAL_MIN_MAGNITUDE_M_S2)) ||
        (magnitude_squared >
         SQUARE(ACCEL_CAL_MAX_MAGNITUDE_M_S2))) {
        return SAMPLE_REJECT_MOTION;
    }

    if ((SQUARE(acceleration_m_s2[0]) +
         SQUARE(acceleration_m_s2[1])) >
        SQUARE(ACCEL_CAL_MAX_INSTANT_HORIZONTAL_M_S2)) {
        return SAMPLE_REJECT_LEVEL;
    }
    return SAMPLE_ACCEPTED;
}

static void reject_window(accel_calibration_t *calibration,
                          sample_result_t result)
{
    if ((calibration->warmup_sample_count != 0U) ||
        (calibration->stable_sample_count != 0U)) {
        increment_saturating(&calibration->restart_count);
    }

    switch (result) {
    case SAMPLE_REJECT_INVALID:
        increment_saturating(&calibration->invalid_sample_count);
        break;
    case SAMPLE_REJECT_LEVEL:
        increment_saturating(&calibration->level_reject_count);
        break;
    case SAMPLE_REJECT_MOTION:
    default:
        increment_saturating(&calibration->motion_reject_count);
        break;
    }
    clear_window(calibration);
}

static bool final_window_is_valid(accel_calibration_t *calibration)
{
    const float denominator =
        (float)(ACCEL_CALIBRATION_REQUIRED_SAMPLES - 1U);
    const float horizontal_squared =
        SQUARE(calibration->mean_m_s2[0]) +
        SQUARE(calibration->mean_m_s2[1]);
    const float mean_z_abs =
        fabsf(calibration->mean_m_s2[2]);
    uint32_t axis;

    if ((horizontal_squared >
         SQUARE(ACCEL_CAL_MAX_MEAN_HORIZONTAL_M_S2)) ||
        (mean_z_abs < ACCEL_CAL_MIN_MEAN_Z_M_S2) ||
        (mean_z_abs > ACCEL_CAL_MAX_MEAN_Z_M_S2)) {
        return false;
    }

    for (axis = 0U; axis < ACCEL_CALIBRATION_AXIS_COUNT; ++axis) {
        const float variance =
            calibration->m2_m2_s4[axis] / denominator;

        if (!isfinite(variance) ||
            (variance > SQUARE(ACCEL_CAL_MAX_STDDEV_M_S2))) {
            return false;
        }
    }

    calibration->candidate_bias_m_s2[0] =
        calibration->mean_m_s2[0];
    calibration->candidate_bias_m_s2[1] =
        calibration->mean_m_s2[1];
    calibration->candidate_bias_m_s2[2] =
        calibration->mean_m_s2[2] -
        copysignf(STANDARD_GRAVITY_M_S2,
                  calibration->mean_m_s2[2]);

    for (axis = 0U; axis < ACCEL_CALIBRATION_AXIS_COUNT; ++axis) {
        if (!isfinite(calibration->candidate_bias_m_s2[axis]) ||
            (fabsf(calibration->candidate_bias_m_s2[axis]) >
             ACCEL_CAL_MAX_BIAS_M_S2)) {
            return false;
        }
    }
    return true;
}

void accel_calibration_initialize(
    accel_calibration_t *calibration,
    const float bias_m_s2[ACCEL_CALIBRATION_AXIS_COUNT],
    bool bias_valid)
{
    if (calibration == NULL) {
        return;
    }

    memset(calibration, 0, sizeof(*calibration));
    if (bias_valid && (bias_m_s2 != NULL)) {
        uint32_t axis;

        for (axis = 0U;
             axis < ACCEL_CALIBRATION_AXIS_COUNT;
             ++axis) {
            if (!isfinite(bias_m_s2[axis]) ||
                (fabsf(bias_m_s2[axis]) >
                 ACCEL_CAL_MAX_BIAS_M_S2)) {
                calibration->state =
                    ACCEL_CALIBRATION_NOT_CALIBRATED;
                return;
            }
        }
        memcpy(calibration->bias_m_s2,
               bias_m_s2,
               sizeof(calibration->bias_m_s2));
        calibration->bias_valid = true;
        calibration->state = ACCEL_CALIBRATION_READY;
    } else {
        calibration->state =
            ACCEL_CALIBRATION_NOT_CALIBRATED;
    }
}

bool accel_calibration_start(accel_calibration_t *calibration)
{
    if ((calibration == NULL) ||
        (calibration->state == ACCEL_CALIBRATION_CALIBRATING) ||
        (calibration->state ==
         ACCEL_CALIBRATION_CANDIDATE_READY)) {
        return false;
    }

    clear_window(calibration);
    calibration->restart_count = 0U;
    calibration->motion_reject_count = 0U;
    calibration->level_reject_count = 0U;
    calibration->invalid_sample_count = 0U;
    calibration->state = ACCEL_CALIBRATION_CALIBRATING;
    return true;
}

accel_calibration_state_t accel_calibration_process(
    accel_calibration_t *calibration,
    const float acceleration_m_s2[ACCEL_CALIBRATION_AXIS_COUNT],
    const float angular_rate_rad_s[ACCEL_CALIBRATION_AXIS_COUNT])
{
    sample_result_t result;
    uint32_t axis;

    if ((calibration == NULL) ||
        (acceleration_m_s2 == NULL) ||
        (angular_rate_rad_s == NULL)) {
        return ACCEL_CALIBRATION_NOT_CALIBRATED;
    }
    if (calibration->state != ACCEL_CALIBRATION_CALIBRATING) {
        return calibration->state;
    }

    result = classify_sample(acceleration_m_s2,
                             angular_rate_rad_s);
    if (result != SAMPLE_ACCEPTED) {
        reject_window(calibration, result);
        return calibration->state;
    }

    if (calibration->warmup_sample_count <
        ACCEL_CALIBRATION_WARMUP_SAMPLES) {
        ++calibration->warmup_sample_count;
        return calibration->state;
    }

    ++calibration->stable_sample_count;
    for (axis = 0U; axis < ACCEL_CALIBRATION_AXIS_COUNT; ++axis) {
        const float count =
            (float)calibration->stable_sample_count;
        const float delta =
            acceleration_m_s2[axis] -
            calibration->mean_m_s2[axis];
        const float next_mean =
            calibration->mean_m_s2[axis] +
            (delta / count);

        calibration->m2_m2_s4[axis] +=
            delta * (acceleration_m_s2[axis] - next_mean);
        calibration->mean_m_s2[axis] = next_mean;
    }

    if (calibration->stable_sample_count <
        ACCEL_CALIBRATION_REQUIRED_SAMPLES) {
        return calibration->state;
    }

    if (!final_window_is_valid(calibration)) {
        reject_window(calibration, SAMPLE_REJECT_LEVEL);
        return calibration->state;
    }

    calibration->state =
        ACCEL_CALIBRATION_CANDIDATE_READY;
    return calibration->state;
}

bool accel_calibration_get_candidate(
    const accel_calibration_t *calibration,
    float candidate_bias_m_s2[ACCEL_CALIBRATION_AXIS_COUNT])
{
    if ((calibration == NULL) ||
        (candidate_bias_m_s2 == NULL) ||
        (calibration->state !=
         ACCEL_CALIBRATION_CANDIDATE_READY)) {
        return false;
    }

    memcpy(candidate_bias_m_s2,
           calibration->candidate_bias_m_s2,
           sizeof(calibration->candidate_bias_m_s2));
    return true;
}

void accel_calibration_mark_persisted(
    accel_calibration_t *calibration)
{
    if ((calibration == NULL) ||
        (calibration->state !=
         ACCEL_CALIBRATION_CANDIDATE_READY)) {
        return;
    }

    memcpy(calibration->bias_m_s2,
           calibration->candidate_bias_m_s2,
           sizeof(calibration->bias_m_s2));
    calibration->bias_valid = true;
    calibration->state = ACCEL_CALIBRATION_READY;
}

void accel_calibration_mark_save_failed(
    accel_calibration_t *calibration)
{
    if ((calibration != NULL) &&
        (calibration->state ==
         ACCEL_CALIBRATION_CANDIDATE_READY)) {
        calibration->state = ACCEL_CALIBRATION_SAVE_FAILED;
    }
}

void accel_calibration_apply(
    const accel_calibration_t *calibration,
    const float acceleration_m_s2[ACCEL_CALIBRATION_AXIS_COUNT],
    float corrected_m_s2[ACCEL_CALIBRATION_AXIS_COUNT])
{
    uint32_t axis;

    if ((calibration == NULL) ||
        (acceleration_m_s2 == NULL) ||
        (corrected_m_s2 == NULL)) {
        return;
    }

    for (axis = 0U; axis < ACCEL_CALIBRATION_AXIS_COUNT; ++axis) {
        corrected_m_s2[axis] =
            acceleration_m_s2[axis] -
            (calibration->bias_valid
                 ? calibration->bias_m_s2[axis]
                 : 0.0f);
    }
}

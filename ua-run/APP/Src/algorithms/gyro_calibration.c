/*
 * gyro_calibration.c —— 上电陀螺静态零偏校准状态机实现
 *
 * 本模块接收已经转换到机体系的 SI 单位 IMU 样本。每个候选样本必须满足三轴
 * 角速度绝对值不超过 5 deg/s，且加速度模长位于 0.8 g～1.2 g；连续 250 个
 * 静止样本用于预热，随后用 Welford 算法累计 2000 个样本。只有三轴标准差均
 * 不超过 0.5 deg/s 时，窗口均值才会冻结为零偏。
 *
 * 运动、无效浮点值或窗口方差超限都会丢弃当前进度并重新等待连续静止窗口。
 * READY 后零偏保持不变，直至调用者因传感器重新初始化而再次 start/reset。
 */
#include "algorithms/gyro_calibration.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define STANDARD_GRAVITY_M_S2 9.80665f
#define GYRO_STATIONARY_MAX_RATE_RAD_S 0.0872664626f
#define GYRO_STATIONARY_MAX_STDDEV_RAD_S 0.0087266463f
#define ACCEL_STATIONARY_MIN_M_S2 (0.8f * STANDARD_GRAVITY_M_S2)
#define ACCEL_STATIONARY_MAX_M_S2 (1.2f * STANDARD_GRAVITY_M_S2)
#define ACCEL_STATIONARY_MIN_SQUARED \
    (ACCEL_STATIONARY_MIN_M_S2 * ACCEL_STATIONARY_MIN_M_S2)
#define ACCEL_STATIONARY_MAX_SQUARED \
    (ACCEL_STATIONARY_MAX_M_S2 * ACCEL_STATIONARY_MAX_M_S2)
#define GYRO_STATIONARY_MAX_VARIANCE_RAD_S2 \
    (GYRO_STATIONARY_MAX_STDDEV_RAD_S * \
     GYRO_STATIONARY_MAX_STDDEV_RAD_S)

_Static_assert(GYRO_CALIBRATION_REQUIRED_SAMPLES > 1U,
               "gyro calibration variance needs at least two samples");
_Static_assert(GYRO_CALIBRATION_REQUIRED_SAMPLES <= UINT16_MAX,
               "gyro calibration sample counter is too small");

static void increment_saturating(uint32_t *counter)
{
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

static void clear_window(gyro_calibration_t *calibration)
{
    calibration->warmup_sample_count = 0U;
    calibration->stable_sample_count = 0U;
    memset(calibration->mean_rad_s, 0, sizeof(calibration->mean_rad_s));
    memset(calibration->m2_rad_s2, 0, sizeof(calibration->m2_rad_s2));
}

static void reject_window(gyro_calibration_t *calibration, bool invalid)
{
    if ((calibration->warmup_sample_count != 0U) ||
        (calibration->stable_sample_count != 0U)) {
        increment_saturating(&calibration->restart_count);
    }

    if (invalid) {
        increment_saturating(&calibration->invalid_sample_count);
    } else {
        increment_saturating(&calibration->motion_reject_count);
    }

    clear_window(calibration);
    calibration->state = GYRO_CALIBRATION_CALIBRATING;
}

static bool sample_is_finite(
    const float angular_rate_rad_s[GYRO_CALIBRATION_AXIS_COUNT],
    const float acceleration_m_s2[GYRO_CALIBRATION_AXIS_COUNT])
{
    uint32_t axis;

    for (axis = 0U; axis < GYRO_CALIBRATION_AXIS_COUNT; ++axis) {
        if (!isfinite(angular_rate_rad_s[axis]) ||
            !isfinite(acceleration_m_s2[axis])) {
            return false;
        }
    }
    return true;
}

static bool sample_is_stationary(
    const float angular_rate_rad_s[GYRO_CALIBRATION_AXIS_COUNT],
    const float acceleration_m_s2[GYRO_CALIBRATION_AXIS_COUNT])
{
    float acceleration_magnitude_squared = 0.0f;
    uint32_t axis;

    for (axis = 0U; axis < GYRO_CALIBRATION_AXIS_COUNT; ++axis) {
        if (fabsf(angular_rate_rad_s[axis]) >
            GYRO_STATIONARY_MAX_RATE_RAD_S) {
            return false;
        }
        acceleration_magnitude_squared +=
            acceleration_m_s2[axis] * acceleration_m_s2[axis];
    }

    return (acceleration_magnitude_squared >=
            ACCEL_STATIONARY_MIN_SQUARED) &&
           (acceleration_magnitude_squared <=
            ACCEL_STATIONARY_MAX_SQUARED);
}

static bool window_variance_is_acceptable(
    const gyro_calibration_t *calibration)
{
    const float denominator =
        (float)(GYRO_CALIBRATION_REQUIRED_SAMPLES - 1U);
    uint32_t axis;

    for (axis = 0U; axis < GYRO_CALIBRATION_AXIS_COUNT; ++axis) {
        const float variance =
            calibration->m2_rad_s2[axis] / denominator;

        if (!isfinite(variance) ||
            (variance > GYRO_STATIONARY_MAX_VARIANCE_RAD_S2)) {
            return false;
        }
    }
    return true;
}

void gyro_calibration_reset(gyro_calibration_t *calibration)
{
    if (calibration == NULL) {
        return;
    }

    memset(calibration, 0, sizeof(*calibration));
    calibration->state = GYRO_CALIBRATION_NOT_STARTED;
}

void gyro_calibration_start(gyro_calibration_t *calibration)
{
    gyro_calibration_reset(calibration);
    if (calibration != NULL) {
        calibration->state = GYRO_CALIBRATION_CALIBRATING;
    }
}

gyro_calibration_state_t gyro_calibration_process(
    gyro_calibration_t *calibration,
    const float angular_rate_rad_s[GYRO_CALIBRATION_AXIS_COUNT],
    const float acceleration_m_s2[GYRO_CALIBRATION_AXIS_COUNT])
{
    uint32_t axis;

    if ((calibration == NULL) ||
        (angular_rate_rad_s == NULL) ||
        (acceleration_m_s2 == NULL)) {
        return GYRO_CALIBRATION_NOT_STARTED;
    }

    if (calibration->state != GYRO_CALIBRATION_CALIBRATING) {
        return calibration->state;
    }

    if (!sample_is_finite(angular_rate_rad_s, acceleration_m_s2)) {
        reject_window(calibration, true);
        return calibration->state;
    }

    if (!sample_is_stationary(angular_rate_rad_s, acceleration_m_s2)) {
        reject_window(calibration, false);
        return calibration->state;
    }

    if (calibration->warmup_sample_count <
        GYRO_CALIBRATION_WARMUP_SAMPLES) {
        ++calibration->warmup_sample_count;
        return calibration->state;
    }

    ++calibration->stable_sample_count;
    for (axis = 0U; axis < GYRO_CALIBRATION_AXIS_COUNT; ++axis) {
        const float count = (float)calibration->stable_sample_count;
        const float delta =
            angular_rate_rad_s[axis] - calibration->mean_rad_s[axis];
        const float next_mean =
            calibration->mean_rad_s[axis] + (delta / count);
        const float delta_after_mean =
            angular_rate_rad_s[axis] - next_mean;

        calibration->mean_rad_s[axis] = next_mean;
        calibration->m2_rad_s2[axis] += delta * delta_after_mean;
    }

    if (calibration->stable_sample_count <
        GYRO_CALIBRATION_REQUIRED_SAMPLES) {
        return calibration->state;
    }

    if (!window_variance_is_acceptable(calibration)) {
        reject_window(calibration, false);
        return calibration->state;
    }

    memcpy(calibration->bias_rad_s,
           calibration->mean_rad_s,
           sizeof(calibration->bias_rad_s));
    calibration->state = GYRO_CALIBRATION_READY;
    return calibration->state;
}

void gyro_calibration_apply(
    const gyro_calibration_t *calibration,
    const float angular_rate_rad_s[GYRO_CALIBRATION_AXIS_COUNT],
    float corrected_rad_s[GYRO_CALIBRATION_AXIS_COUNT])
{
    uint32_t axis;

    if ((calibration == NULL) ||
        (angular_rate_rad_s == NULL) ||
        (corrected_rad_s == NULL)) {
        return;
    }

    for (axis = 0U; axis < GYRO_CALIBRATION_AXIS_COUNT; ++axis) {
        corrected_rad_s[axis] = angular_rate_rad_s[axis];
        if (calibration->state == GYRO_CALIBRATION_READY) {
            corrected_rad_s[axis] -= calibration->bias_rad_s[axis];
        }
    }
}

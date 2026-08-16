/*
 * imu_filter.c —— 按真实dt更新的三轴PT1低通实现
 *
 * Gyro固定100 Hz，Accel固定30 Hz。每轴使用：
 *   alpha = (2*pi*fc*dt) / (1 + 2*pi*fc*dt)
 *   y += alpha * (x - y)
 *
 * 第一版固定截止频率并保留小型、可验证实现；不包含Biquad、Notch、动态滤波
 * 或运行时参数修改。
 */
#include "algorithms/imu_filter.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define TWO_PI_F 6.28318530717958647692f

static bool vectors_are_finite(
    const float acceleration_m_s2[IMU_FILTER_AXIS_COUNT],
    const float angular_rate_rad_s[IMU_FILTER_AXIS_COUNT])
{
    uint32_t axis;

    for (axis = 0U; axis < IMU_FILTER_AXIS_COUNT; ++axis) {
        if (!isfinite(acceleration_m_s2[axis]) ||
            !isfinite(angular_rate_rad_s[axis])) {
            return false;
        }
    }
    return true;
}

static void seed_filter(
    imu_filter_t *filter,
    const float acceleration_m_s2[IMU_FILTER_AXIS_COUNT],
    const float angular_rate_rad_s[IMU_FILTER_AXIS_COUNT],
    float filtered_acceleration_m_s2[IMU_FILTER_AXIS_COUNT],
    float filtered_angular_rate_rad_s[IMU_FILTER_AXIS_COUNT])
{
    memcpy(filter->acceleration_state_m_s2,
           acceleration_m_s2,
           sizeof(filter->acceleration_state_m_s2));
    memcpy(filter->angular_rate_state_rad_s,
           angular_rate_rad_s,
           sizeof(filter->angular_rate_state_rad_s));
    memcpy(filtered_acceleration_m_s2,
           acceleration_m_s2,
           sizeof(filter->acceleration_state_m_s2));
    memcpy(filtered_angular_rate_rad_s,
           angular_rate_rad_s,
           sizeof(filter->angular_rate_state_rad_s));
    filter->initialized = true;
}

void imu_filter_reset(imu_filter_t *filter)
{
    if (filter != NULL) {
        memset(filter, 0, sizeof(*filter));
    }
}

bool imu_filter_process(
    imu_filter_t *filter,
    const float acceleration_m_s2[IMU_FILTER_AXIS_COUNT],
    const float angular_rate_rad_s[IMU_FILTER_AXIS_COUNT],
    float dt_s,
    float filtered_acceleration_m_s2[IMU_FILTER_AXIS_COUNT],
    float filtered_angular_rate_rad_s[IMU_FILTER_AXIS_COUNT])
{
    float acceleration_next[IMU_FILTER_AXIS_COUNT];
    float angular_rate_next[IMU_FILTER_AXIS_COUNT];
    float accel_alpha;
    float gyro_alpha;
    uint32_t axis;

    if ((filter == NULL) || (acceleration_m_s2 == NULL) ||
        (angular_rate_rad_s == NULL) ||
        (filtered_acceleration_m_s2 == NULL) ||
        (filtered_angular_rate_rad_s == NULL)) {
        return false;
    }

    if (!vectors_are_finite(acceleration_m_s2, angular_rate_rad_s)) {
        imu_filter_reset(filter);
        memset(filtered_acceleration_m_s2,
               0,
               sizeof(acceleration_next));
        memset(filtered_angular_rate_rad_s,
               0,
               sizeof(angular_rate_next));
        return false;
    }

    if (!filter->initialized) {
        seed_filter(filter,
                    acceleration_m_s2,
                    angular_rate_rad_s,
                    filtered_acceleration_m_s2,
                    filtered_angular_rate_rad_s);
        return false;
    }

    if (!isfinite(dt_s) || (dt_s < IMU_FILTER_MIN_DT_S) ||
        (dt_s > IMU_FILTER_MAX_DT_S)) {
        imu_filter_reset(filter);
        seed_filter(filter,
                    acceleration_m_s2,
                    angular_rate_rad_s,
                    filtered_acceleration_m_s2,
                    filtered_angular_rate_rad_s);
        return false;
    }

    accel_alpha =
        (TWO_PI_F * IMU_FILTER_ACCEL_CUTOFF_HZ * dt_s) /
        (1.0f + (TWO_PI_F * IMU_FILTER_ACCEL_CUTOFF_HZ * dt_s));
    gyro_alpha =
        (TWO_PI_F * IMU_FILTER_GYRO_CUTOFF_HZ * dt_s) /
        (1.0f + (TWO_PI_F * IMU_FILTER_GYRO_CUTOFF_HZ * dt_s));

    if (!isfinite(accel_alpha) || !isfinite(gyro_alpha) ||
        (accel_alpha <= 0.0f) || (accel_alpha > 1.0f) ||
        (gyro_alpha <= 0.0f) || (gyro_alpha > 1.0f)) {
        imu_filter_reset(filter);
        seed_filter(filter,
                    acceleration_m_s2,
                    angular_rate_rad_s,
                    filtered_acceleration_m_s2,
                    filtered_angular_rate_rad_s);
        return false;
    }

    for (axis = 0U; axis < IMU_FILTER_AXIS_COUNT; ++axis) {
        acceleration_next[axis] =
            filter->acceleration_state_m_s2[axis] +
            accel_alpha *
                (acceleration_m_s2[axis] -
                 filter->acceleration_state_m_s2[axis]);
        angular_rate_next[axis] =
            filter->angular_rate_state_rad_s[axis] +
            gyro_alpha *
                (angular_rate_rad_s[axis] -
                 filter->angular_rate_state_rad_s[axis]);

        if (!isfinite(acceleration_next[axis]) ||
            !isfinite(angular_rate_next[axis])) {
            imu_filter_reset(filter);
            seed_filter(filter,
                        acceleration_m_s2,
                        angular_rate_rad_s,
                        filtered_acceleration_m_s2,
                        filtered_angular_rate_rad_s);
            return false;
        }
    }

    memcpy(filter->acceleration_state_m_s2,
           acceleration_next,
           sizeof(acceleration_next));
    memcpy(filter->angular_rate_state_rad_s,
           angular_rate_next,
           sizeof(angular_rate_next));
    memcpy(filtered_acceleration_m_s2,
           acceleration_next,
           sizeof(acceleration_next));
    memcpy(filtered_angular_rate_rad_s,
           angular_rate_next,
           sizeof(angular_rate_next));
    return true;
}

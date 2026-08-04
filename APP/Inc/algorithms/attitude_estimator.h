/*
 * attitude_estimator.h - Six-axis Mahony quaternion attitude estimator.
 *
 * Coordinate and unit contract:
 *   - Body frame is FRD: X forward, Y right, Z down.
 *   - Angular rate is right-hand-positive body rate in rad/s.
 *   - Accelerometer input is calibrated specific force in m/s^2. The module
 *     negates and normalizes it to obtain the measured down-gravity direction.
 *   - Quaternion order is [w, x, y, z] and rotates body vectors into NED.
 *   - Euler output is roll right-positive, pitch nose-up-positive, and yaw
 *     clockwise-positive when viewed from above.
 *
 * The module has no HAL, RTOS, logging, dynamic-memory, or global-state
 * dependency. The first acceptable sample seeds roll/pitch from gravity and
 * yaw to zero. Invalid vectors or dt reset the estimate; acceleration outside
 * the configured magnitude gate causes a gyro-only update.
 */
#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ATTITUDE_ESTIMATOR_AXIS_COUNT 3U
#define ATTITUDE_ESTIMATOR_QUATERNION_COUNT 4U
#define ATTITUDE_ESTIMATOR_STANDARD_GRAVITY_M_S2 9.80665f
#define ATTITUDE_ESTIMATOR_KP 2.0f
#define ATTITUDE_ESTIMATOR_KI 0.05f
#define ATTITUDE_ESTIMATOR_INTEGRAL_LIMIT_RAD_S 0.25f
#define ATTITUDE_ESTIMATOR_INTEGRAL_SPIN_LIMIT_RAD_S 0.3490658504f
#define ATTITUDE_ESTIMATOR_MIN_ACCEL_G 0.8f
#define ATTITUDE_ESTIMATOR_MAX_ACCEL_G 1.2f
#define ATTITUDE_ESTIMATOR_MIN_DT_S 0.0005f
#define ATTITUDE_ESTIMATOR_MAX_DT_S 0.002f

typedef struct
{
    bool initialized;
    bool ready;
    float quaternion[ATTITUDE_ESTIMATOR_QUATERNION_COUNT];
    float integral_feedback_rad_s[ATTITUDE_ESTIMATOR_AXIS_COUNT];
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint32_t update_count;
    uint32_t reset_count;
    uint32_t invalid_input_count;
    uint32_t accel_rejection_count;
    uint32_t gyro_only_update_count;
} attitude_estimator_t;

void attitude_estimator_initialize(attitude_estimator_t *estimator);
void attitude_estimator_reset(attitude_estimator_t *estimator);
bool attitude_estimator_update(
    attitude_estimator_t *estimator,
    const float acceleration_m_s2[ATTITUDE_ESTIMATOR_AXIS_COUNT],
    const float angular_rate_rad_s[ATTITUDE_ESTIMATOR_AXIS_COUNT],
    float dt_s);

#ifdef __cplusplus
}
#endif

#endif

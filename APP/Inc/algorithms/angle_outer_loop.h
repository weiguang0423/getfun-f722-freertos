/*
 * angle_outer_loop.h - Pure S4.8 Roll/Pitch angle-to-rate controller.
 *
 * The module maps normalized Roll/Pitch sticks to bounded target angles,
 * multiplies attitude error by Angle P, clamps the resulting rate targets and
 * passes Yaw rate through unchanged. It has no HAL, RTOS or global-state
 * dependency; FlightTask owns validity gates and feeds its output to Rate PID.
 */
#ifndef ANGLE_OUTER_LOOP_H
#define ANGLE_OUTER_LOOP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ANGLE_OUTER_LOOP_AXIS_COUNT 3U
#define ANGLE_OUTER_LOOP_LEVEL_AXIS_COUNT 2U

typedef struct
{
    float angle_limit_deg;
    float angle_p;
    float rate_limit_dps;
} angle_outer_loop_profile_t;

typedef struct
{
    bool valid;
    float target_angle_deg[ANGLE_OUTER_LOOP_LEVEL_AXIS_COUNT];
    float current_angle_deg[ANGLE_OUTER_LOOP_LEVEL_AXIS_COUNT];
    float error_angle_deg[ANGLE_OUTER_LOOP_LEVEL_AXIS_COUNT];
    float target_rate_dps[ANGLE_OUTER_LOOP_AXIS_COUNT];
} angle_outer_loop_output_t;

const angle_outer_loop_profile_t *angle_outer_loop_default_profile(void);
bool angle_outer_loop_profile_is_valid(
    const angle_outer_loop_profile_t *profile);
bool angle_outer_loop_compute(
    const angle_outer_loop_profile_t *profile,
    const float normalized_stick[ANGLE_OUTER_LOOP_AXIS_COUNT],
    float yaw_rate_dps,
    const float attitude_deg[ANGLE_OUTER_LOOP_LEVEL_AXIS_COUNT],
    angle_outer_loop_output_t *output);

#ifdef __cplusplus
}
#endif

#endif

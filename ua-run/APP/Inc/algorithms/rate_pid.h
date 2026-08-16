/* Pure S4.5 three-axis rate PID with measurement D and anti-windup. */
#ifndef RATE_PID_H
#define RATE_PID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RATE_PID_AXIS_COUNT 3U

typedef struct
{
    float kp[RATE_PID_AXIS_COUNT];
    float ki[RATE_PID_AXIS_COUNT];
    float kd[RATE_PID_AXIS_COUNT];
    float dterm_lpf1_hz;
    float dterm_lpf2_hz;
    float integral_limit;
    float output_limit;
    float minimum_dt_s;
    float maximum_dt_s;
} rate_pid_profile_t;

typedef struct
{
    float integral[RATE_PID_AXIS_COUNT];
    float previous_measurement_rad_s[RATE_PID_AXIS_COUNT];
    float dterm_lpf1[RATE_PID_AXIS_COUNT];
    float dterm_lpf2[RATE_PID_AXIS_COUNT];
    bool measurement_initialized;
} rate_pid_state_t;

typedef struct
{
    bool valid;
    uint8_t saturated_mask;
    float setpoint_rad_s[RATE_PID_AXIS_COUNT];
    float measurement_rad_s[RATE_PID_AXIS_COUNT];
    float error_rad_s[RATE_PID_AXIS_COUNT];
    float p[RATE_PID_AXIS_COUNT];
    float i[RATE_PID_AXIS_COUNT];
    float d[RATE_PID_AXIS_COUNT];
    float correction[RATE_PID_AXIS_COUNT];
} rate_pid_output_t;

const rate_pid_profile_t *rate_pid_default_profile(void);
bool rate_pid_profile_is_valid(const rate_pid_profile_t *profile);
void rate_pid_reset(rate_pid_state_t *state);
bool rate_pid_update(
    const rate_pid_profile_t *profile,
    rate_pid_state_t *state,
    const float setpoint_rad_s[RATE_PID_AXIS_COUNT],
    const float measurement_rad_s[RATE_PID_AXIS_COUNT],
    float dt_s,
    bool integrator_enabled,
    rate_pid_output_t *output);

#ifdef __cplusplus
}
#endif

#endif

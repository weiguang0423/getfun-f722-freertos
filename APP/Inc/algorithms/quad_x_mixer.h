/* Pure S4.6 board-order Quad-X mixer with bounded desaturation. */
#ifndef QUAD_X_MIXER_H
#define QUAD_X_MIXER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QUAD_X_MIXER_AXIS_COUNT 3U
#define QUAD_X_MIXER_MOTOR_COUNT 4U

typedef enum
{
    QUAD_X_MOTOR_FRONT_LEFT = 0,
    QUAD_X_MOTOR_REAR_LEFT,
    QUAD_X_MOTOR_FRONT_RIGHT,
    QUAD_X_MOTOR_REAR_RIGHT
} quad_x_motor_position_t;

typedef struct
{
    bool valid;
    bool saturated;
    float requested_throttle;
    float applied_throttle;
    float correction_scale;
    float correction[QUAD_X_MIXER_AXIS_COUNT];
    float motor[QUAD_X_MIXER_MOTOR_COUNT];
} quad_x_mixer_output_t;

bool quad_x_mixer_compute(
    float throttle,
    const float correction[QUAD_X_MIXER_AXIS_COUNT],
    quad_x_mixer_output_t *output);

#ifdef __cplusplus
}
#endif

#endif

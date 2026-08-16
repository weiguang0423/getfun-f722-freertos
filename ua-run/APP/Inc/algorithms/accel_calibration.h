/*
 * accel_calibration.h - Horizontal single-face accelerometer calibration.
 *
 * Purpose:
 *   Provides a deterministic pure-C calibration state machine for body-frame
 *   acceleration in m/s^2. It rejects motion, invalid input and non-level
 *   placement, accumulates a fixed Welford window, and produces a candidate
 *   three-axis bias.
 *
 * Core flow:
 *   accel_calibration_initialize() -> start() -> process() ->
 *   get_candidate() -> mark_persisted()/mark_save_failed() -> apply().
 *   A candidate does not become the active bias until persistence succeeds.
 *
 * Constraints:
 *   Input must already be converted to SI units and CW90 body axes. The module
 *   has no HAL, RTOS, Flash, logging or dynamic-memory dependency.
 */
#ifndef ACCEL_CALIBRATION_H
#define ACCEL_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ACCEL_CALIBRATION_AXIS_COUNT 3U
#define ACCEL_CALIBRATION_WARMUP_SAMPLES 250U
#define ACCEL_CALIBRATION_REQUIRED_SAMPLES 2000U

typedef enum
{
    ACCEL_CALIBRATION_NOT_CALIBRATED = 0,
    ACCEL_CALIBRATION_READY,
    ACCEL_CALIBRATION_CALIBRATING,
    ACCEL_CALIBRATION_CANDIDATE_READY,
    ACCEL_CALIBRATION_SAVE_FAILED
} accel_calibration_state_t;

typedef struct
{
    accel_calibration_state_t state;
    bool bias_valid;
    uint16_t warmup_sample_count;
    uint16_t stable_sample_count;
    uint32_t restart_count;
    uint32_t motion_reject_count;
    uint32_t level_reject_count;
    uint32_t invalid_sample_count;
    float bias_m_s2[ACCEL_CALIBRATION_AXIS_COUNT];
    float candidate_bias_m_s2[ACCEL_CALIBRATION_AXIS_COUNT];
    float mean_m_s2[ACCEL_CALIBRATION_AXIS_COUNT];
    float m2_m2_s4[ACCEL_CALIBRATION_AXIS_COUNT];
} accel_calibration_t;

void accel_calibration_initialize(
    accel_calibration_t *calibration,
    const float bias_m_s2[ACCEL_CALIBRATION_AXIS_COUNT],
    bool bias_valid);
bool accel_calibration_start(accel_calibration_t *calibration);
accel_calibration_state_t accel_calibration_process(
    accel_calibration_t *calibration,
    const float acceleration_m_s2[ACCEL_CALIBRATION_AXIS_COUNT],
    const float angular_rate_rad_s[ACCEL_CALIBRATION_AXIS_COUNT]);
bool accel_calibration_get_candidate(
    const accel_calibration_t *calibration,
    float candidate_bias_m_s2[ACCEL_CALIBRATION_AXIS_COUNT]);
void accel_calibration_mark_persisted(
    accel_calibration_t *calibration);
void accel_calibration_mark_save_failed(
    accel_calibration_t *calibration);
void accel_calibration_apply(
    const accel_calibration_t *calibration,
    const float acceleration_m_s2[ACCEL_CALIBRATION_AXIS_COUNT],
    float corrected_m_s2[ACCEL_CALIBRATION_AXIS_COUNT]);

#ifdef __cplusplus
}
#endif

#endif

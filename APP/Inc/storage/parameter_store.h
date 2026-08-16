/*
 * parameter_store.h - STM32F722 internal Flash parameter store interface.
 *
 * Purpose:
 *   Exposes v3 flight configuration and load/save diagnostics for the
 *   dual-sector A/B store. The implementation owns v1 migration, validation,
 *   sequence selection, CRC, erase/program/verify and commit ordering.
 *
 * Core flow:
 *   parameter_store_init() -> parameter_store_get_values()/get_status() ->
 *   parameter_store_save(). Callers receive safe defaults when no valid record
 *   exists and only observe new values after a committed record verifies.
 *
 * Constraints:
 *   The linker script reserves STM32F722 sectors 6 and 7. Save may only be
 *   called from a task while motor outputs are known safe and the craft is
 *   disarmed. No dynamic memory is used.
 */
#ifndef PARAMETER_STORE_H
#define PARAMETER_STORE_H

#include <stdbool.h>
#include <stdint.h>

#include "algorithms/angle_outer_loop.h"
#include "algorithms/rate_pid.h"
#include "algorithms/rc_setpoint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PARAMETER_STORE_AXIS_COUNT 3U
#define PARAMETER_STORE_NAME_LENGTH 16U
#define PARAMETER_STORE_MOTOR_IDLE_DEFAULT 550U

typedef enum
{
    PARAMETER_STORE_LOAD_DEFAULTS_EMPTY = 0,
    PARAMETER_STORE_LOAD_SLOT_A,
    PARAMETER_STORE_LOAD_SLOT_B,
    PARAMETER_STORE_LOAD_RECOVERED_SLOT_A,
    PARAMETER_STORE_LOAD_RECOVERED_SLOT_B,
    PARAMETER_STORE_LOAD_DEFAULTS_CORRUPT
} parameter_store_load_result_t;

typedef enum
{
    PARAMETER_STORE_SAVE_NOT_ATTEMPTED = 0,
    PARAMETER_STORE_SAVE_OK,
    PARAMETER_STORE_SAVE_BAD_ARGUMENT,
    PARAMETER_STORE_SAVE_FLASH_UNLOCK_FAILED,
    PARAMETER_STORE_SAVE_ERASE_FAILED,
    PARAMETER_STORE_SAVE_PROGRAM_FAILED,
    PARAMETER_STORE_SAVE_VERIFY_FAILED
} parameter_store_save_result_t;

typedef enum
{
    PARAMETER_STORE_SLOT_NONE = 0,
    PARAMETER_STORE_SLOT_A,
    PARAMETER_STORE_SLOT_B
} parameter_store_slot_t;

typedef struct
{
    bool accel_calibration_valid;
    float accel_bias_m_s2[PARAMETER_STORE_AXIS_COUNT];
    rc_setpoint_profile_t rc_profile;
    rate_pid_profile_t rate_pid_profile;
    angle_outer_loop_profile_t angle_profile;
    uint16_t motor_idle_percent_x100;
    bool yaw_motors_reversed;
    char craft_name[PARAMETER_STORE_NAME_LENGTH + 1U];
    char pilot_name[PARAMETER_STORE_NAME_LENGTH + 1U];
} parameter_store_values_t;

typedef struct
{
    bool storage_valid;
    parameter_store_load_result_t load_result;
    parameter_store_save_result_t last_save_result;
    parameter_store_slot_t active_slot;
    uint8_t invalid_slot_mask;
    uint32_t sequence;
    uint32_t save_error_count;
    uint32_t last_hal_error;
    uint16_t loaded_record_version;
    bool migration_pending;
} parameter_store_status_t;

void parameter_store_values_set_defaults(parameter_store_values_t *values);
bool parameter_store_values_are_valid(
    const parameter_store_values_t *values);
void parameter_store_init(void);
void parameter_store_get_values(parameter_store_values_t *values);
void parameter_store_get_status(parameter_store_status_t *status);
parameter_store_save_result_t parameter_store_save(
    const parameter_store_values_t *values);

#ifdef __cplusplus
}
#endif

#endif

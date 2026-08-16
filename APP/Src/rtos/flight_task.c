/* 1 kHz control, arming/failsafe, DShot flight output and motor-test owner. */
#include "rtos/flight_task.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "algorithms/angle_outer_loop.h"
#include "algorithms/flight_arming.h"
#include "algorithms/quad_x_mixer.h"
#include "algorithms/rate_pid.h"
#include "algorithms/rc_setpoint.h"
#include "app_state.h"

#define FLIGHT_TASK_STACK_WORDS 768U
#define FLIGHT_TASK_PRIORITY (tskIDLE_PRIORITY + 5U)
#define FLIGHT_TASK_PERIOD_TICKS pdMS_TO_TICKS(1U)
#define FLIGHT_IMU_TIMEOUT_TICKS pdMS_TO_TICKS(5U)
#define FLIGHT_RC_TIMEOUT_TICKS pdMS_TO_TICKS(RC_INPUT_TIMEOUT_MS)
#define FLIGHT_TEST_TIMEOUT_TICKS \
    pdMS_TO_TICKS(FLIGHT_MOTOR_TEST_TIMEOUT_MS)
#define FLIGHT_MAX_CONSECUTIVE_SUBMIT_ERRORS 2U
#define FLIGHT_DEGREES_TO_RADIANS 0.01745329251994329577f
#define FLIGHT_ARM_THROTTLE_MAX 0.0f
#define FLIGHT_DSHOT_INITIAL_DELAY_LOOPS 10U
#define FLIGHT_DSHOT_COMMAND_REPEATS 10U

typedef struct
{
    bool pending;
    TickType_t tick;
    uint16_t values[DSHOT_MOTOR_COUNT];
} motor_test_request_t;

typedef struct
{
    bool pending;
    bool active;
    bool completed;
    bool success;
    uint8_t motor_index;
    uint8_t commands[FLIGHT_DSHOT_COMMAND_MAX_COUNT];
    uint8_t command_count;
    uint8_t command_index;
    uint8_t repeats_remaining;
    uint8_t delay_loops;
} dshot_command_request_t;

static StaticTask_t flight_task_control_block;
static StackType_t flight_task_stack[FLIGHT_TASK_STACK_WORDS];
static TaskHandle_t flight_task_handle;
static motor_test_request_t motor_test_request;
static dshot_command_request_t dshot_command_request;

static bool dshot_command_step(bool can_run)
{
    bool owns_output = false;

    taskENTER_CRITICAL();
    if (dshot_command_request.pending && !dshot_command_request.active) {
        dshot_command_request.active = true;
        dshot_command_request.command_index = 0U;
        dshot_command_request.repeats_remaining =
            FLIGHT_DSHOT_COMMAND_REPEATS;
        dshot_command_request.delay_loops =
            FLIGHT_DSHOT_INITIAL_DELAY_LOOPS;
    }
    owns_output = dshot_command_request.active;
    if (owns_output && !can_run) {
        dshot_command_request.pending = false;
        dshot_command_request.active = false;
        dshot_command_request.completed = true;
        dshot_command_request.success = false;
    }
    taskEXIT_CRITICAL();

    if (!owns_output || !can_run) {
        return owns_output;
    }
    if (dshot_command_request.delay_loops != 0U) {
        --dshot_command_request.delay_loops;
        return true;
    }
    if (dshot_command_request.command_index >=
        dshot_command_request.command_count) {
        taskENTER_CRITICAL();
        dshot_command_request.pending = false;
        dshot_command_request.active = false;
        dshot_command_request.completed = true;
        dshot_command_request.success = true;
        taskEXIT_CRITICAL();
        return true;
    }
    if (!dshot_motor_submit_command(
            dshot_command_request.motor_index,
            dshot_command_request.commands[
                dshot_command_request.command_index])) {
        return true;
    }
    if (--dshot_command_request.repeats_remaining != 0U) {
        return true;
    }
    ++dshot_command_request.command_index;
    if (dshot_command_request.command_index <
        dshot_command_request.command_count) {
        dshot_command_request.repeats_remaining =
            FLIGHT_DSHOT_COMMAND_REPEATS;
        return true;
    }
    return true;
}

static uint32_t flight_safety_flags(const app_state_snapshot_t *snapshot,
                                    TickType_t now)
{
    uint32_t flags = 0U;

    if (!snapshot->imu.present || !snapshot->imu.timing_valid ||
        !snapshot->imu.filter_ready || !snapshot->attitude.valid) {
        flags |= APP_FLIGHT_SAFETY_IMU_INVALID;
    }
    if ((snapshot->imu.sample_count == 0U) ||
        ((TickType_t)(now - snapshot->imu.sample_tick) >
         FLIGHT_IMU_TIMEOUT_TICKS)) {
        flags |= APP_FLIGHT_SAFETY_IMU_STALE;
    }
    if (!snapshot->rc.channels_valid || snapshot->rc.failsafe_active) {
        flags |= APP_FLIGHT_SAFETY_RC_INVALID;
    }
    if ((snapshot->rc.channel_frame_count == 0U) ||
        ((TickType_t)(now - snapshot->rc.last_channel_tick) >=
         FLIGHT_RC_TIMEOUT_TICKS)) {
        flags |= APP_FLIGHT_SAFETY_RC_STALE;
    }
    if (snapshot->arming_inhibit_flags != 0U) {
        flags |= APP_FLIGHT_SAFETY_ARMING_INHIBITED;
    }
    return flags;
}

static bool take_motor_test_request(TickType_t now,
                                    uint16_t values[DSHOT_MOTOR_COUNT],
                                    TickType_t *request_tick,
                                    bool *timed_out)
{
    bool active;

    taskENTER_CRITICAL();
    active = motor_test_request.pending;
    *timed_out = false;
    *request_tick = motor_test_request.tick;
    if (active &&
        ((TickType_t)(now - motor_test_request.tick) >=
         FLIGHT_TEST_TIMEOUT_TICKS)) {
        motor_test_request.pending = false;
        active = false;
        *timed_out = true;
    }
    if (active) {
        memcpy(values, motor_test_request.values,
               sizeof(motor_test_request.values));
    } else {
        memset(values, 0, sizeof(motor_test_request.values));
    }
    taskEXIT_CRITICAL();
    return active;
}

static void reset_control(rate_pid_state_t *pid_state,
                          app_flight_state_t *state)
{
    rate_pid_reset(pid_state);
    memset(&state->angle_outer_loop, 0,
           sizeof(state->angle_outer_loop));
    memset(&state->rate_pid, 0, sizeof(state->rate_pid));
    memset(&state->mixer, 0, sizeof(state->mixer));
    state->rate_pid_integrator_enabled = false;
    state->control_dt_us = 0U;
}

static bool mixer_to_dshot(const quad_x_mixer_output_t *mixer,
                           uint16_t motor_idle_percent_x100,
                           uint16_t output[DSHOT_MOTOR_COUNT])
{
    /* Betaflight motorIdle uses hundredths of one percent: 550 = 5.5%. */
    const float dshot_output_low =
        (float)DSHOT_MIN_THROTTLE_VALUE +
        ((float)motor_idle_percent_x100 / 10000.0f) *
            (float)(DSHOT_MAX_VALUE - DSHOT_MIN_THROTTLE_VALUE);
    const float dshot_output_range =
        (float)DSHOT_MAX_VALUE - dshot_output_low;
    uint32_t motor;

    if ((mixer == NULL) || (output == NULL) || !mixer->valid ||
        (motor_idle_percent_x100 > 2000U)) {
        return false;
    }
    for (motor = 0U; motor < DSHOT_MOTOR_COUNT; ++motor) {
        const float normalized = mixer->motor[motor];

        if (!isfinite(normalized) ||
            (normalized < 0.0f) || (normalized > 1.0f)) {
            memset(output, 0, sizeof(uint16_t) * DSHOT_MOTOR_COUNT);
            return false;
        }
        output[motor] = (uint16_t)(
            dshot_output_low + normalized * dshot_output_range + 0.5f);
    }
    return true;
}

static void copy_arming_state(const flight_arming_t *arming,
                              app_flight_state_t *state)
{
    state->arming_state = arming->state;
    state->armed = flight_arming_is_armed(arming);
    state->last_failsafe_flags = arming->last_failsafe_flags;
    state->arm_count = arming->arm_count;
    state->disarm_count = arming->disarm_count;
    state->flight_failsafe_count = arming->failsafe_count;
}

static void flight_task(void *argument)
{
    app_flight_state_t state;
    flight_arming_t arming;
    rate_pid_state_t pid_state;
    TickType_t last_wake_time;
    uint32_t last_control_sample_count = 0U;
    uint32_t consecutive_submit_errors = 0U;
    rc_setpoint_mode_t previous_mode = RC_SETPOINT_MODE_RATE;
    bool previous_mode_valid = false;
    uint32_t previous_parameter_sequence = 0U;
    bool parameter_sequence_valid = false;

    (void)argument;
    memset(&state, 0, sizeof(state));
    flight_arming_init(&arming);
    rate_pid_reset(&pid_state);
    configASSERT(FLIGHT_TASK_PERIOD_TICKS > 0U);
    configASSERT(FLIGHT_IMU_TIMEOUT_TICKS > 0U);
    configASSERT(FLIGHT_RC_TIMEOUT_TICKS > 0U);
    configASSERT(FLIGHT_TEST_TIMEOUT_TICKS > 0U);
    state.dshot_ready = dshot_motor_init();
    copy_arming_state(&arming, &state);
    app_state_publish_flight(&state);
    last_wake_time = xTaskGetTickCount();

    for (;;) {
        app_state_snapshot_t snapshot;
        dshot_motor_diagnostics_t dshot;
        TickType_t now;
        TickType_t request_tick;
        uint16_t requested[DSHOT_MOTOR_COUNT];
        uint16_t output[DSHOT_MOTOR_COUNT] = {0};
        uint16_t flight_output[DSHOT_MOTOR_COUNT] = {0};
        uint32_t arming_block_flags;
        uint32_t failsafe_flags;
        bool control_output_valid;
        bool control_setpoint_valid;
        bool requested_active;
        bool dshot_command_active;
        bool timed_out;
        const rc_setpoint_profile_t *setpoint_profile;
        const angle_outer_loop_profile_t *angle_profile;
        const rate_pid_profile_t *pid_profile;

        vTaskDelayUntil(&last_wake_time, FLIGHT_TASK_PERIOD_TICKS);
        now = xTaskGetTickCount();
        if ((TickType_t)(now - last_wake_time) > 0U) {
            state.missed_deadline_count +=
                (uint32_t)(now - last_wake_time);
        }
        app_state_get_snapshot(&snapshot);
        if (snapshot.parameters.storage_valid) {
            if (!parameter_sequence_valid ||
                (snapshot.parameters.sequence !=
                 previous_parameter_sequence)) {
                reset_control(&pid_state, &state);
                previous_mode_valid = false;
                previous_parameter_sequence = snapshot.parameters.sequence;
                parameter_sequence_valid = true;
            }
        } else {
            parameter_sequence_valid = false;
        }
        setpoint_profile = &snapshot.parameters.values.rc_profile;
        angle_profile = &snapshot.parameters.values.angle_profile;
        pid_profile = &snapshot.parameters.values.rate_pid_profile;
        dshot_motor_get_diagnostics(&dshot);
        state.dshot_ready = dshot.ready;
        state.dshot_busy = dshot.busy;
        state.dshot_dma_error_count = dshot.dma_error_count;
        state.safety_flags = flight_safety_flags(&snapshot, now);
        state.inputs_ready = state.safety_flags == 0U;
        if ((state.safety_flags & APP_FLIGHT_SAFETY_IMU_STALE) != 0U) {
            ++state.imu_stale_count;
        }
        if ((state.safety_flags & APP_FLIGHT_SAFETY_RC_STALE) != 0U) {
            ++state.rc_stale_count;
        }

        if ((state.safety_flags &
             (APP_FLIGHT_SAFETY_RC_INVALID |
              APP_FLIGHT_SAFETY_RC_STALE)) == 0U) {
            if (rc_setpoint_compute(setpoint_profile,
                                    snapshot.rc.mapped_channel_us,
                                    &state.rc_setpoint)) {
                ++state.rc_setpoint_update_count;
            } else {
                memset(&state.rc_setpoint, 0,
                       sizeof(state.rc_setpoint));
                ++state.rc_setpoint_error_count;
            }
        } else {
            memset(&state.rc_setpoint, 0,
                   sizeof(state.rc_setpoint));
        }

        if (!state.inputs_ready || !state.rc_setpoint.valid) {
            reset_control(&pid_state, &state);
            previous_mode_valid = false;
            last_control_sample_count = snapshot.imu.sample_count;
        } else if (snapshot.imu.sample_count !=
                   last_control_sample_count) {
            float setpoint_rad_s[RATE_PID_AXIS_COUNT];
            float setpoint_dps[RATE_PID_AXIS_COUNT];
            uint32_t axis;

            last_control_sample_count = snapshot.imu.sample_count;
            state.control_sample_count = snapshot.imu.sample_count;
            state.control_dt_us = snapshot.imu.sample_interval_us;
            state.rate_pid_integrator_enabled =
                flight_arming_is_armed(&arming) &&
                state.rc_setpoint.arm_requested &&
                (state.rc_setpoint.throttle > 0.0f);
            control_setpoint_valid = true;
            if (!previous_mode_valid ||
                (previous_mode != state.rc_setpoint.mode)) {
                rate_pid_reset(&pid_state);
                previous_mode = state.rc_setpoint.mode;
                previous_mode_valid = true;
            }

            if (state.rc_setpoint.mode == RC_SETPOINT_MODE_ANGLE) {
                const float attitude_deg[ANGLE_OUTER_LOOP_LEVEL_AXIS_COUNT] = {
                    snapshot.attitude.roll_deg,
                    snapshot.attitude.pitch_deg,
                };

                if (angle_outer_loop_compute(
                        angle_profile,
                        state.rc_setpoint.normalized_stick,
                        state.rc_setpoint.rate_dps[2],
                        attitude_deg,
                        &state.angle_outer_loop)) {
                    memcpy(setpoint_dps,
                           state.angle_outer_loop.target_rate_dps,
                           sizeof(setpoint_dps));
                    ++state.angle_outer_loop_update_count;
                } else {
                    memset(&state.rate_pid, 0,
                           sizeof(state.rate_pid));
                    memset(&state.mixer, 0, sizeof(state.mixer));
                    ++state.angle_outer_loop_error_count;
                    control_setpoint_valid = false;
                }
            } else {
                memset(&state.angle_outer_loop, 0,
                       sizeof(state.angle_outer_loop));
                memcpy(setpoint_dps, state.rc_setpoint.rate_dps,
                       sizeof(setpoint_dps));
            }
            if (control_setpoint_valid) {
                for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
                    setpoint_rad_s[axis] =
                        setpoint_dps[axis] * FLIGHT_DEGREES_TO_RADIANS;
                }
            }

            if (control_setpoint_valid && !rate_pid_update(
                    pid_profile,
                    &pid_state,
                    setpoint_rad_s,
                    snapshot.imu.filtered_angular_rate_rad_s,
                    (float)snapshot.imu.sample_interval_us * 0.000001f,
                    state.rate_pid_integrator_enabled,
                    &state.rate_pid)) {
                memset(&state.mixer, 0, sizeof(state.mixer));
                ++state.rate_pid_error_count;
            }

            if (state.rate_pid.valid) {
                float mixer_correction[RATE_PID_AXIS_COUNT];

                ++state.rate_pid_update_count;
                memcpy(mixer_correction, state.rate_pid.correction,
                       sizeof(mixer_correction));
                if (snapshot.parameters.values.yaw_motors_reversed) {
                    mixer_correction[2] = -mixer_correction[2];
                }
                if (quad_x_mixer_compute(
                        state.rc_setpoint.throttle,
                        mixer_correction,
                        &state.mixer)) {
                    ++state.mixer_update_count;
                } else {
                    memset(&state.mixer, 0, sizeof(state.mixer));
                    ++state.mixer_error_count;
                }
            }
        }

        control_output_valid =
            state.rate_pid.valid && mixer_to_dshot(
                &state.mixer,
                snapshot.parameters.values.motor_idle_percent_x100,
                flight_output);

        requested_active = take_motor_test_request(
            now, requested, &request_tick, &timed_out);
        if (timed_out) {
            ++state.motor_test_timeout_count;
        }
        state.last_motor_test_command_tick = request_tick;
        memcpy(state.requested_motor_value, requested, sizeof(requested));

        arming_block_flags = state.safety_flags;
        if (!dshot.ready || dshot.fault_latched) {
            arming_block_flags |= APP_FLIGHT_SAFETY_DSHOT_NOT_READY;
        }
        if (snapshot.configurator_arming_disabled) {
            arming_block_flags |=
                APP_FLIGHT_SAFETY_CONFIGURATOR_DISABLED;
        }
        if (snapshot.fault_flags != 0U) {
            arming_block_flags |= APP_FLIGHT_SAFETY_SYSTEM_FAULT;
        }
        if (!control_output_valid) {
            arming_block_flags |= APP_FLIGHT_SAFETY_CONTROL_INVALID;
        }
        if (!state.rc_setpoint.valid ||
            (state.rc_setpoint.throttle > FLIGHT_ARM_THROTTLE_MAX)) {
            arming_block_flags |= APP_FLIGHT_SAFETY_THROTTLE_HIGH;
        }
        if (requested_active) {
            arming_block_flags |= APP_FLIGHT_SAFETY_MOTOR_TEST_ACTIVE;
        }
        taskENTER_CRITICAL();
        dshot_command_active = dshot_command_request.pending ||
                               dshot_command_request.active;
        taskEXIT_CRITICAL();
        if (dshot_command_active) {
            arming_block_flags |= APP_FLIGHT_SAFETY_MOTOR_TEST_ACTIVE;
        }
        failsafe_flags = arming_block_flags &
            ~APP_FLIGHT_SAFETY_ARMING_ONLY_MASK;
        state.safety_flags = failsafe_flags;
        state.arming_block_flags = arming_block_flags;
        (void)flight_arming_update(
            &arming,
            state.rc_setpoint.valid &&
                state.rc_setpoint.arm_requested,
            arming_block_flags,
            failsafe_flags);
        copy_arming_state(&arming, &state);

        state.motor_test_active =
            requested_active && state.inputs_ready &&
            dshot.ready && !dshot.fault_latched &&
            (snapshot.fault_flags == 0U) &&
            !state.rc_setpoint.arm_requested && !state.armed;
        if (state.armed) {
            memcpy(output, flight_output, sizeof(output));
        } else if (state.motor_test_active) {
            memcpy(output, requested, sizeof(output));
        }

        dshot_command_active = dshot_command_step(
            !state.armed && !requested_active && dshot.ready &&
            !dshot.fault_latched && !state.rc_setpoint.arm_requested &&
            snapshot.configurator_arming_disabled &&
            (snapshot.fault_flags == 0U));
        if (!dshot_command_active && dshot.ready && !dshot.fault_latched &&
            !dshot_motor_submit(output)) {
            ++state.dshot_submit_error_count;
            ++consecutive_submit_errors;
            if (state.armed ||
                (consecutive_submit_errors >=
                 FLIGHT_MAX_CONSECUTIVE_SUBMIT_ERRORS)) {
                dshot_motor_force_safe();
            }
        } else {
            consecutive_submit_errors = 0U;
        }
        dshot_motor_get_diagnostics(&dshot);
        state.dshot_ready = dshot.ready;
        state.dshot_busy = dshot.busy;
        state.dshot_dma_error_count = dshot.dma_error_count;
        if (state.armed && (!dshot.ready || dshot.fault_latched)) {
            arming_block_flags |= APP_FLIGHT_SAFETY_DSHOT_NOT_READY;
            state.arming_block_flags = arming_block_flags;
            failsafe_flags = arming_block_flags &
                ~APP_FLIGHT_SAFETY_ARMING_ONLY_MASK;
            state.safety_flags = failsafe_flags;
            (void)flight_arming_update(
                &arming,
                state.rc_setpoint.arm_requested,
                arming_block_flags,
                failsafe_flags);
            copy_arming_state(&arming, &state);
            state.motor_test_active = false;
        }
        memcpy(state.output_motor_value, dshot.requested_value,
               sizeof(state.output_motor_value));
        state.last_update_tick = now;
        ++state.loop_count;
        app_state_publish_flight(&state);
    }
}

void flight_task_create(void)
{
    memset(&motor_test_request, 0, sizeof(motor_test_request));
    memset(&dshot_command_request, 0, sizeof(dshot_command_request));
    flight_task_handle = xTaskCreateStatic(
        flight_task,
        "FlightTask",
        FLIGHT_TASK_STACK_WORDS,
        NULL,
        FLIGHT_TASK_PRIORITY,
        flight_task_stack,
        &flight_task_control_block);
    configASSERT(flight_task_handle != NULL);
}

bool flight_task_execute_dshot_commands(
    uint8_t motor_index,
    const uint8_t *commands,
    uint8_t command_count,
    uint32_t timeout_ms)
{
    TickType_t start;
    TickType_t timeout;
    bool completed;
    bool success;
    uint32_t index;

    if ((commands == NULL) || (command_count == 0U) ||
        (command_count > FLIGHT_DSHOT_COMMAND_MAX_COUNT) ||
        ((motor_index >= DSHOT_MOTOR_COUNT) &&
         (motor_index != DSHOT_ALL_MOTORS)) || app_state_is_armed()) {
        return false;
    }
    for (index = 0U; index < command_count; ++index) {
        if (commands[index] > DSHOT_MAX_COMMAND) {
            return false;
        }
    }
    taskENTER_CRITICAL();
    if (motor_test_request.pending) {
        for (index = 0U; index < DSHOT_MOTOR_COUNT; ++index) {
            if (motor_test_request.values[index] != 0U) {
                taskEXIT_CRITICAL();
                return false;
            }
        }
        motor_test_request.pending = false;
    }
    if (dshot_command_request.pending || dshot_command_request.active) {
        taskEXIT_CRITICAL();
        return false;
    }
    dshot_command_request.motor_index = motor_index;
    memcpy(dshot_command_request.commands, commands, command_count);
    dshot_command_request.command_count = command_count;
    dshot_command_request.completed = false;
    dshot_command_request.success = false;
    dshot_command_request.pending = true;
    taskEXIT_CRITICAL();

    start = xTaskGetTickCount();
    timeout = pdMS_TO_TICKS(timeout_ms);
    do {
        taskENTER_CRITICAL();
        completed = dshot_command_request.completed;
        success = dshot_command_request.success;
        taskEXIT_CRITICAL();
        if (completed) {
            return success;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    } while ((TickType_t)(xTaskGetTickCount() - start) < timeout);

    taskENTER_CRITICAL();
    dshot_command_request.pending = false;
    dshot_command_request.active = false;
    dshot_command_request.completed = true;
    dshot_command_request.success = false;
    taskEXIT_CRITICAL();
    return false;
}

bool flight_task_request_motor_test(
    const uint16_t values[DSHOT_MOTOR_COUNT])
{
    bool requests_output = false;
    uint32_t motor;

    if ((values == NULL) || (flight_task_handle == NULL)) {
        return false;
    }
    for (motor = 0U; motor < DSHOT_MOTOR_COUNT; ++motor) {
        if ((values[motor] > DSHOT_MAX_VALUE) ||
            ((values[motor] != 0U) &&
             (values[motor] < DSHOT_MIN_THROTTLE_VALUE))) {
            return false;
        }
        requests_output = requests_output || (values[motor] != 0U);
    }
    if (requests_output && app_state_is_armed()) {
        return false;
    }
    taskENTER_CRITICAL();
    memcpy(motor_test_request.values, values,
           sizeof(motor_test_request.values));
    motor_test_request.tick = xTaskGetTickCount();
    motor_test_request.pending = true;
    taskEXIT_CRITICAL();
    return true;
}

uint32_t flight_task_stack_high_water_mark(void)
{
    if (flight_task_handle == NULL) {
        return 0U;
    }
    return (uint32_t)uxTaskGetStackHighWaterMark(flight_task_handle);
}

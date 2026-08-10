/* 1 kHz control, arming/failsafe, DShot flight output and motor-test owner. */
#include "rtos/flight_task.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "algorithms/flight_arming.h"
#include "algorithms/quad_x_mixer.h"
#include "algorithms/rate_pid.h"
#include "algorithms/rc_setpoint.h"
#include "app_state.h"

#define FLIGHT_TASK_STACK_WORDS 512U
#define FLIGHT_TASK_PRIORITY (tskIDLE_PRIORITY + 5U)
#define FLIGHT_TASK_PERIOD_TICKS pdMS_TO_TICKS(1U)
#define FLIGHT_IMU_TIMEOUT_TICKS pdMS_TO_TICKS(5U)
#define FLIGHT_RC_TIMEOUT_TICKS pdMS_TO_TICKS(RC_INPUT_TIMEOUT_MS)
#define FLIGHT_TEST_TIMEOUT_TICKS \
    pdMS_TO_TICKS(FLIGHT_MOTOR_TEST_TIMEOUT_MS)
#define FLIGHT_MAX_CONSECUTIVE_SUBMIT_ERRORS 2U
#define FLIGHT_DEGREES_TO_RADIANS 0.01745329251994329577f
#define FLIGHT_PID_INTEGRATOR_THROTTLE_MIN 0.05f
#define FLIGHT_ARM_THROTTLE_MAX 0.05f

typedef struct
{
    bool pending;
    TickType_t tick;
    uint16_t values[DSHOT_MOTOR_COUNT];
} motor_test_request_t;

static StaticTask_t flight_task_control_block;
static StackType_t flight_task_stack[FLIGHT_TASK_STACK_WORDS];
static TaskHandle_t flight_task_handle;
static motor_test_request_t motor_test_request;

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
    memset(&state->rate_pid, 0, sizeof(state->rate_pid));
    memset(&state->mixer, 0, sizeof(state->mixer));
    state->rate_pid_integrator_enabled = false;
    state->control_dt_us = 0U;
}

static bool mixer_to_dshot(const quad_x_mixer_output_t *mixer,
                           uint16_t output[DSHOT_MOTOR_COUNT])
{
    const float dshot_range =
        (float)(DSHOT_MAX_VALUE - DSHOT_MIN_THROTTLE_VALUE);
    uint32_t motor;

    if ((mixer == NULL) || (output == NULL) || !mixer->valid) {
        return false;
    }
    for (motor = 0U; motor < DSHOT_MOTOR_COUNT; ++motor) {
        const float normalized = mixer->motor[motor];

        if (!isfinite(normalized) ||
            (normalized < 0.0f) || (normalized > 1.0f)) {
            memset(output, 0, sizeof(uint16_t) * DSHOT_MOTOR_COUNT);
            return false;
        }
        output[motor] =
            DSHOT_MIN_THROTTLE_VALUE +
            (uint16_t)(normalized * dshot_range + 0.5f);
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
    const rc_setpoint_profile_t *setpoint_profile =
        rc_setpoint_default_profile();
    const rate_pid_profile_t *pid_profile =
        rate_pid_default_profile();
    TickType_t last_wake_time;
    uint32_t last_control_sample_count = 0U;
    uint32_t consecutive_submit_errors = 0U;

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
        bool requested_active;
        bool timed_out;

        vTaskDelayUntil(&last_wake_time, FLIGHT_TASK_PERIOD_TICKS);
        now = xTaskGetTickCount();
        if ((TickType_t)(now - last_wake_time) > 0U) {
            state.missed_deadline_count +=
                (uint32_t)(now - last_wake_time);
        }
        app_state_get_snapshot(&snapshot);
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

        if (!state.inputs_ready || !state.rc_setpoint.valid ||
            (state.rc_setpoint.mode != RC_SETPOINT_MODE_RATE)) {
            reset_control(&pid_state, &state);
            last_control_sample_count = snapshot.imu.sample_count;
        } else if (snapshot.imu.sample_count !=
                   last_control_sample_count) {
            float setpoint_rad_s[RATE_PID_AXIS_COUNT];
            uint32_t axis;

            last_control_sample_count = snapshot.imu.sample_count;
            state.control_sample_count = snapshot.imu.sample_count;
            state.control_dt_us = snapshot.imu.sample_interval_us;
            state.rate_pid_integrator_enabled =
                flight_arming_is_armed(&arming) &&
                state.rc_setpoint.arm_requested &&
                (state.rc_setpoint.throttle >
                 FLIGHT_PID_INTEGRATOR_THROTTLE_MIN);
            for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
                setpoint_rad_s[axis] =
                    state.rc_setpoint.rate_dps[axis] *
                    FLIGHT_DEGREES_TO_RADIANS;
            }

            if (rate_pid_update(
                    pid_profile,
                    &pid_state,
                    setpoint_rad_s,
                    snapshot.imu.filtered_angular_rate_rad_s,
                    (float)snapshot.imu.sample_interval_us * 0.000001f,
                    state.rate_pid_integrator_enabled,
                    &state.rate_pid)) {
                ++state.rate_pid_update_count;
                if (quad_x_mixer_compute(
                        state.rc_setpoint.throttle,
                        state.rate_pid.correction,
                        &state.mixer)) {
                    ++state.mixer_update_count;
                } else {
                    memset(&state.mixer, 0, sizeof(state.mixer));
                    ++state.mixer_error_count;
                }
            } else {
                memset(&state.mixer, 0, sizeof(state.mixer));
                ++state.rate_pid_error_count;
            }
        }

        control_output_valid =
            state.rate_pid.valid && mixer_to_dshot(
                &state.mixer, flight_output);

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

        if (dshot.ready && !dshot.fault_latched &&
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

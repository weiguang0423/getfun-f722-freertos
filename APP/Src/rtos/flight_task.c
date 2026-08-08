/* 1 kHz input-freshness gate and propeller-free DShot motor-test owner. */
#include "rtos/flight_task.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"

#define FLIGHT_TASK_STACK_WORDS 384U
#define FLIGHT_TASK_PRIORITY (tskIDLE_PRIORITY + 5U)
#define FLIGHT_TASK_PERIOD_TICKS pdMS_TO_TICKS(1U)
#define FLIGHT_IMU_TIMEOUT_TICKS pdMS_TO_TICKS(5U)
#define FLIGHT_RC_TIMEOUT_TICKS pdMS_TO_TICKS(RC_INPUT_TIMEOUT_MS)
#define FLIGHT_TEST_TIMEOUT_TICKS \
    pdMS_TO_TICKS(FLIGHT_MOTOR_TEST_TIMEOUT_MS)
#define FLIGHT_MAX_CONSECUTIVE_SUBMIT_ERRORS 2U

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

static void flight_task(void *argument)
{
    app_flight_state_t state;
    TickType_t last_wake_time;
    uint32_t consecutive_submit_errors = 0U;

    (void)argument;
    memset(&state, 0, sizeof(state));
    configASSERT(FLIGHT_TASK_PERIOD_TICKS > 0U);
    configASSERT(FLIGHT_IMU_TIMEOUT_TICKS > 0U);
    configASSERT(FLIGHT_RC_TIMEOUT_TICKS > 0U);
    configASSERT(FLIGHT_TEST_TIMEOUT_TICKS > 0U);
    state.dshot_ready = dshot_motor_init();
    app_state_publish_flight(&state);
    last_wake_time = xTaskGetTickCount();

    for (;;) {
        app_state_snapshot_t snapshot;
        dshot_motor_diagnostics_t dshot;
        TickType_t now;
        TickType_t request_tick;
        uint16_t requested[DSHOT_MOTOR_COUNT];
        uint16_t output[DSHOT_MOTOR_COUNT] = {0};
        bool requested_active;
        bool timed_out;

        vTaskDelayUntil(&last_wake_time, FLIGHT_TASK_PERIOD_TICKS);
        now = xTaskGetTickCount();
        if ((TickType_t)(now - last_wake_time) > 0U) {
            state.missed_deadline_count +=
                (uint32_t)(now - last_wake_time);
        }
        app_state_get_snapshot(&snapshot);
        state.safety_flags = flight_safety_flags(&snapshot, now);
        state.inputs_ready = state.safety_flags == 0U;
        if ((state.safety_flags & APP_FLIGHT_SAFETY_IMU_STALE) != 0U) {
            ++state.imu_stale_count;
        }
        if ((state.safety_flags & APP_FLIGHT_SAFETY_RC_STALE) != 0U) {
            ++state.rc_stale_count;
        }

        requested_active = take_motor_test_request(
            now, requested, &request_tick, &timed_out);
        if (timed_out) {
            ++state.motor_test_timeout_count;
        }
        state.last_motor_test_command_tick = request_tick;
        memcpy(state.requested_motor_value, requested, sizeof(requested));
        state.motor_test_active = requested_active && state.inputs_ready;
        if (state.motor_test_active) {
            memcpy(output, requested, sizeof(output));
        }

        if (!dshot_motor_submit(output)) {
            ++state.dshot_submit_error_count;
            ++consecutive_submit_errors;
            if (consecutive_submit_errors >=
                FLIGHT_MAX_CONSECUTIVE_SUBMIT_ERRORS) {
                dshot_motor_force_safe();
            }
        } else {
            consecutive_submit_errors = 0U;
        }
        dshot_motor_get_diagnostics(&dshot);
        state.dshot_ready = dshot.ready;
        state.dshot_busy = dshot.busy;
        state.dshot_dma_error_count = dshot.dma_error_count;
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

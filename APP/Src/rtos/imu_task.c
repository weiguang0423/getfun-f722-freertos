/*
 * imu_task.c - 1 kHz polling owner for the board ICM42688P.
 *
 * Purpose:
 *   Owns sensor initialization/recovery, fixed-period data-ready polling,
 *   SI-unit conversion, CW90 board alignment, statistics, and app_state publish.
 *
 * Core functions:
 *   - imu_task_create(): statically creates ImuTask at idle priority + 4.
 *   - imu_task(): retries initialization, samples at 1 tick, and recovers after
 *     three consecutive bus failures.
 *   - imu_task_stack_high_water_mark(): supports the 1 Hz diagnostic summary.
 *
 * Data flow and constraints:
 *   ICM42688P raw sensor-frame values -> SI conversion -> CW90 body frame ->
 *   app_state_publish_imu(). This task is the only SPI/IMU writer. It performs
 *   no UART/USB output, dynamic allocation, or blocking work beyond bounded
 *   SPI transactions and initialization/retry delays.
 */
#include "rtos/imu_task.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "drivers/icm42688p.h"

#define IMU_TASK_STACK_WORDS 512U
#define IMU_TASK_PRIORITY (tskIDLE_PRIORITY + 4U)
#define IMU_TASK_PERIOD_TICKS pdMS_TO_TICKS(1U)
#define IMU_INITIALIZATION_ATTEMPTS 3U
#define IMU_INITIALIZATION_RETRY_DELAY_MS 10U
#define IMU_OFFLINE_RETRY_DELAY_MS 1000U
#define IMU_MAX_CONSECUTIVE_ERRORS 3U

#define STANDARD_GRAVITY_M_S2 9.80665f
#define ACCEL_COUNTS_PER_G 2048.0f
#define GYRO_COUNTS_PER_DPS 16.4f
#define DEGREES_TO_RADIANS 0.01745329251994329577f
#define TEMPERATURE_COUNTS_PER_C 132.48f
#define TEMPERATURE_OFFSET_C 25.0f

static StaticTask_t imu_task_control_block;
static StackType_t imu_task_stack[IMU_TASK_STACK_WORDS];
static TaskHandle_t imu_task_handle;

static TickType_t milliseconds_to_nonzero_ticks(uint32_t delay_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(delay_ms);

    if (ticks == 0U) {
        ticks = 1U;
    }
    return ticks;
}

static void imu_delay_ms(uint32_t delay_ms)
{
    vTaskDelay(milliseconds_to_nonzero_ticks(delay_ms));
}

static void apply_cw90_alignment(const float sensor[APP_STATE_AXIS_COUNT],
                                 float body[APP_STATE_AXIS_COUNT])
{
    body[0] = sensor[1];
    body[1] = -sensor[0];
    body[2] = sensor[2];
}

static void convert_sample(const icm42688p_raw_sample_t *raw,
                           app_imu_sample_t *sample)
{
    float sensor_acceleration[APP_STATE_AXIS_COUNT];
    float sensor_angular_rate[APP_STATE_AXIS_COUNT];
    uint32_t axis;

    for (axis = 0U; axis < APP_STATE_AXIS_COUNT; ++axis) {
        sensor_acceleration[axis] =
            ((float)raw->acceleration[axis] / ACCEL_COUNTS_PER_G) *
            STANDARD_GRAVITY_M_S2;
        sensor_angular_rate[axis] =
            ((float)raw->angular_rate[axis] / GYRO_COUNTS_PER_DPS) *
            DEGREES_TO_RADIANS;
    }

    apply_cw90_alignment(sensor_acceleration, sample->acceleration_m_s2);
    apply_cw90_alignment(sensor_angular_rate, sample->angular_rate_rad_s);
    sample->temperature_c =
        ((float)raw->temperature / TEMPERATURE_COUNTS_PER_C) +
        TEMPERATURE_OFFSET_C;
}

static bool initialize_sensor(app_imu_sample_t *sample)
{
    icm42688p_diagnostics_t diagnostics;
    icm42688p_status_t status = ICM42688P_STATUS_BUS_ERROR;
    uint32_t attempt;

    for (attempt = 0U; attempt < IMU_INITIALIZATION_ATTEMPTS; ++attempt) {
        status = icm42688p_initialize(imu_delay_ms, &diagnostics);
        sample->who_am_i = diagnostics.who_am_i;
        sample->gyro_config0 = diagnostics.gyro_config0;
        sample->accel_config0 = diagnostics.accel_config0;
        sample->pwr_mgmt0 = diagnostics.pwr_mgmt0;
        sample->last_error = (uint8_t)status;
        sample->last_bus_error = (uint8_t)diagnostics.bus_status;

        if (status == ICM42688P_STATUS_OK) {
            sample->present = true;
            sample->consecutive_error_count = 0U;
            app_state_publish_imu(sample);
            return true;
        }

        ++sample->initialization_error_count;
        sample->present = false;
        app_state_publish_imu(sample);
        if ((attempt + 1U) < IMU_INITIALIZATION_ATTEMPTS) {
            imu_delay_ms(IMU_INITIALIZATION_RETRY_DELAY_MS);
        }
    }
    return false;
}

static bool record_read_failure(app_imu_sample_t *sample,
                                icm42688p_status_t status,
                                imu_bus_status_t bus_status)
{
    ++sample->read_error_count;
    ++sample->consecutive_error_count;
    sample->last_error = (uint8_t)status;
    sample->last_bus_error = (uint8_t)bus_status;

    if (sample->consecutive_error_count >= IMU_MAX_CONSECUTIVE_ERRORS) {
        sample->present = false;
    }
    app_state_publish_imu(sample);
    return sample->present;
}

static void imu_task(void *argument)
{
    app_imu_sample_t sample;

    (void)argument;
    memset(&sample, 0, sizeof(sample));
    configASSERT(IMU_TASK_PERIOD_TICKS > 0U);

    for (;;) {
        TickType_t last_wake_time;

        if (!initialize_sensor(&sample)) {
            vTaskDelay(milliseconds_to_nonzero_ticks(
                IMU_OFFLINE_RETRY_DELAY_MS));
            continue;
        }

        last_wake_time = xTaskGetTickCount();
        while (sample.present) {
            icm42688p_raw_sample_t raw;
            imu_bus_status_t bus_status;
            icm42688p_status_t status;
            TickType_t now;
            bool ready;

            vTaskDelayUntil(&last_wake_time, IMU_TASK_PERIOD_TICKS);
            now = xTaskGetTickCount();
            if ((TickType_t)(now - last_wake_time) > 0U) {
                sample.missed_deadline_count +=
                    (uint32_t)(now - last_wake_time);
            }

            status = icm42688p_data_ready(&ready, &bus_status);
            if (status != ICM42688P_STATUS_OK) {
                if (!record_read_failure(&sample, status, bus_status)) {
                    break;
                }
                continue;
            }

            if (!ready) {
                ++sample.data_not_ready_count;
                sample.consecutive_error_count = 0U;
                sample.last_error = (uint8_t)ICM42688P_STATUS_OK;
                sample.last_bus_error = (uint8_t)IMU_BUS_STATUS_OK;
                app_state_publish_imu(&sample);
                continue;
            }

            status = icm42688p_read_sample(&raw, &bus_status);
            if (status != ICM42688P_STATUS_OK) {
                if (!record_read_failure(&sample, status, bus_status)) {
                    break;
                }
                continue;
            }

            convert_sample(&raw, &sample);
            sample.sample_tick = now;
            ++sample.sample_count;
            sample.consecutive_error_count = 0U;
            sample.last_error = (uint8_t)ICM42688P_STATUS_OK;
            sample.last_bus_error = (uint8_t)IMU_BUS_STATUS_OK;
            app_state_publish_imu(&sample);
        }
    }
}

void imu_task_create(void)
{
    imu_task_handle = xTaskCreateStatic(imu_task,
                                        "ImuTask",
                                        IMU_TASK_STACK_WORDS,
                                        NULL,
                                        IMU_TASK_PRIORITY,
                                        imu_task_stack,
                                        &imu_task_control_block);
    configASSERT(imu_task_handle != NULL);
}

uint32_t imu_task_stack_high_water_mark(void)
{
    if (imu_task_handle == NULL) {
        return 0U;
    }
    return (uint32_t)uxTaskGetStackHighWaterMark(imu_task_handle);
}

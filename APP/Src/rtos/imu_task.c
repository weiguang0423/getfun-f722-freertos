/*
 * imu_task.c - 板载 ICM42688P 的 1 kHz DRDY 门控 DMA 所有者任务。
 *
 * 职责:
 *   负责传感器初始化/恢复、固定周期寄存器 DRDY 轮询、异步 SPI1 DMA 采样读取、
 *   SI单位转换、CW90板级对准、陀螺静态零偏校准、持久化加速度校准、统计以及
 *   app_state发布。
 *
 * 核心函数:
 *   - imu_task_create(): 以 idle 优先级 + 4 静态创建 ImuTask。
 *   - imu_task(): 重试初始化,DRDY 置位后启动 DMA,等待直接任务通知,
 *     连续 3 次失败后进入恢复流程。
 *   - imu_dma_completion_from_isr(): 仅在 DMA ISR 中唤醒 ImuTask。
 *   - imu_task_request_accel_calibration(): 由MSP任务排队一次校准请求。
 *   - imu_task_stack_high_water_mark(): 供 1 Hz 诊断摘要使用。
 *
 * 数据流与约束:
 *   ICM42688P原始传感器坐标系数值 -> SI转换 -> CW90机体系 -> 上电陀螺
 *   静态零偏校准/扣除 -> 水平加速度校准/持久化/扣除 ->
 *   app_state_publish_imu()。本任务是SPI/IMU和参数Flash保存的唯一写者；ISR
 *   只发送DMA任务通知。解析、浮点运算、状态发布、超时与恢复全部留在任务
 *   上下文中。不使用动态内存，也不在1 kHz正常路径输出日志。
 */
#include "rtos/imu_task.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "algorithms/accel_calibration.h"
#include "algorithms/gyro_calibration.h"
#include "app_state.h"
#include "drivers/icm42688p.h"
#include "platform/platform_diag.h"
#include "storage/parameter_store.h"

#define IMU_TASK_STACK_WORDS 512U
#define IMU_TASK_PRIORITY (tskIDLE_PRIORITY + 4U)
#define IMU_TASK_PERIOD_TICKS pdMS_TO_TICKS(1U)
#define IMU_DMA_TIMEOUT_TICKS pdMS_TO_TICKS(2U)
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

_Static_assert(APP_STATE_AXIS_COUNT == GYRO_CALIBRATION_AXIS_COUNT,
               "IMU and gyro calibration axis counts must match");
_Static_assert(APP_STATE_AXIS_COUNT == ACCEL_CALIBRATION_AXIS_COUNT,
               "IMU and accel calibration axis counts must match");
_Static_assert(APP_STATE_AXIS_COUNT == PARAMETER_STORE_AXIS_COUNT,
               "IMU and parameter store axis counts must match");

static StaticTask_t imu_task_control_block;
static StackType_t imu_task_stack[IMU_TASK_STACK_WORDS];
static TaskHandle_t imu_task_handle;
static gyro_calibration_t gyro_calibration;
static accel_calibration_t accel_calibration;
static parameter_store_values_t parameter_values;
static bool accel_calibration_request_pending;

static app_gyro_calibration_state_t calibration_state_for_app(
    gyro_calibration_state_t state)
{
    switch (state) {
    case GYRO_CALIBRATION_CALIBRATING:
        return APP_GYRO_CALIBRATION_CALIBRATING;
    case GYRO_CALIBRATION_READY:
        return APP_GYRO_CALIBRATION_READY;
    case GYRO_CALIBRATION_NOT_STARTED:
    default:
        return APP_GYRO_CALIBRATION_NOT_STARTED;
    }
}

static app_accel_calibration_state_t accel_calibration_state_for_app(
    accel_calibration_state_t state)
{
    switch (state) {
    case ACCEL_CALIBRATION_READY:
        return APP_ACCEL_CALIBRATION_READY;
    case ACCEL_CALIBRATION_CALIBRATING:
        return APP_ACCEL_CALIBRATION_CALIBRATING;
    case ACCEL_CALIBRATION_CANDIDATE_READY:
        return APP_ACCEL_CALIBRATION_CANDIDATE_READY;
    case ACCEL_CALIBRATION_SAVE_FAILED:
        return APP_ACCEL_CALIBRATION_SAVE_FAILED;
    case ACCEL_CALIBRATION_NOT_CALIBRATED:
    default:
        return APP_ACCEL_CALIBRATION_NOT_CALIBRATED;
    }
}

static void sync_calibration_state(app_imu_sample_t *sample)
{
    sample->gyro_calibration_state =
        calibration_state_for_app(gyro_calibration.state);
    sample->gyro_calibration_sample_count =
        gyro_calibration.stable_sample_count;
    sample->gyro_calibration_restart_count =
        gyro_calibration.restart_count;
    sample->gyro_calibration_motion_reject_count =
        gyro_calibration.motion_reject_count;
    sample->gyro_calibration_invalid_sample_count =
        gyro_calibration.invalid_sample_count;
    memcpy(sample->gyro_bias_rad_s,
           gyro_calibration.bias_rad_s,
           sizeof(sample->gyro_bias_rad_s));
    sample->accel_calibration_state =
        accel_calibration_state_for_app(accel_calibration.state);
    sample->accel_calibration_sample_count =
        accel_calibration.stable_sample_count;
    sample->accel_calibration_restart_count =
        accel_calibration.restart_count;
    sample->accel_calibration_motion_reject_count =
        accel_calibration.motion_reject_count;
    sample->accel_calibration_level_reject_count =
        accel_calibration.level_reject_count;
    sample->accel_calibration_invalid_sample_count =
        accel_calibration.invalid_sample_count;
    memcpy(sample->accel_bias_m_s2,
           accel_calibration.bias_m_s2,
           sizeof(sample->accel_bias_m_s2));
}

static app_parameter_load_result_t parameter_load_result_for_app(
    parameter_store_load_result_t result)
{
    switch (result) {
    case PARAMETER_STORE_LOAD_SLOT_A:
        return APP_PARAMETER_LOAD_SLOT_A;
    case PARAMETER_STORE_LOAD_SLOT_B:
        return APP_PARAMETER_LOAD_SLOT_B;
    case PARAMETER_STORE_LOAD_RECOVERED_SLOT_A:
        return APP_PARAMETER_LOAD_RECOVERED_SLOT_A;
    case PARAMETER_STORE_LOAD_RECOVERED_SLOT_B:
        return APP_PARAMETER_LOAD_RECOVERED_SLOT_B;
    case PARAMETER_STORE_LOAD_DEFAULTS_CORRUPT:
        return APP_PARAMETER_LOAD_DEFAULTS_CORRUPT;
    case PARAMETER_STORE_LOAD_DEFAULTS_EMPTY:
    default:
        return APP_PARAMETER_LOAD_DEFAULTS_EMPTY;
    }
}

static app_parameter_save_result_t parameter_save_result_for_app(
    parameter_store_save_result_t result)
{
    switch (result) {
    case PARAMETER_STORE_SAVE_OK:
        return APP_PARAMETER_SAVE_OK;
    case PARAMETER_STORE_SAVE_BAD_ARGUMENT:
        return APP_PARAMETER_SAVE_BAD_ARGUMENT;
    case PARAMETER_STORE_SAVE_FLASH_UNLOCK_FAILED:
        return APP_PARAMETER_SAVE_FLASH_UNLOCK_FAILED;
    case PARAMETER_STORE_SAVE_ERASE_FAILED:
        return APP_PARAMETER_SAVE_ERASE_FAILED;
    case PARAMETER_STORE_SAVE_PROGRAM_FAILED:
        return APP_PARAMETER_SAVE_PROGRAM_FAILED;
    case PARAMETER_STORE_SAVE_VERIFY_FAILED:
        return APP_PARAMETER_SAVE_VERIFY_FAILED;
    case PARAMETER_STORE_SAVE_NOT_ATTEMPTED:
    default:
        return APP_PARAMETER_SAVE_NOT_ATTEMPTED;
    }
}

static app_parameter_slot_t parameter_slot_for_app(
    parameter_store_slot_t slot)
{
    switch (slot) {
    case PARAMETER_STORE_SLOT_A:
        return APP_PARAMETER_SLOT_A;
    case PARAMETER_STORE_SLOT_B:
        return APP_PARAMETER_SLOT_B;
    case PARAMETER_STORE_SLOT_NONE:
    default:
        return APP_PARAMETER_SLOT_NONE;
    }
}

static void publish_parameter_state(void)
{
    parameter_store_status_t source;
    app_parameter_state_t destination;

    parameter_store_get_status(&source);
    memset(&destination, 0, sizeof(destination));
    destination.storage_valid = source.storage_valid;
    destination.load_result =
        parameter_load_result_for_app(source.load_result);
    destination.last_save_result =
        parameter_save_result_for_app(source.last_save_result);
    destination.active_slot =
        parameter_slot_for_app(source.active_slot);
    destination.invalid_slot_mask = source.invalid_slot_mask;
    destination.sequence = source.sequence;
    destination.save_error_count = source.save_error_count;
    destination.last_hal_error = source.last_hal_error;
    app_state_publish_parameters(&destination);
}

static void restore_accel_calibration_from_parameters(void)
{
    accel_calibration_initialize(
        &accel_calibration,
        parameter_values.accel_bias_m_s2,
        parameter_values.accel_calibration_valid);
}

static bool take_accel_calibration_request(void)
{
    bool pending;

    taskENTER_CRITICAL();
    pending = accel_calibration_request_pending;
    accel_calibration_request_pending = false;
    taskEXIT_CRITICAL();
    return pending;
}

static void start_requested_accel_calibration(
    app_imu_sample_t *sample)
{
    if (!take_accel_calibration_request()) {
        return;
    }

    if (sample->present &&
        (gyro_calibration.state == GYRO_CALIBRATION_READY)) {
        (void)accel_calibration_start(&accel_calibration);
        sync_calibration_state(sample);
        app_state_publish_imu(sample);
    }
}

static void persist_accel_calibration_candidate(void)
{
    parameter_store_values_t candidate_values = parameter_values;
    float candidate_bias[APP_STATE_AXIS_COUNT];

    if (!accel_calibration_get_candidate(&accel_calibration,
                                         candidate_bias)) {
        return;
    }

    candidate_values.accel_calibration_valid = true;
    memcpy(candidate_values.accel_bias_m_s2,
           candidate_bias,
           sizeof(candidate_values.accel_bias_m_s2));

    /*
     * v0.5.0 still has no armed state or timer motor output. Force every motor
     * GPIO low immediately before the blocking internal-Flash transaction.
     * The future ARM state machine must additionally reject save while armed.
     */
    platform_motor_outputs_force_safe();
    if (parameter_store_save(&candidate_values) ==
        PARAMETER_STORE_SAVE_OK) {
        parameter_values = candidate_values;
        accel_calibration_mark_persisted(&accel_calibration);
    } else {
        accel_calibration_mark_save_failed(&accel_calibration);
    }
    publish_parameter_state();
}

static void imu_dma_completion_from_isr(void *context)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    (void)context;
    if (imu_task_handle != NULL) {
        vTaskNotifyGiveFromISR(imu_task_handle,
                               &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

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

    gyro_calibration_reset(&gyro_calibration);
    restore_accel_calibration_from_parameters();
    sync_calibration_state(sample);

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
            gyro_calibration_start(&gyro_calibration);
            sync_calibration_state(sample);
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
        gyro_calibration_reset(&gyro_calibration);
        restore_accel_calibration_from_parameters();
        sync_calibration_state(sample);
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
    configASSERT(IMU_DMA_TIMEOUT_TICKS > 0U);
    parameter_store_init();
    parameter_store_get_values(&parameter_values);
    restore_accel_calibration_from_parameters();
    publish_parameter_state();
    sync_calibration_state(&sample);
    app_state_publish_imu(&sample);
    imu_bus_set_dma_callback(imu_dma_completion_from_isr, NULL);

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
            start_requested_accel_calibration(&sample);
            now = xTaskGetTickCount();
            if ((TickType_t)(now - last_wake_time) > 0U) {
                sample.missed_deadline_count +=
                    (uint32_t)(now - last_wake_time);
            }

            ++sample.drdy_poll_count;
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

            (void)ulTaskNotifyTake(pdTRUE, 0U);
            status = icm42688p_start_sample_dma(&bus_status);
            if (status != ICM42688P_STATUS_OK) {
                ++sample.dma_start_error_count;
                if (!record_read_failure(&sample, status, bus_status)) {
                    break;
                }
                continue;
            }

            if (ulTaskNotifyTake(pdTRUE, IMU_DMA_TIMEOUT_TICKS) == 0U) {
                ++sample.dma_timeout_count;
                (void)imu_bus_dma_abort();
                ++sample.dma_abort_count;
                if (!record_read_failure(
                        &sample,
                        ICM42688P_STATUS_BUS_ERROR,
                        IMU_BUS_STATUS_HAL_TIMEOUT)) {
                    break;
                }
                continue;
            }

            status = icm42688p_finish_sample_dma(&raw, &bus_status);
            if (status != ICM42688P_STATUS_OK) {
                ++sample.dma_completion_error_count;
                if (!record_read_failure(&sample, status, bus_status)) {
                    break;
                }
                continue;
            }

            convert_sample(&raw, &sample);
            (void)gyro_calibration_process(
                &gyro_calibration,
                sample.angular_rate_rad_s,
                sample.acceleration_m_s2);
            gyro_calibration_apply(
                &gyro_calibration,
                sample.angular_rate_rad_s,
                sample.angular_rate_rad_s);
            (void)accel_calibration_process(
                &accel_calibration,
                sample.acceleration_m_s2,
                sample.angular_rate_rad_s);
            persist_accel_calibration_candidate();
            accel_calibration_apply(
                &accel_calibration,
                sample.acceleration_m_s2,
                sample.acceleration_m_s2);
            sync_calibration_state(&sample);
            sample.sample_tick = now;
            ++sample.sample_count;
            ++sample.dma_transfer_count;
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

bool imu_task_request_accel_calibration(void)
{
    bool accepted = false;

    taskENTER_CRITICAL();
    if ((imu_task_handle != NULL) &&
        !accel_calibration_request_pending) {
        accel_calibration_request_pending = true;
        accepted = true;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

uint32_t imu_task_stack_high_water_mark(void)
{
    if (imu_task_handle == NULL) {
        return 0U;
    }
    return (uint32_t)uxTaskGetStackHighWaterMark(imu_task_handle);
}

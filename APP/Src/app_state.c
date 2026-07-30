/*
 * app_state.c —— 全局运行态快照实现
 *
 * 本文件持有一份静态的 app_state_snapshot_t state 作为系统当前状态，并提供多任务安全的
 * 读/写接口。它是 app_state.h 中各 publish/set/get 函数的具体实现。
 *
 * 并发同步策略（关键）：
 *   临界区用"关中断 + DMB"实现，而非 FreeRTOS 互斥量——
 *     app_state_lock()   —— 读 PRIMASK、__disable_irq()、__DMB()，返回旧的 PRIMASK；
 *     app_state_unlock() —— __DMB()，仅当原本中断开启时才重新 __enable_irq()。
 *   这样做既能保护 32 位整体拷贝/多字段写入的原子性，又不会让发布者进入阻塞态。
 *
 * 主要内容：
 *   - app_state_init()：在临界区内 memset 清零整份快照，并默认设置 IMU、陀螺、
 *       加速度校准、参数有效性和IMU时间有效性解锁抑制位。
 *   - app_state_get_snapshot()：临界区内整体拷贝 state，退出后把 uptime_ms 替换为
 *       HAL_GetTick()，保证每次读到的都是"当下"的运行时长。
 *   - app_state_set_runtime/set_fault_flags：写入运行时与故障标志。
 *   - app_state_publish_parameters()/publish_imu()：整体复制参数/IMU状态，并原子
 *       更新参数有效性和传感器校准解锁抑制位。
 *   - app_state_publish_attitude/battery：发布姿态和电池数据。
 *   - app_state_set_configurator_arming_disabled/set_host_rtc：回写 Configurator 下发的
 *       解锁状态与 RTC 时间。
 *
 * 约束：发布者必须在临界区内保持短小，禁止调用任何可能阻塞的 API。
 */
#include "app_state.h"

#include <string.h>

#include "stm32f7xx_hal.h"

static app_state_snapshot_t state;

static uint32_t app_state_lock(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

static void app_state_unlock(uint32_t primask)
{
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
}

void app_state_init(void)
{
    const uint32_t primask = app_state_lock();
    memset(&state, 0, sizeof(state));
    state.arming_inhibit_flags =
        APP_ARMING_INHIBIT_IMU_NOT_READY |
        APP_ARMING_INHIBIT_GYRO_NOT_CALIBRATED |
        APP_ARMING_INHIBIT_ACCEL_NOT_CALIBRATED |
        APP_ARMING_INHIBIT_PARAMETERS_INVALID |
        APP_ARMING_INHIBIT_IMU_TIMING_INVALID;
    app_state_unlock(primask);
}

void app_state_get_snapshot(app_state_snapshot_t *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL) {
        return;
    }

    primask = app_state_lock();
    *snapshot = state;
    app_state_unlock(primask);
    snapshot->uptime_ms = HAL_GetTick();
}

void app_state_set_runtime(uint16_t cycle_time_us,
                           uint16_t cpu_load_permille,
                           uint16_t i2c_error_count)
{
    const uint32_t primask = app_state_lock();
    state.cycle_time_us = cycle_time_us;
    state.cpu_load_permille = cpu_load_permille;
    state.i2c_error_count = i2c_error_count;
    app_state_unlock(primask);
}

void app_state_set_fault_flags(uint32_t fault_flags)
{
    const uint32_t primask = app_state_lock();
    state.fault_flags = fault_flags;
    app_state_unlock(primask);
}

void app_state_publish_parameters(
    const app_parameter_state_t *parameters)
{
    const uint32_t primask = app_state_lock();

    if (parameters != NULL) {
        state.parameters = *parameters;
        if (parameters->storage_valid) {
            state.arming_inhibit_flags &=
                ~APP_ARMING_INHIBIT_PARAMETERS_INVALID;
        } else {
            state.arming_inhibit_flags |=
                APP_ARMING_INHIBIT_PARAMETERS_INVALID;
        }
    }
    app_state_unlock(primask);
}

void app_state_publish_imu(const app_imu_sample_t *sample)
{
    const uint32_t primask = app_state_lock();

    if (sample != NULL) {
        state.imu = *sample;
        if (sample->present) {
            state.arming_inhibit_flags &=
                ~APP_ARMING_INHIBIT_IMU_NOT_READY;
        } else {
            state.arming_inhibit_flags |=
                APP_ARMING_INHIBIT_IMU_NOT_READY;
        }

        if (sample->gyro_calibration_state ==
            APP_GYRO_CALIBRATION_READY) {
            state.arming_inhibit_flags &=
                ~APP_ARMING_INHIBIT_GYRO_NOT_CALIBRATED;
        } else {
            state.arming_inhibit_flags |=
                APP_ARMING_INHIBIT_GYRO_NOT_CALIBRATED;
        }

        if (sample->accel_calibration_state ==
            APP_ACCEL_CALIBRATION_READY) {
            state.arming_inhibit_flags &=
                ~APP_ARMING_INHIBIT_ACCEL_NOT_CALIBRATED;
        } else {
            state.arming_inhibit_flags |=
                APP_ARMING_INHIBIT_ACCEL_NOT_CALIBRATED;
        }

        if (sample->present && sample->timing_valid &&
            sample->filter_ready) {
            state.arming_inhibit_flags &=
                ~APP_ARMING_INHIBIT_IMU_TIMING_INVALID;
            state.cycle_time_us =
                sample->sample_interval_us > UINT16_MAX
                    ? UINT16_MAX
                    : (uint16_t)sample->sample_interval_us;
        } else {
            state.arming_inhibit_flags |=
                APP_ARMING_INHIBIT_IMU_TIMING_INVALID;
        }
    }
    app_state_unlock(primask);
}

void app_state_publish_attitude(int16_t roll_deg10,
                                int16_t pitch_deg10,
                                int16_t yaw_deg,
                                bool valid)
{
    const uint32_t primask = app_state_lock();
    state.roll_deg10 = roll_deg10;
    state.pitch_deg10 = pitch_deg10;
    state.yaw_deg = yaw_deg;
    state.attitude_valid = valid;
    app_state_unlock(primask);
}

void app_state_publish_battery(uint8_t cell_count,
                               uint16_t capacity_mah,
                               uint16_t voltage_cv,
                               int16_t current_ca,
                               uint16_t consumed_mah,
                               uint16_t rssi,
                               bool present)
{
    const uint32_t primask = app_state_lock();
    state.battery_cell_count = cell_count;
    state.battery_capacity_mah = capacity_mah;
    state.battery_voltage_cv = voltage_cv;
    state.battery_current_ca = current_ca;
    state.battery_consumed_mah = consumed_mah;
    state.rssi = rssi;
    state.battery_present = present;
    app_state_unlock(primask);
}

void app_state_set_configurator_arming_disabled(bool disabled)
{
    const uint32_t primask = app_state_lock();
    state.configurator_arming_disabled = disabled;
    app_state_unlock(primask);
}

void app_state_set_host_rtc(uint32_t seconds, uint16_t millis)
{
    const uint32_t primask = app_state_lock();
    state.host_rtc_seconds = seconds;
    state.host_rtc_millis = millis;
    app_state_unlock(primask);
}

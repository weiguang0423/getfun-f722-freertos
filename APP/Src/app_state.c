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

void app_state_publish_imu(const int16_t accelerometer[APP_STATE_AXIS_COUNT],
                           const int16_t gyroscope[APP_STATE_AXIS_COUNT],
                           const int16_t magnetometer[APP_STATE_AXIS_COUNT],
                           bool present)
{
    const uint32_t primask = app_state_lock();

    if ((accelerometer != NULL) && (gyroscope != NULL) && (magnetometer != NULL)) {
        memcpy(state.accelerometer, accelerometer, sizeof(state.accelerometer));
        memcpy(state.gyroscope, gyroscope, sizeof(state.gyroscope));
        memcpy(state.magnetometer, magnetometer, sizeof(state.magnetometer));
    }
    state.imu_present = present;
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

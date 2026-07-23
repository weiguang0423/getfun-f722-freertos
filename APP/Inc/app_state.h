#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_STATE_AXIS_COUNT 3U

typedef struct
{
    uint32_t uptime_ms;
    uint16_t cycle_time_us;
    uint16_t i2c_error_count;
    uint16_t cpu_load_permille;
    uint32_t fault_flags;

    bool imu_present;
    bool attitude_valid;
    int16_t accelerometer[APP_STATE_AXIS_COUNT];
    int16_t gyroscope[APP_STATE_AXIS_COUNT];
    int16_t magnetometer[APP_STATE_AXIS_COUNT];
    int16_t roll_deg10;
    int16_t pitch_deg10;
    int16_t yaw_deg;

    bool battery_present;
    uint8_t battery_cell_count;
    uint16_t battery_capacity_mah;
    uint16_t battery_voltage_cv;
    int16_t battery_current_ca;
    uint16_t battery_consumed_mah;
    uint16_t rssi;

    bool configurator_arming_disabled;
    uint32_t host_rtc_seconds;
    uint16_t host_rtc_millis;
} app_state_snapshot_t;

void app_state_init(void);
void app_state_get_snapshot(app_state_snapshot_t *snapshot);

void app_state_set_runtime(uint16_t cycle_time_us,
                           uint16_t cpu_load_permille,
                           uint16_t i2c_error_count);
void app_state_set_fault_flags(uint32_t fault_flags);

void app_state_publish_imu(const int16_t accelerometer[APP_STATE_AXIS_COUNT],
                           const int16_t gyroscope[APP_STATE_AXIS_COUNT],
                           const int16_t magnetometer[APP_STATE_AXIS_COUNT],
                           bool present);
void app_state_publish_attitude(int16_t roll_deg10,
                                int16_t pitch_deg10,
                                int16_t yaw_deg,
                                bool valid);
void app_state_publish_battery(uint8_t cell_count,
                               uint16_t capacity_mah,
                               uint16_t voltage_cv,
                               int16_t current_ca,
                               uint16_t consumed_mah,
                               uint16_t rssi,
                               bool present);

void app_state_set_configurator_arming_disabled(bool disabled);
void app_state_set_host_rtc(uint32_t seconds, uint16_t millis);

#ifdef __cplusplus
}
#endif

#endif

/*
 * app_state.h —— 全局运行态快照（应用层数据中心）接口
 *
 * 本头文件定义整个应用层共享的"当前状态快照"结构，以及各任务对其读/写的接口。
 * 它是【发布者】（传感器/电池等任务）与【消费者】（MspTask 组装回包）之间的唯一数据通道。
 *
 * 主要内容：
 *   - APP_STATE_AXIS_COUNT：三轴常量（3）。
 *   - app_imu_sample_t：IMU 在线状态、配置回读、DRDY/DMA统计量、陀螺/加速度
 *       校准状态和 SI 物理量。
 *   - app_parameter_state_t：参数Flash加载、保存、活动槽、序号和错误状态。
 *   - app_state_snapshot_t：状态快照结构，字段分组——
 *       运行时：uptime / cycle_time / i2c_error / cpu_load / fault_flags；
 *       IMU：完整 app_imu_sample_t + 姿态角(roll/pitch/yaw)；
 *       电池：battery_present + 电芯数/容量/电压/电流/已耗电量/rssi；
 *       宿主：configurator_arming_disabled + 宿主机下发的 RTC 时间。
 *   - 读取：app_state_get_snapshot() —— 整体拷贝一份快照（多任务安全）。
 *   - 发布/写入：app_state_publish_imu/attitude/battery、app_state_set_runtime/
 *       set_fault_flags、app_state_set_configurator_arming_disabled、app_state_set_host_rtc。
 *
 * 设计说明：ImuTask 是 IMU 单写者；并发安全靠 app_state.c 内部的短临界区实现
 * （关中断+DMB，而非互斥量），发布者整体复制，消费者只读快照，互不阻塞。
 */
#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_STATE_AXIS_COUNT 3U
#define APP_ARMING_INHIBIT_IMU_NOT_READY (1UL << 0U)
#define APP_ARMING_INHIBIT_GYRO_NOT_CALIBRATED (1UL << 1U)
#define APP_ARMING_INHIBIT_ACCEL_NOT_CALIBRATED (1UL << 2U)
#define APP_ARMING_INHIBIT_PARAMETERS_INVALID (1UL << 3U)

typedef enum
{
    APP_GYRO_CALIBRATION_NOT_STARTED = 0,
    APP_GYRO_CALIBRATION_CALIBRATING,
    APP_GYRO_CALIBRATION_READY
} app_gyro_calibration_state_t;

typedef enum
{
    APP_ACCEL_CALIBRATION_NOT_CALIBRATED = 0,
    APP_ACCEL_CALIBRATION_READY,
    APP_ACCEL_CALIBRATION_CALIBRATING,
    APP_ACCEL_CALIBRATION_CANDIDATE_READY,
    APP_ACCEL_CALIBRATION_SAVE_FAILED
} app_accel_calibration_state_t;

typedef enum
{
    APP_PARAMETER_LOAD_DEFAULTS_EMPTY = 0,
    APP_PARAMETER_LOAD_SLOT_A,
    APP_PARAMETER_LOAD_SLOT_B,
    APP_PARAMETER_LOAD_RECOVERED_SLOT_A,
    APP_PARAMETER_LOAD_RECOVERED_SLOT_B,
    APP_PARAMETER_LOAD_DEFAULTS_CORRUPT
} app_parameter_load_result_t;

typedef enum
{
    APP_PARAMETER_SAVE_NOT_ATTEMPTED = 0,
    APP_PARAMETER_SAVE_OK,
    APP_PARAMETER_SAVE_BAD_ARGUMENT,
    APP_PARAMETER_SAVE_FLASH_UNLOCK_FAILED,
    APP_PARAMETER_SAVE_ERASE_FAILED,
    APP_PARAMETER_SAVE_PROGRAM_FAILED,
    APP_PARAMETER_SAVE_VERIFY_FAILED
} app_parameter_save_result_t;

typedef enum
{
    APP_PARAMETER_SLOT_NONE = 0,
    APP_PARAMETER_SLOT_A,
    APP_PARAMETER_SLOT_B
} app_parameter_slot_t;

typedef struct
{
    bool storage_valid;
    app_parameter_load_result_t load_result;
    app_parameter_save_result_t last_save_result;
    app_parameter_slot_t active_slot;
    uint8_t invalid_slot_mask;
    uint32_t sequence;
    uint32_t save_error_count;
    uint32_t last_hal_error;
} app_parameter_state_t;

typedef struct
{
    bool present;
    uint8_t who_am_i;
    uint8_t last_error;
    uint8_t last_bus_error;
    uint8_t gyro_config0;
    uint8_t accel_config0;
    uint8_t pwr_mgmt0;
    app_gyro_calibration_state_t gyro_calibration_state;
    uint16_t gyro_calibration_sample_count;
    uint32_t gyro_calibration_restart_count;
    uint32_t gyro_calibration_motion_reject_count;
    uint32_t gyro_calibration_invalid_sample_count;
    float gyro_bias_rad_s[APP_STATE_AXIS_COUNT];
    app_accel_calibration_state_t accel_calibration_state;
    uint16_t accel_calibration_sample_count;
    uint32_t accel_calibration_restart_count;
    uint32_t accel_calibration_motion_reject_count;
    uint32_t accel_calibration_level_reject_count;
    uint32_t accel_calibration_invalid_sample_count;
    float accel_bias_m_s2[APP_STATE_AXIS_COUNT];
    TickType_t sample_tick;
    uint32_t sample_count;
    uint32_t initialization_error_count;
    uint32_t read_error_count;
    uint32_t consecutive_error_count;
    uint32_t data_not_ready_count;
    uint32_t missed_deadline_count;
    uint32_t drdy_poll_count;
    uint32_t dma_transfer_count;
    uint32_t dma_start_error_count;
    uint32_t dma_completion_error_count;
    uint32_t dma_timeout_count;
    uint32_t dma_abort_count;
    float acceleration_m_s2[APP_STATE_AXIS_COUNT];
    float angular_rate_rad_s[APP_STATE_AXIS_COUNT];
    float temperature_c;
} app_imu_sample_t;

typedef struct
{
    uint32_t uptime_ms;
    uint16_t cycle_time_us;
    uint16_t i2c_error_count;
    uint16_t cpu_load_permille;
    uint32_t fault_flags;
    uint32_t arming_inhibit_flags;

    app_parameter_state_t parameters;
    app_imu_sample_t imu;
    bool attitude_valid;
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

void app_state_publish_parameters(
    const app_parameter_state_t *parameters);
void app_state_publish_imu(const app_imu_sample_t *sample);
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

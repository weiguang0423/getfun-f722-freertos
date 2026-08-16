/*
 * app_state.h —— 全局运行态快照（应用层数据中心）接口
 *
 * 本头文件定义整个应用层共享的"当前状态快照"结构，以及各任务对其读/写的接口。
 * 它是【发布者】（传感器/电池等任务）与【消费者】（MspTask 组装回包）之间的唯一数据通道。
 *
 * 主要内容：
 *   - APP_STATE_AXIS_COUNT：三轴常量（3）。
 *   - app_imu_sample_t：IMU 在线状态、配置回读、DRDY/DMA统计量、微秒时间/dt、
 *       低通状态、陀螺/加速度校准状态和 SI 物理量。
 *   - app_parameter_state_t：参数Flash加载、保存、活动槽、序号和错误状态。
 *   - app_state_snapshot_t：状态快照结构，字段分组——
 *       运行时：uptime / cycle_time / i2c_error / cpu_load / fault_flags；
 *       IMU：完整 app_imu_sample_t；
 *       姿态：四元数、欧拉角、READY和Mahony诊断计数；
 *       RC：16路CRSF原始/微秒/映射通道、接收时间、Failsafe、
 *       Link Statistics和接收诊断；
 *       电池：ADC运行状态、原始/滤波值、电芯数、电压、电流、已耗电量和低压状态；
 *       飞行：输入新鲜度、安全位、RC setpoint/AUX请求、Rate PID、Quad-X Mixer、
 *       四路测试/输出值和DShot诊断；
 *       宿主：configurator_arming_disabled + 宿主机下发的 RTC 时间。
 *   - 读取：app_state_get_snapshot() —— 整体拷贝一份快照（多任务安全）。
 *   - 发布/写入：app_state_publish_imu/attitude/rc/battery/flight、app_state_set_runtime/
 *       set_fault_flags、app_state_set_configurator_arming_disabled、app_state_set_host_rtc。
 *
 * 设计说明：ImuTask是IMU/姿态单写者，RcTask是RC单写者；并发安全靠app_state.c内部的短临界区实现
 * （关中断+DMB，而非互斥量），发布者整体复制，消费者只读快照，互不阻塞。
 */
#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "algorithms/angle_outer_loop.h"
#include "algorithms/flight_arming.h"
#include "algorithms/power_monitor.h"
#include "algorithms/quad_x_mixer.h"
#include "algorithms/rate_pid.h"
#include "algorithms/rc_input.h"
#include "algorithms/rc_setpoint.h"
#include "storage/parameter_store.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_STATE_AXIS_COUNT 3U
#define APP_STATE_QUATERNION_COUNT 4U
#define APP_STATE_RC_CHANNEL_COUNT RC_INPUT_CHANNEL_COUNT
#define APP_ARMING_INHIBIT_IMU_NOT_READY (1UL << 0U)
#define APP_ARMING_INHIBIT_GYRO_NOT_CALIBRATED (1UL << 1U)
#define APP_ARMING_INHIBIT_ACCEL_NOT_CALIBRATED (1UL << 2U)
#define APP_ARMING_INHIBIT_PARAMETERS_INVALID (1UL << 3U)
#define APP_ARMING_INHIBIT_IMU_TIMING_INVALID (1UL << 4U)
#define APP_ARMING_INHIBIT_ATTITUDE_NOT_READY (1UL << 5U)
#define APP_ARMING_INHIBIT_RC_NOT_READY (1UL << 6U)
#define APP_ARMING_INHIBIT_BATTERY_NOT_READY (1UL << 7U)
#define APP_FLIGHT_SAFETY_IMU_INVALID (1UL << 0U)
#define APP_FLIGHT_SAFETY_IMU_STALE (1UL << 1U)
#define APP_FLIGHT_SAFETY_RC_INVALID (1UL << 2U)
#define APP_FLIGHT_SAFETY_RC_STALE (1UL << 3U)
#define APP_FLIGHT_SAFETY_ARMING_INHIBITED (1UL << 4U)
#define APP_FLIGHT_SAFETY_DSHOT_NOT_READY (1UL << 5U)
#define APP_FLIGHT_SAFETY_CONFIGURATOR_DISABLED (1UL << 6U)
#define APP_FLIGHT_SAFETY_SYSTEM_FAULT (1UL << 7U)
#define APP_FLIGHT_SAFETY_CONTROL_INVALID (1UL << 8U)
#define APP_FLIGHT_SAFETY_THROTTLE_HIGH (1UL << 9U)
#define APP_FLIGHT_SAFETY_MOTOR_TEST_ACTIVE (1UL << 10U)
#define APP_FLIGHT_SAFETY_ARMING_ONLY_MASK \
    (APP_FLIGHT_SAFETY_THROTTLE_HIGH | \
     APP_FLIGHT_SAFETY_MOTOR_TEST_ACTIVE)
#define APP_STATE_MOTOR_COUNT 4U

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
    uint16_t loaded_record_version;
    bool migration_pending;
    parameter_store_values_t values;
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
    bool timing_source_ready;
    bool timing_valid;
    bool filter_ready;
    uint32_t sample_timestamp_us;
    uint32_t sample_interval_us;
    uint32_t sample_interval_min_us;
    uint32_t sample_interval_max_us;
    uint32_t timing_invalid_count;
    uint32_t timing_reset_count;
    uint32_t filter_reset_count;
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
    float filtered_acceleration_m_s2[APP_STATE_AXIS_COUNT];
    float filtered_angular_rate_rad_s[APP_STATE_AXIS_COUNT];
    float temperature_c;
} app_imu_sample_t;

typedef struct
{
    bool valid;
    float quaternion[APP_STATE_QUATERNION_COUNT];
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint32_t update_count;
    uint32_t reset_count;
    uint32_t invalid_input_count;
    uint32_t accel_rejection_count;
    uint32_t gyro_only_update_count;
} app_attitude_state_t;

typedef struct
{
    bool uart_running;
    bool channels_valid;
    bool failsafe_active;
    bool link_statistics_valid;
    rc_input_phase_t failsafe_phase;
    TickType_t last_channel_tick;
    TickType_t last_link_statistics_tick;
    TickType_t recovery_started_tick;
    TickType_t last_failsafe_tick;
    uint32_t channel_sequence;
    uint32_t channel_frame_count;
    uint32_t link_frame_count;
    uint32_t unsupported_frame_count;
    uint32_t payload_error_count;
    uint32_t parser_valid_frame_count;
    uint32_t parser_crc_error_count;
    uint32_t parser_length_error_count;
    uint32_t parser_sync_drop_count;
    uint32_t uart_start_error_count;
    uint32_t uart_rx_event_count;
    uint32_t uart_idle_event_count;
    uint32_t uart_ring_overflow_count;
    uint32_t uart_error_count;
    uint32_t uart_recovery_count;
    uint32_t last_uart_error;
    uint32_t failsafe_count;
    uint32_t failsafe_recovery_count;
    uint16_t failsafe_recovery_frame_count;
    uint16_t channel_raw[APP_STATE_RC_CHANNEL_COUNT];
    uint16_t channel_us[APP_STATE_RC_CHANNEL_COUNT];
    uint16_t mapped_channel_us[APP_STATE_RC_CHANNEL_COUNT];
    int16_t uplink_rssi_dbm[2];
    uint8_t uplink_link_quality;
    int8_t uplink_snr_db;
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;
    int16_t downlink_rssi_dbm;
    uint8_t downlink_link_quality;
    int8_t downlink_snr_db;
} app_rc_state_t;

typedef struct
{
    bool adc_running;
    bool present;
    power_battery_state_t state;
    uint8_t cell_count;
    TickType_t last_sample_tick;
    uint16_t capacity_mah;
    uint16_t voltage_cv;
    int16_t current_ca;
    uint16_t consumed_mah;
    uint16_t raw[POWER_MONITOR_ADC_CHANNEL_COUNT];
    uint16_t filtered_raw[POWER_MONITOR_ADC_CHANNEL_COUNT];
    uint32_t sample_sequence;
    uint32_t sample_count;
    uint32_t invalid_sample_count;
    uint32_t adc_start_count;
    uint32_t adc_busy_count;
    uint32_t adc_recovery_count;
    uint32_t adc_dma_error_count;
    uint32_t adc_overrun_count;
    uint32_t adc_last_dma_flags;
} app_battery_state_t;

typedef struct
{
    bool inputs_ready;
    bool motor_test_active;
    bool dshot_ready;
    bool dshot_busy;
    bool rate_pid_integrator_enabled;
    bool armed;
    flight_arming_state_t arming_state;
    uint32_t safety_flags;
    uint32_t arming_block_flags;
    uint32_t last_failsafe_flags;
    TickType_t last_update_tick;
    TickType_t last_motor_test_command_tick;
    uint16_t requested_motor_value[APP_STATE_MOTOR_COUNT];
    uint16_t output_motor_value[APP_STATE_MOTOR_COUNT];
    rc_setpoint_output_t rc_setpoint;
    angle_outer_loop_output_t angle_outer_loop;
    rate_pid_output_t rate_pid;
    quad_x_mixer_output_t mixer;
    uint32_t control_sample_count;
    uint32_t control_dt_us;
    uint32_t loop_count;
    uint32_t missed_deadline_count;
    uint32_t imu_stale_count;
    uint32_t rc_stale_count;
    uint32_t motor_test_timeout_count;
    uint32_t dshot_submit_error_count;
    uint32_t dshot_dma_error_count;
    uint32_t rc_setpoint_update_count;
    uint32_t rc_setpoint_error_count;
    uint32_t angle_outer_loop_update_count;
    uint32_t angle_outer_loop_error_count;
    uint32_t rate_pid_update_count;
    uint32_t rate_pid_error_count;
    uint32_t mixer_update_count;
    uint32_t mixer_error_count;
    uint32_t arm_count;
    uint32_t disarm_count;
    uint32_t flight_failsafe_count;
} app_flight_state_t;

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
    app_attitude_state_t attitude;
    app_rc_state_t rc;
    app_battery_state_t battery;
    app_flight_state_t flight;
    uint16_t rssi;

    bool configurator_arming_disabled;
    uint32_t host_rtc_seconds;
    uint16_t host_rtc_millis;
} app_state_snapshot_t;

void app_state_init(void);
void app_state_get_snapshot(app_state_snapshot_t *snapshot);
bool app_state_is_armed(void);

void app_state_set_runtime(uint16_t cycle_time_us,
                           uint16_t cpu_load_permille,
                           uint16_t i2c_error_count);
void app_state_set_fault_flags(uint32_t fault_flags);

void app_state_publish_parameters(
    const app_parameter_state_t *parameters);
void app_state_publish_imu(const app_imu_sample_t *sample);
void app_state_publish_attitude(
    const app_attitude_state_t *attitude);
void app_state_publish_rc(const app_rc_state_t *rc);
void app_state_publish_battery(const app_battery_state_t *battery);
void app_state_publish_flight(const app_flight_state_t *flight);

void app_state_set_configurator_arming_disabled(bool disabled);
void app_state_set_host_rtc(uint32_t seconds, uint16_t millis);

#ifdef __cplusplus
}
#endif

#endif

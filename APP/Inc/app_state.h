/*
 * app_state.h —— 全局运行态快照（应用层数据中心）接口
 *
 * 本头文件定义整个应用层共享的"当前状态快照"结构，以及各任务对其读/写的接口
 * 它是【发布者】（传感器/电池等任务）与【消费者】（MspTask 组装回包）之间的唯一数据通道
 *
 * 主要内容：
 *   - APP_STATE_AXIS_COUNT：三轴常量（3）
 *   - app_imu_sample_t：IMU 在线状态、配置回读、DRDY/DMA统计量、微秒时间/dt、
 *       低通状态、陀螺/加速度校准状态和 SI 物理量
 *   - app_parameter_state_t：参数Flash加载、保存、活动槽、序号和错误状态
 *   - app_state_snapshot_t：状态快照结构，字段分组——
 *       运行时：uptime / cycle_time / i2c_error / cpu_load / fault_flags；
 *       IMU：完整 app_imu_sample_t；
 *       姿态：四元数、欧拉角、READY和Mahony诊断计数；
 *       RC：16路CRSF原始/微秒/映射通道、接收时间、Failsafe、
 *       Link Statistics和接收诊断；
 *       电池：ADC运行状态、原始/滤波值、电芯数、电压、电流、已耗电量和低压状态；
 *       飞行：输入新鲜度、安全位、RC setpoint/AUX请求、Rate PID、Quad-X Mixer、
 *       四路测试/输出值和DShot诊断；
 *       宿主：configurator_arming_disabled + 宿主机下发的 RTC 时间
 *   - 读取：app_state_get_snapshot() —— 整体拷贝一份快照（多任务安全）
 *   - 发布/写入：app_state_publish_imu/attitude/rc/battery/flight、app_state_set_runtime/
 *       set_fault_flags、app_state_set_configurator_arming_disabled、app_state_set_host_rtc
 *
 * 设计说明：ImuTask是IMU/姿态单写者，RcTask是RC单写者；并发安全靠app_state.c内部的短临界区实现
 * （关中断+DMB，而非互斥量），发布者整体复制，消费者只读快照，互不阻塞
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
#include "algorithms/rc_source_arbiter.h"
#include "algorithms/rc_setpoint.h"
#include "storage/parameter_store.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_STATE_AXIS_COUNT 3U                                     /* 三轴数据的元素数量 */
#define APP_STATE_QUATERNION_COUNT 4U                               /* 姿态四元数的元素数量 */
#define APP_STATE_RC_CHANNEL_COUNT RC_INPUT_CHANNEL_COUNT           /* RC 通道数量 */
/* 解锁抑制标志：只要对应条件不满足，就不能解锁 */
#define APP_ARMING_INHIBIT_IMU_NOT_READY (1UL << 0U)                /* IMU 未准备好 */
#define APP_ARMING_INHIBIT_GYRO_NOT_CALIBRATED (1UL << 1U)          /* 陀螺仪未校准 */
#define APP_ARMING_INHIBIT_ACCEL_NOT_CALIBRATED (1UL << 2U)         /* 加速度计未校准 */
#define APP_ARMING_INHIBIT_PARAMETERS_INVALID (1UL << 3U)           /* 参数无效或未加载成功 */
#define APP_ARMING_INHIBIT_IMU_TIMING_INVALID (1UL << 4U)           /* IMU 采样时间或滤波状态无效 */
#define APP_ARMING_INHIBIT_ATTITUDE_NOT_READY (1UL << 5U)           /* 姿态解算结果未准备好 */
#define APP_ARMING_INHIBIT_RC_NOT_READY (1UL << 6U)                 /* RC 输入未准备好或处于失联保护 */
#define APP_ARMING_INHIBIT_BATTERY_NOT_READY (1UL << 7U)            /* 电池未准备好或处于严重低压 */

/* 飞行安全标志：用于判断输入是否安全以及是否需要保护动作 */
#define APP_FLIGHT_SAFETY_IMU_INVALID (1UL << 0U)                   /* IMU 或姿态数据无效 */
#define APP_FLIGHT_SAFETY_IMU_STALE (1UL << 1U)                     /* IMU 数据超时未更新 */
#define APP_FLIGHT_SAFETY_RC_INVALID (1UL << 2U)                    /* RC 通道无效或处于失联保护 */
#define APP_FLIGHT_SAFETY_RC_STALE (1UL << 3U)                      /* RC 通道数据超时未更新 */
#define APP_FLIGHT_SAFETY_ARMING_INHIBITED (1UL << 4U)              /* 存在解锁抑制条件 */
#define APP_FLIGHT_SAFETY_DSHOT_NOT_READY (1UL << 5U)               /* DShot 输出未准备好或存在故障 */
#define APP_FLIGHT_SAFETY_CONFIGURATOR_DISABLED (1UL << 6U)         /* Configurator 禁止解锁 */
#define APP_FLIGHT_SAFETY_SYSTEM_FAULT (1UL << 7U)                  /* 系统存在故障 */
#define APP_FLIGHT_SAFETY_CONTROL_INVALID (1UL << 8U)               /* 控制输出计算结果无效 */
#define APP_FLIGHT_SAFETY_THROTTLE_HIGH (1UL << 9U)                 /* 油门过高或油门输入无效 */
#define APP_FLIGHT_SAFETY_MOTOR_TEST_ACTIVE (1UL << 10U)            /* 电机测试正在进行 */
#define APP_FLIGHT_SAFETY_ARMING_ONLY_MASK \
    (APP_FLIGHT_SAFETY_THROTTLE_HIGH | \
     APP_FLIGHT_SAFETY_MOTOR_TEST_ACTIVE)                             /* 只阻止解锁、不触发飞行中 failsafe */
#define APP_STATE_MOTOR_COUNT 4U                                    /* 电机输出通道数量 */

typedef enum
{
    APP_GYRO_CALIBRATION_NOT_STARTED = 0,                     /* 尚未开始校准 */
    APP_GYRO_CALIBRATION_CALIBRATING,                         /* 正在采集数据并校准 */
    APP_GYRO_CALIBRATION_READY                                        /* 校准完成，结果可用 */
} app_gyro_calibration_state_t;

typedef enum
{
    APP_ACCEL_CALIBRATION_NOT_CALIBRATED = 0,                 /* 未校准或校准结果不可用 */
    APP_ACCEL_CALIBRATION_READY,                              /* 校准完成，结果可用 */
    APP_ACCEL_CALIBRATION_CALIBRATING,                        /* 正在采集数据并校准 */
    APP_ACCEL_CALIBRATION_CANDIDATE_READY,                    /* 新校准结果待确认或保存 */
    APP_ACCEL_CALIBRATION_SAVE_FAILED                                 /* 校准结果保存失败 */
} app_accel_calibration_state_t;

typedef enum
{
    APP_PARAMETER_LOAD_DEFAULTS_EMPTY = 0,                    /* Flash 无有效记录，使用默认参数 */
    APP_PARAMETER_LOAD_SLOT_A,                                /* 从 A 槽加载参数 */
    APP_PARAMETER_LOAD_SLOT_B,                                /* 从 B 槽加载参数 */
    APP_PARAMETER_LOAD_RECOVERED_SLOT_A,                      /* B 槽损坏时从 A 槽恢复参数 */
    APP_PARAMETER_LOAD_RECOVERED_SLOT_B,                      /* A 槽损坏时从 B 槽恢复参数 */
    APP_PARAMETER_LOAD_DEFAULTS_CORRUPT                               /* 记录损坏，使用默认参数 */
} app_parameter_load_result_t;

typedef enum
{
    APP_PARAMETER_SAVE_NOT_ATTEMPTED = 0,                     /* 尚未尝试保存 */
    APP_PARAMETER_SAVE_OK,                                    /* 参数保存成功 */
    APP_PARAMETER_SAVE_BAD_ARGUMENT,                          /* 保存参数无效 */
    APP_PARAMETER_SAVE_FLASH_UNLOCK_FAILED,                   /* Flash 解锁失败 */
    APP_PARAMETER_SAVE_ERASE_FAILED,                          /* Flash 擦除失败 */
    APP_PARAMETER_SAVE_PROGRAM_FAILED,                        /* Flash 写入失败 */
    APP_PARAMETER_SAVE_VERIFY_FAILED                                  /* 写入后校验失败 */
} app_parameter_save_result_t;

typedef enum
{
    APP_PARAMETER_SLOT_NONE = 0,                              /* 当前没有有效参数槽 */
    APP_PARAMETER_SLOT_A,                                     /* 当前使用 A 槽 */
    APP_PARAMETER_SLOT_B                                              /* 当前使用 B 槽 */
} app_parameter_slot_t;

typedef struct
{
    bool storage_valid;                                               /* Flash 中存在可用的参数记录 */
    app_parameter_load_result_t load_result;                          /* 最近一次参数加载结果 */
    app_parameter_save_result_t last_save_result;                     /* 最近一次参数保存结果 */
    app_parameter_slot_t active_slot;                                 /* 当前生效的参数存储槽 */
    uint8_t invalid_slot_mask;                                        /* 参数槽无效标志：bit0=A 槽，bit1=B 槽 */
    uint32_t sequence;                                                /* 当前参数记录的序号 */
    uint32_t save_error_count;                                        /* 参数保存失败的累计次数 */
    uint32_t last_hal_error;                                          /* 最近一次 HAL 错误码 */
    uint16_t loaded_record_version;                                   /* 已加载参数记录的版本号 */
    bool migration_pending;                                           /* 当前参数记录需要迁移到新版本 */
    parameter_store_values_t values;                                  /* 当前生效的具体参数值 */
} app_parameter_state_t;

typedef struct
{
    bool present;                                                     /* IMU 已检测到且初始化成功 */
    uint8_t who_am_i;                                                 /* IMU 芯片身份识别寄存器值 */
    uint8_t last_error;                                               /* 最近一次 IMU 操作错误码 */
    uint8_t last_bus_error;                                           /* 最近一次 IMU 总线错误码 */
    uint8_t gyro_config0;                                             /* IMU 陀螺仪配置寄存器回读值 */
    uint8_t accel_config0;                                            /* IMU 加速度计配置寄存器回读值 */
    uint8_t pwr_mgmt0;                                                /* IMU 电源管理寄存器回读值 */
    app_gyro_calibration_state_t gyro_calibration_state;              /* 陀螺仪校准状态 */
    uint16_t gyro_calibration_sample_count;                           /* 陀螺仪当前校准样本数 */
    uint32_t gyro_calibration_restart_count;                          /* 陀螺仪校准重新开始次数 */
    uint32_t gyro_calibration_motion_reject_count;                    /* 因运动被拒绝的样本次数 */
    uint32_t gyro_calibration_invalid_sample_count;                   /* 无效陀螺仪样本次数 */
    float gyro_bias_rad_s[APP_STATE_AXIS_COUNT];                      /* 陀螺仪零偏，单位：rad/s */
    app_accel_calibration_state_t accel_calibration_state;            /* 加速度计校准状态 */
    uint16_t accel_calibration_sample_count;                          /* 加速度计当前校准样本数 */
    uint32_t accel_calibration_restart_count;                         /* 加速度计校准重新开始次数 */
    uint32_t accel_calibration_motion_reject_count;                   /* 因运动被拒绝的样本次数 */
    uint32_t accel_calibration_level_reject_count;                    /* 因姿态水平度不符被拒绝的次数 */
    uint32_t accel_calibration_invalid_sample_count;                  /* 无效加速度计样本次数 */
    float accel_bias_m_s2[APP_STATE_AXIS_COUNT];                      /* 加速度计零偏，单位：m/s² */
    TickType_t sample_tick;                                           /* 最近一次 IMU 采样的 FreeRTOS 节拍 */
    bool timing_source_ready;                                         /* IMU 时间戳时钟源可用 */
    bool timing_valid;                                                /* 当前采样时间戳和采样间隔有效 */
    bool filter_ready;                                                /* IMU 滤波器已准备好并能输出有效结果 */
    uint32_t sample_timestamp_us;                                     /* 最近一次采样时间戳，单位：微秒 */
    uint32_t sample_interval_us;                                      /* 最近一次采样间隔，单位：微秒 */
    uint32_t sample_interval_min_us;                                  /* 运行期间最小采样间隔，单位：微秒 */
    uint32_t sample_interval_max_us;                                  /* 运行期间最大采样间隔，单位：微秒 */
    uint32_t timing_invalid_count;                                    /* 采样时间无效次数 */
    uint32_t timing_reset_count;                                      /* 时间处理链路重置次数 */
    uint32_t filter_reset_count;                                      /* IMU 滤波器重置次数 */
    uint32_t sample_count;                                            /* 有效采样累计次数 */
    uint32_t initialization_error_count;                              /* IMU 初始化失败次数 */
    uint32_t read_error_count;                                        /* IMU 读取失败次数 */
    uint32_t consecutive_error_count;                                 /* 当前连续读取错误次数 */
    uint32_t data_not_ready_count;                                    /* IMU 数据未准备好次数 */
    uint32_t missed_deadline_count;                                   /* IMU 任务错过周期截止时间次数 */
    uint32_t drdy_poll_count;                                         /* 轮询数据就绪状态的次数 */
    uint32_t dma_transfer_count;                                      /* DMA 传输次数 */
    uint32_t dma_start_error_count;                                   /* DMA 启动失败次数 */
    uint32_t dma_completion_error_count;                              /* DMA 完成状态异常次数 */
    uint32_t dma_timeout_count;                                       /* DMA 超时次数 */
    uint32_t dma_abort_count;                                         /* DMA 中止次数 */
    float acceleration_m_s2[APP_STATE_AXIS_COUNT];                    /* 原始三轴加速度，单位：m/s² */
    float angular_rate_rad_s[APP_STATE_AXIS_COUNT];                   /* 原始三轴角速度，单位：rad/s */
    float filtered_acceleration_m_s2[APP_STATE_AXIS_COUNT];           /* 滤波后三轴加速度，单位：m/s² */
    float filtered_angular_rate_rad_s[APP_STATE_AXIS_COUNT];          /* 滤波后三轴角速度，单位：rad/s */
    float temperature_c;                                              /* IMU 温度，单位：℃ */
} app_imu_sample_t;

typedef struct
{
    bool valid;                                                       /* 当前姿态解算结果有效，可供控制使用 */
    float quaternion[APP_STATE_QUATERNION_COUNT];                     /* 姿态四元数 */
    float roll_deg;                                                   /* 横滚角，单位：度 */
    float pitch_deg;                                                  /* 俯仰角，单位：度 */
    float yaw_deg;                                                    /* 航向角，单位：度 */
    uint32_t update_count;                                            /* 姿态更新累计次数 */
    uint32_t reset_count;                                             /* 姿态解算器重置次数 */
    uint32_t invalid_input_count;                                     /* 姿态输入无效次数 */
    uint32_t accel_rejection_count;                                   /* 加速度计修正被拒绝次数 */
    uint32_t gyro_only_update_count;                                  /* 仅使用陀螺仪更新的次数 */
} app_attitude_state_t;

typedef struct
{
    bool uart_running;                                                /* CRSF UART 已启动并正在运行 */
    bool channels_valid;                                              /* RC 通道数据有效，可供控制使用 */
    bool failsafe_active;                                             /* RC 信号当前处于失联保护状态 */
    bool link_statistics_valid;                                       /* 链路统计数据在有效期内 */
    rc_input_phase_t failsafe_phase;                                  /* RC 信号接收/恢复状态机阶段 */
    TickType_t last_channel_tick;                                     /* 最近一次有效 RC 通道帧时间 */
    TickType_t last_link_statistics_tick;                             /* 最近一次链路统计帧时间 */
    TickType_t recovery_started_tick;                                 /* 本次 RC 恢复开始时间 */
    TickType_t last_failsafe_tick;                                    /* 最近一次进入失联保护的时间 */
    uint32_t channel_sequence;                                        /* RC 通道帧序号 */
    uint32_t channel_frame_count;                                     /* 有效 RC 通道帧累计数 */
    uint32_t link_frame_count;                                        /* 有效链路统计帧累计数 */
    uint32_t unsupported_frame_count;                                 /* 不支持的帧类型累计数 */
    uint32_t payload_error_count;                                     /* 帧负载解码错误累计数 */
    uint32_t parser_valid_frame_count;                                /* 解析器确认有效的帧数 */
    uint32_t parser_crc_error_count;                                  /* 解析器 CRC 错误次数 */
    uint32_t parser_length_error_count;                               /* 解析器长度错误次数 */
    uint32_t parser_sync_drop_count;                                  /* 解析器丢弃不同步数据次数 */
    uint32_t uart_start_error_count;                                  /* UART 启动失败次数 */
    uint32_t uart_rx_event_count;                                     /* UART 接收事件次数 */
    uint32_t uart_idle_event_count;                                   /* UART 空闲事件次数 */
    uint32_t uart_ring_overflow_count;                                /* UART 环形缓冲区溢出次数 */
    uint32_t uart_error_count;                                        /* UART 错误累计次数 */
    uint32_t uart_recovery_count;                                     /* UART 恢复操作次数 */
    uint32_t last_uart_error;                                         /* 最近一次 UART 错误码 */
    uint32_t failsafe_count;                                          /* 进入 RC 失联保护的次数 */
    uint32_t failsafe_recovery_count;                                 /* RC 从失联保护恢复的次数 */
    uint16_t failsafe_recovery_frame_count;                           /* 本次恢复已收到的有效帧数 */
    rc_source_t active_source;                                        /* 当前生效的 RC 输入源 */
    rc_source_exit_reason_t source_last_exit_reason;                  /* 虚拟源最近一次退出原因 */
    bool source_authorization_active;                                 /* 授权通道当前处于有效授权状态 */
    bool source_reauthorization_ready;                                /* 虚拟源退出后已允许再次授权 */
    bool virtual_candidate_valid;                                     /* 虚拟 RC 候选数据有效且未超时 */
    TickType_t virtual_candidate_tick;                                /* 最近一次虚拟 RC 候选数据时间 */
    uint32_t virtual_source_sequence;                                 /* 虚拟 RC 候选的序号 */
    uint32_t virtual_heartbeat;                                       /* 虚拟 RC 候选的心跳值 */
    uint32_t virtual_session_generation;                              /* 虚拟 RC 会话代数 */
    uint32_t source_activation_count;                                 /* 虚拟 RC 源激活次数 */
    uint32_t source_exit_count;                                       /* RC 源退出虚拟控制次数 */
    TickType_t source_last_transition_tick;                           /* 最近一次 RC 源切换时间 */
    uint16_t channel_raw[APP_STATE_RC_CHANNEL_COUNT];                 /* RC 原始通道值 */
    uint16_t channel_us[APP_STATE_RC_CHANNEL_COUNT];                  /* RC 通道值，单位：微秒 */
    uint16_t physical_mapped_channel_us[APP_STATE_RC_CHANNEL_COUNT];  /* 物理 RC 映射值，单位：微秒 */
    uint16_t mapped_channel_us[APP_STATE_RC_CHANNEL_COUNT];           /* 最终映射通道值，单位：微秒 */
    int16_t uplink_rssi_dbm[2];                                       /* 上行 RSSI，单位：dBm */
    uint8_t uplink_link_quality;                                      /* 上行链路质量，单位：百分比 */
    int8_t uplink_snr_db;                                             /* 上行信噪比，单位：dB */
    uint8_t active_antenna;                                           /* 当前使用的接收天线编号 */
    uint8_t rf_mode;                                                  /* 当前射频模式 */
    uint8_t uplink_tx_power;                                          /* 上行发射功率等级 */
    int16_t downlink_rssi_dbm;                                        /* 下行 RSSI，单位：dBm */
    uint8_t downlink_link_quality;                                    /* 下行链路质量，单位：百分比 */
    int8_t downlink_snr_db;                                           /* 下行信噪比，单位：dB */
} app_rc_state_t;

/* Compact, atomic input boundary for RcTask source arbitration. */
typedef struct
{
    bool flight_armed;                                                /* 当前飞行器已解锁 */
    bool authorization_channel_available;                             /* 当前配置允许使用授权通道 */
    uint32_t arming_inhibit_flags;                                    /* 解锁抑制条件位掩码，非 0 表示不能解锁 */
} app_rc_source_context_t;

_Static_assert(sizeof(app_rc_source_context_t) <= 16U,
               "RcTask source context must remain stack-bounded");

typedef struct
{
    bool adc_running;                                                 /* 电池 ADC 采样正在运行 */
    bool present;                                                     /* 已检测到电池存在 */
    power_battery_state_t state;                                      /* 当前电池电压/电源状态 */
    uint8_t cell_count;                                               /* 检测到的电芯数量 */
    TickType_t last_sample_tick;                                      /* 最近一次 ADC 采样的 FreeRTOS 节拍 */
    uint16_t capacity_mah;                                            /* 电池标称容量，单位：mAh */
    uint16_t voltage_cv;                                              /* 电池总电压，单位：0.01 V */
    int16_t current_ca;                                               /* 电池电流，单位：0.01 A */
    uint16_t consumed_mah;                                            /* 已消耗电量，单位：mAh */
    uint16_t raw[POWER_MONITOR_ADC_CHANNEL_COUNT];                    /* ADC 原始采样值 */
    uint16_t filtered_raw[POWER_MONITOR_ADC_CHANNEL_COUNT];           /* ADC 滤波后采样值 */
    uint32_t sample_sequence;                                         /* ADC 采样序号 */
    uint32_t sample_count;                                            /* 电池监测有效采样累计数 */
    uint32_t invalid_sample_count;                                    /* 无效电池采样累计数 */
    uint32_t adc_start_count;                                         /* ADC 启动次数 */
    uint32_t adc_busy_count;                                          /* ADC 忙状态次数 */
    uint32_t adc_recovery_count;                                      /* ADC 恢复操作次数 */
    uint32_t adc_dma_error_count;                                     /* ADC DMA 错误次数 */
    uint32_t adc_overrun_count;                                       /* ADC 溢出次数 */
    uint32_t adc_last_dma_flags;                                      /* 最近一次 ADC DMA 状态标志 */
} app_battery_state_t;

typedef struct
{
    bool inputs_ready;                                                /* IMU、RC 等控制输入均通过安全检查 */
    bool motor_test_active;                                           /* 电机测试输出当前处于激活状态 */
    bool dshot_ready;                                                 /* DShot 输出接口已准备好 */
    bool dshot_busy;                                                  /* DShot 当前正在发送或等待 DMA 完成 */
    bool rate_pid_integrator_enabled;                                 /* Rate PID 积分项当前允许工作 */
    bool armed;                                                       /* 当前飞行器已解锁，可进行正常电机控制 */
    flight_arming_state_t arming_state;                               /* 解锁状态机当前状态 */
    uint32_t safety_flags;                                            /* 当前会触发飞行安全处理的标志掩码 */
    uint32_t arming_block_flags;                                      /* 当前阻止解锁的全部条件标志掩码 */
    uint32_t last_failsafe_flags;                                     /* 最近一次触发 failsafe 的标志掩码 */
    TickType_t last_update_tick;                                      /* 最近一次飞行控制循环的时间 */
    TickType_t last_motor_test_command_tick;                          /* 最近一次电机测试命令的时间 */
    uint16_t requested_motor_value[APP_STATE_MOTOR_COUNT];            /* 请求的各电机输出值 */
    uint16_t output_motor_value[APP_STATE_MOTOR_COUNT];               /* 实际提交的各电机输出值 */
    rc_setpoint_output_t rc_setpoint;                                 /* RC 通道计算出的控制目标 */
    angle_outer_loop_output_t angle_outer_loop;                       /* 姿态外环输出 */
    rate_pid_output_t rate_pid;                                       /* 角速度 PID 输出 */
    quad_x_mixer_output_t mixer;                                      /* 四旋翼混控输出 */
    uint32_t control_sample_count;                                    /* 当前控制使用的 IMU 样本序号 */
    uint32_t control_dt_us;                                           /* 当前控制周期，单位：微秒 */
    uint32_t loop_count;                                              /* 飞行控制循环累计次数 */
    uint32_t missed_deadline_count;                                   /* 飞行控制循环错过截止时间次数 */
    uint32_t imu_stale_count;                                         /* 检测到 IMU 数据过期的次数 */
    uint32_t rc_stale_count;                                          /* 检测到 RC 数据过期的次数 */
    uint32_t motor_test_timeout_count;                                /* 电机测试命令超时次数 */
    uint32_t dshot_submit_error_count;                                /* DShot 输出提交失败次数 */
    uint32_t dshot_dma_error_count;                                   /* DShot DMA 错误次数 */
    uint32_t rc_setpoint_update_count;                                /* RC 控制目标更新成功次数 */
    uint32_t rc_setpoint_error_count;                                 /* RC 控制目标计算失败次数 */
    uint32_t angle_outer_loop_update_count;                           /* 姿态外环更新成功次数 */
    uint32_t angle_outer_loop_error_count;                            /* 姿态外环计算失败次数 */
    uint32_t rate_pid_update_count;                                   /* 角速度 PID 更新成功次数 */
    uint32_t rate_pid_error_count;                                    /* 角速度 PID 计算失败次数 */
    uint32_t mixer_update_count;                                      /* 混控更新成功次数 */
    uint32_t mixer_error_count;                                       /* 混控计算失败次数 */
    uint32_t arm_count;                                               /* 成功解锁次数 */
    uint32_t disarm_count;                                            /* 进入锁定状态次数 */
    uint32_t flight_failsafe_count;                                   /* 飞行中进入 failsafe 的次数 */
} app_flight_state_t;

typedef struct
{
    uint32_t uptime_ms;                                               /* 系统运行时间，单位：毫秒 */
    uint16_t cycle_time_us;                                           /* 最近一次控制周期，单位：微秒 */
    uint16_t i2c_error_count;                                         /* I2C 错误累计次数 */
    uint16_t cpu_load_permille;                                       /* CPU 负载，单位：千分之一 */
    uint32_t fault_flags;                                             /* 系统故障标志掩码，非 0 表示存在故障 */
    uint32_t arming_inhibit_flags;                                    /* 解锁抑制标志掩码，非 0 表示不能解锁 */

    app_parameter_state_t parameters;                                 /* 当前生效的参数状态 */
    app_imu_sample_t imu;                                             /* 当前 IMU 采样和诊断状态 */
    app_attitude_state_t attitude;                                    /* 当前姿态解算状态 */
    app_rc_state_t rc;                                                /* 当前 RC 接收和源仲裁状态 */
    app_battery_state_t battery;                                      /* 当前电池监测状态 */
    app_flight_state_t flight;                                        /* 当前飞行控制状态 */
    uint16_t rssi;                                                    /* 当前 RC 信号强度 */

    bool configurator_arming_disabled;                                /* Configurator 是否禁止解锁 */
    uint32_t host_rtc_seconds;                                        /* 主机 RTC 秒值 */
    uint16_t host_rtc_millis;                                         /* 主机 RTC 当前秒内的毫秒值 */
} app_state_snapshot_t;

/** 初始化全局运行状态，并设置默认的解锁抑制标志 */
void app_state_init(void);

/**
 * 获取当前运行状态的完整快照
 * @param snapshot 用于接收快照的结构体指针，传入 NULL 时不执行操作
 */
void app_state_get_snapshot(app_state_snapshot_t *snapshot);

/**
 * 获取 RC 源仲裁所需的精简状态
 * @param context 用于接收 RC 源上下文的结构体指针，传入 NULL 时不执行操作
 */
void app_state_get_rc_source_context(app_rc_source_context_t *context);

/** 查询当前是否处于已解锁状态 */
bool app_state_is_armed(void);

/**
 * 更新系统运行时统计信息
 * @param cycle_time_us      控制周期，单位：微秒
 * @param cpu_load_permille  CPU 负载，单位：千分之一
 * @param i2c_error_count    I2C 错误计数
 */
void app_state_set_runtime(uint16_t cycle_time_us,
                           uint16_t cpu_load_permille,
                           uint16_t i2c_error_count);

/**
 * 更新系统故障标志
 * @param fault_flags 故障标志位掩码
 */
void app_state_set_fault_flags(uint32_t fault_flags);

/**
 * 发布参数存储状态
 * @param parameters 参数状态指针，传入 NULL 时不执行操作
 */
void app_state_publish_parameters(
    const app_parameter_state_t *parameters);

/**
 * 发布 IMU 采样及校准状态
 * @param sample IMU 状态指针，传入 NULL 时不执行操作
 */
void app_state_publish_imu(const app_imu_sample_t *sample);

/**
 * 发布姿态解算状态
 * @param attitude 姿态状态指针，传入 NULL 时不执行操作
 */
void app_state_publish_attitude(
    const app_attitude_state_t *attitude);

/**
 * 发布 RC 接收状态
 * @param rc RC 状态指针，传入 NULL 时不执行操作
 */
void app_state_publish_rc(const app_rc_state_t *rc);

/**
 * 发布电池监测状态
 * @param battery 电池状态指针，传入 NULL 时不执行操作
 */
void app_state_publish_battery(const app_battery_state_t *battery);

/**
 * 发布飞行控制状态
 * @param flight 飞行状态指针，传入 NULL 时不执行操作
 */
void app_state_publish_flight(const app_flight_state_t *flight);

/**
 * 设置 Configurator 是否禁止解锁
 * @param disabled true 表示禁止解锁，false 表示允许解锁
 */
void app_state_set_configurator_arming_disabled(bool disabled);

/**
 * 更新主机下发的 RTC 时间
 * @param seconds 主机 RTC 的秒值
 * @param millis  当前秒内的毫秒部分
 */
void app_state_set_host_rtc(uint32_t seconds, uint16_t millis);

#ifdef __cplusplus
}
#endif

#endif

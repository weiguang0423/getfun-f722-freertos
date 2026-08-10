/*
 * msp_server.c —— MSP 协议业务层（命令派发）实现
 *
 * 本文件实现 msp_server_process()：根据一帧 MSP 请求的 command，读取 app_state 快照，
 * 把对应数据按 Betaflight Configurator 期望的格式写入响应负载；不支持的命令标记为
 * supported=false。目标是把本板伪装成 Betaflight PID 控制器，让 Configurator 正常连接。
 *
 * 主要内容：
 *   - MSP 命令号宏（MSP_API_VERSION / MSP_FC_VARIANT / MSP_BOARD_INFO / MSP_STATUS 等）。
 *   - payload_writer_t + writer_u8/u16/u32/data/pstring：带容量检查的小端字节写入器，
 *       超出 MSP_MAX_PAYLOAD_SIZE 时置 overflow 标志，最终令该响应判为不支持。
 *   - 各 handle_* 静态函数：按命令逐字段填充响应。MSP_RAW_IMU 从 app_state 的
 *       m/s²、rad/s 转换成 Betaflight App 需要的 2048 LSB/g 和历史陀螺缩放，
 *       磁力计在本阶段固定为 0。
 *   - msp_server_process()：命令分支主体。关键策略——
 *       MSP_FC_VARIANT 回 "BTFL"（Configurator 只对 BTFL 开放正常 UI），
 *       而 MSP_BOARD_INFO 仍报 "GF72"/"GETFUN_F722" 保持真实板子身份。
 *   - 写类命令：MSP_SET_ARMING_DISABLED、MSP_SET_RTC从负载解析参数后写回
 *       app_state；标准MSP_ACC_CALIBRATION向ImuTask排队水平加速度校准。
 *   - GETFUN MSP2 0x4000：返回参数槽、保存结果、校准状态和偏置诊断。
 *   - GETFUN MSP2 0x4001：返回DWT时间、真实dt、低通状态和重置计数。
 *   - GETFUN MSP2 0x4002：返回Mahony READY、四元数、欧拉角和重置诊断。
 *   - Receiver页面：返回Serial RX/CRSF、AETR映射、16通道与只读RX配置；
 *       RC无效时MSP_RC输出安全值，MSP_STATUS_EX报告RXLOSS。
 *   - GETFUN MSP2 0x4003：返回RC年龄、Failsafe阶段/计数和映射通道。
 *   - Power页面：返回ADC电压/电流表、Battery State和只读标定配置；
 *       GETFUN MSP2 0x4004返回ADC3原始值、换算结果和DMA诊断。
 *   - S4.1～S4.3：GETFUN MSP2 0x4005返回Flight/DShot状态，0x4006接收四路
 *       有250 ms刷新期限的显式无桨测试值。
 *   - S4.4：GETFUN MSP2 0x4007返回归一化摇杆、Actual Rates设定值和AUX模式请求。
 *   - S4.5/S4.6：GETFUN MSP2 0x4008返回Rate PID和Quad-X Mixer诊断。
 *   - S4.7：GETFUN MSP2 0x4009返回ARM/Failsafe状态、原因和转换计数。
 *
 * 数据来源：所有只读数据来自 app_state_get_snapshot() 拿到的快照，本层不持有状态。
 */
#include "protocol/msp_server.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include "algorithms/imu_filter.h"
#include "algorithms/quad_x_mixer.h"
#include "algorithms/rate_pid.h"
#include "algorithms/rc_setpoint.h"
#include "app_state.h"
#include "rtos/flight_task.h"
#include "rtos/imu_task.h"
#include "stm32f7xx_hal.h"

#define MSP_API_VERSION 1U
#define MSP_FC_VARIANT 2U
#define MSP_FC_VERSION 3U
#define MSP_BOARD_INFO 4U
#define MSP_BUILD_INFO 5U
#define MSP_NAME 10U
#define MSP_BATTERY_CONFIG 32U
#define MSP_FEATURE_CONFIG 36U
#define MSP_CURRENT_METER_CONFIG 40U
#define MSP_MIXER_CONFIG 42U
#define MSP_RX_CONFIG 44U
#define MSP_RSSI_CONFIG 50U
#define MSP_VOLTAGE_METER_CONFIG 56U
#define MSP_RX_MAP 64U
#define MSP_FAILSAFE_CONFIG 75U
#define MSP_SET_ARMING_DISABLED 99U
#define MSP_STATUS 101U
#define MSP_RAW_IMU 102U
#define MSP_RC 105U
#define MSP_ATTITUDE 108U
#define MSP_ANALOG 110U
#define MSP_RC_TUNING 111U
#define MSP_BOXNAMES 116U
#define MSP_RC_DEADBAND 125U
#define MSP_VOLTAGE_METERS 128U
#define MSP_CURRENT_METERS 129U
#define MSP_BATTERY_STATE 130U
#define MSP_STATUS_EX 150U
#define MSP_UID 160U
#define MSP_ACC_CALIBRATION 205U
#define MSP_SET_RTC 246U

#define MSP2_GET_TEXT 0x3006U
#define MSP2_MCU_INFO 0x300CU
#define MSP2_GETFUN_CALIBRATION_STATUS 0x4000U
#define MSP2_GETFUN_IMU_FILTER_STATUS 0x4001U
#define MSP2_GETFUN_ATTITUDE_STATUS 0x4002U
#define MSP2_GETFUN_RC_STATUS 0x4003U
#define MSP2_GETFUN_POWER_STATUS 0x4004U
#define MSP2_GETFUN_FLIGHT_MOTOR_STATUS 0x4005U
#define MSP2_SET_GETFUN_MOTOR_TEST 0x4006U
#define MSP2_GETFUN_RC_SETPOINT_STATUS 0x4007U
#define MSP2_GETFUN_CONTROL_STATUS 0x4008U
#define MSP2_GETFUN_ARMING_STATUS 0x4009U

#define MSP2TEXT_PILOT_NAME 1U
#define MSP2TEXT_CRAFT_NAME 2U
#define MSP2TEXT_BUILD_KEY 5U

#define MSP_PROTOCOL_VERSION 0U
#define API_VERSION_MAJOR 1U
#define API_VERSION_MINOR 48U

#define SENSOR_ACC (1U << 0U)
#define SENSOR_GYRO (1U << 5U)
#define FEATURE_RX_SERIAL (1UL << 3U)
#define BETAFLIGHT_ARMING_DISABLED_RX_FAILSAFE (1UL << 2U)
#define BETAFLIGHT_ARMING_DISABLED_MSP (1UL << 16U)
#define BETAFLIGHT_ARMING_DISABLE_FLAGS_COUNT 30U

#define TARGET_HAS_VCP (1U << 0U)
#define MCU_TYPE_ID_PROVIDED_BY_NAME 255U

#define STANDARD_GRAVITY_M_S2 9.80665f
#define ACCEL_MSP_COUNTS_PER_G 2048.0f
#define GYRO_SENSOR_COUNTS_PER_DPS 16.4f
#define GYRO_CONFIGURATOR_SCALE_DIVISOR 4.0f

#define GETFUN_IMU_TIME_SOURCE_READY (1U << 0U)
#define GETFUN_IMU_TIMING_VALID (1U << 1U)
#define GETFUN_IMU_FILTER_READY (1U << 2U)
#define GETFUN_ATTITUDE_READY (1U << 0U)
#define RADIANS_TO_DEGREES 57.29577951308232088f
#define BETAFLIGHT_SERIALRX_CRSF 9U
#define BETAFLIGHT_RATE_LIMIT_DPS 1998U
#define BETAFLIGHT_VOLTAGE_METER_ADC 1U
#define BETAFLIGHT_CURRENT_METER_ADC 1U
#define BETAFLIGHT_BATTERY_METER_ID 10U
#define BETAFLIGHT_ADC_SENSOR_TYPE 0U

typedef struct
{
    uint8_t *data;
    uint16_t capacity;
    uint16_t length;
    bool overflow;
} payload_writer_t;

static void writer_u8(payload_writer_t *writer, uint8_t value)
{
    if (writer->length >= writer->capacity) {
        writer->overflow = true;
        return;
    }
    writer->data[writer->length++] = value;
}

static void writer_u16(payload_writer_t *writer, uint16_t value)
{
    writer_u8(writer, (uint8_t)value);
    writer_u8(writer, (uint8_t)(value >> 8U));
}

static void writer_u32(payload_writer_t *writer, uint32_t value)
{
    writer_u16(writer, (uint16_t)value);
    writer_u16(writer, (uint16_t)(value >> 16U));
}

static void writer_i32(payload_writer_t *writer, int32_t value)
{
    writer_u32(writer, (uint32_t)value);
}

static void writer_i16(payload_writer_t *writer, int16_t value)
{
    writer_u16(writer, (uint16_t)value);
}

static int16_t scaled_float_to_i16(float value, float scale)
{
    const float scaled = value * scale;

    if (!isfinite(scaled)) {
        return 0;
    }
    if (scaled >= (float)INT16_MAX) {
        return INT16_MAX;
    }
    if (scaled <= (float)INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)lroundf(scaled);
}

static int32_t scaled_float_to_i32(float value, float scale)
{
    const float scaled = value * scale;

    if (!isfinite(scaled)) {
        return 0;
    }
    if (scaled >= (float)INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled <= (float)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)lroundf(scaled);
}

static void writer_data(payload_writer_t *writer,
                        const void *data,
                        uint16_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint16_t index;

    for (index = 0U; index < length; ++index) {
        writer_u8(writer, bytes[index]);
    }
}

static void writer_pstring(payload_writer_t *writer, const char *text)
{
    size_t length = strlen(text);

    if (length > 255U) {
        length = 255U;
    }
    writer_u8(writer, (uint8_t)length);
    writer_data(writer, text, (uint16_t)length);
}

static uint16_t active_sensor_mask(const app_state_snapshot_t *state)
{
    uint16_t sensors = 0U;

    if (state->imu.present) {
        sensors |= SENSOR_ACC | SENSOR_GYRO;
    }
    return sensors;
}

static void write_status_base(payload_writer_t *writer,
                              const app_state_snapshot_t *state)
{
    writer_u16(writer, state->cycle_time_us);
    writer_u16(writer, state->i2c_error_count);
    writer_u16(writer, active_sensor_mask(state));
    writer_u32(writer, state->flight.armed ? 1UL : 0UL);
    writer_u8(writer, 0U);
}

static void handle_board_info(payload_writer_t *writer)
{
    static const uint8_t empty_signature[32] = {0};

    writer_data(writer, "GF72", 4U);
    writer_u16(writer, 0U);
    writer_u8(writer, 0U);
    writer_u8(writer, TARGET_HAS_VCP);
    writer_pstring(writer, "GETFUN_F722");
    writer_pstring(writer, "GETFUN F722 V3");
    writer_pstring(writer, "GETFUN");
    writer_data(writer, empty_signature, sizeof(empty_signature));
    writer_u8(writer, MCU_TYPE_ID_PROVIDED_BY_NAME);
    writer_u8(writer, 0U);
    writer_u16(writer, 0U);
    writer_u32(writer, 0U);
}

static void handle_build_info(payload_writer_t *writer)
{
    static const char git_revision[7] = "local  ";

    writer_data(writer, __DATE__, 11U);
    writer_data(writer, __TIME__, 8U);
    writer_data(writer, git_revision, sizeof(git_revision));
    writer_u16(writer, 0U);
}

static void handle_status_ex(payload_writer_t *writer,
                             const app_state_snapshot_t *state)
{
    uint32_t arming_disable_flags = 0U;

    if (!state->rc.channels_valid || state->rc.failsafe_active) {
        arming_disable_flags |=
            BETAFLIGHT_ARMING_DISABLED_RX_FAILSAFE;
    }
    if (state->configurator_arming_disabled) {
        arming_disable_flags |= BETAFLIGHT_ARMING_DISABLED_MSP;
    }

    write_status_base(writer, state);
    writer_u16(writer, state->cpu_load_permille);
    writer_u8(writer, 1U);
    writer_u8(writer, 0U);
    writer_u8(writer, 0U);
    writer_u8(writer, BETAFLIGHT_ARMING_DISABLE_FLAGS_COUNT);
    writer_u32(writer, arming_disable_flags);
    writer_u8(writer, 0U);
    writer_u16(writer, 0U);
    writer_u8(writer, 1U);
    writer_u8(writer, 1U);
    writer_u8(writer, 0U);
}

static void handle_raw_imu(payload_writer_t *writer,
                           const app_state_snapshot_t *state)
{
    uint8_t axis;

    if (!state->imu.present) {
        for (axis = 0U; axis < (APP_STATE_AXIS_COUNT * 3U); ++axis) {
            writer_u16(writer, 0U);
        }
        return;
    }

    for (axis = 0U; axis < APP_STATE_AXIS_COUNT; ++axis) {
        const float wire_value =
            (state->imu.acceleration_m_s2[axis] /
             STANDARD_GRAVITY_M_S2) *
            ACCEL_MSP_COUNTS_PER_G;
        int16_t value;

        if (!isfinite(wire_value)) {
            value = 0;
        } else if (wire_value >= (float)INT16_MAX) {
            value = INT16_MAX;
        } else if (wire_value <= (float)INT16_MIN) {
            value = INT16_MIN;
        } else {
            value = (int16_t)lroundf(wire_value);
        }
        writer_u16(writer, (uint16_t)value);
    }
    for (axis = 0U; axis < APP_STATE_AXIS_COUNT; ++axis) {
        const float wire_value =
            state->imu.angular_rate_rad_s[axis] *
            RADIANS_TO_DEGREES *
            GYRO_SENSOR_COUNTS_PER_DPS /
            GYRO_CONFIGURATOR_SCALE_DIVISOR;
        int16_t value;

        if (!isfinite(wire_value)) {
            value = 0;
        } else if (wire_value >= (float)INT16_MAX) {
            value = INT16_MAX;
        } else if (wire_value <= (float)INT16_MIN) {
            value = INT16_MIN;
        } else {
            value = (int16_t)lroundf(wire_value);
        }
        writer_u16(writer, (uint16_t)value);
    }
    for (axis = 0U; axis < APP_STATE_AXIS_COUNT; ++axis) {
        writer_u16(writer, 0U);
    }
}

static void handle_attitude(payload_writer_t *writer,
                            const app_state_snapshot_t *state)
{
    int16_t roll_deg10 = 0;
    int16_t pitch_deg10 = 0;
    int16_t yaw_deg = 0;

    if (state->attitude.valid) {
        roll_deg10 =
            scaled_float_to_i16(state->attitude.roll_deg, 10.0f);
        pitch_deg10 =
            scaled_float_to_i16(state->attitude.pitch_deg, 10.0f);
        if (isfinite(state->attitude.yaw_deg) &&
            (state->attitude.yaw_deg >= 0.0f) &&
            (state->attitude.yaw_deg < 360.0f)) {
            yaw_deg = (int16_t)state->attitude.yaw_deg;
        }
    }

    writer_u16(writer, (uint16_t)roll_deg10);
    writer_u16(writer, (uint16_t)pitch_deg10);
    writer_u16(writer, (uint16_t)yaw_deg);
}

static void handle_rc(payload_writer_t *writer,
                      const app_state_snapshot_t *state)
{
    uint16_t safe_channels[APP_STATE_RC_CHANNEL_COUNT];
    const uint16_t *channels = state->rc.mapped_channel_us;
    uint8_t channel;

    if (!state->rc.channels_valid || state->rc.failsafe_active) {
        rc_input_set_safe_channels(safe_channels);
        channels = safe_channels;
    }

    for (channel = 0U;
         channel < APP_STATE_RC_CHANNEL_COUNT;
         ++channel) {
        writer_u16(writer, channels[channel]);
    }
}

static void handle_rc_tuning(payload_writer_t *writer)
{
    const rc_setpoint_profile_t *profile =
        rc_setpoint_default_profile();

    writer_u8(writer, profile->actual_center_sensitivity[0]);
    writer_u8(writer, profile->expo_percent[0]);
    writer_u8(writer, profile->actual_max_rate[0]);
    writer_u8(writer, profile->actual_max_rate[1]);
    writer_u8(writer, profile->actual_max_rate[2]);
    writer_u8(writer, 0U);
    writer_u8(writer, 50U);
    writer_u8(writer, 0U);
    writer_u16(writer, 0U);
    writer_u8(writer, profile->expo_percent[2]);
    writer_u8(writer, profile->actual_center_sensitivity[2]);
    writer_u8(writer, profile->actual_center_sensitivity[1]);
    writer_u8(writer, profile->expo_percent[1]);
    writer_u8(writer, 0U);
    writer_u8(writer, 100U);
    writer_u16(writer, BETAFLIGHT_RATE_LIMIT_DPS);
    writer_u16(writer, BETAFLIGHT_RATE_LIMIT_DPS);
    writer_u16(writer, BETAFLIGHT_RATE_LIMIT_DPS);
    writer_u8(writer, RC_SETPOINT_RATE_TYPE_ACTUAL);
    writer_u8(writer, 50U);
}

static void handle_rx_config(payload_writer_t *writer)
{
    const rc_setpoint_profile_t *profile =
        rc_setpoint_default_profile();

    writer_u8(writer, BETAFLIGHT_SERIALRX_CRSF);
    writer_u16(writer, 1900U);
    writer_u16(writer, profile->input_mid_us);
    writer_u16(writer, 1050U);
    writer_u8(writer, 0U);
    writer_u16(writer, profile->input_min_us);
    writer_u16(writer, profile->input_max_us);
    writer_u8(writer, 0U);
    writer_u8(writer, 0U);
    writer_u16(writer, 1250U);
    writer_u8(writer, 0U);
    writer_u32(writer, 0U);
    writer_u8(writer, 0U);
    writer_u8(writer, 0U);
    writer_u8(writer, 0U);
    writer_u8(writer, 0U);
    writer_u8(writer, 0U);
    writer_u8(writer, 0U);
    writer_u8(writer, 30U);
    writer_u8(writer, 0U);
    writer_u8(writer, 0U);
    writer_u8(writer, 30U);
    writer_u8(writer, 1U);
    writer_u32(writer, 0U);
    writer_u16(writer, 0U);
    writer_u8(writer, 0U);
}

static void handle_getfun_rc_status(
    payload_writer_t *writer,
    const app_state_snapshot_t *state)
{
    uint8_t flags = 0U;
    uint32_t age_ms = UINT32_MAX;
    uint8_t channel;

    if (state->rc.channel_frame_count != 0U) {
        flags |= (1U << 0U);
        age_ms = state->uptime_ms -
                 (uint32_t)state->rc.last_channel_tick;
    }
    if (state->rc.channels_valid) {
        flags |= (1U << 1U);
    }
    if (state->rc.failsafe_active) {
        flags |= (1U << 2U);
    }
    if (state->rc.failsafe_phase == RC_INPUT_PHASE_RECOVERING) {
        flags |= (1U << 3U);
    }
    if (state->rc.link_statistics_valid) {
        flags |= (1U << 4U);
    }

    writer_u8(writer, 1U);
    writer_u8(writer, flags);
    writer_u8(writer, (uint8_t)state->rc.failsafe_phase);
    writer_u8(writer, 0U);
    writer_u16(writer, RC_INPUT_TIMEOUT_MS);
    writer_u16(writer, RC_INPUT_RECOVERY_MS);
    writer_u16(writer, RC_INPUT_RECOVERY_MIN_FRAMES);
    writer_u16(writer, 0U);
    writer_u32(writer, state->rc.channel_sequence);
    writer_u32(writer, age_ms);
    writer_u32(writer, state->rc.failsafe_count);
    writer_u32(writer, state->rc.failsafe_recovery_count);
    writer_u16(writer, state->rc.failsafe_recovery_frame_count);
    writer_u16(writer, 0U);

    for (channel = 0U;
         channel < APP_STATE_RC_CHANNEL_COUNT;
         ++channel) {
        writer_u16(writer, state->rc.mapped_channel_us[channel]);
    }
}

static void handle_analog(payload_writer_t *writer,
                          const app_state_snapshot_t *state)
{
    const bool present = state->battery.present;
    const uint16_t voltage = present ? state->battery.voltage_cv : 0U;

    writer_u8(writer, (uint8_t)((voltage / 10U) > 255U
                                    ? 255U
                                    : (voltage / 10U)));
    writer_u16(writer, present
                           ? state->battery.consumed_mah
                           : 0U);
    writer_u16(writer, state->rssi);
    writer_u16(writer, present
                           ? (uint16_t)state->battery.current_ca
                           : 0U);
    writer_u16(writer, voltage);
}

static void handle_battery_state(payload_writer_t *writer,
                                 const app_state_snapshot_t *state)
{
    const bool present = state->battery.present;
    const uint16_t voltage = present ? state->battery.voltage_cv : 0U;

    writer_u8(writer, present ? state->battery.cell_count : 0U);
    writer_u16(writer, present ? state->battery.capacity_mah : 0U);
    writer_u8(writer, (uint8_t)((voltage / 10U) > 255U
                                    ? 255U
                                    : (voltage / 10U)));
    writer_u16(writer, present ? state->battery.consumed_mah : 0U);
    writer_u16(writer, present ? (uint16_t)state->battery.current_ca : 0U);
    writer_u8(writer, (uint8_t)state->battery.state);
    writer_u16(writer, voltage);
}

static void handle_voltage_meters(payload_writer_t *writer,
                                  const app_state_snapshot_t *state)
{
    const uint16_t voltage = state->battery.present
                                 ? state->battery.voltage_cv
                                 : 0U;

    writer_u8(writer, BETAFLIGHT_BATTERY_METER_ID);
    writer_u8(writer, (uint8_t)((voltage / 10U) > 255U
                                    ? 255U
                                    : (voltage / 10U)));
}

static void handle_current_meters(payload_writer_t *writer,
                                  const app_state_snapshot_t *state)
{
    int32_t current_ma = state->battery.present
                             ? (int32_t)state->battery.current_ca * 10
                             : 0;

    if (current_ma < 0) {
        current_ma = 0;
    } else if (current_ma > UINT16_MAX) {
        current_ma = UINT16_MAX;
    }

    writer_u8(writer, BETAFLIGHT_BATTERY_METER_ID);
    writer_u16(writer, state->battery.present
                           ? state->battery.consumed_mah
                           : 0U);
    writer_u16(writer, (uint16_t)current_ma);
}

static void handle_voltage_meter_config(payload_writer_t *writer)
{
    writer_u8(writer, 1U);
    writer_u8(writer, 5U);
    writer_u8(writer, BETAFLIGHT_BATTERY_METER_ID);
    writer_u8(writer, BETAFLIGHT_ADC_SENSOR_TYPE);
    writer_u8(writer, POWER_MONITOR_VOLTAGE_SCALE);
    writer_u8(writer, POWER_MONITOR_VOLTAGE_DIVIDER);
    writer_u8(writer, POWER_MONITOR_VOLTAGE_MULTIPLIER);
}

static void handle_current_meter_config(payload_writer_t *writer)
{
    writer_u8(writer, 1U);
    writer_u8(writer, 6U);
    writer_u8(writer, BETAFLIGHT_BATTERY_METER_ID);
    writer_u8(writer, BETAFLIGHT_ADC_SENSOR_TYPE);
    writer_u16(writer, POWER_MONITOR_CURRENT_SCALE);
    writer_u16(writer, (uint16_t)POWER_MONITOR_CURRENT_OFFSET_MA);
}

static void handle_battery_config(payload_writer_t *writer)
{
    writer_u8(writer,
              (POWER_MONITOR_CRITICAL_CELL_CV + 5U) / 10U);
    writer_u8(writer,
              (POWER_MONITOR_MAX_CELL_CV + 5U) / 10U);
    writer_u8(writer,
              (POWER_MONITOR_WARNING_CELL_CV + 5U) / 10U);
    writer_u16(writer, 0U);
    writer_u8(writer, BETAFLIGHT_VOLTAGE_METER_ADC);
    writer_u8(writer, BETAFLIGHT_CURRENT_METER_ADC);
    writer_u16(writer, POWER_MONITOR_CRITICAL_CELL_CV);
    writer_u16(writer, POWER_MONITOR_MAX_CELL_CV);
    writer_u16(writer, POWER_MONITOR_WARNING_CELL_CV);
}

static void handle_getfun_power_status(
    payload_writer_t *writer,
    const app_state_snapshot_t *state)
{
    uint8_t flags = 0U;
    uint8_t channel;

    if (state->battery.adc_running) {
        flags |= (1U << 0U);
    }
    if (state->battery.present) {
        flags |= (1U << 1U);
    }
    if (state->battery.state == POWER_BATTERY_WARNING) {
        flags |= (1U << 2U);
    }
    if (state->battery.state == POWER_BATTERY_CRITICAL) {
        flags |= (1U << 3U);
    }

    writer_u8(writer, 1U);
    writer_u8(writer, flags);
    writer_u8(writer, (uint8_t)state->battery.state);
    writer_u8(writer, state->battery.cell_count);
    writer_u16(writer, state->battery.voltage_cv);
    writer_u16(writer, (uint16_t)state->battery.current_ca);
    writer_u16(writer, state->battery.consumed_mah);
    writer_u16(writer, state->battery.capacity_mah);
    for (channel = 0U;
         channel < POWER_MONITOR_ADC_CHANNEL_COUNT;
         ++channel) {
        writer_u16(writer, state->battery.raw[channel]);
    }
    for (channel = 0U;
         channel < POWER_MONITOR_ADC_CHANNEL_COUNT;
         ++channel) {
        writer_u16(writer, state->battery.filtered_raw[channel]);
    }
    writer_u32(writer, state->battery.sample_sequence);
    writer_u32(writer, state->battery.sample_count);
    writer_u32(writer, state->battery.invalid_sample_count);
    writer_u32(writer, state->battery.adc_start_count);
    writer_u32(writer, state->battery.adc_busy_count);
    writer_u32(writer, state->battery.adc_recovery_count);
    writer_u32(writer, state->battery.adc_dma_error_count);
    writer_u32(writer, state->battery.adc_overrun_count);
    writer_u32(writer, state->battery.adc_last_dma_flags);
}

static void handle_getfun_flight_motor_status(
    payload_writer_t *writer,
    const app_state_snapshot_t *state)
{
    uint32_t motor;

    writer_u8(writer, state->flight.dshot_ready ? 1U : 0U);
    writer_u8(writer, state->flight.inputs_ready ? 1U : 0U);
    writer_u8(writer, state->flight.motor_test_active ? 1U : 0U);
    writer_u8(writer, state->flight.dshot_busy ? 1U : 0U);
    writer_u32(writer, state->flight.safety_flags);
    writer_u32(writer, state->flight.loop_count);
    writer_u32(writer, state->flight.missed_deadline_count);
    writer_u32(writer, state->flight.imu_stale_count);
    writer_u32(writer, state->flight.rc_stale_count);
    writer_u32(writer, state->flight.motor_test_timeout_count);
    writer_u32(writer, state->flight.dshot_submit_error_count);
    writer_u32(writer, state->flight.dshot_dma_error_count);
    writer_u32(writer, state->flight.last_update_tick);
    writer_u32(writer, state->flight.last_motor_test_command_tick);
    for (motor = 0U; motor < APP_STATE_MOTOR_COUNT; ++motor) {
        writer_u16(writer, state->flight.requested_motor_value[motor]);
    }
    for (motor = 0U; motor < APP_STATE_MOTOR_COUNT; ++motor) {
        writer_u16(writer, state->flight.output_motor_value[motor]);
    }
    writer_u32(writer, flight_task_stack_high_water_mark());
}

static void handle_getfun_rc_setpoint_status(
    payload_writer_t *writer,
    const app_state_snapshot_t *state)
{
    const rc_setpoint_profile_t *profile =
        rc_setpoint_default_profile();
    uint8_t flags = 0U;
    uint32_t axis;
    uint32_t age_ms = UINT32_MAX;

    if (state->flight.rc_setpoint.valid) {
        flags |= (1U << 0U);
    }
    if (state->flight.rc_setpoint.arm_requested) {
        flags |= (1U << 1U);
    }
    if (state->flight.rc_setpoint.mode == RC_SETPOINT_MODE_ANGLE) {
        flags |= (1U << 2U);
    }
    if (state->rc.channel_frame_count != 0U) {
        age_ms = state->uptime_ms -
                 (uint32_t)state->rc.last_channel_tick;
    }

    writer_u8(writer, 1U);
    writer_u8(writer, flags);
    writer_u8(writer, (uint8_t)state->flight.rc_setpoint.mode);
    writer_u8(writer, RC_SETPOINT_RATE_TYPE_ACTUAL);
    writer_u16(writer, profile->input_min_us);
    writer_u16(writer, profile->input_mid_us);
    writer_u16(writer, profile->input_max_us);
    writer_u8(writer, profile->deadband_us);
    writer_u8(writer, profile->yaw_deadband_us);
    for (axis = 0U; axis < RC_SETPOINT_AXIS_COUNT; ++axis) {
        writer_u8(writer, profile->actual_center_sensitivity[axis]);
    }
    for (axis = 0U; axis < RC_SETPOINT_AXIS_COUNT; ++axis) {
        writer_u8(writer, profile->actual_max_rate[axis]);
    }
    for (axis = 0U; axis < RC_SETPOINT_AXIS_COUNT; ++axis) {
        writer_u8(writer, profile->expo_percent[axis]);
    }
    writer_u8(writer, profile->arm_aux_channel);
    writer_u8(writer, profile->angle_aux_channel);
    writer_u8(writer, 0U);
    writer_u16(writer, profile->arm_range_min_us);
    writer_u16(writer, profile->arm_range_max_us);
    writer_u16(writer, profile->angle_range_min_us);
    writer_u16(writer, profile->angle_range_max_us);
    for (axis = 0U; axis < RC_SETPOINT_AXIS_COUNT; ++axis) {
        writer_i16(writer,
                   scaled_float_to_i16(
                       state->flight.rc_setpoint.normalized_stick[axis],
                       1000.0f));
    }
    writer_u16(writer,
               (uint16_t)scaled_float_to_i16(
                   state->flight.rc_setpoint.throttle, 1000.0f));
    for (axis = 0U; axis < RC_SETPOINT_AXIS_COUNT; ++axis) {
        writer_i16(writer,
                   scaled_float_to_i16(
                       state->flight.rc_setpoint.rate_dps[axis],
                       10.0f));
    }
    writer_u32(writer, state->flight.rc_setpoint_update_count);
    writer_u32(writer, state->flight.rc_setpoint_error_count);
    writer_u32(writer, state->rc.channel_sequence);
    writer_u32(writer, age_ms);
    writer_u16(writer, 0U);
}

static void handle_getfun_control_status(
    payload_writer_t *writer,
    const app_state_snapshot_t *state)
{
    const rate_pid_profile_t *profile = rate_pid_default_profile();
    uint8_t flags = 0U;
    uint32_t axis;
    uint32_t motor;

    if (state->flight.rate_pid.valid) {
        flags |= (1U << 0U);
    }
    if (state->flight.mixer.valid) {
        flags |= (1U << 1U);
    }
    if (state->flight.mixer.saturated) {
        flags |= (1U << 2U);
    }
    if (state->flight.rate_pid_integrator_enabled) {
        flags |= (1U << 3U);
    }
    if (state->flight.rc_setpoint.mode == RC_SETPOINT_MODE_RATE) {
        flags |= (1U << 4U);
    }

    writer_u8(writer, 1U);
    writer_u8(writer, flags);
    writer_u8(writer, state->flight.rate_pid.saturated_mask);
    writer_u8(writer, QUAD_X_MIXER_MOTOR_COUNT);
    writer_u32(writer, state->flight.control_dt_us);
    writer_u32(writer, state->flight.control_sample_count);
    writer_u32(writer, state->flight.rate_pid_update_count);
    writer_u32(writer, state->flight.rate_pid_error_count);
    writer_u32(writer, state->flight.mixer_update_count);
    writer_u32(writer, state->flight.mixer_error_count);

    for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
        writer_i16(writer,
                   scaled_float_to_i16(profile->kp[axis], 10000.0f));
    }
    for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
        writer_i16(writer,
                   scaled_float_to_i16(profile->ki[axis], 10000.0f));
    }
    for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
        writer_i16(writer,
                   scaled_float_to_i16(profile->kd[axis], 10000.0f));
    }
    writer_i16(writer,
               scaled_float_to_i16(profile->integral_limit, 10000.0f));
    writer_i16(writer,
               scaled_float_to_i16(profile->output_limit, 10000.0f));

    for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
        writer_i16(writer,
                   scaled_float_to_i16(
                       state->flight.rate_pid.setpoint_rad_s[axis],
                       1000.0f));
    }
    for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
        writer_i16(writer,
                   scaled_float_to_i16(
                       state->flight.rate_pid.measurement_rad_s[axis],
                       1000.0f));
    }
    for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
        writer_i16(writer,
                   scaled_float_to_i16(
                       state->flight.rate_pid.error_rad_s[axis],
                       1000.0f));
    }
    for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
        writer_i16(writer,
                   scaled_float_to_i16(state->flight.rate_pid.p[axis],
                                       10000.0f));
    }
    for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
        writer_i16(writer,
                   scaled_float_to_i16(state->flight.rate_pid.i[axis],
                                       10000.0f));
    }
    for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
        writer_i16(writer,
                   scaled_float_to_i16(state->flight.rate_pid.d[axis],
                                       10000.0f));
    }
    for (axis = 0U; axis < RATE_PID_AXIS_COUNT; ++axis) {
        writer_i16(writer,
                   scaled_float_to_i16(
                       state->flight.rate_pid.correction[axis],
                       10000.0f));
    }
    writer_i16(writer,
               scaled_float_to_i16(
                   state->flight.mixer.requested_throttle,
                   10000.0f));
    writer_i16(writer,
               scaled_float_to_i16(
                   state->flight.mixer.applied_throttle,
                   10000.0f));
    writer_i16(writer,
               scaled_float_to_i16(
                   state->flight.mixer.correction_scale,
                   10000.0f));
    for (motor = 0U; motor < QUAD_X_MIXER_MOTOR_COUNT; ++motor) {
        writer_i16(writer,
                   scaled_float_to_i16(state->flight.mixer.motor[motor],
                                       10000.0f));
    }
}

static void handle_getfun_arming_status(
    payload_writer_t *writer,
    const app_state_snapshot_t *state)
{
    uint8_t flags = 0U;
    uint32_t motor;

    if (state->flight.armed) {
        flags |= (1U << 0U);
    }
    if (state->flight.rc_setpoint.arm_requested) {
        flags |= (1U << 1U);
    }
    if (state->flight.inputs_ready) {
        flags |= (1U << 2U);
    }
    if (state->flight.motor_test_active) {
        flags |= (1U << 3U);
    }

    writer_u8(writer, 1U);
    writer_u8(writer, (uint8_t)state->flight.arming_state);
    writer_u8(writer, flags);
    writer_u8(writer, state->flight.dshot_ready ? 1U : 0U);
    writer_u32(writer, state->arming_inhibit_flags);
    writer_u32(writer, state->flight.safety_flags);
    writer_u32(writer, state->flight.arming_block_flags);
    writer_u32(writer, state->flight.last_failsafe_flags);
    writer_u32(writer, state->flight.arm_count);
    writer_u32(writer, state->flight.disarm_count);
    writer_u32(writer, state->flight.flight_failsafe_count);
    writer_u16(writer,
               (uint16_t)scaled_float_to_i16(
                   state->flight.rc_setpoint.throttle, 1000.0f));
    for (motor = 0U; motor < APP_STATE_MOTOR_COUNT; ++motor) {
        writer_u16(writer, state->flight.output_motor_value[motor]);
    }
}

static void handle_get_text(const msp_request_t *request,
                            payload_writer_t *writer)
{
    const uint8_t text_type =
        request->payload_length > 0U ? request->payload[0] : 0U;

    writer_u8(writer, text_type);
    switch (text_type) {
    case MSP2TEXT_CRAFT_NAME:
        writer_pstring(writer, "GETFUN F722");
        break;
    case MSP2TEXT_PILOT_NAME:
    case MSP2TEXT_BUILD_KEY:
    default:
        writer_pstring(writer, "");
        break;
    }
}

static void handle_getfun_calibration_status(
    payload_writer_t *writer,
    const app_state_snapshot_t *state)
{
    uint32_t axis;

    writer_u8(writer, 1U);
    writer_u8(writer, state->parameters.storage_valid ? 1U : 0U);
    writer_u8(writer, (uint8_t)state->parameters.load_result);
    writer_u8(writer, (uint8_t)state->parameters.last_save_result);
    writer_u8(writer, (uint8_t)state->parameters.active_slot);
    writer_u8(writer, state->parameters.invalid_slot_mask);
    writer_u8(writer, (uint8_t)state->imu.accel_calibration_state);
    writer_u8(writer, 0U);
    writer_u32(writer, state->parameters.sequence);
    writer_u32(writer, state->parameters.save_error_count);
    writer_u32(writer, state->parameters.last_hal_error);
    writer_u32(writer, state->arming_inhibit_flags);
    writer_u16(writer, state->imu.accel_calibration_sample_count);
    writer_u16(writer, 0U);
    writer_u32(writer, state->imu.accel_calibration_restart_count);
    writer_u32(writer,
               state->imu.accel_calibration_motion_reject_count);
    writer_u32(writer,
               state->imu.accel_calibration_level_reject_count);
    writer_u32(writer,
               state->imu.accel_calibration_invalid_sample_count);

    for (axis = 0U; axis < APP_STATE_AXIS_COUNT; ++axis) {
        const float milli_m_s2 =
            state->imu.accel_bias_m_s2[axis] * 1000.0f;
        int32_t value;

        if (!isfinite(milli_m_s2)) {
            value = 0;
        } else if (milli_m_s2 >= (float)INT32_MAX) {
            value = INT32_MAX;
        } else if (milli_m_s2 <= (float)INT32_MIN) {
            value = INT32_MIN;
        } else {
            value = (int32_t)lroundf(milli_m_s2);
        }
        writer_i32(writer, value);
    }
}

static void handle_getfun_imu_filter_status(
    payload_writer_t *writer,
    const app_state_snapshot_t *state)
{
    uint8_t flags = 0U;

    if (state->imu.timing_source_ready) {
        flags |= GETFUN_IMU_TIME_SOURCE_READY;
    }
    if (state->imu.timing_valid) {
        flags |= GETFUN_IMU_TIMING_VALID;
    }
    if (state->imu.filter_ready) {
        flags |= GETFUN_IMU_FILTER_READY;
    }

    writer_u8(writer, 1U);
    writer_u8(writer, flags);
    writer_u16(writer, 0U);
    writer_u32(writer, state->imu.sample_timestamp_us);
    writer_u32(writer, state->imu.sample_interval_us);
    writer_u32(writer, state->imu.sample_interval_min_us);
    writer_u32(writer, state->imu.sample_interval_max_us);
    writer_u32(writer, state->imu.timing_invalid_count);
    writer_u32(writer, state->imu.timing_reset_count);
    writer_u32(writer, state->imu.filter_reset_count);
    writer_u32(writer,
               (uint32_t)(IMU_FILTER_GYRO_CUTOFF_HZ * 1000.0f));
    writer_u32(writer,
               (uint32_t)(IMU_FILTER_ACCEL_CUTOFF_HZ * 1000.0f));
}

static void handle_getfun_attitude_status(
    payload_writer_t *writer,
    const app_state_snapshot_t *state)
{
    uint8_t flags = 0U;
    uint32_t element;

    if (state->attitude.valid) {
        flags |= GETFUN_ATTITUDE_READY;
    }

    writer_u8(writer, 1U);
    writer_u8(writer, flags);
    writer_u16(writer, 0U);
    writer_u32(writer, state->attitude.update_count);
    writer_u32(writer, state->attitude.reset_count);
    writer_u32(writer, state->attitude.invalid_input_count);
    writer_u32(writer, state->attitude.accel_rejection_count);
    writer_u32(writer, state->attitude.gyro_only_update_count);

    for (element = 0U;
         element < APP_STATE_QUATERNION_COUNT;
         ++element) {
        writer_i32(
            writer,
            scaled_float_to_i32(
                state->attitude.quaternion[element],
                1000000000.0f));
    }
    writer_i32(
        writer,
        scaled_float_to_i32(state->attitude.roll_deg, 1000.0f));
    writer_i32(
        writer,
        scaled_float_to_i32(state->attitude.pitch_deg, 1000.0f));
    writer_i32(
        writer,
        scaled_float_to_i32(state->attitude.yaw_deg, 1000.0f));
}

static uint32_t request_u32(const msp_request_t *request, uint16_t offset)
{
    return (uint32_t)request->payload[offset] |
           ((uint32_t)request->payload[offset + 1U] << 8U) |
           ((uint32_t)request->payload[offset + 2U] << 16U) |
           ((uint32_t)request->payload[offset + 3U] << 24U);
}

static uint16_t request_u16(const msp_request_t *request, uint16_t offset)
{
    return (uint16_t)request->payload[offset] |
           ((uint16_t)request->payload[offset + 1U] << 8U);
}

void msp_server_init(void)
{
}

void msp_server_process(const msp_request_t *request,
                        msp_response_t *response)
{
    app_state_snapshot_t state;
    payload_writer_t writer;

    if ((request == NULL) || (response == NULL)) {
        return;
    }

    memset(response, 0, sizeof(*response));
    response->supported = true;
    writer.data = response->payload;
    writer.capacity = MSP_MAX_PAYLOAD_SIZE;
    writer.length = 0U;
    writer.overflow = false;
    app_state_get_snapshot(&state);

    switch (request->command) {
    case MSP_API_VERSION:
        writer_u8(&writer, MSP_PROTOCOL_VERSION);
        writer_u8(&writer, API_VERSION_MAJOR);
        writer_u8(&writer, API_VERSION_MINOR);
        break;

    case MSP_FC_VARIANT:
        /*
         * The stock Betaflight Configurator only opens its normal UI for BTFL.
         * Board identity remains GF72/GETFUN_F722 in MSP_BOARD_INFO.
         */
        writer_data(&writer, "BTFL", 4U);
        break;

    case MSP_FC_VERSION:
        writer_u8(&writer, 0U);
        writer_u8(&writer, 1U);
        writer_u8(&writer, 0U);
        break;

    case MSP_BOARD_INFO:
        handle_board_info(&writer);
        break;

    case MSP_BUILD_INFO:
        handle_build_info(&writer);
        break;

    case MSP_NAME:
        writer_data(&writer, "GETFUN F722", 10U);
        break;

    case MSP_STATUS:
        write_status_base(&writer, &state);
        break;

    case MSP_STATUS_EX:
        handle_status_ex(&writer, &state);
        break;

    case MSP_RAW_IMU:
        handle_raw_imu(&writer, &state);
        break;

    case MSP_RC:
        handle_rc(&writer, &state);
        break;

    case MSP_ATTITUDE:
        handle_attitude(&writer, &state);
        break;

    case MSP_ANALOG:
        handle_analog(&writer, &state);
        break;

    case MSP_BATTERY_STATE:
        handle_battery_state(&writer, &state);
        break;

    case MSP_VOLTAGE_METERS:
        handle_voltage_meters(&writer, &state);
        break;

    case MSP_CURRENT_METERS:
        handle_current_meters(&writer, &state);
        break;

    case MSP_BOXNAMES:
        break;

    case MSP_FEATURE_CONFIG:
        writer_u32(&writer, FEATURE_RX_SERIAL);
        break;

    case MSP_MIXER_CONFIG:
        /* Betaflight mixerMode_e: 3 = Quad X; reverseMotorDir = false. */
        writer_u8(&writer, 3U);
        writer_u8(&writer, 0U);
        break;

    case MSP_RX_CONFIG:
        handle_rx_config(&writer);
        break;

    case MSP_RSSI_CONFIG:
        writer_u8(&writer, 0U);
        break;

    case MSP_RX_MAP:
        writer_data(&writer,
                    rc_input_aetr_map,
                    RC_INPUT_MAPPABLE_CHANNEL_COUNT);
        break;

    case MSP_FAILSAFE_CONFIG:
        writer_u8(&writer, RC_INPUT_TIMEOUT_MS / 100U);
        writer_u8(&writer, 0U);
        writer_u16(&writer, 1000U);
        writer_u8(&writer, 0U);
        writer_u16(&writer, 0U);
        writer_u8(&writer, 1U);
        break;

    case MSP_RC_TUNING:
        handle_rc_tuning(&writer);
        break;

    case MSP_RC_DEADBAND:
        writer_u8(&writer,
                  rc_setpoint_default_profile()->deadband_us);
        writer_u8(&writer,
                  rc_setpoint_default_profile()->yaw_deadband_us);
        writer_u8(&writer, 0U);
        writer_u16(&writer, 50U);
        break;

    case MSP_BATTERY_CONFIG:
        handle_battery_config(&writer);
        break;

    case MSP_VOLTAGE_METER_CONFIG:
        handle_voltage_meter_config(&writer);
        break;

    case MSP_CURRENT_METER_CONFIG:
        handle_current_meter_config(&writer);
        break;

    case MSP_UID:
        writer_u32(&writer, *(const uint32_t *)(UID_BASE));
        writer_u32(&writer, *(const uint32_t *)(UID_BASE + 4U));
        writer_u32(&writer, *(const uint32_t *)(UID_BASE + 8U));
        break;

    case MSP_ACC_CALIBRATION:
        if ((request->payload_length != 0U) ||
            state.flight.armed ||
            !state.imu.present ||
            (state.imu.gyro_calibration_state !=
             APP_GYRO_CALIBRATION_READY) ||
            (state.imu.accel_calibration_state ==
             APP_ACCEL_CALIBRATION_CALIBRATING) ||
            (state.imu.accel_calibration_state ==
             APP_ACCEL_CALIBRATION_CANDIDATE_READY) ||
            !imu_task_request_accel_calibration()) {
            response->supported = false;
        }
        break;

    case MSP_SET_ARMING_DISABLED:
        if (request->payload_length < 1U) {
            response->supported = false;
        } else {
            app_state_set_configurator_arming_disabled(
                request->payload[0] != 0U);
        }
        break;

    case MSP_SET_RTC:
        if (request->payload_length < 6U) {
            response->supported = false;
        } else {
            app_state_set_host_rtc(request_u32(request, 0U),
                                   request_u16(request, 4U));
        }
        break;

    case MSP2_GET_TEXT:
        handle_get_text(request, &writer);
        break;

    case MSP2_MCU_INFO:
        writer_u8(&writer, MCU_TYPE_ID_PROVIDED_BY_NAME);
        writer_pstring(&writer, "STM32F722");
        break;

    case MSP2_GETFUN_CALIBRATION_STATUS:
        handle_getfun_calibration_status(&writer, &state);
        break;

    case MSP2_GETFUN_IMU_FILTER_STATUS:
        handle_getfun_imu_filter_status(&writer, &state);
        break;

    case MSP2_GETFUN_ATTITUDE_STATUS:
        handle_getfun_attitude_status(&writer, &state);
        break;

    case MSP2_GETFUN_RC_STATUS:
        handle_getfun_rc_status(&writer, &state);
        break;

    case MSP2_GETFUN_POWER_STATUS:
        handle_getfun_power_status(&writer, &state);
        break;

    case MSP2_GETFUN_FLIGHT_MOTOR_STATUS:
        handle_getfun_flight_motor_status(&writer, &state);
        break;

    case MSP2_SET_GETFUN_MOTOR_TEST:
        if (request->payload_length != (DSHOT_MOTOR_COUNT * 2U)) {
            response->supported = false;
        } else {
            uint16_t values[DSHOT_MOTOR_COUNT];
            uint32_t motor;
            bool requests_output = false;

            for (motor = 0U; motor < DSHOT_MOTOR_COUNT; ++motor) {
                values[motor] = request_u16(
                    request, (uint16_t)(motor * 2U));
                requests_output = requests_output || (values[motor] != 0U);
            }
            if ((requests_output &&
                 (state.flight.armed ||
                  state.flight.rc_setpoint.arm_requested ||
                  !state.flight.dshot_ready ||
                  !state.flight.inputs_ready)) ||
                !flight_task_request_motor_test(values)) {
                response->supported = false;
            }
        }
        break;

    case MSP2_GETFUN_RC_SETPOINT_STATUS:
        handle_getfun_rc_setpoint_status(&writer, &state);
        break;

    case MSP2_GETFUN_CONTROL_STATUS:
        handle_getfun_control_status(&writer, &state);
        break;

    case MSP2_GETFUN_ARMING_STATUS:
        handle_getfun_arming_status(&writer, &state);
        break;

    default:
        response->supported = false;
        break;
    }

    if (writer.overflow) {
        response->supported = false;
        response->payload_length = 0U;
    } else {
        response->payload_length = writer.length;
    }
}

#include "protocol/msp_server.h"

#include <string.h>

#include "app_state.h"
#include "stm32f7xx_hal.h"

#define MSP_API_VERSION 1U
#define MSP_FC_VARIANT 2U
#define MSP_FC_VERSION 3U
#define MSP_BOARD_INFO 4U
#define MSP_BUILD_INFO 5U
#define MSP_NAME 10U
#define MSP_BATTERY_CONFIG 32U
#define MSP_FEATURE_CONFIG 36U
#define MSP_SET_ARMING_DISABLED 99U
#define MSP_STATUS 101U
#define MSP_RAW_IMU 102U
#define MSP_ATTITUDE 108U
#define MSP_ANALOG 110U
#define MSP_BOXNAMES 116U
#define MSP_BATTERY_STATE 130U
#define MSP_STATUS_EX 150U
#define MSP_UID 160U
#define MSP_SET_RTC 246U

#define MSP2_GET_TEXT 0x3006U
#define MSP2_MCU_INFO 0x300CU

#define MSP2TEXT_PILOT_NAME 1U
#define MSP2TEXT_CRAFT_NAME 2U
#define MSP2TEXT_BUILD_KEY 5U

#define MSP_PROTOCOL_VERSION 0U
#define API_VERSION_MAJOR 1U
#define API_VERSION_MINOR 48U

#define SENSOR_ACC (1U << 0U)
#define SENSOR_MAG (1U << 2U)
#define SENSOR_GYRO (1U << 5U)

#define TARGET_HAS_VCP (1U << 0U)
#define MCU_TYPE_ID_PROVIDED_BY_NAME 255U

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

    if (state->imu_present) {
        sensors |= SENSOR_ACC | SENSOR_GYRO;
        if ((state->magnetometer[0] != 0) ||
            (state->magnetometer[1] != 0) ||
            (state->magnetometer[2] != 0)) {
            sensors |= SENSOR_MAG;
        }
    }
    return sensors;
}

static void write_status_base(payload_writer_t *writer,
                              const app_state_snapshot_t *state)
{
    writer_u16(writer, state->cycle_time_us);
    writer_u16(writer, state->i2c_error_count);
    writer_u16(writer, active_sensor_mask(state));
    writer_u32(writer, 0U);
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
    write_status_base(writer, state);
    writer_u16(writer, state->cpu_load_permille);
    writer_u8(writer, 1U);
    writer_u8(writer, 0U);
    writer_u8(writer, 0U);
    writer_u8(writer, 0U);
    writer_u32(writer, 0U);
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

    for (axis = 0U; axis < APP_STATE_AXIS_COUNT; ++axis) {
        writer_u16(writer, (uint16_t)state->accelerometer[axis]);
    }
    for (axis = 0U; axis < APP_STATE_AXIS_COUNT; ++axis) {
        writer_u16(writer, (uint16_t)state->gyroscope[axis]);
    }
    for (axis = 0U; axis < APP_STATE_AXIS_COUNT; ++axis) {
        writer_u16(writer, (uint16_t)state->magnetometer[axis]);
    }
}

static void handle_analog(payload_writer_t *writer,
                          const app_state_snapshot_t *state)
{
    uint16_t voltage = state->battery_present ? state->battery_voltage_cv : 0U;

    writer_u8(writer, (uint8_t)((voltage / 10U) > 255U
                                    ? 255U
                                    : (voltage / 10U)));
    writer_u16(writer, state->battery_present
                           ? state->battery_consumed_mah
                           : 0U);
    writer_u16(writer, state->rssi);
    writer_u16(writer, state->battery_present
                           ? (uint16_t)state->battery_current_ca
                           : 0U);
    writer_u16(writer, voltage);
}

static void handle_battery_state(payload_writer_t *writer,
                                 const app_state_snapshot_t *state)
{
    const bool present = state->battery_present;
    const uint16_t voltage = present ? state->battery_voltage_cv : 0U;

    writer_u8(writer, present ? state->battery_cell_count : 0U);
    writer_u16(writer, present ? state->battery_capacity_mah : 0U);
    writer_u8(writer, (uint8_t)((voltage / 10U) > 255U
                                    ? 255U
                                    : (voltage / 10U)));
    writer_u16(writer, present ? state->battery_consumed_mah : 0U);
    writer_u16(writer, present ? (uint16_t)state->battery_current_ca : 0U);
    writer_u8(writer, 0U);
    writer_u16(writer, voltage);
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

    case MSP_ATTITUDE:
        writer_u16(&writer, (uint16_t)state.roll_deg10);
        writer_u16(&writer, (uint16_t)state.pitch_deg10);
        writer_u16(&writer, (uint16_t)state.yaw_deg);
        break;

    case MSP_ANALOG:
        handle_analog(&writer, &state);
        break;

    case MSP_BATTERY_STATE:
        handle_battery_state(&writer, &state);
        break;

    case MSP_BOXNAMES:
        break;

    case MSP_FEATURE_CONFIG:
        writer_u32(&writer, 0U);
        break;

    case MSP_BATTERY_CONFIG:
        writer_u8(&writer, 0U);
        writer_u8(&writer, 0U);
        writer_u8(&writer, 0U);
        writer_u16(&writer, 0U);
        writer_u8(&writer, 0U);
        writer_u8(&writer, 0U);
        writer_u16(&writer, 0U);
        writer_u16(&writer, 0U);
        writer_u16(&writer, 0U);
        break;

    case MSP_UID:
        writer_u32(&writer, *(const uint32_t *)(UID_BASE));
        writer_u32(&writer, *(const uint32_t *)(UID_BASE + 4U));
        writer_u32(&writer, *(const uint32_t *)(UID_BASE + 8U));
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

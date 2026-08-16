/*
 * msp_transport.c —— MSP 协议传输层实现（纯协议层，无 RTOS 依赖）
 *
 * 本文件实现 MSP 帧的逐字节解析与响应帧构造，是 msp_transport.h 中接口的具体逻辑。
 *
 * 主要内容：
 *   - 解析状态机枚举（PARSER_IDLE -> HEADER -> DIRECTION -> ... -> CHECKSUM）：
 *       同时支持 V1（"$M<" + size + cmd + payload + XOR 校验）
 *       和 V2（"$X<" + flags + cmd16 + size16 + payload + CRC8-DVB-S2 校验）。
 *   - crc8_dvb_s2()：V2 使用的 CRC8 校验（多项式 0xD5）。
 *   - msp_parser_process_byte()：核心状态机，每次喂入一个字节；当一帧完整且校验通过时，
 *       把请求拷贝输出并复位解析器，返回 true。任一字节不符合则复位等待下一个 '$'。
 *   - msp_transport_build_response()：按请求的协议版本构造回包字节流（方向符 '>' 成功/
 *       '!' 不支持），完成对应校验（V1 用 XOR，V2 用 CRC8-DVB-S2），返回帧总长度。
 *
 * 边界处理：V2 负载超过 MSP_MAX_PAYLOAD_SIZE 时直接丢弃该帧；构造响应时容量不足则返回 0。
 */
#include "protocol/msp_transport.h"

#include <string.h>

enum
{
    PARSER_IDLE = 0,
    PARSER_HEADER,
    PARSER_DIRECTION,
    PARSER_V1_SIZE,
    PARSER_V1_COMMAND,
    PARSER_V1_PAYLOAD,
    PARSER_V1_CHECKSUM,
    PARSER_V2_FLAGS,
    PARSER_V2_COMMAND_LOW,
    PARSER_V2_COMMAND_HIGH,
    PARSER_V2_SIZE_LOW,
    PARSER_V2_SIZE_HIGH,
    PARSER_V2_PAYLOAD,
    PARSER_V2_CHECKSUM
};

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t data)
{
    uint8_t bit;

    crc ^= data;
    for (bit = 0U; bit < 8U; ++bit) {
        crc = (crc & 0x80U) != 0U
                  ? (uint8_t)((crc << 1U) ^ 0xD5U)
                  : (uint8_t)(crc << 1U);
    }
    return crc;
}

static void parser_reset(msp_parser_t *parser)
{
    parser->state = PARSER_IDLE;
    parser->payload_offset = 0U;
    parser->checksum = 0U;
}

void msp_parser_init(msp_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->state = PARSER_IDLE;
}

static bool parser_complete(msp_parser_t *parser, msp_request_t *request)
{
    *request = parser->request;
    parser_reset(parser);
    return true;
}

bool msp_parser_process_byte(msp_parser_t *parser,
                             uint8_t byte,
                             msp_request_t *request)
{
    if ((parser == NULL) || (request == NULL)) {
        return false;
    }

    switch (parser->state) {
    case PARSER_IDLE:
        if (byte == '$') {
            parser->state = PARSER_HEADER;
        }
        break;

    case PARSER_HEADER:
        if (byte == 'M') {
            parser->request.protocol = MSP_PROTOCOL_V1;
            parser->state = PARSER_DIRECTION;
        } else if (byte == 'X') {
            parser->request.protocol = MSP_PROTOCOL_V2;
            parser->state = PARSER_DIRECTION;
        } else {
            parser_reset(parser);
        }
        break;

    case PARSER_DIRECTION:
        if (byte != '<') {
            parser_reset(parser);
        } else if (parser->request.protocol == MSP_PROTOCOL_V1) {
            parser->state = PARSER_V1_SIZE;
        } else {
            parser->state = PARSER_V2_FLAGS;
        }
        break;

    case PARSER_V1_SIZE:
        parser->request.payload_length = byte;
        parser->payload_offset = 0U;
        parser->checksum = byte;
        parser->state = PARSER_V1_COMMAND;
        break;

    case PARSER_V1_COMMAND:
        parser->request.command = byte;
        parser->request.flags = 0U;
        parser->checksum ^= byte;
        parser->state = parser->request.payload_length == 0U
                            ? PARSER_V1_CHECKSUM
                            : PARSER_V1_PAYLOAD;
        break;

    case PARSER_V1_PAYLOAD:
        parser->request.payload[parser->payload_offset++] = byte;
        parser->checksum ^= byte;
        if (parser->payload_offset >= parser->request.payload_length) {
            parser->state = PARSER_V1_CHECKSUM;
        }
        break;

    case PARSER_V1_CHECKSUM:
        if (byte == parser->checksum) {
            return parser_complete(parser, request);
        }
        parser_reset(parser);
        break;

    case PARSER_V2_FLAGS:
        parser->request.flags = byte;
        parser->checksum = crc8_dvb_s2(0U, byte);
        parser->state = PARSER_V2_COMMAND_LOW;
        break;

    case PARSER_V2_COMMAND_LOW:
        parser->request.command = byte;
        parser->checksum = crc8_dvb_s2(parser->checksum, byte);
        parser->state = PARSER_V2_COMMAND_HIGH;
        break;

    case PARSER_V2_COMMAND_HIGH:
        parser->request.command |= (uint16_t)byte << 8U;
        parser->checksum = crc8_dvb_s2(parser->checksum, byte);
        parser->state = PARSER_V2_SIZE_LOW;
        break;

    case PARSER_V2_SIZE_LOW:
        parser->request.payload_length = byte;
        parser->checksum = crc8_dvb_s2(parser->checksum, byte);
        parser->state = PARSER_V2_SIZE_HIGH;
        break;

    case PARSER_V2_SIZE_HIGH:
        parser->request.payload_length |= (uint16_t)byte << 8U;
        parser->checksum = crc8_dvb_s2(parser->checksum, byte);
        parser->payload_offset = 0U;
        if (parser->request.payload_length > MSP_MAX_PAYLOAD_SIZE) {
            parser_reset(parser);
        } else {
            parser->state = parser->request.payload_length == 0U
                                ? PARSER_V2_CHECKSUM
                                : PARSER_V2_PAYLOAD;
        }
        break;

    case PARSER_V2_PAYLOAD:
        parser->request.payload[parser->payload_offset++] = byte;
        parser->checksum = crc8_dvb_s2(parser->checksum, byte);
        if (parser->payload_offset >= parser->request.payload_length) {
            parser->state = PARSER_V2_CHECKSUM;
        }
        break;

    case PARSER_V2_CHECKSUM:
        if (byte == parser->checksum) {
            return parser_complete(parser, request);
        }
        parser_reset(parser);
        break;

    default:
        parser_reset(parser);
        break;
    }

    return false;
}

size_t msp_transport_build_response(const msp_request_t *request,
                                    const msp_response_t *response,
                                    uint8_t *frame,
                                    size_t capacity)
{
    size_t offset = 0U;
    uint16_t index;
    uint8_t checksum;

    if ((request == NULL) || (response == NULL) || (frame == NULL)) {
        return 0U;
    }

    if (request->protocol == MSP_PROTOCOL_V1) {
        if ((response->payload_length > 255U) ||
            (capacity < (size_t)response->payload_length + 6U)) {
            return 0U;
        }

        frame[offset++] = '$';
        frame[offset++] = 'M';
        frame[offset++] = response->supported ? '>' : '!';
        frame[offset++] = (uint8_t)response->payload_length;
        frame[offset++] = (uint8_t)request->command;
        checksum = frame[3] ^ frame[4];

        for (index = 0U; index < response->payload_length; ++index) {
            frame[offset++] = response->payload[index];
            checksum ^= response->payload[index];
        }
        frame[offset++] = checksum;
        return offset;
    }

    if (capacity < (size_t)response->payload_length + 9U) {
        return 0U;
    }

    frame[offset++] = '$';
    frame[offset++] = 'X';
    frame[offset++] = response->supported ? '>' : '!';
    frame[offset++] = request->flags;
    frame[offset++] = (uint8_t)request->command;
    frame[offset++] = (uint8_t)(request->command >> 8U);
    frame[offset++] = (uint8_t)response->payload_length;
    frame[offset++] = (uint8_t)(response->payload_length >> 8U);

    checksum = 0U;
    for (index = 3U; index < offset; ++index) {
        checksum = crc8_dvb_s2(checksum, frame[index]);
    }

    for (index = 0U; index < response->payload_length; ++index) {
        frame[offset++] = response->payload[index];
        checksum = crc8_dvb_s2(checksum, response->payload[index]);
    }
    frame[offset++] = checksum;
    return offset;
}

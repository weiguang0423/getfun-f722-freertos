/*
 * crsf.c - bounded CRSF parser and payload decoders.
 *
 * Frame layout: address, length, type, payload, CRC. The length covers type,
 * payload and CRC, so total frame size is length + 2. CRC covers type/payload.
 */
#include "protocol/crsf.h"

#include <string.h>

#define CRSF_MIN_LENGTH_FIELD 2U
#define CRSF_MAX_LENGTH_FIELD 62U
#define CRSF_CRC_POLYNOMIAL 0xD5U

static void crsf_parser_reset(crsf_parser_t *parser)
{
    parser->index = 0U;
    parser->expected_size = 0U;
}

void crsf_parser_init(crsf_parser_t *parser)
{
    if (parser != NULL) {
        memset(parser, 0, sizeof(*parser));
    }
}

uint8_t crsf_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0U;
    size_t index;

    if ((data == NULL) && (length != 0U)) {
        return 0U;
    }

    for (index = 0U; index < length; ++index) {
        uint8_t bit;

        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = ((crc & 0x80U) != 0U)
                      ? (uint8_t)((crc << 1U) ^ CRSF_CRC_POLYNOMIAL)
                      : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

bool crsf_parser_process_byte(crsf_parser_t *parser,
                              uint8_t byte,
                              crsf_frame_t *frame)
{
    uint8_t length_field;
    uint8_t payload_length;
    uint8_t received_crc;
    uint8_t calculated_crc;

    if ((parser == NULL) || (frame == NULL)) {
        return false;
    }

    if (parser->index == 0U) {
        if (byte != CRSF_DEVICE_ADDRESS_FLIGHT_CONTROLLER) {
            parser->sync_drop_count++;
            return false;
        }
        parser->frame[parser->index++] = byte;
        return false;
    }

    if (parser->index == 1U) {
        if ((byte < CRSF_MIN_LENGTH_FIELD) ||
            (byte > CRSF_MAX_LENGTH_FIELD)) {
            parser->length_error_count++;
            if (byte == CRSF_DEVICE_ADDRESS_FLIGHT_CONTROLLER) {
                parser->frame[0] = byte;
                parser->index = 1U;
                parser->expected_size = 0U;
            } else {
                crsf_parser_reset(parser);
            }
            return false;
        }
        parser->frame[parser->index++] = byte;
        parser->expected_size = (uint8_t)(byte + 2U);
        return false;
    }

    if ((parser->index >= parser->expected_size) ||
        (parser->index >= CRSF_MAX_FRAME_SIZE)) {
        parser->length_error_count++;
        crsf_parser_reset(parser);
        return false;
    }

    parser->frame[parser->index++] = byte;
    if (parser->index != parser->expected_size) {
        return false;
    }

    length_field = parser->frame[1];
    payload_length = (uint8_t)(length_field - 2U);
    received_crc = parser->frame[parser->expected_size - 1U];
    calculated_crc = crsf_crc8(&parser->frame[2],
                               (size_t)length_field - 1U);

    if (received_crc != calculated_crc) {
        parser->crc_error_count++;
        crsf_parser_reset(parser);
        return false;
    }

    frame->type = parser->frame[2];
    frame->payload_length = payload_length;
    if (payload_length != 0U) {
        memcpy(frame->payload, &parser->frame[3], payload_length);
    }
    parser->valid_frame_count++;
    crsf_parser_reset(parser);
    return true;
}

static uint16_t crsf_channel_ticks_to_us(uint16_t ticks)
{
    const int32_t offset = (int32_t)ticks - (int32_t)CRSF_CHANNEL_CENTER;
    int32_t pulse_us = 1500 + ((offset * 5) / 8);

    if (pulse_us < 0) {
        pulse_us = 0;
    } else if (pulse_us > (int32_t)UINT16_MAX) {
        pulse_us = (int32_t)UINT16_MAX;
    }
    return (uint16_t)pulse_us;
}

bool crsf_decode_channels(const crsf_frame_t *frame,
                          crsf_channels_t *channels)
{
    uint32_t bit_buffer = 0U;
    uint8_t bits_available = 0U;
    uint8_t input_index = 0U;
    uint8_t channel_index;

    if ((frame == NULL) || (channels == NULL) ||
        (frame->type != CRSF_FRAME_TYPE_RC_CHANNELS_PACKED) ||
        (frame->payload_length < CRSF_RC_CHANNEL_PAYLOAD_SIZE)) {
        return false;
    }

    for (channel_index = 0U;
         channel_index < CRSF_CHANNEL_COUNT;
         ++channel_index) {
        uint16_t raw;

        while (bits_available < 11U) {
            bit_buffer |= ((uint32_t)frame->payload[input_index++])
                          << bits_available;
            bits_available = (uint8_t)(bits_available + 8U);
        }
        raw = (uint16_t)(bit_buffer & 0x07FFU);
        bit_buffer >>= 11U;
        bits_available = (uint8_t)(bits_available - 11U);

        channels->raw[channel_index] = raw;
        channels->pulse_us[channel_index] =
            crsf_channel_ticks_to_us(raw);
    }
    return true;
}

bool crsf_decode_link_statistics(
    const crsf_frame_t *frame,
    crsf_link_statistics_t *statistics)
{
    const uint8_t *payload;

    if ((frame == NULL) || (statistics == NULL) ||
        (frame->type != CRSF_FRAME_TYPE_LINK_STATISTICS) ||
        (frame->payload_length < CRSF_LINK_STATISTICS_PAYLOAD_SIZE)) {
        return false;
    }

    payload = frame->payload;
    statistics->uplink_rssi_dbm[0] = -(int16_t)payload[0];
    statistics->uplink_rssi_dbm[1] = -(int16_t)payload[1];
    statistics->uplink_link_quality = payload[2];
    statistics->uplink_snr_db = (int8_t)payload[3];
    statistics->active_antenna = payload[4];
    statistics->rf_mode = payload[5];
    statistics->uplink_tx_power = payload[6];
    statistics->downlink_rssi_dbm = -(int16_t)payload[7];
    statistics->downlink_link_quality = payload[8];
    statistics->downlink_snr_db = (int8_t)payload[9];
    return true;
}

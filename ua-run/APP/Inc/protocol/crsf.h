/*
 * crsf.h - CRSF byte-stream parser and S3.7 payload decoders.
 *
 * The module is independent from HAL and FreeRTOS. It accepts frames addressed
 * to the flight controller (0xC8), verifies the CRSF length and CRC-8/DVB-S2,
 * and exposes the two payloads needed by S3.7: packed RC channels (0x16) and
 * link statistics (0x14).
 */
#ifndef CRSF_H
#define CRSF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CRSF_DEVICE_ADDRESS_FLIGHT_CONTROLLER 0xC8U
#define CRSF_FRAME_TYPE_LINK_STATISTICS 0x14U
#define CRSF_FRAME_TYPE_RC_CHANNELS_PACKED 0x16U
#define CRSF_CHANNEL_COUNT 16U
#define CRSF_MAX_FRAME_SIZE 64U
#define CRSF_MAX_PAYLOAD_SIZE 60U
#define CRSF_RC_CHANNEL_PAYLOAD_SIZE 22U
#define CRSF_LINK_STATISTICS_PAYLOAD_SIZE 10U
#define CRSF_CHANNEL_CENTER 992U

typedef struct
{
    uint8_t type;
    uint8_t payload_length;
    uint8_t payload[CRSF_MAX_PAYLOAD_SIZE];
} crsf_frame_t;

typedef struct
{
    uint8_t frame[CRSF_MAX_FRAME_SIZE];
    uint8_t index;
    uint8_t expected_size;
    uint32_t valid_frame_count;
    uint32_t crc_error_count;
    uint32_t length_error_count;
    uint32_t sync_drop_count;
} crsf_parser_t;

typedef struct
{
    uint16_t raw[CRSF_CHANNEL_COUNT];
    uint16_t pulse_us[CRSF_CHANNEL_COUNT];
} crsf_channels_t;

typedef struct
{
    int16_t uplink_rssi_dbm[2];
    uint8_t uplink_link_quality;
    int8_t uplink_snr_db;
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;
    int16_t downlink_rssi_dbm;
    uint8_t downlink_link_quality;
    int8_t downlink_snr_db;
} crsf_link_statistics_t;

void crsf_parser_init(crsf_parser_t *parser);
bool crsf_parser_process_byte(crsf_parser_t *parser,
                              uint8_t byte,
                              crsf_frame_t *frame);
uint8_t crsf_crc8(const uint8_t *data, size_t length);
bool crsf_decode_channels(const crsf_frame_t *frame,
                          crsf_channels_t *channels);
bool crsf_decode_link_statistics(
    const crsf_frame_t *frame,
    crsf_link_statistics_t *statistics);

#ifdef __cplusplus
}
#endif

#endif

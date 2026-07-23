#ifndef MSP_TRANSPORT_H
#define MSP_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MSP_MAX_PAYLOAD_SIZE 256U
#define MSP_MAX_FRAME_SIZE (MSP_MAX_PAYLOAD_SIZE + 9U)

typedef enum
{
    MSP_PROTOCOL_V1 = 1,
    MSP_PROTOCOL_V2 = 2
} msp_protocol_t;

typedef struct
{
    msp_protocol_t protocol;
    uint8_t flags;
    uint16_t command;
    uint16_t payload_length;
    uint8_t payload[MSP_MAX_PAYLOAD_SIZE];
} msp_request_t;

typedef struct
{
    bool supported;
    uint16_t payload_length;
    uint8_t payload[MSP_MAX_PAYLOAD_SIZE];
} msp_response_t;

typedef struct
{
    uint8_t state;
    msp_request_t request;
    uint16_t payload_offset;
    uint8_t checksum;
} msp_parser_t;

void msp_parser_init(msp_parser_t *parser);
bool msp_parser_process_byte(msp_parser_t *parser,
                             uint8_t byte,
                             msp_request_t *request);
size_t msp_transport_build_response(const msp_request_t *request,
                                    const msp_response_t *response,
                                    uint8_t *frame,
                                    size_t capacity);

#ifdef __cplusplus
}
#endif

#endif

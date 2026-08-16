/*
 * msp_transport.h —— MSP 协议传输层（纯协议层，无 RTOS 依赖）
 *
 * 本头文件声明 MSP（Multiwii Serial Protocol，Betaflight 飞控通用串口协议）
 * 的帧解析与构造所需的类型和接口，是协议层与上层（msp_server / MspTask）之间的契约。
 *
 * 主要内容：
 *   - 容量宏：MSP_MAX_PAYLOAD_SIZE（单帧负载上限 256 字节）、MSP_MAX_FRAME_SIZE
 *   - msp_protocol_t：协议版本枚举（V1 = "$M<"，V2 = "$X<"）
 *   - msp_request_t：解析后的一帧请求（版本/标志/命令号/负载长度/负载）
 *   - msp_response_t：待发送的响应（是否支持/负载长度/负载）
 *   - msp_parser_t：逐字节状态机解析器的上下文（状态/当前请求/偏移/校验）
 *   - 接口函数：
 *       msp_parser_init()              —— 初始化解析器
 *       msp_parser_process_byte()      —— 喂入一个字节，帧完整且校验通过时输出请求
 *       msp_transport_build_response() —— 把响应打包成可发送的字节流帧
 *
 * 设计说明：本层只管"帧的字节如何拆/拼"，不碰 USB 收发细节，也不读取任何业务状态，
 * 可被任意任务复用，便于移植和单元测试。
 */
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

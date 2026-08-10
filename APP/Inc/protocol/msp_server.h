/*
 * msp_server.h —— MSP 协议业务层（命令派发）接口
 *
 * 本头文件声明把"已解析的一帧 MSP 请求"翻译成"具体响应内容"的入口，是协议命令
 * 与板载业务状态（app_state）之间的桥梁。它依赖 msp_transport.h 定义的请求/响应结构。
 *
 * 主要内容：
 *   - msp_server_init()         —— 业务层初始化（当前为空，预留扩展点）
 *   - msp_server_process()      —— 命令派发主入口：接收一个请求，按 command 分支
 *                                   读 app_state 快照、填充响应负载、排队校准请求
 *                                   或标记为"不支持"
 *
 * 设计说明：本层只关心"每个命令该回什么数据"，不碰帧的字节级拆/拼（那是
 * msp_transport的职责），也不直接接触USB收发。标准MSP_ACC_CALIBRATION由本层
 * 检查快照后交给ImuTask；GETFUN MSP2 0x4000～0x4009分别提供校准参数、
 * IMU时间/低通、Mahony姿态、RC Failsafe、ADC电源、Flight/DShot、RC setpoint和
 * Rate PID/Quad-X Mixer、ARM/Failsafe诊断，
 * 详见msp_server.c的命令列表。
 */
#ifndef MSP_SERVER_H
#define MSP_SERVER_H

#include "protocol/msp_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

void msp_server_init(void);
void msp_server_process(const msp_request_t *request,
                        msp_response_t *response);

#ifdef __cplusplus
}
#endif

#endif

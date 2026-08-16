/*
 * app_task.c —— 应用层 RTOS 任务的统一创建与 MspTask 运行
 *
 * 本文件是应用层的数据流主轴：把 USB CDC 收到的字节流，逐字节解析成 MSP 请求，
 * 派发处理后再把响应帧通过 USB CDC 发回，对外伪装成 Betaflight PID 控制器。
 *
 * 主要内容：
 *   - app_tasks_init()：由 CubeMX 的 MX_FREERTOS_Init() 调用。先初始化 app_state 和
 *       usb_cdc_transport，再静态创建ImuTask、RcTask、BatteryTask、FlightTask与MspTask。各采样
 *       职责见对应rtos源文件；MspTask栈768 words、优先级tskIDLE_PRIORITY+2。
 *   - msp_task()：MspTask 任务体，整条 USB→解析→状态→回包链路的驱动者——
 *       1) 初始化解析器/服务端，并 usb_cdc_transport_bind_current_task() 绑定自身，
 *          以便 USB ISR 收到数据后能唤醒本任务；
 *       2) 主循环：usb_cdc_transport_read() 取出若干字节 → msp_parser_process_byte()
 *          逐字节喂入状态机，每凑成一帧就 msp_server_process() 派发 + 构造回包 +
 *          usb_cdc_transport_write() 回送（超时 50ms）；
 *       3) 环缓冲空时 ulTaskNotifyTake() 阻塞等待通知（最多 20ms 超时兜底）。
 *
 * 数据流：USB ISR -> transport 环缓冲 -> 本任务读取 -> msp_transport 解析 ->
 *         msp_server 派发(读 app_state 快照) -> msp_transport 组帧 -> transport 发回。
 */
#include "rtos/app_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "bsp/usb_cdc_transport.h"
#include "protocol/msp_server.h"
#include "protocol/msp_transport.h"
#include "rtos/battery_task.h"
#include "rtos/flight_task.h"
#include "rtos/imu_task.h"
#include "rtos/rc_task.h"

#define MSP_TASK_STACK_WORDS 1024U
#define MSP_TASK_PRIORITY (tskIDLE_PRIORITY + 2U)
#define MSP_TX_TIMEOUT_MS 50U

static StaticTask_t msp_task_control_block;
static StackType_t msp_task_stack[MSP_TASK_STACK_WORDS];
static TaskHandle_t msp_task_handle;

static void msp_task(void *argument)
{
    msp_parser_t parser;
    msp_request_t request;
    msp_response_t response;
    uint8_t input[64];
    uint8_t output[MSP_MAX_FRAME_SIZE];

    (void)argument;
    msp_parser_init(&parser);
    msp_server_init();
    usb_cdc_transport_bind_current_task();

    for (;;) {
        size_t count;

        while ((count = usb_cdc_transport_read(input, sizeof(input))) != 0U) {
            size_t index;

            for (index = 0U; index < count; ++index) {
                if (msp_parser_process_byte(&parser, input[index], &request)) {
                    size_t frame_length;

                    msp_server_process(&request, &response);
                    frame_length = msp_transport_build_response(&request,
                                                                &response,
                                                                output,
                                                                sizeof(output));

                    if (frame_length != 0U) {
                        (void)usb_cdc_transport_write(output,
                                                      frame_length,
                                                      MSP_TX_TIMEOUT_MS);
                    }
                }
            }
        }

        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20U));
    }
}

void app_tasks_init(void)
{
    app_state_init();
    usb_cdc_transport_init();
    imu_task_create();
    rc_task_create();
    battery_task_create();
    flight_task_create();

    msp_task_handle = xTaskCreateStatic(msp_task,
                                        "MspTask",
                                        MSP_TASK_STACK_WORDS,
                                        NULL,
                                        MSP_TASK_PRIORITY,
                                        msp_task_stack,
                                        &msp_task_control_block);
    configASSERT(msp_task_handle != NULL);
}

#include "rtos/app_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "bsp/usb_cdc_transport.h"
#include "protocol/msp_server.h"
#include "protocol/msp_transport.h"

#define MSP_TASK_STACK_WORDS 768U
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

    msp_task_handle = xTaskCreateStatic(msp_task,
                                        "MspTask",
                                        MSP_TASK_STACK_WORDS,
                                        NULL,
                                        MSP_TASK_PRIORITY,
                                        msp_task_stack,
                                        &msp_task_control_block);
    configASSERT(msp_task_handle != NULL);
}

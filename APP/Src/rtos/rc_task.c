/*
 * rc_task.c - S3.7 CRSF receiver task.
 *
 * UART/DMA callbacks only enqueue bytes. RcTask verifies frames, decodes RC
 * channels and link statistics, then atomically publishes app_state.rc.
 * Timeout/failsafe invalidation is deliberately deferred to S3.8.
 */
#include "rtos/rc_task.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_state.h"
#include "bsp/crsf_uart.h"
#include "protocol/crsf.h"

#define RC_TASK_STACK_WORDS 512U
#define RC_TASK_PRIORITY (tskIDLE_PRIORITY + 3U)
#define RC_TASK_WAIT_MS 20U

static StaticTask_t rc_task_control_block;
static StackType_t rc_task_stack[RC_TASK_STACK_WORDS];
static TaskHandle_t rc_task_handle;

static void rc_task_update_diagnostics(app_rc_state_t *rc,
                                       const crsf_parser_t *parser)
{
    crsf_uart_diagnostics_t uart;

    crsf_uart_get_diagnostics(&uart);
    rc->uart_running = uart.running;
    rc->uart_start_error_count = uart.start_error_count;
    rc->uart_rx_event_count = uart.rx_event_count;
    rc->uart_idle_event_count = uart.idle_event_count;
    rc->uart_ring_overflow_count = uart.ring_overflow_count;
    rc->uart_error_count = uart.uart_error_count;
    rc->uart_recovery_count = uart.recovery_count;
    rc->last_uart_error = uart.last_uart_error;
    rc->parser_valid_frame_count = parser->valid_frame_count;
    rc->parser_crc_error_count = parser->crc_error_count;
    rc->parser_length_error_count = parser->length_error_count;
    rc->parser_sync_drop_count = parser->sync_drop_count;
}

static void rc_task_handle_frame(app_rc_state_t *rc,
                                 const crsf_frame_t *frame)
{
    if (frame->type == CRSF_FRAME_TYPE_RC_CHANNELS_PACKED) {
        crsf_channels_t channels;

        if (crsf_decode_channels(frame, &channels)) {
            memcpy(rc->channel_raw,
                   channels.raw,
                   sizeof(rc->channel_raw));
            memcpy(rc->channel_us,
                   channels.pulse_us,
                   sizeof(rc->channel_us));
            rc->channels_valid = true;
            rc->last_channel_tick = xTaskGetTickCount();
            rc->channel_sequence++;
            rc->channel_frame_count++;
        } else {
            rc->payload_error_count++;
        }
        return;
    }

    if (frame->type == CRSF_FRAME_TYPE_LINK_STATISTICS) {
        crsf_link_statistics_t statistics;

        if (crsf_decode_link_statistics(frame, &statistics)) {
            rc->link_statistics_valid = true;
            rc->last_link_statistics_tick = xTaskGetTickCount();
            rc->link_frame_count++;
            rc->uplink_rssi_dbm[0] = statistics.uplink_rssi_dbm[0];
            rc->uplink_rssi_dbm[1] = statistics.uplink_rssi_dbm[1];
            rc->uplink_link_quality = statistics.uplink_link_quality;
            rc->uplink_snr_db = statistics.uplink_snr_db;
            rc->active_antenna = statistics.active_antenna;
            rc->rf_mode = statistics.rf_mode;
            rc->uplink_tx_power = statistics.uplink_tx_power;
            rc->downlink_rssi_dbm = statistics.downlink_rssi_dbm;
            rc->downlink_link_quality =
                statistics.downlink_link_quality;
            rc->downlink_snr_db = statistics.downlink_snr_db;
        } else {
            rc->payload_error_count++;
        }
        return;
    }

    rc->unsupported_frame_count++;
}

static void rc_task(void *argument)
{
    crsf_parser_t parser;
    crsf_frame_t frame;
    app_rc_state_t rc;
    uint8_t input[64];

    (void)argument;
    memset(&rc, 0, sizeof(rc));
    crsf_parser_init(&parser);
    crsf_uart_init();
    crsf_uart_bind_current_task();
    (void)crsf_uart_start();
    rc_task_update_diagnostics(&rc, &parser);
    app_state_publish_rc(&rc);

    for (;;) {
        size_t count;

        crsf_uart_service();
        while ((count = crsf_uart_read(input, sizeof(input))) != 0U) {
            size_t index;

            for (index = 0U; index < count; ++index) {
                if (crsf_parser_process_byte(&parser,
                                             input[index],
                                             &frame)) {
                    rc_task_handle_frame(&rc, &frame);
                }
            }
        }

        rc_task_update_diagnostics(&rc, &parser);
        app_state_publish_rc(&rc);
        (void)ulTaskNotifyTake(pdTRUE,
                               pdMS_TO_TICKS(RC_TASK_WAIT_MS));
    }
}

void rc_task_create(void)
{
    rc_task_handle = xTaskCreateStatic(rc_task,
                                       "RcTask",
                                       RC_TASK_STACK_WORDS,
                                       NULL,
                                       RC_TASK_PRIORITY,
                                       rc_task_stack,
                                       &rc_task_control_block);
    configASSERT(rc_task_handle != NULL);
}

uint32_t rc_task_stack_high_water_mark(void)
{
    if (rc_task_handle == NULL) {
        return 0U;
    }
    return (uint32_t)uxTaskGetStackHighWaterMark(rc_task_handle);
}

/*
 * rc_task.c - S3.7 CRSF receiver task.
 *
 * UART/DMA callbacks only enqueue bytes. RcTask verifies frames, decodes RC
 * channels and link statistics, then atomically publishes app_state.rc.
 * S3.8 adds AETR mapping plus bounded loss and recovery handling. S7.7 keeps
 * the physical snapshot as authority for ARM/AUX, validates the USART6 Linux
 * candidate, and publishes only the arbiter's effective mapped channels.
 */
#include "rtos/rc_task.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f7xx_hal.h"

#include "app_state.h"
#include "bsp/crsf_uart.h"
#include "bsp/linux_rc_monitor.h"
#include "protocol/crsf.h"

#define RC_TASK_STACK_WORDS 512U
#define RC_TASK_PRIORITY (tskIDLE_PRIORITY + 3U)
#define RC_TASK_WAIT_MS 20U
#define RC_LINK_STATISTICS_TIMEOUT_MS 1000U
#define RC_TASK_MIN_STACK_BYTES 1536U

_Static_assert(configTICK_RATE_HZ == 1000U,
               "RcTask timestamps require a 1 kHz FreeRTOS tick");
_Static_assert((RC_TASK_STACK_WORDS * sizeof(StackType_t)) >=
                   RC_TASK_MIN_STACK_BYTES,
               "RcTask stack is below the verified minimum budget");

static StaticTask_t rc_task_control_block;
static StackType_t rc_task_stack[RC_TASK_STACK_WORDS];
static TaskHandle_t rc_task_handle;

static void rc_task_copy_failsafe(app_rc_state_t *rc,
                                  const rc_input_failsafe_t *failsafe)
{
    rc->channels_valid = rc_input_control_valid(failsafe);
    rc->failsafe_active = failsafe->failsafe_active;
    rc->failsafe_phase = failsafe->phase;
    rc->recovery_started_tick =
        (TickType_t)failsafe->recovery_started_ms;
    rc->last_failsafe_tick =
        (TickType_t)failsafe->last_failsafe_ms;
    rc->failsafe_count = failsafe->failsafe_count;
    rc->failsafe_recovery_count = failsafe->recovery_count;
    rc->failsafe_recovery_frame_count =
        failsafe->recovery_frame_count;
}

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
                                 rc_input_failsafe_t *failsafe,
                                 const crsf_frame_t *frame,
                                 TickType_t now_tick)
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
            rc_input_map_aetr(rc->channel_us,
                              rc->physical_mapped_channel_us);
            rc->last_channel_tick = now_tick;
            rc->channel_sequence++;
            rc->channel_frame_count++;
            rc_input_failsafe_on_frame(failsafe,
                                       (uint32_t)now_tick);
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
    rc_input_failsafe_t failsafe;
    rc_source_arbiter_t arbiter;
    app_rc_state_t rc;
    uint8_t input[64];

    (void)argument;
    memset(&rc, 0, sizeof(rc));
    rc_input_set_safe_channels(rc.channel_us);
    rc_input_set_safe_channels(rc.physical_mapped_channel_us);
    rc_input_set_safe_channels(rc.mapped_channel_us);
    rc_input_failsafe_init(&failsafe);
    rc_source_arbiter_init(&arbiter, HAL_GetTick());
    rc_task_copy_failsafe(&rc, &failsafe);
    crsf_parser_init(&parser);
    crsf_uart_init();
    crsf_uart_bind_current_task();
    (void)crsf_uart_start();
    rc_task_update_diagnostics(&rc, &parser);
    app_state_publish_rc(&rc);

    for (;;) {
        size_t count;
        TickType_t now_tick;
        app_rc_source_context_t source_context;
        rc_virtual_candidate_t candidate;

        crsf_uart_service();
        while ((count = crsf_uart_read(input, sizeof(input))) != 0U) {
            size_t index;

            for (index = 0U; index < count; ++index) {
                if (crsf_parser_process_byte(&parser,
                                             input[index],
                                             &frame)) {
                    now_tick = xTaskGetTickCount();
                    rc_task_handle_frame(&rc,
                                         &failsafe,
                                         &frame,
                                         now_tick);
                }
            }
        }

        now_tick = xTaskGetTickCount();
        rc_input_failsafe_update(&failsafe,
                                 (uint32_t)now_tick);
        rc_task_copy_failsafe(&rc, &failsafe);
        app_state_get_rc_source_context(&source_context);
        linux_rc_monitor_get_candidate(&candidate);
        rc_source_arbiter_update(
            &arbiter,
            rc.physical_mapped_channel_us,
            rc.channels_valid,
            source_context.flight_armed,
            source_context.arming_inhibit_flags,
            source_context.authorization_channel_available,
            &candidate,
            HAL_GetTick(),
            rc.mapped_channel_us);
        rc.active_source = arbiter.active_source;
        rc.source_last_exit_reason = arbiter.last_exit_reason;
        rc.source_authorization_active = arbiter.authorization_active;
        rc.source_reauthorization_ready = arbiter.authorization_seen_low;
        rc.virtual_candidate_valid = candidate.valid &&
            ((uint32_t)(HAL_GetTick() - candidate.received_ms) <
             RC_SOURCE_VIRTUAL_TIMEOUT_MS);
        rc.virtual_candidate_tick = (TickType_t)candidate.received_ms;
        rc.virtual_source_sequence = candidate.source_sequence;
        rc.virtual_heartbeat = candidate.heartbeat;
        rc.virtual_session_generation = candidate.session_generation;
        rc.source_activation_count = arbiter.activation_count;
        rc.source_exit_count = arbiter.exit_count;
        rc.source_last_transition_tick =
            (TickType_t)arbiter.last_transition_ms;
        if (rc.link_statistics_valid &&
            ((TickType_t)(now_tick -
                          rc.last_link_statistics_tick) >=
             pdMS_TO_TICKS(RC_LINK_STATISTICS_TIMEOUT_MS))) {
            rc.link_statistics_valid = false;
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

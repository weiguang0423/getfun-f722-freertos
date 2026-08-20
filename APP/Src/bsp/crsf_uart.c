/*
 * crsf_uart.c - UART2 RX circular DMA + IDLE transport.
 *
 * DMA uses a 128-byte circular buffer. HAL RX events expose the current write
 * position; the callback copies only the new span into a 512-byte SPSC ring.
 * RcTask drains that ring and owns error recovery.
 */
#include "bsp/crsf_uart.h"

#include <string.h>

#include "FreeRTOS.h"
#include "bsp/linux_rc_monitor.h"
#include "task.h"
#include "usart.h"

#define CRSF_UART_DMA_BUFFER_SIZE 128U
#define CRSF_UART_RING_SIZE 512U

static uint8_t dma_buffer[CRSF_UART_DMA_BUFFER_SIZE];
static uint8_t ring_buffer[CRSF_UART_RING_SIZE];
static volatile uint16_t ring_head;
static volatile uint16_t ring_tail;
static volatile uint16_t dma_last_position;
static volatile bool recovery_requested;
static TaskHandle_t rc_task_handle;
static crsf_uart_diagnostics_t diagnostics;

static uint16_t crsf_uart_ring_next(uint16_t position)
{
    position++;
    return (position >= CRSF_UART_RING_SIZE) ? 0U : position;
}

static void crsf_uart_ring_write_from_isr(const uint8_t *data,
                                          uint16_t length)
{
    uint16_t index;

    for (index = 0U; index < length; ++index) {
        const uint16_t next = crsf_uart_ring_next(ring_head);

        if (next == ring_tail) {
            diagnostics.ring_overflow_count++;
            break;
        }
        ring_buffer[ring_head] = data[index];
        __DMB();
        ring_head = next;
    }
}

static void crsf_uart_notify_from_isr(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (rc_task_handle != NULL) {
        vTaskNotifyGiveFromISR(rc_task_handle,
                               &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

static bool crsf_uart_start_receive(void)
{
    dma_last_position = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2,
                                    dma_buffer,
                                    CRSF_UART_DMA_BUFFER_SIZE) != HAL_OK) {
        diagnostics.start_error_count++;
        diagnostics.running = false;
        recovery_requested = true;
        return false;
    }
    diagnostics.running = true;
    return true;
}

void crsf_uart_init(void)
{
    taskENTER_CRITICAL();
    memset(dma_buffer, 0, sizeof(dma_buffer));
    memset(ring_buffer, 0, sizeof(ring_buffer));
    memset(&diagnostics, 0, sizeof(diagnostics));
    ring_head = 0U;
    ring_tail = 0U;
    dma_last_position = 0U;
    recovery_requested = false;
    rc_task_handle = NULL;
    taskEXIT_CRITICAL();
}

void crsf_uart_bind_current_task(void)
{
    taskENTER_CRITICAL();
    rc_task_handle = xTaskGetCurrentTaskHandle();
    taskEXIT_CRITICAL();
}

bool crsf_uart_start(void)
{
    return crsf_uart_start_receive();
}

void crsf_uart_service(void)
{
    bool recover;

    taskENTER_CRITICAL();
    recover = recovery_requested;
    recovery_requested = false;
    taskEXIT_CRITICAL();

    if (!recover) {
        return;
    }

    (void)HAL_UART_AbortReceive(&huart2);
    if (crsf_uart_start_receive()) {
        diagnostics.recovery_count++;
    } else {
        taskENTER_CRITICAL();
        recovery_requested = true;
        taskEXIT_CRITICAL();
    }
}

size_t crsf_uart_read(uint8_t *data, size_t capacity)
{
    size_t count = 0U;

    if ((data == NULL) || (capacity == 0U)) {
        return 0U;
    }

    while ((count < capacity) && (ring_tail != ring_head)) {
        data[count++] = ring_buffer[ring_tail];
        __DMB();
        ring_tail = crsf_uart_ring_next(ring_tail);
    }
    return count;
}

void crsf_uart_get_diagnostics(crsf_uart_diagnostics_t *result)
{
    if (result == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *result = diagnostics;
    taskEXIT_CRITICAL();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t position;
    HAL_UART_RxEventTypeTypeDef event_type;

    if ((huart == NULL) || (huart->Instance != USART2) ||
        (size > CRSF_UART_DMA_BUFFER_SIZE)) {
        return;
    }

    position = (size == CRSF_UART_DMA_BUFFER_SIZE) ? 0U : size;
    if (position > dma_last_position) {
        crsf_uart_ring_write_from_isr(&dma_buffer[dma_last_position],
                                      (uint16_t)(position -
                                                 dma_last_position));
    } else if (position < dma_last_position) {
        crsf_uart_ring_write_from_isr(
            &dma_buffer[dma_last_position],
            (uint16_t)(CRSF_UART_DMA_BUFFER_SIZE - dma_last_position));
        if (position != 0U) {
            crsf_uart_ring_write_from_isr(dma_buffer, position);
        }
    }
    dma_last_position = position;

    diagnostics.rx_event_count++;
    event_type = HAL_UARTEx_GetRxEventType(huart);
    if (event_type == HAL_UART_RXEVENT_IDLE) {
        diagnostics.idle_event_count++;
    } else if (event_type == HAL_UART_RXEVENT_HT) {
        diagnostics.half_event_count++;
    } else if (event_type == HAL_UART_RXEVENT_TC) {
        diagnostics.complete_event_count++;
    }
    crsf_uart_notify_from_isr();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return;
    }

    if (huart->Instance == USART6) {
        linux_rc_monitor_uart_error();
        usart6_receive_restart();
        return;
    }
    if (huart->Instance != USART2) {
        return;
    }

    diagnostics.uart_error_count++;
    diagnostics.last_uart_error = HAL_UART_GetError(huart);
    diagnostics.running = false;
    recovery_requested = true;
    crsf_uart_notify_from_isr();
}

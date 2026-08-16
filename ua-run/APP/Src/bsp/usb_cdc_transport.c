#include "bsp/usb_cdc_transport.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "usbd_cdc_if.h"

#define USB_CDC_RX_BUFFER_SIZE 1024U
#define USB_CDC_TX_BUFFER_SIZE 320U

static uint8_t rx_buffer[USB_CDC_RX_BUFFER_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_dropped;
static TaskHandle_t rx_task;

static uint8_t tx_buffer[USB_CDC_TX_BUFFER_SIZE];
static volatile bool tx_idle;

void usb_cdc_transport_init(void)
{
    rx_head = 0U;
    rx_tail = 0U;
    rx_dropped = 0U;
    rx_task = NULL;
    tx_idle = true;
}

void usb_cdc_transport_bind_current_task(void)
{
    taskENTER_CRITICAL();
    rx_task = xTaskGetCurrentTaskHandle();
    taskEXIT_CRITICAL();
}

void usb_cdc_transport_receive_from_isr(const uint8_t *data, uint32_t length)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    uint32_t index;

    if (data == NULL) {
        return;
    }

    for (index = 0U; index < length; ++index) {
        const uint16_t next =
            (uint16_t)((rx_head + 1U) % USB_CDC_RX_BUFFER_SIZE);

        if (next == rx_tail) {
            ++rx_dropped;
            break;
        }

        rx_buffer[rx_head] = data[index];
        __DMB();
        rx_head = next;
    }

    if (rx_task != NULL) {
        vTaskNotifyGiveFromISR(rx_task, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void usb_cdc_transport_transmit_complete_from_isr(void)
{
    tx_idle = true;
    __DMB();
}

size_t usb_cdc_transport_read(uint8_t *data, size_t capacity)
{
    size_t count = 0U;

    if (data == NULL) {
        return 0U;
    }

    while ((count < capacity) && (rx_tail != rx_head)) {
        data[count++] = rx_buffer[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % USB_CDC_RX_BUFFER_SIZE);
    }

    return count;
}

static bool usb_cdc_transport_wait_tx_idle(TickType_t start,
                                           TickType_t timeout)
{
    while (!tx_idle) {
        if ((xTaskGetTickCount() - start) >= timeout) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return true;
}

bool usb_cdc_transport_write(const uint8_t *data,
                             size_t length,
                             uint32_t timeout_ms)
{
    const TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    if ((data == NULL) || (length == 0U) ||
        (length > USB_CDC_TX_BUFFER_SIZE)) {
        return false;
    }

    if (timeout == 0U) {
        timeout = 1U;
    }

    if (!usb_cdc_transport_wait_tx_idle(start, timeout)) {
        return false;
    }

    memcpy(tx_buffer, data, length);

    do {
        tx_idle = false;
        __DMB();

        if (CDC_Transmit_FS(tx_buffer, (uint16_t)length) == USBD_OK) {
            return usb_cdc_transport_wait_tx_idle(start, timeout);
        }

        tx_idle = true;
        if ((xTaskGetTickCount() - start) >= timeout) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    } while (true);
}

uint32_t usb_cdc_transport_rx_dropped(void)
{
    return rx_dropped;
}

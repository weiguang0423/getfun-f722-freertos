#ifndef USB_CDC_TRANSPORT_H
#define USB_CDC_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void usb_cdc_transport_init(void);
void usb_cdc_transport_bind_current_task(void);

void usb_cdc_transport_receive_from_isr(const uint8_t *data, uint32_t length);
void usb_cdc_transport_transmit_complete_from_isr(void);

size_t usb_cdc_transport_read(uint8_t *data, size_t capacity);
bool usb_cdc_transport_write(const uint8_t *data,
                             size_t length,
                             uint32_t timeout_ms);
uint32_t usb_cdc_transport_rx_dropped(void);

#ifdef __cplusplus
}
#endif

#endif

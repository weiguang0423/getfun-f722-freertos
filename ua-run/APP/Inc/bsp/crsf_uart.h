/*
 * crsf_uart.h - UART2 circular-DMA transport for the CRSF receiver.
 *
 * ISR callbacks only copy newly arrived bytes into a software ring and notify
 * RcTask. Parsing and recovery run in task context.
 */
#ifndef CRSF_UART_H
#define CRSF_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    bool running;
    uint32_t start_error_count;
    uint32_t rx_event_count;
    uint32_t idle_event_count;
    uint32_t half_event_count;
    uint32_t complete_event_count;
    uint32_t ring_overflow_count;
    uint32_t uart_error_count;
    uint32_t recovery_count;
    uint32_t last_uart_error;
} crsf_uart_diagnostics_t;

void crsf_uart_init(void);
void crsf_uart_bind_current_task(void);
bool crsf_uart_start(void);
void crsf_uart_service(void);
size_t crsf_uart_read(uint8_t *data, size_t capacity);
void crsf_uart_get_diagnostics(crsf_uart_diagnostics_t *diagnostics);

#ifdef __cplusplus
}
#endif

#endif

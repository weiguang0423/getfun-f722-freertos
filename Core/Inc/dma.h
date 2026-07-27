/*
 * dma.h - DMA controller initialization interface for SPI1 IMU transfers.
 *
 * Purpose:
 *   Exposes the generated-style DMA clock and NVIC initialization used before
 *   SPI1 is initialized.
 *
 * Core interface:
 *   - MX_DMA_Init(): enables DMA2 and configures Stream 0/3 IRQ priority 5.
 *
 * Constraints:
 *   Both IRQs may call FreeRTOS FromISR APIs through the SPI completion path,
 *   so their preemption priority must not be numerically lower than 5.
 */
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void MX_DMA_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */

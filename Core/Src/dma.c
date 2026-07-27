/*
 * dma.c - DMA2 clock and interrupt initialization for SPI1 IMU transfers.
 *
 * Purpose:
 *   Enables the DMA controller and the RX/TX stream interrupts selected for
 *   SPI1 full-duplex transfers.
 *
 * Core function:
 *   - MX_DMA_Init(): enables DMA2 and configures Stream 0/3 at priority 5.
 *
 * Data flow and constraints:
 *   Stream 0 is SPI1 RX and Stream 3 is SPI1 TX. Transfer parameters and HAL
 *   handle linkage are configured in HAL_SPI_MspInit(). Priority 5 is required
 *   because the completion path notifies ImuTask with a FreeRTOS FromISR API.
 */
#include "dma.h"

void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

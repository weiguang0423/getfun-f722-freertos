/*
 * spi.c - SPI1 peripheral initialization for the board-mounted ICM42688P.
 *
 * Purpose:
 *   Configures SPI1 as an 8-bit, MSB-first, full-duplex master using Mode 0.
 *
 * Core function:
 *   - MX_SPI1_Init(): creates the HAL SPI1 configuration used by imu_bus.
 *
 * Data flow and constraints:
 *   SPI1 clocks data on PA5/PA6/PA7. PA4 chip select is deliberately managed
 *   by imu_bus so one CS-low window covers the address and complete payload.
 *   RX/TX DMA handles are linked by HAL_SPI_MspInit().
 *   The initial prescaler is /64: 108 MHz PCLK2 / 64 = 1.6875 MHz.
 */
#include "spi.h"

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

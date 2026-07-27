/*
 * spi.h - SPI1 peripheral interface for the board-mounted ICM42688P.
 *
 * Purpose:
 *   Exposes the CubeMX-style SPI1 handle and initialization entry used by
 *   main.c and the application IMU bus adapter.
 *
 * Core interface:
 *   - hspi1: HAL handle for SPI1 in master, full-duplex, Mode 0 operation.
 *   - hdma_spi1_rx/tx: DMA2 Stream 0/3 handles used by IMU sample transfers.
 *   - MX_SPI1_Init(): configures SPI1 for an initial 1.6875 MHz bus clock.
 *
 * Constraints:
 *   Chip select is not controlled by this module. PA4 is a software GPIO
 *   owned by imu_bus, and must remain high outside a complete transaction.
 */
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

void MX_SPI1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

/*
 * imu_bus.h —— ICM42688P 的阻塞/DMA SPI 事务适配层接口
 *
 * 作用:
 *   把 HAL SPI1 与 PA4 片选处理从传感器寄存器驱动中隔离出来,传感器层只看到
 *   有边界的读/写事务。
 *
 * 核心接口:
 *   - imu_bus_init(): 校验 SPI1,并使片选保持非激活高电平。
 *   - imu_bus_read()/imu_bus_write(): 执行一次完整的 CS 拉低事务。
 *   - imu_bus_read_dma_start()/finish(): 执行一次异步乒乓读取。
 *   - imu_bus_dma_abort(): 终止已超时的传输并恢复 CS 高电平。
 *   - imu_bus_set_dma_callback(): 绑定 ISR 完成通知回调。
 *   - imu_bus_clock_hz(): 返回配置的基线总线时钟频率。
 *
 * 数据流与约束:
 *   ImuTask 是唯一的任务侧调用者。阻塞调用保留 5 ms 超时;采样读取使用两个
 *   静态对齐的 DMA 槽位。完成回调运行在 ISR 上下文,只能通知绑定的任务。
 *   每一条退出路径都会恢复 CS 为高电平。
 */
#ifndef IMU_BUS_H
#define IMU_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    IMU_BUS_STATUS_OK = 0,
    IMU_BUS_STATUS_BAD_ARGUMENT,
    IMU_BUS_STATUS_NOT_READY,
    IMU_BUS_STATUS_HAL_ERROR,
    IMU_BUS_STATUS_HAL_BUSY,
    IMU_BUS_STATUS_HAL_TIMEOUT,
    IMU_BUS_STATUS_DMA_NOT_COMPLETE,
    IMU_BUS_STATUS_DMA_ABORTED
} imu_bus_status_t;

typedef void (*imu_bus_dma_callback_t)(void *context);

imu_bus_status_t imu_bus_init(void);
imu_bus_status_t imu_bus_read(uint8_t register_address,
                              uint8_t *data,
                              size_t length);
imu_bus_status_t imu_bus_write(uint8_t register_address,
                               const uint8_t *data,
                               size_t length);
void imu_bus_set_dma_callback(imu_bus_dma_callback_t callback,
                              void *context);
imu_bus_status_t imu_bus_read_dma_start(uint8_t register_address,
                                        size_t length);
imu_bus_status_t imu_bus_read_dma_finish(uint8_t *data,
                                         size_t length);
imu_bus_status_t imu_bus_dma_abort(void);
bool imu_bus_dma_busy(void);
uint32_t imu_bus_clock_hz(void);

#ifdef __cplusplus
}
#endif

#endif

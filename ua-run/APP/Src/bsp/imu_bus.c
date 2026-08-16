/*
 * imu_bus.c —— ICM42688P 的阻塞/DMA SPI1/PA4 传输层
 *
 * 作用:
 *   把有边界的寄存器读/写转换为 HAL 全双工 SPI 传输,并负责片选时序、对齐的
 *   乒乓 DMA 缓冲、Cache 维护、ISR 完成处理和错误清理。
 *
 * 核心函数:
 *   - imu_bus_init(): 检查 CubeMX 生成的 SPI1 句柄并释放 CS(拉高)。
 *   - imu_bus_read(): 发送 address|0x80 加填充字节,返回有效数据。
 *   - imu_bus_write(): 发送 address&0x7f 后跟传入的有效数据。
 *   - imu_bus_read_dma_start()/finish(): 异步双槽位采样读取。
 *   - imu_bus_dma_abort(): 安全地取消已超时的事务。
 *   - HAL_SPI_TxRxCpltCallback()/ErrorCallback(): 关闭 CS 并通知 ImuTask。
 *
 * 数据流与约束:
 *   初始化/状态查询调用保持阻塞。运行时采样使用 DMA2 和静态 32 字节对齐缓冲。
 *   本模块不依赖 FreeRTOS;其 ISR 回调把任务通知转交给已注册的所有者。
 *   ImuTask 必须保持唯一所有者,不允许嵌套事务。
 */
#include "bsp/imu_bus.h"

#include <string.h>

#include "main.h"
#include "spi.h"

#define IMU_BUS_MAX_PAYLOAD_SIZE 32U
#define IMU_BUS_SPI_TIMEOUT_MS 5U
#define IMU_BUS_READ_BIT 0x80U
#define IMU_BUS_CONFIGURED_CLOCK_HZ 1687500UL
#define IMU_BUS_DMA_SLOT_COUNT 2U
#define IMU_BUS_DMA_CACHE_LINE_SIZE 32U
#define IMU_BUS_DMA_BUFFER_SIZE 64U

typedef struct
{
    uint8_t transmit[IMU_BUS_DMA_BUFFER_SIZE];
    uint8_t receive[IMU_BUS_DMA_BUFFER_SIZE];
} imu_bus_dma_slot_t;

static imu_bus_dma_slot_t dma_slots[IMU_BUS_DMA_SLOT_COUNT]
    __attribute__((aligned(IMU_BUS_DMA_CACHE_LINE_SIZE)));
static volatile bool dma_active;
static volatile bool dma_completed;
static volatile uint8_t dma_active_slot;
static volatile uint8_t dma_completed_slot;
static uint8_t dma_next_slot;
static volatile size_t dma_active_payload_length;
static volatile size_t dma_completed_payload_length;
static volatile imu_bus_status_t dma_completion_status;
static imu_bus_dma_callback_t dma_callback;
static void *dma_callback_context;

static uint32_t imu_bus_lock(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();
    return primask;
}

static void imu_bus_unlock(uint32_t primask)
{
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
}

static bool imu_bus_dcache_enabled(void)
{
    return (SCB->CCR & SCB_CCR_DC_Msk) != 0U;
}

static void imu_bus_dma_prepare_cache(imu_bus_dma_slot_t *slot)
{
    if (!imu_bus_dcache_enabled()) {
        return;
    }

    SCB_CleanDCache_by_Addr((uint32_t *)slot->transmit,
                            IMU_BUS_DMA_BUFFER_SIZE);
    SCB_InvalidateDCache_by_Addr((uint32_t *)slot->receive,
                                 IMU_BUS_DMA_BUFFER_SIZE);
}

static void imu_bus_dma_complete_cache(imu_bus_dma_slot_t *slot)
{
    if (imu_bus_dcache_enabled()) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)slot->receive,
                                     IMU_BUS_DMA_BUFFER_SIZE);
    }
}

static imu_bus_status_t imu_bus_status_from_hal(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return IMU_BUS_STATUS_OK;
    case HAL_BUSY:
        return IMU_BUS_STATUS_HAL_BUSY;
    case HAL_TIMEOUT:
        return IMU_BUS_STATUS_HAL_TIMEOUT;
    case HAL_ERROR:
    default:
        return IMU_BUS_STATUS_HAL_ERROR;
    }
}

static imu_bus_status_t imu_bus_transfer(const uint8_t *transmit,
                                         uint8_t *receive,
                                         size_t length)
{
    HAL_StatusTypeDef hal_status;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    hal_status = HAL_SPI_TransmitReceive(&hspi1,
                                        (uint8_t *)transmit,
                                        receive,
                                        (uint16_t)length,
                                        IMU_BUS_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return imu_bus_status_from_hal(hal_status);
}

static void imu_bus_dma_complete_from_isr(imu_bus_status_t status)
{
    imu_bus_dma_callback_t callback;
    void *callback_context;
    uint8_t completed_slot;

    if (!dma_active) {
        return;
    }

    completed_slot = dma_active_slot;
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    imu_bus_dma_complete_cache(&dma_slots[completed_slot]);

    dma_completion_status = status;
    dma_completed_slot = completed_slot;
    dma_completed_payload_length = dma_active_payload_length;
    dma_active = false;
    dma_completed = true;
    __DMB();

    callback = dma_callback;
    callback_context = dma_callback_context;
    if (callback != NULL) {
        callback(callback_context);
    }
}

imu_bus_status_t imu_bus_init(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    if (hspi1.Instance != SPI1) {
        return IMU_BUS_STATUS_NOT_READY;
    }
    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
        return IMU_BUS_STATUS_NOT_READY;
    }
    return IMU_BUS_STATUS_OK;
}

imu_bus_status_t imu_bus_read(uint8_t register_address,
                              uint8_t *data,
                              size_t length)
{
    uint8_t transmit[IMU_BUS_MAX_PAYLOAD_SIZE + 1U] = {0};
    uint8_t receive[IMU_BUS_MAX_PAYLOAD_SIZE + 1U] = {0};
    imu_bus_status_t status;

    if ((data == NULL) || (length == 0U) ||
        (length > IMU_BUS_MAX_PAYLOAD_SIZE)) {
        return IMU_BUS_STATUS_BAD_ARGUMENT;
    }
    if (imu_bus_dma_busy()) {
        return IMU_BUS_STATUS_HAL_BUSY;
    }

    transmit[0] = register_address | IMU_BUS_READ_BIT;
    status = imu_bus_transfer(transmit, receive, length + 1U);
    if (status == IMU_BUS_STATUS_OK) {
        memcpy(data, &receive[1], length);
    }
    return status;
}

imu_bus_status_t imu_bus_write(uint8_t register_address,
                               const uint8_t *data,
                               size_t length)
{
    uint8_t transmit[IMU_BUS_MAX_PAYLOAD_SIZE + 1U] = {0};
    uint8_t receive[IMU_BUS_MAX_PAYLOAD_SIZE + 1U] = {0};

    if ((data == NULL) || (length == 0U) ||
        (length > IMU_BUS_MAX_PAYLOAD_SIZE)) {
        return IMU_BUS_STATUS_BAD_ARGUMENT;
    }
    if (imu_bus_dma_busy()) {
        return IMU_BUS_STATUS_HAL_BUSY;
    }

    transmit[0] = register_address & (uint8_t)~IMU_BUS_READ_BIT;
    memcpy(&transmit[1], data, length);
    return imu_bus_transfer(transmit, receive, length + 1U);
}

void imu_bus_set_dma_callback(imu_bus_dma_callback_t callback,
                              void *context)
{
    const uint32_t primask = imu_bus_lock();

    dma_callback = callback;
    dma_callback_context = context;
    imu_bus_unlock(primask);
}

imu_bus_status_t imu_bus_read_dma_start(uint8_t register_address,
                                        size_t length)
{
    imu_bus_dma_slot_t *slot;
    HAL_StatusTypeDef hal_status;
    uint8_t selected_slot;
    uint32_t primask;

    if ((length == 0U) || (length > IMU_BUS_MAX_PAYLOAD_SIZE) ||
        ((length + 1U) > IMU_BUS_DMA_BUFFER_SIZE)) {
        return IMU_BUS_STATUS_BAD_ARGUMENT;
    }
    if ((hspi1.Instance != SPI1) ||
        (hspi1.hdmarx == NULL) ||
        (hspi1.hdmatx == NULL)) {
        return IMU_BUS_STATUS_NOT_READY;
    }

    primask = imu_bus_lock();
    if (dma_active || dma_completed) {
        imu_bus_unlock(primask);
        return IMU_BUS_STATUS_HAL_BUSY;
    }
    selected_slot = dma_next_slot;
    dma_next_slot =
        (uint8_t)((dma_next_slot + 1U) % IMU_BUS_DMA_SLOT_COUNT);
    imu_bus_unlock(primask);

    slot = &dma_slots[selected_slot];
    memset(slot, 0, sizeof(*slot));
    slot->transmit[0] = register_address | IMU_BUS_READ_BIT;
    imu_bus_dma_prepare_cache(slot);

    primask = imu_bus_lock();
    dma_active_slot = selected_slot;
    dma_active_payload_length = length;
    dma_completion_status = IMU_BUS_STATUS_DMA_NOT_COMPLETE;
    dma_active = true;
    dma_completed = false;
    imu_bus_unlock(primask);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    hal_status = HAL_SPI_TransmitReceive_DMA(
        &hspi1,
        slot->transmit,
        slot->receive,
        (uint16_t)(length + 1U));
    if (hal_status != HAL_OK) {
        primask = imu_bus_lock();
        dma_active = false;
        dma_completed = false;
        imu_bus_unlock(primask);
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    }
    return imu_bus_status_from_hal(hal_status);
}

imu_bus_status_t imu_bus_read_dma_finish(uint8_t *data,
                                         size_t length)
{
    imu_bus_status_t status;
    size_t completed_length;
    uint8_t completed_slot;
    uint32_t primask;

    if ((data == NULL) || (length == 0U) ||
        (length > IMU_BUS_MAX_PAYLOAD_SIZE)) {
        return IMU_BUS_STATUS_BAD_ARGUMENT;
    }

    primask = imu_bus_lock();
    if (dma_active || !dma_completed) {
        imu_bus_unlock(primask);
        return IMU_BUS_STATUS_DMA_NOT_COMPLETE;
    }

    completed_slot = dma_completed_slot;
    completed_length = dma_completed_payload_length;
    status = dma_completion_status;
    if (completed_length != length) {
        imu_bus_unlock(primask);
        return IMU_BUS_STATUS_BAD_ARGUMENT;
    }
    dma_completed = false;
    imu_bus_unlock(primask);

    if (status == IMU_BUS_STATUS_OK) {
        memcpy(data,
               &dma_slots[completed_slot].receive[1],
               completed_length);
    }
    return status;
}

imu_bus_status_t imu_bus_dma_abort(void)
{
    HAL_StatusTypeDef hal_status;
    uint32_t primask = imu_bus_lock();

    if (!dma_active) {
        dma_completed = false;
        imu_bus_unlock(primask);
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        return IMU_BUS_STATUS_OK;
    }

    dma_active = false;
    dma_completed = false;
    imu_bus_unlock(primask);

    hal_status = HAL_SPI_Abort(&hspi1);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    if (hal_status == HAL_OK) {
        return IMU_BUS_STATUS_DMA_ABORTED;
    }
    return imu_bus_status_from_hal(hal_status);
}

bool imu_bus_dma_busy(void)
{
    return dma_active || dma_completed;
}

uint32_t imu_bus_clock_hz(void)
{
    return IMU_BUS_CONFIGURED_CLOCK_HZ;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi != NULL) && (hspi->Instance == SPI1)) {
        imu_bus_dma_complete_from_isr(IMU_BUS_STATUS_OK);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi != NULL) && (hspi->Instance == SPI1)) {
        imu_bus_dma_complete_from_isr(IMU_BUS_STATUS_HAL_ERROR);
    }
}

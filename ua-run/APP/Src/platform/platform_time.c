/*
 * platform_time.c —— Cortex-M7 DWT CYCCNT微秒时间实现
 *
 * 初始化时解锁DWT并开启CYCCNT。任务侧把相邻32位周期计数做无符号相减，
 * 再按SystemCoreClock/1 MHz换算成微秒；不能整除的周期余数保留到下一次，
 * 避免每个样本独立截断造成累计漂移。
 *
 * 模块只有ImuTask一个状态写者。ISR只读取CYCCNT，不修改扩展状态。
 */
#include "platform/platform_time.h"

#include <stddef.h>

#include "stm32f7xx.h"

#define PLATFORM_TIME_HZ 1000000UL
#define DWT_LOCK_ACCESS_KEY 0xC5ACCE55UL
#define DWT_LSR_LOCK_IMPLEMENTED (1UL << 0U)
#define DWT_LSR_LOCKED (1UL << 1U)

static bool time_ready;
static uint32_t cycles_per_us;
static uint32_t previous_cycles;
static uint32_t fractional_cycles;
static uint32_t elapsed_us;

bool platform_time_initialize(void)
{
    time_ready = false;
    cycles_per_us = 0U;
    previous_cycles = 0U;
    fractional_cycles = 0U;
    elapsed_us = 0U;

    if ((SystemCoreClock < PLATFORM_TIME_HZ) ||
        ((SystemCoreClock % PLATFORM_TIME_HZ) != 0U)) {
        return false;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    __DSB();
    __ISB();

    if (((DWT->LSR & DWT_LSR_LOCK_IMPLEMENTED) != 0U) &&
        ((DWT->LSR & DWT_LSR_LOCKED) != 0U)) {
        DWT->LAR = DWT_LOCK_ACCESS_KEY;
    }

    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U) {
        return false;
    }

    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        return false;
    }

    cycles_per_us = SystemCoreClock / PLATFORM_TIME_HZ;
    previous_cycles = DWT->CYCCNT;
    time_ready = true;
    __DMB();
    return true;
}

bool platform_time_ready(void)
{
    return time_ready &&
           ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
}

uint32_t platform_time_capture_cycles(void)
{
    if (!platform_time_ready()) {
        return 0U;
    }
    return DWT->CYCCNT;
}

bool platform_time_resolve_cycles(uint32_t cycle_count,
                                  uint32_t *timestamp_us)
{
    uint64_t total_cycles;
    uint32_t delta_cycles;

    if ((timestamp_us == NULL) || !platform_time_ready() ||
        (cycles_per_us == 0U)) {
        time_ready = false;
        return false;
    }

    delta_cycles = cycle_count - previous_cycles;
    previous_cycles = cycle_count;
    total_cycles = (uint64_t)fractional_cycles + delta_cycles;
    elapsed_us += (uint32_t)(total_cycles / cycles_per_us);
    fractional_cycles = (uint32_t)(total_cycles % cycles_per_us);
    *timestamp_us = elapsed_us;
    return true;
}

bool platform_time_maintain(uint32_t *timestamp_us)
{
    if (!platform_time_ready()) {
        return false;
    }
    return platform_time_resolve_cycles(DWT->CYCCNT, timestamp_us);
}

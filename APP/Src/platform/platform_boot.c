#include "platform/platform_boot.h"

#include <stdint.h>

#include "bsp/dshot_motor.h"
#include "platform/platform_diag.h"
#include "stm32f7xx.h"

#define ROM_DFU_SYSTEM_MEMORY_BASE 0x1FF00000UL
#define ROM_DFU_REQUEST_MAGIC 0x47464455UL

/* Survives NVIC_SystemReset(); the complement makes accidental entry negligible. */
static volatile uint32_t rom_dfu_request[2]
    __attribute__((section(".noinit.rom_dfu_request"), used));

void platform_boot_enter_rom_dfu_if_requested(void)
{
    uint32_t stack_pointer;
    uint32_t reset_handler;

    if ((rom_dfu_request[0] != ROM_DFU_REQUEST_MAGIC) ||
        (rom_dfu_request[1] != ~ROM_DFU_REQUEST_MAGIC)) {
        return;
    }

    rom_dfu_request[0] = 0U;
    rom_dfu_request[1] = 0U;
    __DSB();

    stack_pointer = *(const uint32_t *)ROM_DFU_SYSTEM_MEMORY_BASE;
    reset_handler = *(const uint32_t *)(ROM_DFU_SYSTEM_MEMORY_BASE + 4U);

    __disable_irq();
    SysTick->CTRL = 0U;
    SCB->VTOR = ROM_DFU_SYSTEM_MEMORY_BASE;
    __DSB();
    __ISB();
    __set_MSP(stack_pointer);
    __enable_irq();
    ((void (*)(void))reset_handler)();

    for (;;) {
    }
}

void platform_boot_request_rom_dfu(void)
{
    __disable_irq();
    dshot_motor_force_safe();
    platform_motor_outputs_force_safe();
    rom_dfu_request[0] = ROM_DFU_REQUEST_MAGIC;
    rom_dfu_request[1] = ~ROM_DFU_REQUEST_MAGIC;
    __DSB();
    NVIC_SystemReset();

    for (;;) {
    }
}

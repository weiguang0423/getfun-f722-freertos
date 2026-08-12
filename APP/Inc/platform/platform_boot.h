#ifndef PLATFORM_BOOT_H
#define PLATFORM_BOOT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Called before HAL/peripheral initialization on every application boot. */
void platform_boot_enter_rom_dfu_if_requested(void);

/* Stops all motor activity, records a one-shot request, then resets the MCU. */
void platform_boot_request_rom_dfu(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif

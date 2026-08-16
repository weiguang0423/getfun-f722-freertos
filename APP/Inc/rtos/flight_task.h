#ifndef FLIGHT_TASK_H
#define FLIGHT_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp/dshot_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLIGHT_MOTOR_TEST_TIMEOUT_MS 250U
#define FLIGHT_DSHOT_COMMAND_MAX_COUNT 8U

void flight_task_create(void);
bool flight_task_request_motor_test(
    const uint16_t values[DSHOT_MOTOR_COUNT]);
bool flight_task_execute_dshot_commands(
    uint8_t motor_index,
    const uint8_t *commands,
    uint8_t command_count,
    uint32_t timeout_ms);
uint32_t flight_task_stack_high_water_mark(void);

#ifdef __cplusplus
}
#endif

#endif

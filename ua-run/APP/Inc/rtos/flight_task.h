#ifndef FLIGHT_TASK_H
#define FLIGHT_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp/dshot_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLIGHT_MOTOR_TEST_TIMEOUT_MS 250U

void flight_task_create(void);
bool flight_task_request_motor_test(
    const uint16_t values[DSHOT_MOTOR_COUNT]);
uint32_t flight_task_stack_high_water_mark(void);

#ifdef __cplusplus
}
#endif

#endif

/* Static 50 Hz ADC3 battery monitor task. */
#ifndef BATTERY_TASK_H
#define BATTERY_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void battery_task_create(void);
uint32_t battery_task_stack_high_water_mark(void);

#ifdef __cplusplus
}
#endif

#endif

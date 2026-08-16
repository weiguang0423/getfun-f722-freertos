/* RcTask owns CRSF parsing and publishes the single coherent RC snapshot. */
#ifndef RC_TASK_H
#define RC_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void rc_task_create(void);
uint32_t rc_task_stack_high_water_mark(void);

#ifdef __cplusplus
}
#endif

#endif

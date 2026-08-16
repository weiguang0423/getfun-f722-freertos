#ifndef DSHOT_MOTOR_H
#define DSHOT_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DSHOT_MOTOR_COUNT 4U
#define DSHOT_MIN_THROTTLE_VALUE 48U
#define DSHOT_MAX_VALUE 2047U
#define DSHOT_MAX_COMMAND 47U
#define DSHOT_ALL_MOTORS 255U

typedef struct
{
    bool ready;
    bool busy;
    bool fault_latched;
    uint16_t requested_value[DSHOT_MOTOR_COUNT];
    uint16_t encoded_frame[DSHOT_MOTOR_COUNT];
    uint32_t submit_count;
    uint32_t complete_count;
    uint32_t busy_reject_count;
    uint32_t dma_error_count;
    uint32_t last_dma_flags;
} dshot_motor_diagnostics_t;

bool dshot_motor_init(void);
uint16_t dshot_encode_frame(uint16_t value, bool telemetry);
bool dshot_motor_submit(const uint16_t values[DSHOT_MOTOR_COUNT]);
bool dshot_motor_submit_command(uint8_t motor_index, uint8_t command);
void dshot_motor_force_safe(void);
void dshot_motor_get_diagnostics(dshot_motor_diagnostics_t *diagnostics);
void dshot_motor_dma_irq_handler(void);

#ifdef __cplusplus
}
#endif

#endif

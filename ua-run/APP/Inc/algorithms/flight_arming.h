/*
 * Pure S4.7 PREARM/DISARMED/ARMED/FAILSAFE state machine.
 *
 * The caller owns all hardware checks and passes a complete blocking mask plus
 * the subset that must force an already armed aircraft into FAILSAFE.  After
 * boot or FAILSAFE, ARM must be observed low with no blockers before a new ARM
 * request can succeed.
 */
#ifndef FLIGHT_ARMING_H
#define FLIGHT_ARMING_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    FLIGHT_ARMING_PREARM = 0,
    FLIGHT_ARMING_DISARMED,
    FLIGHT_ARMING_ARMED,
    FLIGHT_ARMING_FAILSAFE
} flight_arming_state_t;

typedef struct
{
    flight_arming_state_t state;
    uint32_t last_failsafe_flags;
    uint32_t arm_count;
    uint32_t disarm_count;
    uint32_t failsafe_count;
} flight_arming_t;

void flight_arming_init(flight_arming_t *arming);
bool flight_arming_update(flight_arming_t *arming,
                          bool arm_requested,
                          uint32_t blocking_flags,
                          uint32_t failsafe_flags);
bool flight_arming_is_armed(const flight_arming_t *arming);

#ifdef __cplusplus
}
#endif

#endif

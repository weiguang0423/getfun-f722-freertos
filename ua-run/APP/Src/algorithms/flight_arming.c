/*
 * S4.7 arming state transitions.  PREARM is the mandatory safe-switch
 * handshake used after every boot or FAILSAFE; it is not a configurable AUX
 * mode.  Configurable PREARM ranges remain a later Modes/parameter feature.
 */
#include "algorithms/flight_arming.h"

#include <stddef.h>
#include <string.h>

void flight_arming_init(flight_arming_t *arming)
{
    if (arming == NULL) {
        return;
    }
    memset(arming, 0, sizeof(*arming));
    arming->state = FLIGHT_ARMING_PREARM;
}

bool flight_arming_update(flight_arming_t *arming,
                          bool arm_requested,
                          uint32_t blocking_flags,
                          uint32_t failsafe_flags)
{
    if (arming == NULL) {
        return false;
    }

    switch (arming->state) {
    case FLIGHT_ARMING_PREARM:
        if (!arm_requested && (blocking_flags == 0U)) {
            arming->state = FLIGHT_ARMING_DISARMED;
        }
        break;

    case FLIGHT_ARMING_DISARMED:
        if (blocking_flags != 0U) {
            arming->state = FLIGHT_ARMING_PREARM;
        } else if (arm_requested) {
            arming->state = FLIGHT_ARMING_ARMED;
            ++arming->arm_count;
        }
        break;

    case FLIGHT_ARMING_ARMED:
        if (failsafe_flags != 0U) {
            arming->state = FLIGHT_ARMING_FAILSAFE;
            arming->last_failsafe_flags = failsafe_flags;
            ++arming->failsafe_count;
        } else if (!arm_requested) {
            arming->state = FLIGHT_ARMING_DISARMED;
            ++arming->disarm_count;
        }
        break;

    case FLIGHT_ARMING_FAILSAFE:
    default:
        if (!arm_requested) {
            arming->state = FLIGHT_ARMING_PREARM;
        }
        break;
    }

    return arming->state == FLIGHT_ARMING_ARMED;
}

bool flight_arming_is_armed(const flight_arming_t *arming)
{
    return (arming != NULL) &&
           (arming->state == FLIGHT_ARMING_ARMED);
}

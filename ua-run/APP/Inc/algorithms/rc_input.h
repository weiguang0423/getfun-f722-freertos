/* Pure RC channel mapping and loss/recovery state machine. */
#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RC_INPUT_CHANNEL_COUNT 16U
#define RC_INPUT_MAPPABLE_CHANNEL_COUNT 8U
#define RC_INPUT_TIMEOUT_MS 300U
#define RC_INPUT_RECOVERY_MS 100U
#define RC_INPUT_RECOVERY_MIN_FRAMES 5U

typedef enum
{
    RC_INPUT_PHASE_WAITING_FOR_SIGNAL = 0,
    RC_INPUT_PHASE_RECOVERING,
    RC_INPUT_PHASE_HEALTHY,
    RC_INPUT_PHASE_LOST
} rc_input_phase_t;

typedef struct
{
    rc_input_phase_t phase;
    bool frame_received;
    bool failsafe_active;
    uint32_t last_frame_ms;
    uint32_t recovery_started_ms;
    uint32_t last_failsafe_ms;
    uint32_t failsafe_count;
    uint32_t recovery_count;
    uint16_t recovery_frame_count;
} rc_input_failsafe_t;

extern const uint8_t
    rc_input_aetr_map[RC_INPUT_MAPPABLE_CHANNEL_COUNT];

void rc_input_map_aetr(
    const uint16_t raw_channel_us[RC_INPUT_CHANNEL_COUNT],
    uint16_t mapped_channel_us[RC_INPUT_CHANNEL_COUNT]);
void rc_input_set_safe_channels(
    uint16_t channel_us[RC_INPUT_CHANNEL_COUNT]);

void rc_input_failsafe_init(rc_input_failsafe_t *state);
void rc_input_failsafe_on_frame(rc_input_failsafe_t *state,
                                uint32_t now_ms);
void rc_input_failsafe_update(rc_input_failsafe_t *state,
                              uint32_t now_ms);
bool rc_input_control_valid(const rc_input_failsafe_t *state);

#ifdef __cplusplus
}
#endif

#endif

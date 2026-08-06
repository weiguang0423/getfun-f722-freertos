/* Pure ADC-to-battery conversion, filtering and low-voltage state machine. */
#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POWER_MONITOR_ADC_CHANNEL_COUNT 4U
#define POWER_MONITOR_ADC_FULL_SCALE 4095U
#define POWER_MONITOR_VREF_MV 3300U

/* GETFUNF722V3 archived Betaflight defaults; real S1.7 calibration is required. */
#define POWER_MONITOR_VOLTAGE_SCALE 110U
#define POWER_MONITOR_VOLTAGE_DIVIDER 10U
#define POWER_MONITOR_VOLTAGE_MULTIPLIER 1U
#define POWER_MONITOR_CURRENT_SCALE 100U
#define POWER_MONITOR_CURRENT_OFFSET_MA 0

#define POWER_MONITOR_SAMPLE_PERIOD_MS 20U
#define POWER_MONITOR_PRESENT_CONFIRM_MS 200U
#define POWER_MONITOR_STATE_CONFIRM_MS 1000U
#define POWER_MONITOR_PRESENT_ON_CV 100U
#define POWER_MONITOR_PRESENT_OFF_CV 80U
#define POWER_MONITOR_MAX_CELL_CV 430U
#define POWER_MONITOR_WARNING_CELL_CV 350U
#define POWER_MONITOR_CRITICAL_CELL_CV 330U
#define POWER_MONITOR_STATE_HYSTERESIS_CELL_CV 10U
#define POWER_MONITOR_MAX_CELL_COUNT 8U

typedef enum
{
    POWER_BATTERY_OK = 0,
    POWER_BATTERY_WARNING,
    POWER_BATTERY_CRITICAL,
    POWER_BATTERY_NOT_PRESENT,
    POWER_BATTERY_INIT
} power_battery_state_t;

typedef struct
{
    bool filter_initialized;
    bool present;
    uint8_t cell_count;
    power_battery_state_t state;
    power_battery_state_t candidate_state;
    uint16_t raw[POWER_MONITOR_ADC_CHANNEL_COUNT];
    uint16_t filtered_raw[POWER_MONITOR_ADC_CHANNEL_COUNT];
    uint32_t filtered_q8[POWER_MONITOR_ADC_CHANNEL_COUNT];
    uint16_t voltage_cv;
    int16_t current_ca;
    uint16_t consumed_mah;
    uint32_t present_candidate_ms;
    uint32_t absent_candidate_ms;
    uint32_t state_candidate_ms;
    uint64_t consumed_ca_ms_remainder;
    uint32_t update_count;
    uint32_t invalid_sample_count;
} power_monitor_t;

uint16_t power_monitor_raw_to_voltage_cv(uint16_t raw);
int16_t power_monitor_raw_to_current_ca(uint16_t raw);
uint8_t power_monitor_detect_cell_count(uint16_t voltage_cv);
void power_monitor_init(power_monitor_t *monitor);
bool power_monitor_update(
    power_monitor_t *monitor,
    const uint16_t raw[POWER_MONITOR_ADC_CHANNEL_COUNT],
    uint32_t elapsed_ms);
void power_monitor_mark_stale(power_monitor_t *monitor);

#ifdef __cplusplus
}
#endif

#endif

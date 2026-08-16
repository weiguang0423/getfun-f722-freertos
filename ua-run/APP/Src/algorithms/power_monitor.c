#include "algorithms/power_monitor.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define POWER_MONITOR_FILTER_SHIFT 3U
#define POWER_MONITOR_Q8_HALF 128U
#define POWER_MONITOR_CONSUMED_CA_MS_PER_MAH 360000ULL

static uint32_t saturating_add_u32(uint32_t value, uint32_t increment)
{
    if (UINT32_MAX - value < increment) {
        return UINT32_MAX;
    }
    return value + increment;
}

static power_battery_state_t classify_voltage(
    const power_monitor_t *monitor)
{
    const uint32_t voltage = monitor->voltage_cv;
    const uint32_t warning =
        (uint32_t)monitor->cell_count *
        POWER_MONITOR_WARNING_CELL_CV;
    const uint32_t critical =
        (uint32_t)monitor->cell_count *
        POWER_MONITOR_CRITICAL_CELL_CV;
    const uint32_t hysteresis =
        (uint32_t)monitor->cell_count *
        POWER_MONITOR_STATE_HYSTERESIS_CELL_CV;

    if (!monitor->present || (monitor->cell_count == 0U)) {
        return POWER_BATTERY_NOT_PRESENT;
    }

    if (monitor->state == POWER_BATTERY_CRITICAL) {
        if (voltage < critical + hysteresis) {
            return POWER_BATTERY_CRITICAL;
        }
    } else if (voltage <= critical) {
        return POWER_BATTERY_CRITICAL;
    }

    if ((monitor->state == POWER_BATTERY_WARNING) ||
        (monitor->state == POWER_BATTERY_CRITICAL)) {
        if (voltage < warning + hysteresis) {
            return POWER_BATTERY_WARNING;
        }
    } else if (voltage <= warning) {
        return POWER_BATTERY_WARNING;
    }

    return POWER_BATTERY_OK;
}

uint16_t power_monitor_raw_to_voltage_cv(uint16_t raw)
{
    const uint64_t numerator =
        (uint64_t)raw * POWER_MONITOR_VOLTAGE_SCALE *
        POWER_MONITOR_VREF_MV;
    const uint64_t denominator =
        (uint64_t)POWER_MONITOR_ADC_FULL_SCALE * 10U *
        POWER_MONITOR_VOLTAGE_DIVIDER *
        POWER_MONITOR_VOLTAGE_MULTIPLIER;
    const uint64_t value = (numerator + (denominator / 2U)) /
                           denominator;

    return value > UINT16_MAX ? UINT16_MAX : (uint16_t)value;
}

int16_t power_monitor_raw_to_current_ca(uint16_t raw)
{
    const int64_t millivolts =
        ((int64_t)raw * POWER_MONITOR_VREF_MV) / 4096;
    int64_t current_ca;

    if (POWER_MONITOR_CURRENT_SCALE == 0U) {
        return 0;
    }

    current_ca =
        ((millivolts * 10000LL /
          POWER_MONITOR_CURRENT_SCALE) +
         POWER_MONITOR_CURRENT_OFFSET_MA) /
        10LL;
    if (current_ca > INT16_MAX) {
        return INT16_MAX;
    }
    if (current_ca < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)current_ca;
}

uint8_t power_monitor_detect_cell_count(uint16_t voltage_cv)
{
    uint32_t cells;

    if (voltage_cv < POWER_MONITOR_PRESENT_ON_CV) {
        return 0U;
    }

    cells = ((uint32_t)voltage_cv +
             POWER_MONITOR_MAX_CELL_CV - 1U) /
            POWER_MONITOR_MAX_CELL_CV;
    if (cells == 0U) {
        cells = 1U;
    }
    if (cells > POWER_MONITOR_MAX_CELL_COUNT) {
        cells = POWER_MONITOR_MAX_CELL_COUNT;
    }
    return (uint8_t)cells;
}

void power_monitor_init(power_monitor_t *monitor)
{
    if (monitor == NULL) {
        return;
    }

    memset(monitor, 0, sizeof(*monitor));
    monitor->state = POWER_BATTERY_INIT;
    monitor->candidate_state = POWER_BATTERY_INIT;
}

bool power_monitor_update(
    power_monitor_t *monitor,
    const uint16_t raw[POWER_MONITOR_ADC_CHANNEL_COUNT],
    uint32_t elapsed_ms)
{
    uint8_t channel;
    power_battery_state_t classified;

    if ((monitor == NULL) || (raw == NULL) || (elapsed_ms == 0U)) {
        return false;
    }

    for (channel = 0U;
         channel < POWER_MONITOR_ADC_CHANNEL_COUNT;
         ++channel) {
        if (raw[channel] > POWER_MONITOR_ADC_FULL_SCALE) {
            monitor->invalid_sample_count++;
            return false;
        }
    }

    memcpy(monitor->raw, raw, sizeof(monitor->raw));
    if (!monitor->filter_initialized) {
        for (channel = 0U;
             channel < POWER_MONITOR_ADC_CHANNEL_COUNT;
             ++channel) {
            monitor->filtered_q8[channel] =
                (uint32_t)raw[channel] << 8U;
        }
        monitor->filter_initialized = true;
    } else {
        for (channel = 0U;
             channel < POWER_MONITOR_ADC_CHANNEL_COUNT;
             ++channel) {
            const int32_t target = (int32_t)raw[channel] << 8U;
            const int32_t current =
                (int32_t)monitor->filtered_q8[channel];

            monitor->filtered_q8[channel] =
                (uint32_t)(current +
                           ((target - current) /
                            (1L << POWER_MONITOR_FILTER_SHIFT)));
        }
    }

    for (channel = 0U;
         channel < POWER_MONITOR_ADC_CHANNEL_COUNT;
         ++channel) {
        monitor->filtered_raw[channel] =
            (uint16_t)((monitor->filtered_q8[channel] +
                        POWER_MONITOR_Q8_HALF) >>
                       8U);
    }
    monitor->voltage_cv =
        power_monitor_raw_to_voltage_cv(monitor->filtered_raw[0]);
    monitor->current_ca =
        power_monitor_raw_to_current_ca(monitor->filtered_raw[1]);
    monitor->update_count++;

    if (!monitor->present) {
        monitor->absent_candidate_ms = 0U;
        if (monitor->voltage_cv >= POWER_MONITOR_PRESENT_ON_CV) {
            monitor->present_candidate_ms = saturating_add_u32(
                monitor->present_candidate_ms,
                elapsed_ms);
            if (monitor->present_candidate_ms >=
                POWER_MONITOR_PRESENT_CONFIRM_MS) {
                monitor->present = true;
                monitor->cell_count =
                    power_monitor_detect_cell_count(
                        monitor->voltage_cv);
                monitor->state = classify_voltage(monitor);
                monitor->candidate_state = monitor->state;
                monitor->state_candidate_ms = 0U;
            }
        } else {
            monitor->present_candidate_ms = 0U;
            monitor->state = POWER_BATTERY_NOT_PRESENT;
            monitor->candidate_state = monitor->state;
        }
    } else if (monitor->voltage_cv <= POWER_MONITOR_PRESENT_OFF_CV) {
        monitor->absent_candidate_ms = saturating_add_u32(
            monitor->absent_candidate_ms,
            elapsed_ms);
        if (monitor->absent_candidate_ms >=
            POWER_MONITOR_PRESENT_CONFIRM_MS) {
            monitor->present = false;
            monitor->cell_count = 0U;
            monitor->consumed_mah = 0U;
            monitor->consumed_ca_ms_remainder = 0U;
            monitor->state = POWER_BATTERY_NOT_PRESENT;
            monitor->candidate_state = monitor->state;
            monitor->present_candidate_ms = 0U;
            monitor->state_candidate_ms = 0U;
        }
    } else {
        monitor->absent_candidate_ms = 0U;
    }

    if (monitor->present) {
        classified = classify_voltage(monitor);
        if (classified == monitor->state) {
            monitor->candidate_state = classified;
            monitor->state_candidate_ms = 0U;
        } else if (classified != monitor->candidate_state) {
            monitor->candidate_state = classified;
            monitor->state_candidate_ms = elapsed_ms;
        } else {
            monitor->state_candidate_ms = saturating_add_u32(
                monitor->state_candidate_ms,
                elapsed_ms);
            if (monitor->state_candidate_ms >=
                POWER_MONITOR_STATE_CONFIRM_MS) {
                monitor->state = classified;
                monitor->state_candidate_ms = 0U;
            }
        }

        if (monitor->current_ca > 0) {
            monitor->consumed_ca_ms_remainder +=
                (uint64_t)(uint16_t)monitor->current_ca * elapsed_ms;
            while (monitor->consumed_ca_ms_remainder >=
                   POWER_MONITOR_CONSUMED_CA_MS_PER_MAH) {
                monitor->consumed_ca_ms_remainder -=
                    POWER_MONITOR_CONSUMED_CA_MS_PER_MAH;
                if (monitor->consumed_mah < UINT16_MAX) {
                    monitor->consumed_mah++;
                }
            }
        }
    }

    return true;
}

void power_monitor_mark_stale(power_monitor_t *monitor)
{
    if (monitor == NULL) {
        return;
    }

    monitor->present = false;
    monitor->cell_count = 0U;
    monitor->state = POWER_BATTERY_INIT;
    monitor->candidate_state = POWER_BATTERY_INIT;
    monitor->present_candidate_ms = 0U;
    monitor->absent_candidate_ms = 0U;
    monitor->state_candidate_ms = 0U;
}

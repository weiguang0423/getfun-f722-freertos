/*
 * BatteryTask triggers one ADC3 DMA scan every 20 ms, filters and converts the
 * coherent four-channel sample, then atomically publishes app_state.battery.
 */
#include "rtos/battery_task.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "algorithms/power_monitor.h"
#include "app_state.h"
#include "bsp/power_adc.h"

#define BATTERY_TASK_STACK_WORDS 384U
#define BATTERY_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)
#define BATTERY_ADC_STALE_MS 100U

_Static_assert(configTICK_RATE_HZ == 1000U,
               "BatteryTask timestamps require a 1 kHz FreeRTOS tick");

static StaticTask_t battery_task_control_block;
static StackType_t battery_task_stack[BATTERY_TASK_STACK_WORDS];
static TaskHandle_t battery_task_handle;

static void publish_battery(const power_monitor_t *monitor,
                            const power_adc_snapshot_t *adc,
                            TickType_t last_sample_tick,
                            bool adc_running)
{
    app_battery_state_t battery;

    memset(&battery, 0, sizeof(battery));
    battery.adc_running = adc_running;
    battery.present = adc_running && monitor->present;
    battery.state = adc_running
                        ? monitor->state
                        : POWER_BATTERY_INIT;
    battery.cell_count = battery.present
                             ? monitor->cell_count
                             : 0U;
    battery.last_sample_tick = last_sample_tick;
    battery.voltage_cv = battery.present
                             ? monitor->voltage_cv
                             : 0U;
    battery.current_ca = battery.present
                             ? monitor->current_ca
                             : 0;
    battery.consumed_mah = battery.present
                               ? monitor->consumed_mah
                               : 0U;
    memcpy(battery.raw, monitor->raw, sizeof(battery.raw));
    memcpy(battery.filtered_raw,
           monitor->filtered_raw,
           sizeof(battery.filtered_raw));
    battery.sample_sequence = adc->sequence;
    battery.sample_count = monitor->update_count;
    battery.invalid_sample_count = monitor->invalid_sample_count;
    battery.adc_start_count = adc->start_count;
    battery.adc_busy_count = adc->busy_count;
    battery.adc_recovery_count = adc->recovery_count;
    battery.adc_dma_error_count = adc->dma_error_count;
    battery.adc_overrun_count = adc->adc_overrun_count;
    battery.adc_last_dma_flags = adc->last_dma_flags;
    app_state_publish_battery(&battery);
}

static void battery_task(void *argument)
{
    power_monitor_t monitor;
    power_adc_snapshot_t adc;
    TickType_t wake_tick;
    TickType_t last_sample_tick = 0U;
    uint32_t last_sequence = 0U;
    bool have_sample = false;
    const bool initialized = power_adc_init();

    (void)argument;
    power_monitor_init(&monitor);
    memset(&adc, 0, sizeof(adc));
    wake_tick = xTaskGetTickCount();
    power_adc_get_snapshot(&adc);
    publish_battery(&monitor, &adc, last_sample_tick, false);

    if (initialized) {
        (void)power_adc_start_conversion();
    }

    for (;;) {
        TickType_t now_tick;
        uint32_t elapsed_ms;
        bool adc_running;

        vTaskDelayUntil(&wake_tick,
                        pdMS_TO_TICKS(
                            POWER_MONITOR_SAMPLE_PERIOD_MS));
        now_tick = xTaskGetTickCount();
        power_adc_get_snapshot(&adc);

        if (adc.sequence != last_sequence) {
            elapsed_ms = have_sample
                             ? (uint32_t)(now_tick - last_sample_tick)
                             : POWER_MONITOR_SAMPLE_PERIOD_MS;
            if (power_monitor_update(&monitor,
                                     adc.raw,
                                     elapsed_ms)) {
                last_sample_tick = now_tick;
                have_sample = true;
            }
            last_sequence = adc.sequence;
        }

        adc_running = initialized && have_sample &&
                      ((TickType_t)(now_tick - last_sample_tick) <
                       pdMS_TO_TICKS(BATTERY_ADC_STALE_MS));
        if (!adc_running) {
            power_monitor_mark_stale(&monitor);
        }
        publish_battery(&monitor,
                        &adc,
                        last_sample_tick,
                        adc_running);

        if (initialized) {
            (void)power_adc_start_conversion();
        }
    }
}

void battery_task_create(void)
{
    battery_task_handle = xTaskCreateStatic(
        battery_task,
        "BatteryTask",
        BATTERY_TASK_STACK_WORDS,
        NULL,
        BATTERY_TASK_PRIORITY,
        battery_task_stack,
        &battery_task_control_block);
    configASSERT(battery_task_handle != NULL);
}

uint32_t battery_task_stack_high_water_mark(void)
{
    if (battery_task_handle == NULL) {
        return 0U;
    }
    return (uint32_t)uxTaskGetStackHighWaterMark(
        battery_task_handle);
}

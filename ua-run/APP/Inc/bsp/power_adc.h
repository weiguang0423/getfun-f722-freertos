/* ADC3 one-shot scan transport for VBAT, Current, RSSI and External ADC. */
#ifndef POWER_ADC_H
#define POWER_ADC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POWER_ADC_CHANNEL_COUNT 4U

typedef enum
{
    POWER_ADC_VBAT_INDEX = 0,
    POWER_ADC_CURRENT_INDEX,
    POWER_ADC_RSSI_INDEX,
    POWER_ADC_EXTERNAL_INDEX
} power_adc_channel_index_t;

typedef struct
{
    bool initialized;
    bool conversion_busy;
    uint16_t raw[POWER_ADC_CHANNEL_COUNT];
    uint32_t sequence;
    uint32_t start_count;
    uint32_t busy_count;
    uint32_t recovery_count;
    uint32_t dma_error_count;
    uint32_t adc_overrun_count;
    uint32_t last_dma_flags;
} power_adc_snapshot_t;

bool power_adc_init(void);
bool power_adc_start_conversion(void);
void power_adc_get_snapshot(power_adc_snapshot_t *snapshot);
void power_adc_dma_irq_handler(void);

#ifdef __cplusplus
}
#endif

#endif

/*
 * PC0..PC3 are ADC3 channels 10..13 on GETFUNF722V3.  A bounded one-shot
 * four-channel scan runs at the BatteryTask rate.  DMA2 Stream1/Channel2 is
 * used because the archived target's Stream0 option conflicts with SPI1 RX.
 */
#include "bsp/power_adc.h"

#include <stddef.h>

#include "main.h"

#define POWER_ADC_DMA_STREAM DMA2_Stream1
#define POWER_ADC_DMA_ALL_FLAGS \
    (DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 | \
     DMA_LIFCR_CTEIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1)
#define POWER_ADC_DMA_ERROR_FLAGS \
    (DMA_LISR_FEIF1 | DMA_LISR_DMEIF1 | DMA_LISR_TEIF1)
#define POWER_ADC_BUSY_RECOVERY_ATTEMPTS 3U
#define POWER_ADC_DISABLE_SPIN_LIMIT 10000U

static volatile uint16_t dma_samples[POWER_ADC_CHANNEL_COUNT];
static volatile power_adc_snapshot_t adc_snapshot;
static uint32_t consecutive_busy_count;

static uint32_t irq_lock(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void irq_unlock(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static bool disable_dma_stream(void)
{
    uint32_t spin = POWER_ADC_DISABLE_SPIN_LIMIT;

    POWER_ADC_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    while (((POWER_ADC_DMA_STREAM->CR & DMA_SxCR_EN) != 0U) &&
           (spin != 0U)) {
        spin--;
    }
    return (POWER_ADC_DMA_STREAM->CR & DMA_SxCR_EN) == 0U;
}

static void configure_gpio(void)
{
    const uint32_t pin_mask =
        (3UL << GPIO_MODER_MODER0_Pos) |
        (3UL << GPIO_MODER_MODER1_Pos) |
        (3UL << GPIO_MODER_MODER2_Pos) |
        (3UL << GPIO_MODER_MODER3_Pos);
    const uint32_t pull_mask =
        (3UL << GPIO_PUPDR_PUPDR0_Pos) |
        (3UL << GPIO_PUPDR_PUPDR1_Pos) |
        (3UL << GPIO_PUPDR_PUPDR2_Pos) |
        (3UL << GPIO_PUPDR_PUPDR3_Pos);

    GPIOC->MODER = (GPIOC->MODER & ~pin_mask) | pin_mask;
    GPIOC->PUPDR &= ~pull_mask;
}

static void configure_adc(void)
{
    ADC3->CR1 = ADC_CR1_SCAN;
    /*
     * Keep the ADC DMA request path enabled between software-triggered scan
     * sequences.  With DDS cleared, STM32F7 stops issuing DMA requests after
     * the first completed DMA transfer; merely OR-ing DMA again does not
     * create the required disable/enable edge.  CONT remains cleared, so each
     * BatteryTask request is still one bounded four-channel scan.
     */
    ADC3->CR2 = ADC_CR2_DMA | ADC_CR2_DDS;
    ADC3->SMPR1 =
        (7UL << ADC_SMPR1_SMP10_Pos) |
        (7UL << ADC_SMPR1_SMP11_Pos) |
        (7UL << ADC_SMPR1_SMP12_Pos) |
        (7UL << ADC_SMPR1_SMP13_Pos);
    ADC3->SMPR2 = 0U;
    ADC3->SQR1 = 3UL << ADC_SQR1_L_Pos;
    ADC3->SQR2 = 0U;
    ADC3->SQR3 =
        (10UL << 0U) |
        (11UL << 5U) |
        (12UL << 10U) |
        (13UL << 15U);
    ADC3->SR = 0U;
    ADC3->CR2 |= ADC_CR2_ADON;
}

static void configure_dma(void)
{
    DMA2->LIFCR = POWER_ADC_DMA_ALL_FLAGS;
    POWER_ADC_DMA_STREAM->CR =
        DMA_SxCR_CHSEL_1 |
        DMA_SxCR_PL_0 |
        DMA_SxCR_MSIZE_0 |
        DMA_SxCR_PSIZE_0 |
        DMA_SxCR_MINC |
        DMA_SxCR_TCIE |
        DMA_SxCR_TEIE |
        DMA_SxCR_DMEIE;
    POWER_ADC_DMA_STREAM->NDTR = POWER_ADC_CHANNEL_COUNT;
    POWER_ADC_DMA_STREAM->PAR = (uint32_t)&ADC3->DR;
    POWER_ADC_DMA_STREAM->M0AR = (uint32_t)dma_samples;
    POWER_ADC_DMA_STREAM->FCR = 0U;
}

bool power_adc_init(void)
{
    uint32_t channel;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    __DSB();

    if (!disable_dma_stream()) {
        return false;
    }

    for (channel = 0U; channel < POWER_ADC_CHANNEL_COUNT; ++channel) {
        dma_samples[channel] = 0U;
        adc_snapshot.raw[channel] = 0U;
    }
    adc_snapshot.sequence = 0U;
    adc_snapshot.start_count = 0U;
    adc_snapshot.busy_count = 0U;
    adc_snapshot.recovery_count = 0U;
    adc_snapshot.dma_error_count = 0U;
    adc_snapshot.adc_overrun_count = 0U;
    adc_snapshot.last_dma_flags = 0U;
    adc_snapshot.conversion_busy = false;
    consecutive_busy_count = 0U;

    configure_gpio();
    ADC123_COMMON->CCR =
        (ADC123_COMMON->CCR & ~ADC_CCR_ADCPRE) |
        ADC_CCR_ADCPRE_0;
    configure_adc();
    configure_dma();

    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    adc_snapshot.initialized = true;
    return true;
}

bool power_adc_start_conversion(void)
{
    uint32_t primask;

    if (!adc_snapshot.initialized) {
        return false;
    }

    primask = irq_lock();
    if (adc_snapshot.conversion_busy) {
        adc_snapshot.busy_count++;
        consecutive_busy_count++;
        if (consecutive_busy_count <
            POWER_ADC_BUSY_RECOVERY_ATTEMPTS) {
            irq_unlock(primask);
            return false;
        }

        ADC3->CR2 &= ~ADC_CR2_DMA;
        if (!disable_dma_stream()) {
            adc_snapshot.dma_error_count++;
            irq_unlock(primask);
            return false;
        }
        DMA2->LIFCR = POWER_ADC_DMA_ALL_FLAGS;
        adc_snapshot.conversion_busy = false;
        adc_snapshot.recovery_count++;
    }

    consecutive_busy_count = 0U;
    if (!disable_dma_stream()) {
        adc_snapshot.dma_error_count++;
        irq_unlock(primask);
        return false;
    }
    if ((ADC3->SR & ADC_SR_OVR) != 0U) {
        adc_snapshot.adc_overrun_count++;
    }
    ADC3->SR = 0U;
    DMA2->LIFCR = POWER_ADC_DMA_ALL_FLAGS;
    POWER_ADC_DMA_STREAM->NDTR = POWER_ADC_CHANNEL_COUNT;
    POWER_ADC_DMA_STREAM->M0AR = (uint32_t)dma_samples;
    POWER_ADC_DMA_STREAM->CR |= DMA_SxCR_EN;
    adc_snapshot.conversion_busy = true;
    adc_snapshot.start_count++;
    ADC3->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;
    ADC3->CR2 |= ADC_CR2_SWSTART;
    irq_unlock(primask);
    return true;
}

void power_adc_get_snapshot(power_adc_snapshot_t *snapshot)
{
    uint32_t primask;
    uint32_t channel;

    if (snapshot == NULL) {
        return;
    }

    primask = irq_lock();
    snapshot->initialized = adc_snapshot.initialized;
    snapshot->conversion_busy = adc_snapshot.conversion_busy;
    for (channel = 0U; channel < POWER_ADC_CHANNEL_COUNT; ++channel) {
        snapshot->raw[channel] = adc_snapshot.raw[channel];
    }
    snapshot->sequence = adc_snapshot.sequence;
    snapshot->start_count = adc_snapshot.start_count;
    snapshot->busy_count = adc_snapshot.busy_count;
    snapshot->recovery_count = adc_snapshot.recovery_count;
    snapshot->dma_error_count = adc_snapshot.dma_error_count;
    snapshot->adc_overrun_count = adc_snapshot.adc_overrun_count;
    snapshot->last_dma_flags = adc_snapshot.last_dma_flags;
    irq_unlock(primask);
}

void power_adc_dma_irq_handler(void)
{
    const uint32_t flags = DMA2->LISR;
    uint32_t channel;

    DMA2->LIFCR = POWER_ADC_DMA_ALL_FLAGS;
    POWER_ADC_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    if ((flags & POWER_ADC_DMA_ERROR_FLAGS) != 0U) {
        adc_snapshot.last_dma_flags =
            flags & POWER_ADC_DMA_ERROR_FLAGS;
        adc_snapshot.dma_error_count++;
        adc_snapshot.conversion_busy = false;
        return;
    }

    if ((flags & DMA_LISR_TCIF1) != 0U) {
        __DMB();
        for (channel = 0U;
             channel < POWER_ADC_CHANNEL_COUNT;
             ++channel) {
            adc_snapshot.raw[channel] = dma_samples[channel];
        }
        __DMB();
        adc_snapshot.sequence++;
        adc_snapshot.conversion_busy = false;
    }
}

/*
 * Four-output DShot600 transport for GETFUN F722 V3.
 *
 * This follows Betaflight's default STM32F7 DShot bitbang path: TIM8_CH1 is
 * only a DMA pacer, and DMA2 Stream2/Channel7 writes all four GPIOA outputs
 * through GPIOA->BSRR. M1..M4 are PA15/PA10/PA9/PA8 respectively.
 */
#include "bsp/dshot_motor.h"

#include <string.h>

#include "main.h"

#define DSHOT_BIT_RATE_HZ 600000UL
#define DSHOT_STATES_PER_BIT 3U
#define DSHOT_FRAME_BITS 16U
#define DSHOT_HOLD_STATES 3U
#define DSHOT_DMA_WORD_COUNT \
    ((DSHOT_FRAME_BITS * DSHOT_STATES_PER_BIT) + DSHOT_HOLD_STATES)

#define DSHOT_PACER_TIMER_CLOCK_HZ 216000000UL
#define DSHOT_PACER_HZ (DSHOT_BIT_RATE_HZ * DSHOT_STATES_PER_BIT)
#define DSHOT_PACER_PERIOD_TICKS \
    (DSHOT_PACER_TIMER_CLOCK_HZ / DSHOT_PACER_HZ)
#define DSHOT_PACER_COMPARE_TICKS 10U

#define DSHOT_DMA_STREAM DMA2_Stream2
#define DSHOT_DMA_CHANNEL 7UL
#define DSHOT_DMA_ALL_FLAGS \
    (DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2 | \
     DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2)
#define DSHOT_DMA_ERROR_FLAGS (DMA_LISR_DMEIF2 | DMA_LISR_TEIF2)

#define DSHOT_MOTOR_PIN_MASK \
    ((uint32_t)(MOTOR1_Pin | MOTOR2_Pin | MOTOR3_Pin | MOTOR4_Pin))
#define DSHOT_MOTOR_PIN_RESET_MASK (DSHOT_MOTOR_PIN_MASK << 16U)

_Static_assert((DSHOT_PACER_TIMER_CLOCK_HZ % DSHOT_PACER_HZ) == 0U,
               "TIM8 must divide exactly to Betaflight DShot600 pacing");
_Static_assert(DSHOT_PACER_PERIOD_TICKS == 120U,
               "TIM8 DShot600 pacer period changed");
_Static_assert(DSHOT_DMA_WORD_COUNT == 51U,
               "Betaflight DShot bitbang frame length changed");

static const uint16_t motor_pins[DSHOT_MOTOR_COUNT] = {
    MOTOR1_Pin, MOTOR2_Pin, MOTOR3_Pin, MOTOR4_Pin
};
static uint32_t dma_values[DSHOT_DMA_WORD_COUNT];
static dshot_motor_diagnostics_t diagnostics;

static bool disable_stream(void)
{
    uint32_t attempts = 100000U;

    DSHOT_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    while (((DSHOT_DMA_STREAM->CR & DMA_SxCR_EN) != 0U) &&
           (attempts != 0U)) {
        --attempts;
    }
    return (DSHOT_DMA_STREAM->CR & DMA_SxCR_EN) == 0U;
}

static void configure_motor_pins(void)
{
    GPIO_InitTypeDef gpio = {0};

    GPIOA->BSRR = DSHOT_MOTOR_PIN_RESET_MASK;
    gpio.Pin = (uint32_t)DSHOT_MOTOR_PIN_MASK;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    GPIOA->BSRR = DSHOT_MOTOR_PIN_RESET_MASK;
}

static bool motor_pins_are_outputs(void)
{
    static const uint32_t pin_numbers[DSHOT_MOTOR_COUNT] = {
        15U, 10U, 9U, 8U
    };
    uint32_t index;

    for (index = 0U; index < DSHOT_MOTOR_COUNT; ++index) {
        if (((GPIOA->MODER >> (pin_numbers[index] * 2U)) & 3U) != 1U) {
            return false;
        }
    }
    return true;
}

static void configure_pacer_timer(void)
{
    TIM8->CR1 = 0U;
    TIM8->CR2 = 0U;
    TIM8->SMCR = 0U;
    TIM8->DIER = 0U;
    TIM8->CCER = 0U;
    TIM8->PSC = 0U;
    TIM8->ARR = DSHOT_PACER_PERIOD_TICKS - 1U;
    TIM8->RCR = 0U;
    TIM8->CCR1 = DSHOT_PACER_COMPARE_TICKS;
    TIM8->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC1PE;
    TIM8->CCER = TIM_CCER_CC1E;
    TIM8->BDTR = TIM_BDTR_MOE;
    TIM8->EGR = TIM_EGR_UG;
    TIM8->SR = 0U;
    TIM8->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void configure_dma(void)
{
    (void)disable_stream();
    DMA2->LIFCR = DSHOT_DMA_ALL_FLAGS;

    DSHOT_DMA_STREAM->PAR = (uint32_t)&GPIOA->BSRR;
    DSHOT_DMA_STREAM->M0AR = (uint32_t)dma_values;
    DSHOT_DMA_STREAM->CR =
        (DSHOT_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_DIR_0 | DMA_SxCR_MINC |
        DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1 |
        DMA_SxCR_PL_0 | DMA_SxCR_PL_1 |
        DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    DSHOT_DMA_STREAM->FCR = DMA_SxFCR_DMDIS;
}

uint16_t dshot_encode_frame(uint16_t value, bool telemetry)
{
    uint16_t payload;
    uint16_t checksum;

    if (value > DSHOT_MAX_VALUE) {
        value = DSHOT_MAX_VALUE;
    }
    payload = (uint16_t)((value << 1U) | (telemetry ? 1U : 0U));
    checksum = (uint16_t)(payload ^ (payload >> 4U) ^ (payload >> 8U));
    return (uint16_t)((payload << 4U) | (checksum & 0x0FU));
}

static void build_dma_values(const uint16_t frames[DSHOT_MOTOR_COUNT])
{
    uint32_t bit;
    uint32_t motor;

    for (bit = 0U; bit < DSHOT_FRAME_BITS; ++bit) {
        const uint32_t base = bit * DSHOT_STATES_PER_BIT;
        const uint16_t bit_mask = (uint16_t)(1U << (15U - bit));

        dma_values[base] = DSHOT_MOTOR_PIN_MASK;
        dma_values[base + 1U] = 0U;
        dma_values[base + 2U] = DSHOT_MOTOR_PIN_RESET_MASK;
        for (motor = 0U; motor < DSHOT_MOTOR_COUNT; ++motor) {
            if ((frames[motor] & bit_mask) == 0U) {
                dma_values[base + 1U] |=
                    (uint32_t)motor_pins[motor] << 16U;
            }
        }
    }

    dma_values[DSHOT_FRAME_BITS * DSHOT_STATES_PER_BIT] =
        DSHOT_MOTOR_PIN_RESET_MASK;
    dma_values[(DSHOT_FRAME_BITS * DSHOT_STATES_PER_BIT) + 1U] = 0U;
    dma_values[(DSHOT_FRAME_BITS * DSHOT_STATES_PER_BIT) + 2U] = 0U;
}

bool dshot_motor_init(void)
{
    memset(&diagnostics, 0, sizeof(diagnostics));
    memset(dma_values, 0, sizeof(dma_values));
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    configure_motor_pins();
    configure_pacer_timer();
    configure_dma();
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    diagnostics.ready = motor_pins_are_outputs();
    if (!diagnostics.ready) {
        dshot_motor_force_safe();
    }
    return diagnostics.ready;
}

bool dshot_motor_submit(const uint16_t values[DSHOT_MOTOR_COUNT])
{
    uint16_t frames[DSHOT_MOTOR_COUNT];
    uint32_t motor;

    if ((values == NULL) || !diagnostics.ready ||
        diagnostics.fault_latched || !motor_pins_are_outputs()) {
        dshot_motor_force_safe();
        return false;
    }
    if (diagnostics.busy) {
        ++diagnostics.busy_reject_count;
        return false;
    }
    for (motor = 0U; motor < DSHOT_MOTOR_COUNT; ++motor) {
        if ((values[motor] > DSHOT_MAX_VALUE) ||
            ((values[motor] != 0U) &&
             (values[motor] < DSHOT_MIN_THROTTLE_VALUE))) {
            return false;
        }
        frames[motor] = dshot_encode_frame(values[motor], false);
    }

    if (!disable_stream()) {
        dshot_motor_force_safe();
        return false;
    }
    build_dma_values(frames);
    memcpy(diagnostics.requested_value, values,
           sizeof(diagnostics.requested_value));
    memcpy(diagnostics.encoded_frame, frames,
           sizeof(diagnostics.encoded_frame));

    TIM8->DIER &= ~TIM_DIER_CC1DE;
    DMA2->LIFCR = DSHOT_DMA_ALL_FLAGS;
    DSHOT_DMA_STREAM->M0AR = (uint32_t)dma_values;
    DSHOT_DMA_STREAM->NDTR = DSHOT_DMA_WORD_COUNT;
    GPIOA->BSRR = DSHOT_MOTOR_PIN_RESET_MASK;
    diagnostics.busy = true;
    ++diagnostics.submit_count;
    __DMB();
    DSHOT_DMA_STREAM->CR |= DMA_SxCR_EN;
    TIM8->DIER |= TIM_DIER_CC1DE;
    return true;
}

void dshot_motor_force_safe(void)
{
    uint32_t motor;

    TIM8->DIER &= ~TIM_DIER_CC1DE;
    (void)disable_stream();
    GPIOA->BSRR = DSHOT_MOTOR_PIN_RESET_MASK;
    configure_motor_pins();
    diagnostics.ready = false;
    diagnostics.busy = false;
    diagnostics.fault_latched = true;
    for (motor = 0U; motor < DSHOT_MOTOR_COUNT; ++motor) {
        diagnostics.requested_value[motor] = 0U;
    }
}

void dshot_motor_get_diagnostics(dshot_motor_diagnostics_t *destination)
{
    uint32_t primask;

    if (destination == NULL) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    *destination = diagnostics;
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
}

void dshot_motor_dma_irq_handler(void)
{
    const uint32_t flags = DMA2->LISR &
        (DMA_LISR_FEIF2 | DMA_LISR_DMEIF2 | DMA_LISR_TEIF2 |
         DMA_LISR_HTIF2 | DMA_LISR_TCIF2);

    if (flags == 0U) {
        return;
    }
    DMA2->LIFCR = flags;
    if ((flags & DSHOT_DMA_ERROR_FLAGS) != 0U) {
        diagnostics.last_dma_flags = flags;
        ++diagnostics.dma_error_count;
        dshot_motor_force_safe();
        return;
    }
    if ((flags & DMA_LISR_TCIF2) != 0U) {
        TIM8->DIER &= ~TIM_DIER_CC1DE;
        DSHOT_DMA_STREAM->CR &= ~DMA_SxCR_EN;
        GPIOA->BSRR = DSHOT_MOTOR_PIN_RESET_MASK;
        diagnostics.busy = false;
        ++diagnostics.complete_count;
    }
}

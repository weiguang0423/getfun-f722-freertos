/*
 * Four-output DShot300 transport for GETFUN F722 V3.
 *
 * M1 uses PA15/TIM2_CH1 and TIM2_UP on DMA1 Stream1/Channel3. M2..M4
 * use PA10/PA9/PA8 (TIM1_CH3/CH2/CH1) and one TIM1 DMA burst on
 * DMA2 Stream5/Channel6. The .ioc keeps these pins as safe GPIO outputs;
 * this application-owned driver switches only M1..M4 to AF1 at runtime.
 */
#include "bsp/dshot_motor.h"

#include <string.h>

#include "main.h"
#include "stm32f7xx_hal.h"

#define DSHOT_BIT_RATE_HZ 300000UL
#define DSHOT_FRAME_BITS 16U
#define DSHOT_RESET_SLOTS 2U
#define DSHOT_DMA_UPDATE_COUNT (DSHOT_FRAME_BITS + DSHOT_RESET_SLOTS)
#define DSHOT_DMA_BURST_LENGTH 4U

#define TIM1_CLOCK_HZ 216000000UL
#define TIM2_CLOCK_HZ 108000000UL
#define TIM1_PERIOD_TICKS (TIM1_CLOCK_HZ / DSHOT_BIT_RATE_HZ)
#define TIM2_PERIOD_TICKS (TIM2_CLOCK_HZ / DSHOT_BIT_RATE_HZ)
#define TIM1_DUTY_ZERO_TICKS ((TIM1_PERIOD_TICKS * 3U) / 8U)
#define TIM1_DUTY_ONE_TICKS ((TIM1_PERIOD_TICKS * 3U) / 4U)
#define TIM2_DUTY_ZERO_TICKS ((TIM2_PERIOD_TICKS * 3U) / 8U)
#define TIM2_DUTY_ONE_TICKS ((TIM2_PERIOD_TICKS * 3U) / 4U)

#define TIM1_DMA_STREAM DMA2_Stream5
#define TIM2_DMA_STREAM DMA1_Stream1
#define TIM1_DMA_CHANNEL 6UL
#define TIM2_DMA_CHANNEL 3UL

#define TIM1_DMA_ALL_FLAGS \
    (DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | \
     DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5)
#define TIM2_DMA_ALL_FLAGS \
    (DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CTEIF1 | \
     DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1)
#define TIM1_DMA_ERROR_FLAGS \
    (DMA_HISR_FEIF5 | DMA_HISR_DMEIF5 | DMA_HISR_TEIF5)
#define TIM2_DMA_ERROR_FLAGS \
    (DMA_LISR_FEIF1 | DMA_LISR_DMEIF1 | DMA_LISR_TEIF1)

_Static_assert((TIM1_CLOCK_HZ % DSHOT_BIT_RATE_HZ) == 0U,
               "TIM1 must divide exactly to DShot300");
_Static_assert((TIM2_CLOCK_HZ % DSHOT_BIT_RATE_HZ) == 0U,
               "TIM2 must divide exactly to DShot300");
_Static_assert(TIM1_PERIOD_TICKS == 720U,
               "TIM1 DShot300 period changed");
_Static_assert(TIM2_PERIOD_TICKS == 360U,
               "TIM2 DShot300 period changed");

static uint32_t tim1_dma_values[DSHOT_DMA_UPDATE_COUNT]
                                [DSHOT_DMA_BURST_LENGTH];
static uint32_t tim2_dma_values[DSHOT_DMA_UPDATE_COUNT]
                                [DSHOT_DMA_BURST_LENGTH];
static dshot_motor_diagnostics_t diagnostics;
static volatile bool tim1_transfer_complete;
static volatile bool tim2_transfer_complete;

static bool disable_stream(DMA_Stream_TypeDef *stream)
{
    uint32_t attempts = 100000U;

    stream->CR &= ~DMA_SxCR_EN;
    while (((stream->CR & DMA_SxCR_EN) != 0U) && (attempts != 0U)) {
        --attempts;
    }
    return (stream->CR & DMA_SxCR_EN) == 0U;
}

static void configure_motor_pins_af(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = MOTOR1_Pin | MOTOR2_Pin | MOTOR3_Pin | MOTOR4_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static bool motor_pins_are_af(void)
{
    const uint32_t pins[] = {15U, 10U, 9U, 8U};
    uint32_t index;

    for (index = 0U; index < DSHOT_MOTOR_COUNT; ++index) {
        if (((GPIOA->MODER >> (pins[index] * 2U)) & 3U) != 2U) {
            return false;
        }
    }
    return true;
}

static void configure_timers(void)
{
    TIM1->CR1 = 0U;
    TIM1->CR2 = 0U;
    TIM1->SMCR = 0U;
    TIM1->DIER = 0U;
    TIM1->CCER = 0U;
    TIM1->PSC = 0U;
    TIM1->ARR = TIM1_PERIOD_TICKS - 1U;
    TIM1->RCR = 0U;
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM1->CCR4 = 0U;
    TIM1->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS;
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;

    TIM2->CR1 = 0U;
    TIM2->CR2 = 0U;
    TIM2->SMCR = 0U;
    TIM2->DIER = 0U;
    TIM2->CCER = 0U;
    TIM2->PSC = 0U;
    TIM2->ARR = TIM2_PERIOD_TICKS - 1U;
    TIM2->CCR1 = 0U;
    TIM2->CCR2 = 0U;
    TIM2->CCR3 = 0U;
    TIM2->CCR4 = 0U;
    TIM2->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    TIM2->CCER = TIM_CCER_CC1E;
    TIM2->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
}

static void configure_dma(void)
{
    (void)disable_stream(TIM1_DMA_STREAM);
    (void)disable_stream(TIM2_DMA_STREAM);
    DMA2->HIFCR = TIM1_DMA_ALL_FLAGS;
    DMA1->LIFCR = TIM2_DMA_ALL_FLAGS;

    TIM1_DMA_STREAM->PAR = (uint32_t)&TIM1->DMAR;
    TIM1_DMA_STREAM->CR =
        (TIM1_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_PSIZE_1 |
        DMA_SxCR_MSIZE_1 | DMA_SxCR_PL_1 |
        DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    TIM1_DMA_STREAM->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH |
                           DMA_SxFCR_FEIE;

    TIM2_DMA_STREAM->PAR = (uint32_t)&TIM2->DMAR;
    TIM2_DMA_STREAM->CR =
        (TIM2_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_PSIZE_1 |
        DMA_SxCR_MSIZE_1 | DMA_SxCR_PL_1 |
        DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    TIM2_DMA_STREAM->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH |
                           DMA_SxFCR_FEIE;
}

static void force_gpio_low(void)
{
    GPIO_InitTypeDef gpio = {0};

    TIM1->DIER &= ~TIM_DIER_UDE;
    TIM2->DIER &= ~TIM_DIER_UDE;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    (void)disable_stream(TIM1_DMA_STREAM);
    (void)disable_stream(TIM2_DMA_STREAM);
    GPIOA->BSRR = (uint32_t)(MOTOR1_Pin | MOTOR2_Pin | MOTOR3_Pin |
                             MOTOR4_Pin) << 16U;
    gpio.Pin = MOTOR1_Pin | MOTOR2_Pin | MOTOR3_Pin | MOTOR4_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
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

static uint16_t duty_for_bit(uint16_t frame,
                             uint32_t bit,
                             uint16_t zero_ticks,
                             uint16_t one_ticks)
{
    return ((frame & (uint16_t)(1U << (15U - bit))) != 0U)
               ? one_ticks
               : zero_ticks;
}

static void build_dma_values(const uint16_t frames[DSHOT_MOTOR_COUNT])
{
    uint32_t update;

    for (update = 0U; update < DSHOT_DMA_UPDATE_COUNT; ++update) {
        const uint32_t slot = update + 1U;

        if (slot < DSHOT_FRAME_BITS) {
            tim2_dma_values[update][0] = duty_for_bit(
                frames[0], slot, TIM2_DUTY_ZERO_TICKS,
                TIM2_DUTY_ONE_TICKS);
            tim1_dma_values[update][0] = duty_for_bit(
                frames[3], slot, TIM1_DUTY_ZERO_TICKS,
                TIM1_DUTY_ONE_TICKS);
            tim1_dma_values[update][1] = duty_for_bit(
                frames[2], slot, TIM1_DUTY_ZERO_TICKS,
                TIM1_DUTY_ONE_TICKS);
            tim1_dma_values[update][2] = duty_for_bit(
                frames[1], slot, TIM1_DUTY_ZERO_TICKS,
                TIM1_DUTY_ONE_TICKS);
        } else {
            tim2_dma_values[update][0] = 0U;
            tim1_dma_values[update][0] = 0U;
            tim1_dma_values[update][1] = 0U;
            tim1_dma_values[update][2] = 0U;
        }
        tim1_dma_values[update][3] = 0U;
        tim2_dma_values[update][1] = 0U;
        tim2_dma_values[update][2] = 0U;
        tim2_dma_values[update][3] = 0U;
    }
}

bool dshot_motor_init(void)
{
    memset(&diagnostics, 0, sizeof(diagnostics));
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    configure_motor_pins_af();
    configure_timers();
    configure_dma();
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    diagnostics.ready = motor_pins_are_af();
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
        diagnostics.fault_latched || !motor_pins_are_af()) {
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

    if (!disable_stream(TIM1_DMA_STREAM) ||
        !disable_stream(TIM2_DMA_STREAM)) {
        dshot_motor_force_safe();
        return false;
    }
    build_dma_values(frames);
    memcpy(diagnostics.requested_value, values,
           sizeof(diagnostics.requested_value));
    memcpy(diagnostics.encoded_frame, frames,
           sizeof(diagnostics.encoded_frame));

    TIM1->CCR1 = duty_for_bit(frames[3], 0U,
                              TIM1_DUTY_ZERO_TICKS,
                              TIM1_DUTY_ONE_TICKS);
    TIM1->CCR2 = duty_for_bit(frames[2], 0U,
                              TIM1_DUTY_ZERO_TICKS,
                              TIM1_DUTY_ONE_TICKS);
    TIM1->CCR3 = duty_for_bit(frames[1], 0U,
                              TIM1_DUTY_ZERO_TICKS,
                              TIM1_DUTY_ONE_TICKS);
    TIM2->CCR1 = duty_for_bit(frames[0], 0U,
                              TIM2_DUTY_ZERO_TICKS,
                              TIM2_DUTY_ONE_TICKS);

    DMA2->HIFCR = TIM1_DMA_ALL_FLAGS;
    DMA1->LIFCR = TIM2_DMA_ALL_FLAGS;
    TIM1_DMA_STREAM->M0AR = (uint32_t)tim1_dma_values;
    TIM1_DMA_STREAM->NDTR =
        DSHOT_DMA_UPDATE_COUNT * DSHOT_DMA_BURST_LENGTH;
    TIM2_DMA_STREAM->M0AR = (uint32_t)tim2_dma_values;
    TIM2_DMA_STREAM->NDTR =
        DSHOT_DMA_UPDATE_COUNT * DSHOT_DMA_BURST_LENGTH;
    TIM1->CNT = 0U;
    TIM2->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM2->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;
    TIM2->SR = 0U;
    tim1_transfer_complete = false;
    tim2_transfer_complete = false;
    diagnostics.busy = true;
    ++diagnostics.submit_count;
    __DMB();
    TIM1_DMA_STREAM->CR |= DMA_SxCR_EN;
    TIM2_DMA_STREAM->CR |= DMA_SxCR_EN;
    TIM1->DIER |= TIM_DIER_UDE;
    TIM2->DIER |= TIM_DIER_UDE;
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM1->CR1 |= TIM_CR1_CEN;
    return true;
}

void dshot_motor_force_safe(void)
{
    uint32_t motor;

    force_gpio_low();
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
    const uint32_t tim1_flags = DMA2->HISR &
        (DMA_HISR_FEIF5 | DMA_HISR_DMEIF5 | DMA_HISR_TEIF5 |
         DMA_HISR_HTIF5 | DMA_HISR_TCIF5);
    const uint32_t tim2_flags = DMA1->LISR &
        (DMA_LISR_FEIF1 | DMA_LISR_DMEIF1 | DMA_LISR_TEIF1 |
         DMA_LISR_HTIF1 | DMA_LISR_TCIF1);

    if (tim1_flags != 0U) {
        DMA2->HIFCR = tim1_flags;
        if ((tim1_flags & TIM1_DMA_ERROR_FLAGS) != 0U) {
            diagnostics.last_tim1_dma_flags = tim1_flags;
            ++diagnostics.dma_error_count;
            dshot_motor_force_safe();
            return;
        }
        if ((tim1_flags & DMA_HISR_TCIF5) != 0U) {
            TIM1->DIER &= ~TIM_DIER_UDE;
            TIM1->CR1 &= ~TIM_CR1_CEN;
            TIM1_DMA_STREAM->CR &= ~DMA_SxCR_EN;
            tim1_transfer_complete = true;
        }
    }
    if (tim2_flags != 0U) {
        DMA1->LIFCR = tim2_flags;
        if ((tim2_flags & TIM2_DMA_ERROR_FLAGS) != 0U) {
            diagnostics.last_tim2_dma_flags = tim2_flags;
            ++diagnostics.dma_error_count;
            dshot_motor_force_safe();
            return;
        }
        if ((tim2_flags & DMA_LISR_TCIF1) != 0U) {
            TIM2->DIER &= ~TIM_DIER_UDE;
            TIM2->CR1 &= ~TIM_CR1_CEN;
            TIM2_DMA_STREAM->CR &= ~DMA_SxCR_EN;
            tim2_transfer_complete = true;
        }
    }
    if (tim1_transfer_complete && tim2_transfer_complete &&
        diagnostics.busy) {
        TIM1->CCR1 = 0U;
        TIM1->CCR2 = 0U;
        TIM1->CCR3 = 0U;
        TIM1->CCR4 = 0U;
        TIM2->CCR1 = 0U;
        TIM2->CCR2 = 0U;
        TIM2->CCR3 = 0U;
        TIM2->CCR4 = 0U;
        diagnostics.busy = false;
        ++diagnostics.complete_count;
    }
}

#include "config.hpp"
#if ENCODER_TYPE == ABZ

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_gpio.h>

#include "ABZ.hpp"
#include "configs/encoder.hpp"
#include "debug.hpp"
#include "error.hpp"

static TIM_HandleTypeDef timer;

static void gpio_init(void);
static void timer_init(void);
static void z_irq(void* args);

static volatile int64_t rollover_count = 0;
static volatile uint32_t z_pulses = 0;
static volatile int32_t delta = 0;
static volatile int32_t count_offset = 0;

int64_t encoder::get_count(void) {
    debug::trace("count offset: %i", count_offset);
    return ENCODER_POLARITY *
               ((int64_t)__HAL_TIM_GetCounter(&timer) +
                rollover_count * ((int64_t)ENCODER_TIMER_PERIOD + 1)) +
           count_offset;
}

void encoder::init(void) {
    debug::trace("Encoder ABZ: Initialising encoder timer.");
    timer_init();
    debug::trace("Encoder ABZ: Initialising encoder GPIO.");
    gpio_init();
    debug::trace("Encoder ABZ: Initialised encoder.");
}

void encoder::timer_irq(void) { HAL_TIM_IRQHandler(&timer); }

void encoder::rollover_irq(void) {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&timer)) {
        rollover_count -= 1;
    } else {
        rollover_count += 1;
    }
}

uint32_t encoder::get_z_pulses(void) { return z_pulses; }

int64_t encoder::get_rollovers(void) { return rollover_count; }

int32_t encoder::get_delta(void) { return delta; }

void encoder::set_count(int32_t count) {
    count_offset = 0;
    count_offset = count - get_count();
    return;
}

static void z_irq([[maybe_unused]] void* args) {
    z_pulses += 1;

    int64_t current_count =
        ENCODER_POLARITY * (encoder::get_count() - count_offset);
    // integer division always rounds towards zero, values that should get
    // rounded up/down are pushed over the next threshold
    int64_t corrected_count = 0;
    if (current_count >= 0) {
        corrected_count =
            ((current_count + ENCODER_RESOLUTION / 2) / ENCODER_RESOLUTION) *
            ENCODER_RESOLUTION;
    } else {
        corrected_count =
            ((current_count - ENCODER_RESOLUTION / 2) / ENCODER_RESOLUTION) *
            ENCODER_RESOLUTION;
    }
    delta = corrected_count - current_count;
    int64_t current_counter = __HAL_TIM_GetCounter(&timer);

    // calculate correct counter, together with rollovers
    int64_t corrected_counter = current_counter + delta;
    if (corrected_counter < 0) {
        rollover_count -= 1;
        corrected_counter += ENCODER_TIMER_PERIOD;
    } else if (corrected_counter > ENCODER_TIMER_PERIOD) {
        rollover_count += 1;
        corrected_counter -= ENCODER_TIMER_PERIOD;
    }

    if (z_pulses == 1) {
        count_offset -= ENCODER_POLARITY * delta;
    }

    __HAL_TIM_SetCounter(&timer, corrected_counter);
    return;
}

static void gpio_init(void) {
    // configure A, B and Z (optionally) channels
    gpio::init(ENCODER_A, gpio::Mode::AF_PP, gpio::Pull::UP,
               gpio::Speed::MEDIUM);
    gpio::init(ENCODER_B, gpio::Mode::AF_PP, gpio::Pull::UP,
               gpio::Speed::MEDIUM);
#ifdef USE_ENCODER_Z
    gpio::attach_interrupt(ENCODER_Z, gpio::Mode::INT_RISING, gpio::Pull::DOWN,
                           gpio::Speed::LOW, z_irq, NULL);
#endif
}

static void timer_init(void) {
    // WARN : If using other timers than TIM4, will break!
    // TODO : Use macro substitution instead to do this (and many others)
    // enable timer
    if (ENCODER_TIMER == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
    } else if (ENCODER_TIMER == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }

    timer.Instance = ENCODER_TIMER;  // encoder mode
    timer.Init.Prescaler = 0;        // count every edge
    timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer.Init.Period = ENCODER_TIMER_PERIOD;
    timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef encoder_config = {};
    // count on both edges (4x resolution)
    encoder_config.EncoderMode = TIM_ENCODERMODE_TI12;

    encoder_config.IC1Polarity = ENCODER_PIN_POLARITY;
    encoder_config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC1Filter = ENCODER_FILTER;

    encoder_config.IC2Polarity = ENCODER_PIN_POLARITY;
    encoder_config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC2Filter = ENCODER_FILTER;

    if (HAL_TIM_Encoder_Init(&timer, &encoder_config) != HAL_OK) {
        debug::fatal("Encoder ABZ: Timer initialisation failed.");
        error::handler();
    }

    // not necessary, but good practice
    TIM_MasterConfigTypeDef master_config = {};
    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&timer, &master_config) !=
        HAL_OK) {
        debug::fatal(
            "Encoder ABZ: Timer master config synchronisation failed.");
        error::handler();
    }

    if (ENCODER_TIMER == TIM4) {
        HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
    } else if (ENCODER_TIMER == TIM3) {
        HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }

    HAL_TIM_Encoder_Start_IT(&timer, ENCODER_A_CHANNEL | ENCODER_B_CHANNEL);
    HAL_TIM_Base_Start_IT(&timer);

    rollover_count = 0;
}

#endif

#include "ABZ.hpp"

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_gpio.h>

#include "config.hpp"
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

int64_t encoder::get_count(void) {
    return ENCODER_POLARITY * (__HAL_TIM_GetCounter(&timer) +
                               rollover_count * (ENCODER_TIMER_PERIOD + 1));
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

int32_t encoder::get_delta(void) { return delta; }

static void z_irq([[maybe_unused]] void* args) {
    z_pulses += 1;

    int64_t current_count = encoder::get_count();
    // integer division always rounds down, values that should get rounded up
    // are pushed over the next threshold
    int64_t corrected_count =
        ((current_count + ENCODER_RESOLUTION / 2) / ENCODER_RESOLUTION) *
        ENCODER_RESOLUTION;
    delta = corrected_count - current_count;
    int32_t current_counter = __HAL_TIM_GetCounter(&timer);

    // calculate correct counter, together with rollovers
    int32_t corrected_counter = current_counter + delta;
    if (corrected_counter < 0) {
        rollover_count -= 1;
        corrected_counter += ENCODER_TIMER_PERIOD;
    } else if (corrected_counter > ENCODER_TIMER_PERIOD) {
        rollover_count += 1;
        corrected_counter -= ENCODER_TIMER_PERIOD;
    }

    __HAL_TIM_SetCounter(&timer, corrected_counter);
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

    // enable level shifter
    gpio::init(ENCODER_SHIFTER_OE, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(ENCODER_SHIFTER_OE, gpio::HIGH);
}

static void timer_init(void) {
    // WARN : If using other timers than TIM4, will break!
    // TODO : Use macro substitution instead to do this (and many others)
    // enable timer
    if (ENCODER_TIMER == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
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
    }

    HAL_TIM_Encoder_Start_IT(&timer, ENCODER_A_CHANNEL | ENCODER_B_CHANNEL);
}

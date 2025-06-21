#include "ABZ.hpp"

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_gpio.h>

#include "config.hpp"
#include "debug.hpp"
#include "error.hpp"

static TIM_HandleTypeDef timer;

static void gpio_init(void);
static void timer_init(void);

uint16_t encoder::get_count(void) { return __HAL_TIM_GetCounter(&timer); }

void encoder::init(void) {
    debug::trace("Encoder ABZ: Initialising encoder timer.");
    timer_init();
    debug::trace("Encoder ABZ: Initialising encoder GPIO.");
    gpio_init();
    debug::trace("Encoder ABZ: Initialised encoder.");
}

static void gpio_init(void) {
    gpio::init(ENCODER_A, gpio::GPIOMode::AF_PP, gpio::GPIOPull::UP,
               gpio::GPIOSpeed::MEDIUM);
    gpio::init(ENCODER_B, gpio::GPIOMode::AF_PP, gpio::GPIOPull::UP,
               gpio::GPIOSpeed::MEDIUM);
}

static void timer_init(void) {
    if (ENCODER_TIMER == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
    }

    timer.Instance = ENCODER_TIMER;
    timer.Init.Prescaler = 0;  // count every edge
    timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    // 16 bit timer, count to 65535
    timer.Init.Period = 0xFFFF;
    timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef encoder_config = {};
    // count on both edges (4x resolution)
    encoder_config.EncoderMode = TIM_ENCODERMODE_TI12;

    encoder_config.IC1Polarity = ENCODER_POLARITY;
    encoder_config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC1Filter = ENCODER_FILTER;

    encoder_config.IC2Polarity = ENCODER_POLARITY;
    encoder_config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC2Filter = ENCODER_FILTER;

    if (HAL_TIM_Encoder_Init(&timer, &encoder_config) != HAL_OK) {
        debug::fatal("Encoder ABZ: Timer initialisation failed.");
        error::handler();
    }

    TIM_MasterConfigTypeDef master_config = {};
    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&timer, &master_config) !=
        HAL_OK) {
        debug::fatal(
            "Encoder ABZ: Timer master config synchronisation failed.");
        error::handler();
    }

    HAL_TIM_Encoder_Start(&timer, ENCODER_A_CHANNEL | ENCODER_B_CHANNEL);
}

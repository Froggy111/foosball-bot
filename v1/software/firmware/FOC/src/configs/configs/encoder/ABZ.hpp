#pragma once

#include <stm32g4xx_hal.h>

#include "gpio.hpp"

#define ENCODER_TIMER TIM4
const gpio::PinConfig ENCODER_A = {GPIOB, gpio::GPIOPin::PIN7,
                                   gpio::GPIOAF::AF2_TIM4};
#define ENCODER_A_CHANNEL TIM_CHANNEL_2

const gpio::PinConfig ENCODER_B = {GPIOB, gpio::GPIOPin::PIN6,
                                   gpio::GPIOAF::AF2_TIM4};
#define ENCODER_B_CHANNEL TIM_CHANNEL_1

const gpio::PinConfig ENCODER_Z = {GPIOA, gpio::GPIOPin::PIN4,
                                   gpio::GPIOAF::NONE};

const uint8_t ENCODER_FILTER = 4;
const uint8_t ENCODER_POLARITY = TIM_ICPOLARITY_RISING;

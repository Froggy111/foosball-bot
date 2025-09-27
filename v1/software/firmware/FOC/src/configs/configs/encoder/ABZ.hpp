#pragma once

#include <stm32g4xx_hal.h>

#include "gpio.hpp"

// NOTE : uncomment to use encoder Z channel (currently very noisy from motor
// NOTE : power phase wires, due to level shifter)
#define USE_ENCODER_Z

const int64_t ENCODER_POLARITY = -1;

#define ENCODER_TIMER TIM3
const gpio::PinConfig ENCODER_A = {GPIOC, gpio::Pin::PIN6, gpio::AF::AF2_TIM3};
#define ENCODER_A_CHANNEL TIM_CHANNEL_2

const gpio::PinConfig ENCODER_B = {GPIOA, gpio::Pin::PIN4, gpio::AF::AF2_TIM3};
#define ENCODER_B_CHANNEL TIM_CHANNEL_1

const gpio::PinConfig ENCODER_Z = {GPIOC, gpio::Pin::PIN5, gpio::AF::NONE};

const uint8_t ENCODER_FILTER = 4;
const uint8_t ENCODER_PIN_POLARITY = TIM_ICPOLARITY_FALLING;
const uint32_t ENCODER_RESOLUTION = 4000;

const uint16_t ENCODER_TIMER_PERIOD = 65535;  // using full 16 bits

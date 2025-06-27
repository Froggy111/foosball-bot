#pragma once

#include <stm32g4xx_hal.h>

#include "gpio.hpp"

// NOTE : uncomment to use encoder Z channel (currently very noisy from motor
// NOTE : power phase wires, due to level shifter)
// #define USE_ENCODER_Z

#define ENCODER_TIMER TIM4
const gpio::PinConfig ENCODER_A = {GPIOB, gpio::Pin::PIN7, gpio::AF::AF2_TIM4};
#define ENCODER_A_CHANNEL TIM_CHANNEL_2

const gpio::PinConfig ENCODER_B = {GPIOB, gpio::Pin::PIN6, gpio::AF::AF2_TIM4};
#define ENCODER_B_CHANNEL TIM_CHANNEL_1

const gpio::PinConfig ENCODER_Z = {GPIOB, gpio::Pin::PIN3, gpio::AF::NONE};

const uint8_t ENCODER_FILTER = 4;
const uint8_t ENCODER_POLARITY = TIM_ICPOLARITY_RISING;
const uint32_t ENCODER_RESOLUTION = 4000;

const uint16_t ENCODER_TIMER_PERIOD = 65535;  // using full 16 bits

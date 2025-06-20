#pragma once

#include <stdint.h>
#include <stm32g4xx_hal.h>

#include "gpio.hpp"

/**
 * @brief PWM peripheral settings.
 */
// PWM resolution must be above 1 << PWM_MIN_RESOLUTION. This is prioritised
// over frequency deviation
const uint32_t PWM_MIN_RESOLUTION = 6;  // 6 bits of resolution at minimum
// PWM frequency is as far as possible maintained to deviate less than
// PWM_MAX_FREQ_DEVIATION
const float PWM_MAX_FREQ_DEVIATION = 0.05;  // 5% max deviation

// Configuration for drive phase
#define DRIVE_PHASE_TIMER TIM1  // U, V, W phases on TIM1
#define DRIVE_PHASE_CLOCK PCLK2
#define PCLK1 1
#define PCLK2 2

#define U_PHASE_CHANNEL TIM_CHANNEL_1
#define V_PHASE_CHANNEL TIM_CHANNEL_2
#define W_PHASE_CHANNEL TIM_CHANNEL_3

/**
 * PC13     ------> TIM1_CH1N (AF4)
 * PB1     ------> TIM1_CH3N (AF6)
 * PB14     ------> TIM1_CH2N (AF6)
 * PA8     ------> TIM1_CH1 (AF6)
 * PA9     ------> TIM1_CH2 (AF6)
 * PA10     ------> TIM1_CH3 (AF6)
 */

// --- Inverter Leg U (High and Low Side) ---
const gpio::PinConfig INVERTER_U = {GPIOA, gpio::GPIOPin::PIN8,
                                    gpio::GPIOAF::AF6_TIM1};
const gpio::PinConfig INVERTER_U_N = {GPIOC, gpio::GPIOPin::PIN13,
                                      gpio::GPIOAF::AF4_TIM1};

// --- Inverter Leg V (High and Low Side) ---
const gpio::PinConfig INVERTER_V = {GPIOA, gpio::GPIOPin::PIN9,
                                    gpio::GPIOAF::AF6_TIM1};
const gpio::PinConfig INVERTER_V_N = {GPIOB, gpio::GPIOPin::PIN14,
                                      gpio::GPIOAF::AF6_TIM1};

// --- Inverter Leg W (High and Low Side) ---
const gpio::PinConfig INVERTER_W = {GPIOA, gpio::GPIOPin::PIN10,
                                    gpio::GPIOAF::AF6_TIM1};
const gpio::PinConfig INVERTER_W_N = {GPIOB, gpio::GPIOPin::PIN1,
                                      gpio::GPIOAF::AF6_TIM1};

const uint32_t DRIVE_BREAKTIME = 500;  // in ns

// maximum high-side on-time, due to bootstrap circuit used
const float MAX_PWM_ONTIME = 0.95;  // 95% should be safe

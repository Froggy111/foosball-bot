#pragma once

#include <stm32g4xx_hal.h>

// port, pin, alternate function
#define ENCODER_A_PORT GPIOB
#define ENCODER_A_PIN GPIO_PIN_7
#define ENCODER_A_AF GPIO_AF2_TIM4  // TIM4_CH2

#define ENCODER_B_PORT GPIOB
#define ENCODER_B_PIN GPIO_PIN_6
#define ENCODER_B_AF GPIO_AF2_TIM4  // TIM4_CH1

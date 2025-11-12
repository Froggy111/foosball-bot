#pragma once

#include <stm32g4xx_hal.h>

#include "gpio.hpp"

const gpio::PinConfig UART_TX = {GPIOB, gpio::Pin::PIN6, gpio::AF::AF7_USART1};
const gpio::PinConfig UART_RX = {GPIOB, gpio::Pin::PIN7, gpio::AF::AF7_USART1};

#define UART_INSTANCE USART1
#define UART_TX_DMA_INSTANCE DMA1_Channel3
#define UART_RX_DMA_INSTANCE DMA1_Channel4

// uncomment for RS485
// NOTE : also define UART_DE PinConfig
// #define USE_RS485

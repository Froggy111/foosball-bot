#pragma once

#include <stm32g4xx_hal.h>

#include "gpio.hpp"
#include "spi.hpp"

const uint32_t ENCODER_RESOLUTION = 1 << 14;

const int8_t ENCODER_DIRECTION = -1;

const gpio::PinConfig MT6701_NCS = {GPIOA, gpio::Pin::PIN4, gpio::AF::NONE};
const gpio::PinConfig MT6701_SCLK = {GPIOA, gpio::Pin::PIN5,
                                     gpio::AF::AF5_SPI1};
const gpio::PinConfig MT6701_MISO = {GPIOA, gpio::Pin::PIN6,
                                     gpio::AF::AF5_SPI1};
#define MT6701_SPI_INSTANCE SPI1

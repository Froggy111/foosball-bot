#pragma once

#include <stm32g4xx_hal.h>

#include "gpio.hpp"

namespace spi {

enum class Mode : uint32_t { master = SPI_MODE_MASTER, slave = SPI_MODE_SLAVE };
enum class Direction : uint32_t {
    bidirectional = SPI_DIRECTION_2LINES,
    rx_only = SPI_DIRECTION_2LINES_RXONLY
};

/**
 * INFO : SPI modes
 * Polarity: what the clock line idles at
 * Phase: which edge of the clock the data is captured on
 * first = capture on change from idle state to active state
 * second = capture on change from active state to idle state
 */
enum class Polarity : uint32_t {
    low = SPI_POLARITY_LOW,
    high = SPI_POLARITY_HIGH
};
enum class Phase : uint32_t {
    first = SPI_PHASE_1EDGE,
    second = SPI_PHASE_2EDGE
};

/**
 * INFO : SPI input clock = SYSCLK. SYSCLK = 160MHz
 */
enum class BaudRate : uint32_t {
    _80Mb = SPI_BAUDRATEPRESCALER_2,
    _40Mb = SPI_BAUDRATEPRESCALER_4,
    _20Mb = SPI_BAUDRATEPRESCALER_8,
    _10Mb = SPI_BAUDRATEPRESCALER_16,
    _5Mb = SPI_BAUDRATEPRESCALER_32,
    _2_5Mb = SPI_BAUDRATEPRESCALER_64,
    _1_25Mb = SPI_BAUDRATEPRESCALER_128,
    _625Kb = SPI_BAUDRATEPRESCALER_256
};

enum class FirstBit : uint32_t {
    MSB = SPI_FIRSTBIT_MSB,
    LSB = SPI_FIRSTBIT_LSB
};

struct Config {
    SPI_TypeDef* instance;
    Mode mode;
    Direction direction;
    Polarity polarity;
    Phase phase;
    BaudRate baud_rate;
    FirstBit first_bit;

    gpio::PinConfig SCLK;
    gpio::PinConfig MISO;
    gpio::PinConfig MOSI;
    gpio::PinConfig NCS;

    SPI_HandleTypeDef handle;
};

void init(Config& spi);
bool receive(Config& spi, uint8_t* buffer, uint16_t size,
             uint32_t timeout = HAL_MAX_DELAY);
}  // namespace spi

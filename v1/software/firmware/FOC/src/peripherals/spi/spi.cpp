#include "spi.hpp"

#include <stm32g4xx_hal.h>

#include "debug.hpp"
#include "error.hpp"
#include "gpio.hpp"

// INFO : SYSCLK = 160MHz, 100ns = 16 cycles
inline void delay_100ns(void) {
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
}

void spi::init(Config &spi) {
    debug::debug("Initialising SPI...");
    if (spi.instance == SPI1) {
        debug::debug("Enabling SPI1 clock");
        __HAL_RCC_SPI1_CLK_ENABLE();
    } else if (spi.instance == SPI2) {
        __HAL_RCC_SPI2_CLK_ENABLE();
    } else if (spi.instance == SPI3) {
        __HAL_RCC_SPI3_CLK_ENABLE();
    }
    spi.handle.Instance = spi.instance;
    spi.handle.Init.Mode = (uint32_t)spi.mode;
    spi.handle.Init.Direction = (uint32_t)spi.direction;
    spi.handle.Init.DataSize = SPI_DATASIZE_8BIT;
    spi.handle.Init.CLKPolarity = (uint32_t)spi.polarity;
    spi.handle.Init.CLKPhase = (uint32_t)spi.phase;
    spi.handle.Init.NSS = SPI_NSS_SOFT;
    spi.handle.Init.BaudRatePrescaler = (uint32_t)spi.baud_rate;
    spi.handle.Init.FirstBit = (uint32_t)spi.first_bit;
    spi.handle.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi.handle.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&spi.handle) != HAL_OK) {
        debug::error("SPI initialisation failed.");
        error::handler();
    }

    gpio::init(spi.SCLK, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);
    gpio::init(spi.MISO, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);
    if (spi.direction == Direction::bidirectional) {
        gpio::init(spi.MOSI, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
                   gpio::Speed::VERY_HIGH);
    }
    gpio::init(spi.NCS, gpio::Mode::OUTPUT_PP, gpio::Pull::UP,
               gpio::Speed::VERY_HIGH);

    debug::debug("SPI initialisation successful.");
    return;
}

bool spi::receive(Config &spi, uint8_t *buffer, uint16_t size,
                  uint32_t timeout) {
    gpio::write(spi.NCS, gpio::LOW);
    delay_100ns();
    HAL_StatusTypeDef status =
        HAL_SPI_Receive(&spi.handle, buffer, size, timeout);
    delay_100ns();
    gpio::write(spi.NCS, gpio::HIGH);
    if (status != HAL_OK) {
        debug::error("SPI receive failed.");
        return false;
    }
    return true;
}

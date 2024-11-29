#pragma once
#include "types.h"

namespace esp32 {

const types::u8 INTERNAL_LED = 2; // must be floating or low during boot
const types::u8 SPI0_MOSI = 23;
const types::u8 SPI0_MISO = 19;
const types::u8 SPI0_CLK = 18;
const types::u8 SPI0_CS = 5;
const types::u8 SPI1_MOSI = 13;
const types::u8 SPI1_MISO = 12; // must not be pulled high during boot
const types::u8 SPI1_CLK = 14;
const types::u8 SPI1_CS = 15;
const types::u8 UART0_TX = 1; // debug output during boot
const types::u8 UART0_RX = 3; // must be high during boot
const types::u8 UART2_TX = 17;
const types::u8 UART2_RX = 16;
const types::u8 SCL = 22;
const types::u8 SDA = 21;

// GPIO 34, 35, 36, 39 are read-only

}

#include "swo.hpp"

#include <stm32g4xx_hal.h>

#include "config.hpp"

void swo::init(void) {
    gpio::init(SWO, gpio::Mode::AF_PP, gpio::Pull::UP, gpio::Speed::HIGH);
    return;
}

void swo::write(uint8_t *buf, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        ITM_SendChar(buf[i]);
    }
    return;
}

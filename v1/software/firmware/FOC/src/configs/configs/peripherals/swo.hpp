#pragma once

#include "gpio.hpp"

const gpio::PinConfig SWO = {GPIOB, gpio::Pin::PIN3, gpio::AF::AF0_SWJ};

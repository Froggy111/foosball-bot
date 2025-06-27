#pragma once
#include "configs/encoder/ABZ.hpp"
#include "gpio.hpp"

#define ENCODER_TYPE ABZ
#define ABZ 0

const gpio::PinConfig ENCODER_SHIFTER_OE = {GPIOB, gpio::Pin::PIN11,
                                            gpio::AF::NONE};

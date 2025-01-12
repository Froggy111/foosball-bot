#pragma once

#include <Arduino.h>
#include <RP2040_PWM.h>
#include "types.hpp"

namespace arm {


class Servo {
public:
  Servo(types::u8 servo_pin, types::u8 gear_ratio = 4, types::u16 min_pulse = 500, types::u16 max_pulse = 2500, types::u16 dead_band = 7);
};

}

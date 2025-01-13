#include <Arduino.h>
#include <Servo.h>
#include "stepper.hpp"
#include "types.hpp"

namespace arm {

bool home_stepper(Stepper &stepper, types::u8 endstop_pin, types::u8 moveforward_mm, types::u8 movebackward_mm, types::u8 midpoint_pos, types::u32 speed, types::u32 accel);

}

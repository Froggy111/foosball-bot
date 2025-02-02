#include <Arduino.h>
#include "stepper.hpp"
#include "types.hpp"
#include "homing.hpp"

using namespace types;

namespace arm {

bool home_stepper(Stepper &stepper, u8 endstop_pin, u8 moveforward_mm, u8 movebackward_mm, u8 target_pos, u32 speed, u32 accel) { // speed should be low, accel should be high.
  pinMode(endstop_pin, INPUT_PULLUP); // endstop is wired NC gnd and signal.
  stepper.move(stepper.um_coord() / 1000 + moveforward_mm, speed, accel);
  stepper.setup_move(stepper.um_coord() / 1000 - movebackward_mm, speed, accel);
  while (digitalRead(endstop_pin) && stepper.next_step()); // if its high, endstop is not triggered.
  stepper.set_step_coord(0);
  stepper.setup_move(target_pos);
  while (stepper.next_step());
  return true;
}

}

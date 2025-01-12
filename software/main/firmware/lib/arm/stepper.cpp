#include <Arduino.h>
#include "types.hpp"
#include "stepper.hpp"
#include "debug.hpp"

using namespace types;

namespace arm {

const u32 min_step_pulse_duration = 2; // step must be on high for at least 2 microseconds
const u32 wakeup_time = 1700;

void delay_from_time_us(u32 duration, u64 time) {
  delayMicroseconds(duration - (time - micros()));
}

Stepper::Stepper(u8 nfault_pin, u8 nreset_pin, u8 nsleep_pin,
                 u8 enable_pin, u8 step_pin, u8 dir_pin,
                 u8 m0_pin, u8 m1_pin, u8 m2_pin,
                 u16 steps_per_rev, u8 mm_per_rev) {
  _pins.nfault_pin = nfault_pin;
  _pins.nreset_pin = nreset_pin;
  _pins.nsleep_pin = nsleep_pin;
  _pins.enable_pin = enable_pin;
  _pins.step_pin = step_pin;
  _pins.dir_pin = dir_pin;
  _pins.m0_pin = m0_pin;
  _pins.m1_pin = m1_pin;
  _pins.m2_pin = m2_pin;

  _steps_per_rev = steps_per_rev;
  _mm_per_rev = mm_per_rev;
  _um_per_step = mm_per_rev * 1000 / steps_per_rev; /* micrometers per step. This should be an integer value for regular pulleys. (200 for GT2-20T)*/
}

bool Stepper::begin(u32 max_speed, u32 max_accel, u8 microsteps) {
  _max_speed = max_speed;
  _max_accel = max_accel;
  set_microsteps(microsteps);

  _max_steps_per_second = mm_to_steps<u32>(max_speed);
  _max_steps_accel = mm_to_steps<u32>(max_accel);

  // not reset, not sleep, and not enable. No fault interrupt, as it will be ran on second core without interrupts.
  // Fault will be polled in between pulses.
  pinMode(_pins.nreset_pin, OUTPUT);
  digitalWrite(_pins.nreset_pin, HIGH);
  pinMode(_pins.nsleep_pin, OUTPUT);
  digitalWrite(_pins.nsleep_pin, HIGH);
  pinMode(_pins.enable_pin, OUTPUT);
  digitalWrite(_pins.enable_pin, LOW);
  pinMode(_pins.nfault_pin, INPUT);

  // // hardcoded set to no microstepping (M0 low, M1 low, M2 low)
  // pinMode(_pins.m0_pin, OUTPUT);
  // digitalWrite(_pins.m0_pin, LOW);
  // pinMode(_pins.m1_pin, OUTPUT);
  // digitalWrite(_pins.m1_pin, LOW);
  // pinMode(_pins.m2_pin, OUTPUT);
  // digitalWrite(_pins.m2_pin, LOW);
  //
  return true;
}

// will energise the coils! make sure to set current limit before this.
bool Stepper::enable(void) {
  digitalWrite(_pins.enable_pin, HIGH);
  return true;
}

// can also use nsleep.
bool Stepper::disable(void) {
  digitalWrite(_pins.enable_pin, LOW);
  return true;
}

bool Stepper::faulted(void) {
  return !digitalRead(_pins.nfault_pin);
}

bool Stepper::reset(void) {
  // enter reset mode
  digitalWrite(_pins.nreset_pin, LOW);
  delayMicroseconds(wakeup_time);
  // exit reset mode
  digitalWrite(_pins.nreset_pin, HIGH);
  // if still faulted, return false
  return !faulted();
}

bool Stepper::move(i32 pos, u32 speed, u32 accel) {
  setup_move(pos, speed, accel);
  while (next_step());
  return true;
}

bool Stepper::setup_move(i32 pos, u32 speed, u32 accel) {
  if (speed == 0) {
    speed = _max_speed;
  }
  if (accel == 0) {
    accel = _max_accel;
  }
  speed = min(speed, _max_speed);
  accel = min(accel, _max_accel);

  StepperLinearAccel &sla = _stepper_linear_accel; // alias to prevent 100 long lines

  // convert to microsteps units
  speed = mm_to_steps<u32>(speed) * _microsteps;
  accel = mm_to_steps<u32>(accel) * _microsteps;
  sla.speed = speed;
  sla.accel = accel;
  i32 move_steps = (mm_to_steps<i32>(pos) - _step_coord) * (i32) _microsteps - (i32) _microstep_counter;
  if (move_steps == 0) {
    debug::printf("move_steps in setup_move is zero for some stupid reason\n");
    return false;
  }
  sla.move_steps = (u32) abs(move_steps);
  sla.direction = move_steps > 0;

  // calculate how many steps to accelerate for
  sla.accel_steps = speed * speed / (accel * 2);
  // cannot reach specified speed, clamp to accelerate halfway
  if (sla.accel_steps * 2 > sla.move_steps) {
    sla.accel_steps = sla.move_steps / 2;
  }

  // setup timers
  // https://ww1.microchip.com/downloads/en/Appnotes/doc8017.pdf (Linear speed control of stepper motor)
  sla.step_timing = (1e+6)*0.676*sqrt(2.0f/accel);
  sla.cruise_step_timing = 1e+6 / speed;
  sla.last_step_time = micros();

  // setup counters
  sla.step_count = 0;
  debug::printf("setup_move values: speed: %u, accel: %u, direction: %u, move_steps: %i, accel_steps: %i, step_count: %u, \
steps_remaining: %u, cruise_step_timing: %u, step_timing: %u, step_timing_remainder: %u\n",
                sla.speed, sla.accel, (u32) sla.direction, sla.move_steps, sla.accel_steps, sla.step_count,
                sla.move_steps - sla.step_count, sla.cruise_step_timing, sla.step_timing, sla.step_timing_remainder);
  return true;
}

bool Stepper::setup_move_override(i32 pos, u32 speed, u32 accel) {
  // setup move that is overriding a previous move.
  StepperLinearAccel &sla = _stepper_linear_accel; // alias to prevent 100 long lines
  if (speed == 0) {
    speed = _max_speed;
  }
  if (accel == 0) {
    accel = _max_accel;
  }
  speed = min(speed, _max_speed);
  accel = min(accel, _max_accel);

  // convert to steps units
  speed = mm_to_steps<u32>(speed);
  accel = mm_to_steps<u32>(accel);
  i32 move_steps = mm_to_steps<i32>(pos) - _step_coord;
  if (move_steps == 0) {
    return false;
  }
  bool direction = move_steps > 0;
  sla.speed = speed;
  sla.accel = accel;

  // check what is needed to override
  if (sla.move_steps == 0) { // no move to override, do normal.
    setup_move(pos, speed, accel);
  }
  else if (direction == sla.direction) { // going in the same direction. modify accelerations and speeds.
    u32 current_speed = 1e+6 / sla.step_timing; // steps per second. this would have a resolution of 0.2mm/s (using the current design), which is enough.
    u32 min_decel_steps = current_speed * current_speed / (accel * 2);
    // first check if move_steps is enough to decelerate.
    if ((u32) abs(move_steps) < min_decel_steps) { // will overshoot, thus do decel, accel then decel
      sla.initial_decel_steps = min_decel_steps; // schedule decel first, which will overshoot
      sla.initial_decel_direction = sla.direction;
      u32 remaining_move_steps = min_decel_steps - move_steps;
      sla.accel_steps = speed * speed / (accel * 2);
      // cannot reach specified speed, clamp to accelerate halfway
      if (sla.accel_steps * 2 > remaining_move_steps) {
        sla.accel_steps = remaining_move_steps / 2;
      }
      sla.accel_direction = !sla.direction; // accelerate back
      sla.final_decel_direction = sla.accel_direction;
      sla.final_decel_steps = sla.accel_steps;
      sla.move_steps = sla.initial_decel_steps + remaining_move_steps;
    }
    else { // does not overshoot. Move to position aggressively.
      if (speed < current_speed) { // target max speed lower, need to decel first.
        // do a deceleration to speed, cruise, then decelerate to 0.
        u32 initial_decel_speed = current_speed - speed;
        sla.initial_decel_steps = initial_decel_speed * initial_decel_speed / (accel * 2);
        sla.initial_decel_direction = sla.direction;
        // do not accel
        sla.accel_steps = 0;
        sla.accel_direction = sla.direction;
        // cruise
        sla.move_steps = (u32) abs(move_steps);
        // decel again
        sla.final_decel_steps = speed * speed / (accel * 2);
        sla.final_decel_direction = sla.direction;
      }
      else { // target max speed equal or higher, accelerate then decelerate.
        u32 max_accel_speed = speed - current_speed;
        u32 max_accel_steps = max_accel_speed * max_accel_speed / (accel * 2);
        u32 max_decel_steps = speed * speed / (accel * 2);
        if (max_accel_steps + max_decel_steps > (u32) abs(move_steps)) { // need to clamp
          u32 clamp_amount = max_accel_steps + max_decel_steps - (u32) abs(move_steps);
          sla.initial_decel_steps = 0;
          sla.initial_decel_direction = sla.accel_direction;
          sla.accel_steps = max_accel_steps - clamp_amount / 2;
          sla.accel_direction = sla.accel_direction;
          sla.final_decel_steps = max_decel_steps - clamp_amount / 2;
          sla.final_decel_direction = sla.accel_direction;
        }
        else {
          sla.initial_decel_steps = 0;
          sla.initial_decel_direction = sla.accel_direction;
          sla.accel_steps = max_accel_steps;
          sla.accel_direction = sla.accel_direction;
          sla.final_decel_steps = max_decel_steps;
          sla.final_decel_direction = sla.accel_direction;
        }
      }
    }
  }
  else { // going in opposite direction. Decelerate to a stop first, then setup as normal.
    u32 current_speed = 1e+6 / sla.step_timing; // steps per second. this would have a resolution of 0.2mm/s (using the current design), which is enough.
    // initial decel
    sla.initial_decel_steps = current_speed * current_speed / (accel * 2);
    sla.initial_decel_direction = sla.direction;
    // normal setup
    u32 remaining_move_steps = (u32) abs(move_steps) + sla.initial_decel_steps;
    sla.accel_steps = speed * speed / (accel * 2);
    // cannot reach specified speed, clamp to accelerate halfway
    if (sla.accel_steps * 2 > remaining_move_steps) {
      sla.accel_steps = remaining_move_steps / 2;
    }
    sla.accel_direction = !sla.direction; // accelerate back
    sla.final_decel_direction = sla.accel_direction;
    sla.final_decel_steps = sla.accel_steps;
    sla.move_steps = sla.initial_decel_steps + remaining_move_steps;
  }

  // calculate how many steps to accelerate for
  sla.accel_steps = speed * speed / (accel * 2);
  // cannot reach specified speed, clamp to accelerate halfway
  if (sla.accel_steps * 2 > (u32) abs(move_steps)) {
    sla.accel_steps = move_steps / 2;
  }

  // setup timers
  // https://ww1.microchip.com/downloads/en/Appnotes/doc8017.pdf (Linear speed control of stepper motor)
  sla.step_timing = (1e+6)*0.676*sqrt(2.0f/accel);
  sla.cruise_step_timing = 1e+6 / speed;
  sla.last_step_time = micros();

  // setup counters
  sla.step_count = 0;
  return true;
}

// reset all the stepper accel move profile values to 0
bool Stepper::cancel_move(void) {
  StepperLinearAccel &sla = _stepper_linear_accel; // alias to prevent 100 long lines
  sla.speed = 0;
  sla.accel = 0;

  sla.direction = 0;
  sla.move_steps = 0;
  sla.initial_decel_steps = 0;
  sla.initial_decel_direction = 0;
  sla.accel_steps = 0;
  sla.accel_direction = 0;
  sla.final_decel_steps = 0;
  sla.final_decel_direction = 0;
  sla.cruise_step_timing = 0;
  sla.cruise_step_timing = 0;

  sla.step_count = 0;
  sla.last_step_time = 0;
  sla.step_timing = 0;
  sla.step_timing_remainder = 0;
  sla.current_direction = 0;

  return true;
}

bool Stepper::setup_stop(u32 accel) {
  // setup a stop for the current move, using the current acceleration.

}

void Stepper::calc_timing(void) {
  // to do something that will work from some speed to some other speed:
  // the maximum number of stages, is one deceleration, one acceleration, one cruise, then another deceleration.
  // assume decelerations are kept the same as accelerations for simplicity.
  // thus, an initial deceleration_steps, then an acceleration_steps is needed to describe this movement.
  StepperLinearAccel &sla = _stepper_linear_accel; // alias to prevent 100 long lines
  if (sla.step_count >= sla.move_steps){  // this should not happen, but avoids strange calculations
    return;
  }
  sla.step_count ++;
  if (sla.current_direction) {
    if (_microstep_counter < _microsteps - 1) { // another microstep does not finish a full step
      _microstep_counter ++;
    }
    else {
      _step_coord ++;
      _microstep_counter = 0;
    }
  }
  else {
    if (_microstep_counter > 0) {
      _microstep_counter --;
    }
    else {
      _step_coord --;
      _microstep_counter = _microsteps - 1;
    }
  }

  if (sla.step_count < sla.initial_decel_steps) { // need to run an initial deceleration.
    // "accel steps" for this initial deceleration is initial_decel_steps - step_count.
    i32 n = (i32) sla.initial_decel_steps - (i32) sla.step_count;
    sla.step_timing = sla.step_timing - (2 * (i32) sla.step_timing + (i32) sla.step_timing_remainder) / (-4 * n + 1);
    sla.step_timing_remainder = (2 * sla.step_timing + sla.step_timing_remainder) % (-4 * n + 1);
    digitalWrite(_pins.dir_pin, sla.initial_decel_direction);
    sla.current_direction = sla.initial_decel_direction;
  }
  else if (sla.step_count >= sla.initial_decel_steps && sla.step_count < sla.accel_steps + sla.initial_decel_steps) { // accelerating
    if (sla.step_count == sla.initial_decel_steps) {
      sla.step_timing_remainder = 0;
    }
    // "accel steps" for this acceleration is step_count - initial_decel_steps.
    i32 n = (i32) sla.step_count - (i32) sla.initial_decel_steps;
    sla.step_timing = sla.step_timing - (2 * sla.step_timing + sla.step_timing_remainder) / (4 * n + 1);
    sla.step_timing_remainder = (2 * sla.step_timing + sla.step_timing_remainder) % (4 * n + 1);
    digitalWrite(_pins.dir_pin, sla.accel_direction);
    sla.current_direction = sla.accel_direction;
  }
  else if (sla.step_count == sla.accel_steps + sla.initial_decel_steps && sla.step_count < sla.move_steps - sla.accel_steps) {
    sla.step_timing = sla.cruise_step_timing;
    sla.step_timing_remainder = 0;
    digitalWrite(_pins.dir_pin, sla.accel_direction);
    sla.current_direction = sla.accel_direction;
  }
  else if (sla.step_count >= sla.move_steps - sla.accel_steps && sla.step_count < sla.move_steps) { // decelerating.
    if (sla.step_count == sla.move_steps - sla.accel_steps) {
      sla.step_timing_remainder = 0;
    }
    // "accel steps" for this acceleration is move_steps - step_count.
    i32 n = (i32) sla.move_steps - (i32) sla.step_count;
    sla.step_timing = sla.step_timing - (2 * (i32) sla.step_timing + (i32) sla.step_timing_remainder) / (-4 * n + 1);
    sla.step_timing_remainder = (2 * (i32) sla.step_timing + (i32) sla.step_timing_remainder) % (-4 * n + 1);
    digitalWrite(_pins.dir_pin, sla.final_decel_direction);
    sla.current_direction = sla.final_decel_direction;
  }
  else if (sla.step_count == sla.move_steps) { // move completed.
    cancel_move();
  }
  else return;
}

u32 Stepper::next_step(void) {
  StepperLinearAccel &sla = _stepper_linear_accel; // alias to prevent 100 long lines
  delay_from_time_us(sla.step_timing, sla.last_step_time);
  digitalWrite(_pins.step_pin, HIGH);
  calc_timing();
  delayMicroseconds(min_step_pulse_duration);
  digitalWrite(_pins.step_pin, LOW);
  sla.last_step_time = micros();
  debug::printf("next_step values: speed: %u, accel: %u, direction: %u, move_steps: %i, accel_steps: %i, step_count: %u, \
steps_remaining: %u, cruise_step_timing: %u, step_timing: %u, step_timing_remainder: %u, microstep_counter: %u\n",
                sla.speed, sla.accel, (u32) sla.direction, sla.move_steps, sla.accel_steps, sla.step_count,
                sla.move_steps - sla.step_count, sla.cruise_step_timing, sla.step_timing, sla.step_timing_remainder, (u32) _microstep_counter);
  return sla.step_timing;
}

// settings and constants
u32 Stepper::max_speed(void) {
  return _max_speed;
}
u32 Stepper::max_accel(void) {
  return _max_accel;
}
u16 Stepper::steps_per_rev(void) {
  return _steps_per_rev;
}
u8 Stepper::mm_per_rev(void) {
  return _mm_per_rev;
}
u16 Stepper::um_per_step(void) {
  return _um_per_step;
}
u32 Stepper::max_steps_per_second(void) {
  return _max_steps_per_second;
}
u32 Stepper::max_steps_accel(void) {
  return _max_steps_accel;
}
u8 Stepper::microsteps(void) {
  return _microsteps;
}
bool Stepper::set_max_speed(u32 max_speed) {
  _max_speed = max_speed;
  return true;
}
bool Stepper::set_max_accel(u32 max_accel) {
  _max_accel = max_accel;
  return true;
}
bool Stepper::set_steps_per_rev(u16 steps_per_rev) {
  _steps_per_rev = steps_per_rev;
  return true;
}
bool Stepper::set_mm_per_rev(u8 mm_per_rev) {
  _mm_per_rev = mm_per_rev;
  return true;
}
bool Stepper::set_microsteps(u8 microsteps) {
  switch (microsteps) {
  case 0:
    digitalWrite(_pins.m0_pin, LOW);
    digitalWrite(_pins.m1_pin, LOW);
    digitalWrite(_pins.m2_pin, LOW);
    _microsteps = 1;
    break;
  case 1:
    digitalWrite(_pins.m0_pin, LOW);
    digitalWrite(_pins.m1_pin, LOW);
    digitalWrite(_pins.m2_pin, LOW);
    _microsteps = 1;
    break;
  case 2:
    digitalWrite(_pins.m0_pin, HIGH);
    digitalWrite(_pins.m1_pin, LOW);
    digitalWrite(_pins.m2_pin, LOW);
    _microsteps = 2;
    break;
  case 4:
    digitalWrite(_pins.m0_pin, LOW);
    digitalWrite(_pins.m1_pin, HIGH);
    digitalWrite(_pins.m2_pin, LOW);
    _microsteps = 4;
    break;
  case 8:
    digitalWrite(_pins.m0_pin, HIGH);
    digitalWrite(_pins.m1_pin, HIGH);
    digitalWrite(_pins.m2_pin, LOW);
    _microsteps = 8;
    break;
  case 16:
    digitalWrite(_pins.m0_pin, LOW);
    digitalWrite(_pins.m1_pin, LOW);
    digitalWrite(_pins.m2_pin, HIGH);
    _microsteps = 16;
    break;
  case 32:
    digitalWrite(_pins.m0_pin, HIGH);
    digitalWrite(_pins.m1_pin, LOW);
    digitalWrite(_pins.m2_pin, HIGH);
    _microsteps = 32;
    break;
  };
  return true;
}

// variables
i32 Stepper::step_coord(void) {
  return _step_coord;
}
i32 Stepper::um_coord(void) {
  return _step_coord * _um_per_step;
}
bool Stepper::set_step_coord(i32 step_coord) {
  _step_coord = step_coord;
  return true;
}

}

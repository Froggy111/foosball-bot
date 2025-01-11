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
  sla.move_steps = move_steps;
  sla.direction = sla.move_steps > 0;

  // calculate how many steps to accelerate for
  sla.accel_steps = speed * speed / (accel * 2);
  // cannot reach specified speed, clamp to accelerate halfway
  if (sla.accel_steps * 2 > (u32) abs(move_steps)) {
    sla.accel_steps = (u32) abs(move_steps) / 2;
  }

  // setup timers
  // https://ww1.microchip.com/downloads/en/Appnotes/doc8017.pdf (Linear speed control of stepper motor)
  sla.step_timing = (1e+6)*0.676*sqrt(2.0f/accel);
  sla.cruise_step_timing = 1e+6 / speed;
  sla.last_step_time = micros();

  // setup counters
  sla.step_count = 0;
  sla.steps_remaining = abs(sla.move_steps);
  debug::printf("setup_move values: speed: %u, accel: %u, direction: %u, move_steps: %i, accel_steps: %i, step_count: %u, \
steps_remaining: %u, cruise_step_timing: %u, step_timing: %u, rest: %u\n",
                sla.speed, sla.accel, (u32) sla.direction, sla.move_steps, sla.accel_steps, sla.step_count,
                sla.steps_remaining, sla.cruise_step_timing, sla.step_timing, sla.rest);
  return true;
}

// bool Stepper::setup_move_override(i32 pos, u32 speed, u32 accel, u32 time) {
//   // testing for overriding a previous move.
//   if (speed == 0) {
//     speed = _max_speed;
//   }
//   if (accel == 0) {
//     accel = _max_accel;
//   }
//   speed = min(speed, _max_speed);
//   accel = min(accel, _max_accel);
//
//   StepperLinearAccel &sla = _stepper_linear_accel; // alias to prevent 100 long lines
//
//   // convert to steps units
//   speed = mm_to_steps<u32>(speed);
//   accel = mm_to_steps<u32>(accel);
//   sla.speed = speed;
//   sla.accel = accel;
//   i32 move_steps = mm_to_steps<i32>(pos) - _step_coord;
//   if (move_steps == 0) {
//     return false;
//   }
//   sla.move_steps = move_steps;
//   sla.direction = sla.move_steps > 0;
//
//   // calculate how many steps to accelerate for
//   sla.accel_steps = speed * speed / (accel * 2);
//   // cannot reach specified speed, clamp to accelerate halfway
//   if (sla.accel_steps * 2 > (u32) abs(move_steps)) {
//     sla.accel_steps = move_steps / 2;
//   }
//
//   // setup timers
//   // https://ww1.microchip.com/downloads/en/Appnotes/doc8017.pdf (Linear speed control of stepper motor)
//   sla.step_timing = (1e+6)*0.676*sqrt(2.0f/accel);
//   sla.cruise_step_timing = 1e+6 / speed;
//   sla.last_step_time = micros();
//
//   // setup counters
//   sla.step_count = 0;
//   sla.steps_remaining = sla.move_steps;
//   return true;
// }

// reset all the stepper accel move profile values to 0
bool Stepper::cancel_move(void) {
  StepperLinearAccel &sla = _stepper_linear_accel; // alias to prevent 100 long lines
  sla.speed = 0;
  sla.accel = 0;
  sla.direction = 0;
  sla.move_steps = 0;
  sla.accel_steps = 0;
  sla.step_count = 0;
  sla.steps_remaining = 0;
  sla.cruise_step_timing = 0;
  sla.last_step_time = 0;
  sla.step_timing = 0;
  sla.rest = 0;
  return true;
}

void Stepper::calc_timing(void) {
  StepperLinearAccel &sla = _stepper_linear_accel; // alias to prevent 100 long lines
  if (sla.steps_remaining <= 0){  // this should not happen, but avoids strange calculations
    return;
  }
  sla.steps_remaining --;
  sla.step_count ++;
  if (sla.direction) {
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

  if (sla.step_count < sla.accel_steps) { // accelerating
    sla.step_timing = sla.step_timing - (2 * sla.step_timing + sla.rest) / (4 * sla.step_count + 1);
    sla.rest = (sla.step_count < sla.accel_steps) ? (2 * sla.step_timing + sla.rest) % (4 * sla.step_count + 1) : 0;
  }
  else if (sla.step_count == sla.accel_steps) {
    sla.step_timing = sla.cruise_step_timing;
  }
  else if (sla.steps_remaining <= sla.accel_steps) { // decelerating.
    sla.step_timing = sla.step_timing - (2 * (i32) sla.step_timing + (i32) sla.rest) / (-4 * (i32) sla.steps_remaining + 1);
    debug::printf("new step timing when decelerating: %u\n", sla.step_timing);
    sla.rest = (2 * (i32) sla.step_timing + (i32) sla.rest) % (-4 * (i32) sla.steps_remaining + 1);
  }
  else return;
}

u32 Stepper::next_step(void) {
  StepperLinearAccel &sla = _stepper_linear_accel; // alias to prevent 100 long lines
  if (sla.steps_remaining > 0) {
    delay_from_time_us(sla.step_timing, sla.last_step_time);
    digitalWrite(_pins.dir_pin, sla.direction);
    digitalWrite(_pins.step_pin, HIGH);
    calc_timing();
    delayMicroseconds(min_step_pulse_duration);
    digitalWrite(_pins.step_pin, LOW);
    sla.last_step_time = micros();
    debug::printf("next_step values: speed: %u, accel: %u, direction: %u, move_steps: %i, accel_steps: %i, step_count: %u, \
steps_remaining: %u, cruise_step_timing: %u, step_timing: %u, rest: %u, microstep_counter: %u\n",
                  sla.speed, sla.accel, (u32) sla.direction, sla.move_steps, sla.accel_steps, sla.step_count,
                  sla.steps_remaining, sla.cruise_step_timing, sla.step_timing, sla.rest, (u32) _microstep_counter);
    return sla.step_timing;
  }
  else {
    cancel_move(); // this is so that setup_move will not pick up a completed move as an override.
  }
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

#include <Arduino.h>
#include <DRV8825.h>
#include <RPi_Pico_TimerInterrupt.h>
#include "TimerInterrupt_Generic_Debug.h"
#include "types.hpp"
#include "stepper.hpp"

using namespace types;

namespace arm {

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

bool Stepper::begin(u32 max_speed, u32 max_accel) {
  _max_speed = max_speed;
  _max_accel = max_accel;

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

  // hardcoded set to no microstepping (M0 low, M1 low, M2 low)
  pinMode(_pins.m0_pin, OUTPUT);
  digitalWrite(_pins.m0_pin, LOW);
  pinMode(_pins.m1_pin, OUTPUT);
  digitalWrite(_pins.m1_pin, LOW);
  pinMode(_pins.m2_pin, OUTPUT);
  digitalWrite(_pins.m2_pin, LOW);

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

bool Stepper::reset(u32 reset_time) {
  // enter reset mode
  digitalWrite(_pins.nreset_pin, LOW);
  delayMicroseconds(reset_time);
  // exit reset mode
  digitalWrite(_pins.nreset_pin, HIGH);
  // if still faulted, return false
  return !faulted();
}

bool Stepper::move(i32 pos, u32 speed, u32 accel) {
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

  return true;
}

bool Stepper::setup_move(i32 pos, u32 speed, u32 accel, u32 time) {
  if (speed == 0) {
    speed = _max_speed;
  }
  if (accel == 0) {
    accel = _max_accel;
  }
  speed = min(speed, _max_speed);
  accel = min(accel, _max_accel);

  StepperLinearAccel &sla = _stepper_linear_accel; // alias to prevent 100 long lines

  // convert to steps units
  speed = mm_to_steps<u32>(speed);
  accel = mm_to_steps<u32>(accel);
  sla.speed = speed;
  sla.accel = accel;
  i32 move_steps = mm_to_steps<i32>(pos) - _step_coord;
  sla.move_steps = move_steps;

  // calculate how many steps to accelerate for
  sla.accel_steps = speed * speed / (accel * 2);
  u32 cruise_steps;
  // cannot reach specified speed, clamp to accelerate halfway
  if (sla.accel_steps * 2 > abs(move_steps)) {
    sla.accel_steps = move_steps / 2;
  }

  // setup timers
  // https://ww1.microchip.com/downloads/en/Appnotes/doc8017.pdf (Linear speed control of stepper motor)
  sla.step_timing = (1e+6)*0.676*sqrt(2.0f/accel);
  sla.cruise_step_timing = 1e+6 / speed;
  sla.last_step_time = micros();

  // setup counters
  sla.step_count = 0;
  sla.steps_remaining = sla.move_steps;
  return true;
}

u32 Stepper::calc_timing(void) {
  StepperLinearAccel &sla = _stepper_linear_accel; // alias to prevent 100 long lines
  if (sla.step_count < sla.accel_steps) { // accelerating
    sla.step_timing = sla.step_timing - (2 * sla.step_timing + sla.rest) / (4 * sla.step_count + 1);
    sla.rest = (sla.step_count < sla.accel_steps) ? (2 * sla.step_timing + sla.rest) % (4 * sla.step_count + 1) : 0;
  }
  else if (sla.step_count == sla.accel_steps) {
    sla.step_timing = sla.cruise_step_timing;
  }
  else if ()
            if (step_count < steps_to_cruise){
                step_pulse = step_pulse - (2*step_pulse+rest)/(4*step_count+1);
                rest = (step_count < steps_to_cruise) ? (2*step_pulse+rest) % (4*step_count+1) : 0;
            } else {
                // The series approximates target, set the final value to what it should be instead
                step_pulse = cruise_step_pulse;
            }
}

u32 Stepper::next_step(void) {

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

// variables
i32 Stepper::step_coord(void) {
  return _step_coord;
}
i32 Stepper::um_coord(void) {
  return _step_coord * _um_per_step;
}
u32 Stepper::total_steps_moved(void) {
  return _total_steps_moved;
}
u64 Stepper::um_moved(void) {
  return (u64) _total_steps_moved * (u64) _um_per_step;
}
bool Stepper::set_step_coord(i32 step_coord) {
  _step_coord = step_coord;
  return true;
}
bool Stepper::set_total_steps_moved(u32 total_steps_moved) {
  _total_steps_moved = 0;
  return true;
}

}

#pragma once

#include <Arduino.h>
#include <types.hpp>

namespace arm {

struct StepperPins {
  types::u8 nfault_pin = 0;
  types::u8 nreset_pin = 0;
  types::u8 nsleep_pin = 0;
  types::u8 enable_pin = 0;
  types::u8 step_pin = 0;
  types::u8 dir_pin = 0;
  types::u8 m0_pin = 0;
  types::u8 m1_pin = 0;
  types::u8 m2_pin = 0;
};

// Displacement units are in steps. Timings are in microseconds.
struct StepperLinearAccel {
  types::u32 speed = 0;
  types::u32 accel = 0;

  // step timing profile descriptors
  bool direction = 0; // the direction of the net move.
  types::u32 move_steps = 0;
  types::u32 initial_decel_steps = 0; // this is for when overriding movement when it is already moving.
  bool initial_decel_direction = 0;
  types::u32 accel_steps = 0;
  bool accel_direction = 0;
  types::u32 final_decel_steps = 0; // when stopping.
  bool final_decel_direction = 0;
  types::u32 cruise_step_timing = 0; // fixed speed
  types::u32 from_zero_step_timing = 0; // constant for some accel value

  // trackers of step timing profile
  types::u32 step_count = 0;
  types::u64 last_step_time = 0;
  types::u32 step_timing = 0;
  types::u32 step_timing_remainder = 0;
  bool current_direction = 0;
};

// Currently does not implement microstepping. Will be noisy, but 0.2mm resolution is plenty for foosball.
class Stepper {
public:
  // max speed and accel are in mm/s and mm/s^2 units
  Stepper(types::u8 nfault_pin, types::u8 nreset_pin, types::u8 nsleep_pin,
          types::u8 enable_pin, types::u8 step_pin, types::u8 dir_pin,
          types::u8 m0_pin, types::u8 m1_pin, types::u8 m2_pin,
          types::u16 steps_per_rev = 200, types::u8 mm_per_rev = 40);

  bool begin(types::u32 max_speed, types::u32 max_accel, types::u8 microsteps = 1);

  bool enable(void);
  bool disable(void);
  bool faulted(void);
  bool reset(void);
  // blocking
  bool move(types::i32 pos, types::u32 speed = 0, types::u32 accel = 0); // move and come to a stop at some position, while keeping to the max speed and accels specified.
  // non blocking
  bool setup_move(types::i32 pos, types::u32 speed = 0, types::u32 accel = 0); // will override any existing moves by default.
  // non blocking
  bool start_move_continous(types::u32 speed, types::u32 accel);
  types::u32 next_step(void);
  types::u32 next_step_override(void);

  // get and set for settings and constants
  types::u32 max_speed(void); // this is in mm/s
  types::u32 max_accel(void); // this is in mm/s^2
  types::u16 steps_per_rev(void);
  types::u8 mm_per_rev(void);
  types::u16 um_per_step(void);
  types::u32 max_steps_per_second(void);
  types::u32 max_steps_accel(void); // this is in steps/s^2
  types::u8 microsteps(void);
  bool set_max_speed(types::u32 max_speed);
  bool set_max_accel(types::u32 max_accel);
  bool set_steps_per_rev(types::u16 steps_per_rev);
  bool set_mm_per_rev(types::u8 mm_per_rev);
  bool set_microsteps(types::u8 microsteps);

  // get and set for variables
  types::i32 step_coord(void);
  types::i32 um_coord(void); // coordinate in micrometers
  bool set_step_coord(types::i32 step_coord);
  bool set_total_steps_moved(types::u32 total_steps_moved);

private:
  template<typename T>
  T mm_to_steps(T mm) { // this is probably unsafe, but im too lazy to limit it to only ints.
    T steps = mm * 1000 / _um_per_step;
    return steps;
  }
  void calc_timing(void);
  bool clear_move(void);
  // pins
  StepperPins _pins;

  // constants
  types::u16 _steps_per_rev = 0;
  types::u8 _mm_per_rev = 0;

  // computed constants
  types::u16 _um_per_step = 0; // micrometers per step (no floating point due to drift)

  // settings
  types::u8 _microsteps = 0; // 1. 2. 4. 8. 16. 32
  types::u32 _max_speed = 0; // this is in mm/s
  types::u32 _max_accel = 0; // this is in mm/s^2

  // computed settings
  types::u32 _max_steps_per_second = 0;
  types::u32 _max_steps_accel = 0; // this is in steps/s^2

  // variables
  types::u8 _microstep_counter = 0; // which microstep the stepper is currently in.
  types::i32 _step_coord = 0; // coordinate in steps
  
  // acceleration handling
  StepperLinearAccel _stepper_linear_accel;
};

}

#include <Arduino.h>
#include <Servo.h>
#include "pico/mutex.h"
#include "stepper.hpp"
#include "homing.hpp"
#include "debug.hpp"
#include "types.hpp"
#include "pinouts.hpp"

using namespace types;

bool core1_separate_stack = true;

const u16 stepper_cmd_buffer_size = 512;
mutex_t stepper_cmd_mutex;
bool stepper_cmd_updated = false;
u32 stepper_cmd_len = 0;
u8 stepper_cmd[stepper_cmd_buffer_size] = {0}; // 512 bytes should be much more than enough.

const u16 stepper_response_buffer_size = 512;

bool core1_ready = false;

// stepper
const u16 steps_per_rev = 200;
const u8 mm_per_rev = 40;
const u32 default_max_speed = 5000;
const u32 default_max_accel = 50000;
const u8 default_microsteps = 1;
u32 steps_moved_in_current_move = 0;
arm::Stepper stepper(DRV_NFAULT, DRV_NRESET, DRV_NSLEEP,
                     DRV_ENABLE, DRV_STEP, DRV_DIR,
                     DRV_MODE0, DRV_MODE1, DRV_MODE2, steps_per_rev, mm_per_rev);

// servo
const u8 servo_gear_ratio = 4;
const u16 servo_range = 180; // in degrees
const u16 servo_geared_range = servo_range * servo_gear_ratio; // in degrees
const u16 servo_min_pulse = 500;
const u16 servo_max_pulse = 2500;
const u16 servo_deadband = 7;
Servo servo;

/* Command format:
 * Byte 1: MSB 0 = command for core0, MSB 1 = command for core1. Same for responses.
 * The rest of the bits should be its payload length in bytes.
 */

const u8 which_core_bitmask = 0b10000000;
const u8 core0_val = 0b00000000;
const u8 core1_val = 0b10000000;
const u8 payload_length_bitmask = 0b01111111;

enum class StepperCommands : u8 {
  enable = 0,
  disable = 1,
  reset = 2,
  move = 3,
  home = 4,

  fault_state = 5,
  get_max_speed = 6,
  get_max_accel = 7,
  get_steps_per_rev = 8,
  get_mm_per_rev = 9,
  get_um_per_step = 10,
  get_max_steps_per_second = 11,
  get_max_steps_accel = 12,
  get_microsteps = 13,

  set_max_speed = 14,
  set_max_accel = 15,
  set_steps_per_rev = 16,
  set_mm_per_rev = 17,
  set_microsteps = 18,

  get_step_coord = 19,
  get_um_coord = 20,
  get_current_speed = 21,
  set_step_coord = 22,
};

bool estopped = false;
enum class Core0Commands : u8 {
  estop = 0,
  restart = 1,
  
  move_servo = 2,
};

/* Response format:
 * Byte 1: MSB 0 = command failed, MSB 1 = command success
 */
// for stepper move, respond with a special code
const u8 stepper_move_complete_response = 0b01010101;

// Multicore notes:
// Serial class writes are already protected by mutex. Initialise in one core, and can write from both. Keep one full packet as one write. Flush after.
// Serial reads should be done on core 0.
void setup() {
  while (!core1_ready); // wait until core1 is ready so that commands do not get voided.

  Serial.begin();
  // Serial.println("setting up core 0");
  
  // servo
  pinMode(SERVO_DIR, OUTPUT);
  digitalWrite(SERVO_DIR, HIGH); // set direction from SERVO_CTRL to SERVO_CTRL on 5V.
  servo.attach(SERVO_CTRL, servo_min_pulse, servo_max_pulse);
  // Serial.println("set up core 0");
}

void loop() {
  // delay(100);
  // poll if any serial sent.
  if (Serial.available()) {
    // Serial.println("Serial recieved");
    u8 first_byte = Serial.read();

    // whether on first core or not
    bool is_core0_cmd = (first_byte & which_core_bitmask) == core0_val;

    // read payload
    u8 payload_length = first_byte & payload_length_bitmask;
    // Serial.print("payload length: "); Serial.println(payload_length);
    u8 payload_buffer[stepper_cmd_buffer_size] = {0};
    for (u16 i = 0; i < payload_length; i++) {
      while (!Serial.available());
      payload_buffer[i] = Serial.read();
      // Serial.println("read byte");
    }

    if (estopped) { // ignore anything other than restart command.
      if (is_core0_cmd && (Core0Commands) payload_buffer[0] == Core0Commands::restart) {
        watchdog_reboot(0, 0, 0); // restart entire program. This is simpler.
      }
    }
    else if (is_core0_cmd) { // now update command
      Core0Commands command = (Core0Commands) payload_buffer[0];
      switch (command) {
        case Core0Commands::estop: {
          estopped = true;
          rp2040.idleOtherCore(); // stop core1
          break;
        }
        case Core0Commands::move_servo: { // no homing for now...
          u16 target_angle;
          memcpy(&target_angle, payload_buffer + 1, sizeof(target_angle));
          servo.write((float) target_angle / servo_gear_ratio);
          break;
        }
        case Core0Commands::restart: { // ignore
          break;
        }
      }
    }
    else { // core1 command
      // Serial.println("is core1 command");
      // grab mutex
      mutex_enter_blocking(&stepper_cmd_mutex);
      // Serial.println("grabbed mutex");
      memcpy(stepper_cmd, payload_buffer, sizeof(stepper_cmd));
      stepper_cmd_updated = true;
      // Serial.println("set command for core1");
      // release mutex
      mutex_exit(&stepper_cmd_mutex);
      // Serial.println("released mutex");
    }
  }
}

void setup1() { // runs the stepper.
  core1_ready = false;

  mutex_init(&stepper_cmd_mutex);
  mutex_enter_blocking(&stepper_cmd_mutex);
  stepper_cmd_updated = false;
  mutex_exit(&stepper_cmd_mutex);
  stepper.begin(default_max_speed, default_max_accel, default_microsteps);
  stepper.enable();

  core1_ready = true;
}

void loop1() {
  // delay(100);
  // Serial.println("core 1 grabbed mutex");
  mutex_enter_blocking(&stepper_cmd_mutex);
  if (stepper_cmd_updated) {
    StepperCommands command = (StepperCommands) stepper_cmd[0];
    switch (command) {
      case StepperCommands::disable: {
        stepper.disable();
        Serial.write(true);
        break;
      }
      case StepperCommands::enable: {
        stepper.enable();
        Serial.write(true);
        break;
      }
      case StepperCommands::reset: {
        stepper.reset();
        Serial.write(true);
        break;
      }
      case StepperCommands::move: {
        // Serial.println("Detected move command");
        i32 move_pos;
        memcpy(&move_pos, stepper_cmd + 1, sizeof(move_pos));
        u32 move_speed;
        memcpy(&move_speed, stepper_cmd + 1 + sizeof(move_pos), sizeof(move_speed));
        u32 move_accel;
        memcpy(&move_accel, stepper_cmd + 1 + sizeof(move_pos) + sizeof(move_speed), sizeof(move_accel));
        // Serial.println(steps_moved_in_current_move);
        // Serial.println(move_pos); Serial.println(move_speed); Serial.println(move_accel);
        stepper.setup_move(move_pos, move_speed, move_accel);
        steps_moved_in_current_move = 0;
        Serial.write(true);
        break;
      }
      case StepperCommands::home: {
        u8 moveforward_mm = stepper_cmd[1];
        u8 movebackward_mm = stepper_cmd[2];
        u8 midpoint_pos = stepper_cmd[3];
        u32 home_speed;
        memcpy(&home_speed, stepper_cmd + 4, sizeof(home_speed));
        u32 home_accel;
        memcpy(&home_accel, stepper_cmd + 4 + sizeof(home_speed), sizeof(home_accel));
        arm::home_stepper(stepper, ENDSTOP, moveforward_mm, movebackward_mm, midpoint_pos, home_speed, home_accel);
        Serial.write(true);
        break;
      }

      case StepperCommands::fault_state: {
        u8 fault_state = (u8) stepper.faulted();
        Serial.write((u8*) &fault_state, sizeof(fault_state));
        break;
      }
      case StepperCommands::get_max_speed: {
        u32 max_speed = stepper.max_speed();
        Serial.write((u8*) &max_speed, sizeof(max_speed));
        break;
      }
      case StepperCommands::get_max_accel: {
        u32 max_accel = stepper.max_accel();
        Serial.write((u8*) &max_accel, sizeof(max_accel));
        break;
      }
      case StepperCommands::get_steps_per_rev: {
        u16 steps_per_rev = stepper.steps_per_rev();
        Serial.write((u8*) &steps_per_rev, sizeof(steps_per_rev));
        break;
      }
      case StepperCommands::get_mm_per_rev: {
        u8 mm_per_rev = stepper.mm_per_rev();
        Serial.write((u8*) &mm_per_rev, sizeof(mm_per_rev));
        break;
      }
      case StepperCommands::get_um_per_step: {
        u16 um_per_step = stepper.um_per_step();
        Serial.write((u8*) &um_per_step, sizeof(um_per_step));
        break;
      }
      case StepperCommands::get_max_steps_per_second: {
        u32 max_steps_per_second = stepper.max_steps_per_second();
        Serial.write((u8*) &max_steps_per_second, sizeof(max_steps_per_second));
        break;
      }
      case StepperCommands::get_max_steps_accel: {
        u32 max_steps_accel = stepper.max_steps_accel();
        Serial.write((u8*) &max_steps_accel, sizeof(max_steps_accel));
        break;
      }
      case StepperCommands::get_microsteps: {
        u8 microsteps = stepper.microsteps();
        Serial.write((u8*) &microsteps, sizeof(microsteps));
        break;
      }

      case StepperCommands::set_max_speed: {
        u32 max_speed;
        memcpy(&max_speed, stepper_cmd + 1, sizeof(max_speed));
        stepper.set_max_speed(max_speed);
        break;
      }
      case StepperCommands::set_max_accel: {
        u32 max_accel;
        memcpy(&max_accel, stepper_cmd + 1, sizeof(max_accel));
        stepper.set_max_accel(max_accel);
        break;
      }
      case StepperCommands::set_steps_per_rev: {
        u16 steps_per_rev;
        memcpy(&steps_per_rev, stepper_cmd + 1, sizeof(steps_per_rev));
        stepper.set_steps_per_rev(steps_per_rev);
        Serial.write(true);
        break;
      }
      case StepperCommands::set_mm_per_rev: {
        u8 mm_per_rev;
        memcpy(&mm_per_rev, stepper_cmd + 1, sizeof(mm_per_rev));
        stepper.set_mm_per_rev(mm_per_rev);
        Serial.write(true);
        break;
      }
      case StepperCommands::set_microsteps: {
        u8 microsteps;
        memcpy(&microsteps, stepper_cmd + 1, sizeof(microsteps));
        stepper.set_microsteps(microsteps);
        Serial.write(true);
        break;
      }

      case StepperCommands::get_step_coord: {
        i32 step_coord = stepper.step_coord();
        Serial.write((u8*) &step_coord, sizeof(step_coord));
        break;
      }
      case StepperCommands::get_um_coord: {
        i32 um_coord = stepper.um_coord();
        Serial.write((u8*) &um_coord, sizeof(um_coord));
        break;
      }
      case StepperCommands::get_current_speed: {
        i32 current_speed = stepper.current_speed();
        Serial.write((u8*) &current_speed, sizeof(current_speed));
        break;
      }
      case StepperCommands::set_step_coord: {
        i32 step_coord;
        memcpy(&step_coord, stepper_cmd + 1, sizeof(step_coord));
        stepper.set_step_coord(step_coord);
        Serial.write(true);
        break;
      }
    }
    stepper_cmd_updated = false;
    memset(stepper_cmd, 0, sizeof(stepper_cmd));
  }
  mutex_exit(&stepper_cmd_mutex);
  // Serial.println("core 1 released mutex");
  if (stepper.in_move()) {
    steps_moved_in_current_move += 1;
    if (!stepper.next_step()) {
      Serial.write(stepper_move_complete_response);
    }
  }
}

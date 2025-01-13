#include <Arduino.h>
#include "stepper.hpp"
#include "servo.hpp"
#include "debug.hpp"
#include "types.hpp"
#include "pinouts.hpp"

using namespace types;

bool core1_separate_stack = true;

mutex_t stepper_cmd_updated_mutex;
bool stepper_cmd_updated = false;

mutex_t core0_ready_mutex;
bool core0_ready = false;
const u32 core1_check_core0_ready_micros = 100;

bool idk = __isFreeRTOS;

void setup() {
  Serial.begin();
}

void loop() {
  Serial.println(idk);
}

// enum class Core1Commands : u32 {
// };

// Multicore notes:
// Serial class writes are already protected by mutex. Initialise in one core, and can write from both. Keep one full packet as one write. Flush after.
// Serial reads should be done on core 0.
// void setup() {
//   CoreMutex ready_m(&core0_ready_mutex); // grab core0_ready_mutex
//   core0_ready = false;
//   delete &ready_m; // release core0_ready_mutex
//   Serial.begin();
// }
//
// void setup1() {
//   bool run = false;
//   do {
//     CoreMutex ready_m(&core0_ready_mutex); // grab core0_ready_mutex
//     if (core0_ready = false) {
//       delayMicroseconds(core1_check_core0_ready_micros);
//     }
//
//   }
// }

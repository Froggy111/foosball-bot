#include <Arduino.h>
#include "types.hpp"
#include "stepper.hpp"
#include "debug.hpp"

using namespace types;

const u8 DRV_ENABLE = 8;
const u8 DRV_DIR = 1;
const u8 DRV_STEP = 2;
const u8 DRV_NSLEEP = 3;
const u8 DRV_MODE0 = 7;
const u8 DRV_MODE1 = 6;
const u8 DRV_MODE2 = 5;
const u8 DRV_NFAULT = 0;
const u8 DRV_NRESET = 4;

const u8 led_pin = 25;

arm::Stepper stepper(DRV_NFAULT, DRV_NRESET, DRV_NSLEEP, DRV_ENABLE, DRV_STEP, DRV_DIR, DRV_MODE0, DRV_MODE1, DRV_MODE2);

// #define TEST_OVERRIDE

void setup() {
  Serial.begin(115200);
  delay(5000);
  debug::printf("Starting setup\n");
  #ifdef TEST_OVERRIDE
  stepper.begin(100, 1000, 16);
  #else
  // pinMode(led_pin, OUTPUT);
  // digitalWrite(led_pin, LOW);
  stepper.begin(100, 500, 1);
  stepper.enable();
  #endif
  debug::printf("Completed setup\n");
}

void loop() {
  #ifdef TEST_OVERRIDE
  debug::printf("move forward\n");
  stepper.setup_move_override(100);
  for (u8 i = 0; i < 50; i++) {
    stepper.next_step();
  }
  debug::printf("move backward with override\n");
  stepper.setup_move_override(0);
  while (stepper.next_step());
  #else
  debug::printf("move forward\n");
  stepper.setup_move(100);
  while(stepper.next_step()) {
    delay(10);
  }
  debug::printf("move backward with override\n");
  stepper.setup_move(0);
  while (stepper.next_step()) {
    delay(10);
  }
  #endif
}

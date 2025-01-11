#include "types.hpp"
#include "stepper.hpp"

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

void setup() {
  stepper.begin(100, 100);
}

void loop() {
  stepper.move(50, 100, 100);
  stepper.move(0, 100, 100);
}

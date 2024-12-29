/*
 * Simple demo, should work with any driver board
 *
 * Connect STEP, DIR as indicated
 *
 * Copyright (C)2015-2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <SPI.h>
#include <Arduino.h>
#include <DRV8825.h>
#include <BasicStepperDriver.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
const uint8_t steps_per_rev = 200;
const uint8_t target_rpm = 10;

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
const uint8_t DRV_MICROSTEPS = 4;

const uint8_t DRV_ENABLE = 8;
const uint8_t DRV_DIR = 1;
const uint8_t DRV_STEP = 2;
const uint8_t DRV_SLEEP = 3;
const uint8_t DRV_MODE0 = 7;
const uint8_t DRV_MODE1 = 6;
const uint8_t DRV_MODE2 = 5;
const uint8_t DRV_FAULT = 0;

const uint8_t led_pin = 25;

DRV8825 stepper(steps_per_rev, DRV_DIR, DRV_STEP, DRV_SLEEP, DRV_MODE0, DRV_MODE1, DRV_MODE2);

//Uncomment linep to use enable/disable functionality
// BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, SLEEP);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup");
  pinMode(DRV_ENABLE, OUTPUT);
  digitalWrite(DRV_ENABLE, LOW);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);
  pinMode(DRV_FAULT, INPUT);
  stepper.begin(target_rpm, DRV_MICROSTEPS);
  // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
  // stepper.setEnableActiveState(LOW);
  Serial.println("Completed setup");
}

void loop() {
  digitalWrite(led_pin, HIGH);
  Serial.println("Loop");
  Serial.println(digitalRead(DRV_FAULT));

  // energize coils - the motor will hold position
  // stepper.enable();

  /*
   * Moving motor one full revolution using the degree notation
   */
  delay(10);
  digitalWrite(led_pin, LOW);
  stepper.rotate(360);
  delay(1000);
  stepper.rotate(-360);
}

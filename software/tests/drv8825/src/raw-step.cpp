#include <SPI.h>
#include <Arduino.h>

const uint8_t steps_per_rev = 200;
const uint8_t target_rpm = 10;

const uint8_t DRV_ENABLE = 8;
const uint8_t DRV_DIR = 1;
const uint8_t DRV_STEP = 2;
const uint8_t DRV_SLEEP = 3;
const uint8_t DRV_MODE0 = 7;
const uint8_t DRV_MODE1 = 6;
const uint8_t DRV_MODE2 = 5;
const uint8_t DRV_FAULT = 0;
const uint8_t DRV_RESET = 4;

const uint8_t led_pin = 25;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup");
  pinMode(DRV_ENABLE, OUTPUT);
  digitalWrite(DRV_ENABLE, LOW);
  pinMode(DRV_DIR, OUTPUT);
  pinMode(DRV_STEP, OUTPUT);
  pinMode(DRV_SLEEP, OUTPUT);
  digitalWrite(DRV_SLEEP, HIGH);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);
  pinMode(DRV_FAULT, INPUT);

  pinMode(DRV_RESET, OUTPUT);
  digitalWrite(DRV_RESET, LOW);
  delay(1000);
  digitalWrite(DRV_RESET, HIGH);

  Serial.println("Completed setup");
}

void loop() {
  digitalWrite(led_pin, HIGH);
  Serial.println("Loop");
  Serial.println(digitalRead(DRV_FAULT));
  delay(10);
  digitalWrite(led_pin, LOW);

  digitalWrite(DRV_DIR, HIGH);
  for (int i = 0; i < steps_per_rev; i++) {
    // These four lines result in 1 step:
    digitalWrite(DRV_STEP, HIGH);
    delayMicroseconds(2000);
    digitalWrite(DRV_STEP, LOW);
    delayMicroseconds(2000);
  }

  digitalWrite(DRV_DIR, LOW);
  for (int i = 0; i < steps_per_rev; i++) {
    // These four lines result in 1 step:
    digitalWrite(DRV_STEP, HIGH);
    delayMicroseconds(2000);
    digitalWrite(DRV_STEP, LOW);
    delayMicroseconds(2000);
  }
}

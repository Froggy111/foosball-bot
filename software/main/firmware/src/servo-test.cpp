#include <Arduino.h>
#include <RP2040_PWM.h>
#include "types.hpp"
#include "stepper.hpp"
#include "servo.hpp"
#include "debug.hpp"

#define _PWM_LOGLEVEL_ 3

using namespace types;

const u8 servo_pin_num = 14;
const u8 servo_dir = 15;

const u8 led_pin = 25;

arm::Servo servo(servo_pin_num, servo_dir);

void setup() {
    Serial.begin(115200);
    pinMode(25, OUTPUT);
    digitalWrite(25, HIGH);
    pinMode(15, OUTPUT);
    digitalWrite(15, HIGH);
    servo.begin();
    digitalWrite(25, LOW);
    delay(1000);
}

void loop() {
    delay(3000);
    digitalWrite(25, HIGH);
    servo.move(1800);
    delay(3000);
    digitalWrite(25, LOW);
    servo.move(0);
}

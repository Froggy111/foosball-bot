#include <Arduino.h>
#include "types.hpp"
#include "stepper.hpp"
#include "debug.hpp"
#include <Servo.h>

#define _PWM_LOGLEVEL_ 3

using namespace types;

const u8 servo_pin_num = 14;
const u8 servo_dir = 15;

const u8 led_pin = 25;

Servo servo;

void setup() {
    Serial.begin(115200);
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, HIGH);
    pinMode(servo_dir, OUTPUT);
    digitalWrite(servo_dir, HIGH);
    servo.attach(servo_pin_num);
    delay(1000);
}

void loop() {
    servo.write(180);
    delay(1000);
    servo.write(0);
    delay(1000);
}

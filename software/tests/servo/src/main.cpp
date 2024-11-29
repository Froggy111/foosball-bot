#include <Arduino.h>
#include <Servo.h>

const uint8_t servo_pin = 0;

Servo servo;

void setup() {
  Serial.begin(115200);
  // while (!Serial);
  delay(100);
  Serial.println("Starting setup");
  // pinMode(servo_pin, OUTPUT);
  // analogWriteResolution(8);
  servo.detach();
  Serial.println(servo.attach(servo_pin, 500, 2500));
  if (servo.attached()) {
    Serial.println("Servo is attached.");
  }
  else {
    Serial.println("Servo not attached.");
  }
  servo.write(0);
  Serial.println("Completed setup");
}

void loop() {
  Serial.println("Loop");
  servo.write(180);
  delay(1000);
  servo.write(0);
  delay(1000);
  // for (int i = 0; i <= 180; i++) {
  //   // analogWrite(servo_pin, i);
  //   servo.write(i);
  //   delay(10);
  // }
  // for (int i = 180; i >= 0; i--) {
  //   // analogWrite(servo_pin, i);
  //   servo.write(i);
  //   delay(10);
  // }
}

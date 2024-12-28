#include <Arduino.h>

const uint8_t led_pin = 25;

void setup() {
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);
}

void loop() {
  delay(100);
  digitalWrite(led_pin, 1);
  delay(100);
  digitalWrite(led_pin, 0);
}

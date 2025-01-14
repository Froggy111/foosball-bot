#include <Arduino.h>

void setup(){
    pinMode(25, OUTPUT);
    Serial.begin();
}

void loop(){
    if(Serial.available()){
        Serial.read();
        digitalWrite(25, HIGH);
        delay(1000);
        digitalWrite(25, LOW);
    }
}
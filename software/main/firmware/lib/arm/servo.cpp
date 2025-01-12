#include <Arduino.h>
#include "types.hpp"
#include "servo.hpp"
#include "debug.hpp"

#define _PWM_LOGLEVEL_ 3

using namespace types;

namespace arm {

Servo::Servo(u8 servo_pin, u8 servo_dir, u8 gear_ratio, u16 min_pulse, u16 max_pulse, u16 dead_band, u16 max_speed){
    _servo_pin = servo_pin;
    _servo_dir = servo_dir;
    _gear_ratio = gear_ratio;
    _min_pulse = min_pulse;
    _max_pulse = max_pulse;
    _dead_band = dead_band;
}

void Servo::begin(){
    // pinMode(_servo_pin, OUTPUT);
    // pinMode(_servo_dir, OUTPUT);
    // digitalWrite(_servo_dir, HIGH);
    PWM_Instance = new RP2040_PWM(_servo_pin, 50, 0);
    // _curr_ang = 0;
}

void Servo::move(u16 target_ang){
    if (target_ang > 7200){
        target_ang = target_ang % 7200;
    }
    f64 pulsewidth = target_ang * ((_max_pulse-_min_pulse)/7200) + _min_pulse;  
    f64 duty_cycle = pulsewidth / _period;
    delay(1000);
    PWM_Instance->setPWM(_servo_pin, 50, duty_cycle);
}

}
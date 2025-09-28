#include "PID.hpp"

namespace FOC {

PID::PID(float Kp, float Ki, float Kd, float frequency)
    : Kp(Kp), Ki(Ki), Kd(Kd), frequency(frequency) {}

void PID::update(float actual) {
    past_err = curr_err;
    curr_err = (target - actual) * frequency;
    sum_err += curr_err / frequency;
    this->actual = actual;
    return;
}

void PID::set(float target) {
    this->target = target;
    return;
}

float PID::get(void) {
    float p = curr_err * Kp;
    float i = sum_err * Ki;
    float d = (curr_err - past_err) * Kd;
    float pid = p + i + d;
    return pid;
}

float PID::get_target(void) { return target; }

float PID::get_actual(void) { return actual; }

void PID::set_params(float Kp, float Ki, float Kd, float frequency) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->frequency = frequency;
    return;
}

}  // namespace FOC

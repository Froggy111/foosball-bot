#include "PID.hpp"

namespace FOC {

PID::PID(float Kp, float Ki, float Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {}

void PID::update(float actual) {
    past_err = curr_err;
    curr_err = target - actual;
    sum_err += curr_err;
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

void PID::set_params(float Kp, float Ki, float Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    return;
}

}  // namespace FOC

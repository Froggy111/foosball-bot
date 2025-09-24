#pragma once

namespace FOC {

class PID {
   public:
    PID(float Kp, float Ki, float Kd);
    void update(float actual);
    void set(float target);
    float get(void);
    float get_target(void);
    void set_params(float Kp, float Ki, float Kd);

   private:
    float Kp = 0, Ki = 0, Kd = 0;
    float curr_err = 0, sum_err = 0, past_err = 0;
    float target = 0;
};

}  // namespace FOC

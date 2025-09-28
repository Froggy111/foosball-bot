#pragma once

namespace FOC {

// INFO : The frequency is how frequently the loop runs (to keep units
// INFO : in seconds for intuitive Ki and Kd values). D errors are multiplied by
// INFO : this and I errors are divided by this.
class PID {
   public:
    PID(float Kp, float Ki, float Kd, float frequency);
    void update(float actual);
    void set(float target);
    float get(void);
    float get_target(void);
    float get_actual(void);
    void set_params(float Kp, float Ki, float Kd, float frequency);

   private:
    float Kp = 0, Ki = 0, Kd = 0, frequency = 1.0f;
    float curr_err = 0, sum_err = 0, past_err = 0;
    float target = 0;
    float actual = 0;
};

}  // namespace FOC

#pragma once

namespace FOC {

// first-order low pass
class LowPassFilter {
   public:
    LowPassFilter(float alpha) : alpha(alpha), last_output(0.0f) {}
    float process(float input) {
        last_output = alpha * input + (1.0f - alpha) * last_output;
        return last_output;
    }

   private:
    float alpha = 0, last_output = 0;
};

}  // namespace FOC

#pragma once

namespace cordic {
struct SinCosVal {
    float sin = 0;
    float cos = 0;
};
void init(void);
SinCosVal sincos(float theta);
}  // namespace cordic

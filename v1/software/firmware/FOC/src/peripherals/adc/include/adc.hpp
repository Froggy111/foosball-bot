#pragma once

namespace adc {

void init(void);

float read_VMOT(void);
float read_12V(void);
float read_5V(void);

float read_U_current(void);
float read_V_current(void);
float read_W_current(void);

}  // namespace adc

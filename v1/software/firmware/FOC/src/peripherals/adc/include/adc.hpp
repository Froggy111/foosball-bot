#pragma once

namespace adc {

void init(void);

// start the conversion for VMOT
void start_VMOT_read(void);
// block for value of VMOT
float read_VMOT(void);

// start the conversion for 12V
void start_12V_read(void);
// block for value of 12V
float read_12V(void);

// start the conversion for 5V
void start_5V_read(void);
// block for value of 5V
float read_5V(void);

float read_U_current(void);
float read_V_current(void);
float read_W_current(void);

}  // namespace adc

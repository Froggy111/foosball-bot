#pragma once

#include "config.hpp"

namespace adc {

struct Values {
    float voltage_VMOT = 0;
#ifdef HAVE_12V_SENSE
    float voltage_12V = 0;
#endif
#ifdef HAVE_5V_SENSE
    float voltage_5V = 0;
#endif
    float current_U = 0;
    float current_V = 0;
    float current_W = 0;
};

void init(void);

// start conversions
void start_conversions(void);
// read values
Values read(void);

// used in core_interrupts.cpp
void DMA_ADC1_handler(void);
void DMA_ADC2_handler(void);
void ADC1_conversion_complete_callback(void);
void ADC2_conversion_complete_callback(void);

}  // namespace adc

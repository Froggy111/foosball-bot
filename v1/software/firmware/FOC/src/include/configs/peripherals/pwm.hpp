#pragma once

#include <stdint.h>

/**
 * @brief PWM peripheral settings.
 */
// PWM resolution must be above 1 << PWM_MIN_RESOLUTION. This is prioritised
// over frequency deviation
const uint32_t PWM_MIN_RESOLUTION = 6;  // 6 bits of resolution at minimum
// PWM frequency is as far as possible maintained to deviate less than
// PWM_MAX_FREQ_DEVIATION
const float PWM_MAX_FREQ_DEVIATION = 0.05;  // 5% max deviation

// Configuration for drive phase
#define DRIVE_PHASE_TIMER TIM1  // U, V, W phases on TIM1
#define DRIVE_PHASE_CLOCK PCLK2
#define PCLK1 1
#define PCLK2 2

#define U_PHASE_CHANNEL TIM_CHANNEL_1
#define V_PHASE_CHANNEL TIM_CHANNEL_2
#define W_PHASE_CHANNEL TIM_CHANNEL_3

const uint32_t DRIVE_BREAKTIME = 500;  // in ns

#pragma once
// configs for FOC firmware

#include <stdint.h>
#include <stm32g474xx.h>

/**
 * @brief Target for stdio functions (printf)
 * @options USB, CANFD, UART
 */
#define STDIO_TARGET STDIO_USB
#define STDIO_NULL 0
#define STDIO_USB 1
#define STDIO_CANFD 2
#define STDIO_UART 3

/**
 * @brief Debug output settings. Comment out defines to disable a log level.
 */
#ifndef NDEBUG

#define DEBUG_ENABLED
#define DEBUG_LEVEL_TRACE
#define DEBUG_LEVEL_DEBUG
#define DEBUG_LEVEL_LOG
#define DEBUG_LEVEL_NOTE
#define DEBUG_LEVEL_WARN
#define DEBUG_LEVEL_ERROR
#define DEBUG_LEVEL_FATAL

#endif

/**
 * @brief USB CDC settings
 */
#define USB_TX_BUFFER_SIZE 256  // number of bytes for TX buffer

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

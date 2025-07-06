#pragma once

#include <stm32g4xx.h>

#include "gpio.hpp"

/**
 * NOTE : SYSCLK is 160MHz
 * NOTE : Maximum ADC clock is 60MHz
 * NOTE : ADC clock here is 40MHz
 * NOTE : All measurement times will be 1.5x longer
 */
#define ADC_CLK_PRESCALER ADC_CLOCK_ASYNC_DIV4

/**
 * NOTE : For STM32G4 ADCs at 12bit:
 * NOTE : Fast ADCs: ADCx_CHANNEL_1 - CHANNEL_5, rest are slow (minimum 6.5)
 * NOTE : Format: <sampling cycle>, <sampling time (@60MHz)>, <time @40MHz>:
 * NOTE : <fast maximum source impedance>, <slow maximum source impedance>
 * 2.5, 41.67ns, 62.51ns: 100, N/A
 * 6.5, 108.33ns, 162.50ns: 330, 100
 * 12.5, 208.33ns, 312.50ns: 680, 470
 * 24.5, 408.33ns, 612.50ns: 1.5k, 1.2k
 * 47.5, 791.67ns, 1187.51ns: 2.2k, 1.8k
 * 92.5, 1541.67ns, 2312.51ns: 4.7k, 3.9k
 * 247.5, 4.125us, 6.188us: 12k, 10k
 * 640.5, 10.675us, 16.013us: 39k, 33k
 */

// PA0, 490K-1K divider
// source impedance = 490K // 1K = 998Ohm
// minimum sampling cycle: 24.5, 612.50ns @40MHz
const gpio::PinConfig VSENSE_VMOT_PIN = {GPIOA, gpio::Pin::PIN0,
                                         gpio::AF::NONE};
#define VSENSE_VMOT_ADC ADC1
#define VSENSE_VMOT_CHANNEL ADC_CHANNEL_1
#define VSENSE_VMOT_SAMPLETIME ADC_SAMPLETIME_24CYCLES_5

// PA6, 32.4K-10K divider
// source impedance = 32.4K // 10K = 7.7kOhm
// minimum sampling cycle: 247.5, 6.188us @40MHz
const gpio::PinConfig VSENSE_12V_PIN = {GPIOA, gpio::Pin::PIN6, gpio::AF::NONE};
#define VSENSE_12V_ADC ADC2
#define VSENSE_12V_CHANNEL ADC_CHANNEL_3
#define VSENSE_12V_SAMPLETIME ADC_SAMPLETIME_247CYCLES_5

// PA5, 32.4K-10K divider
// source impedance = 32.4K // 10K = 7.7kOhm
// minimum sampling cycle: 247.5, 6.188us @40MHz
const gpio::PinConfig VSENSE_5V_PIN = {GPIOA, gpio::Pin::PIN5, gpio::AF::NONE};
#define VSENSE_5V_ADC ADC2
#define VSENSE_5V_CHANNEL ADC_CHANNEL_13
#define VSENSE_5V_SAMPLETIME ADC_SAMPLETIME_247CYCLES_5

const gpio::PinConfig ISENSE_U_PHASE_PIN = {GPIOA, gpio::Pin::PIN1,
                                            gpio::AF::NONE};
#define ISENSE_U_PHASE_OPAMP OPAMP3
#define ISENSE_U_PHASE_ADC ADC1
#define ISENSE_U_PHASE_CHANNEL ADC_CHANNEL_12
#define ISENSE_U_PHASE_SAMPLETIME ADC_SAMPLETIME_6CYCLES_5

const gpio::PinConfig ISENSE_V_PHASE_PIN = {GPIOB, gpio::Pin::PIN0,
                                            gpio::AF::NONE};
#define ISENSE_V_PHASE_OPAMP OPAMP2
#define ISENSE_V_PHASE_ADC ADC2
#define ISENSE_V_PHASE_CHANNEL ADC_CHANNEL_16
#define ISENSE_V_PHASE_SAMPLETIME ADC_SAMPLETIME_6CYCLES_5

const gpio::PinConfig ISENSE_W_PHASE_PIN = {GPIOA, gpio::Pin::PIN7,
                                            gpio::AF::NONE};
#define ISENSE_W_PHASE_OPAMP OPAMP1
#define ISENSE_W_PHASE_ADC ADC1
#define ISENSE_W_PHASE_CHANNEL ADC_CHANNEL_13
#define ISENSE_W_PHASE_SAMPLETIME ADC_SAMPLETIME_6CYCLES_5

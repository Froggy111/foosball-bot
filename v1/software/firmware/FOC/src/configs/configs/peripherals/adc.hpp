#pragma once

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
const uint8_t ADC_POLL_TIMEOUT = 1;        // should never ever exceed 1ms
const uint16_t ADC_RANGE = (1 << 12) - 1;  // 12 bit ADC
const float ADC_VOLTAGE = 3.3f;
const float ADC_VOLTAGE_MULTIPLIER = ADC_VOLTAGE / ADC_RANGE;

// INFO : VOLTAGE SENSING CONFIGS
// PC4, 43K-1K divider
// source impedance = 43K // 1K = 977Ohm
// minimum sampling cycle: 24.5, 612.50ns @40MHz
const gpio::PinConfig VSENSE_VMOT_PIN = {GPIOC, gpio::Pin::PIN4,
                                         gpio::AF::NONE};
#define VSENSE_VMOT_ADC ADC2
#define VSENSE_VMOT_CHANNEL ADC_CHANNEL_5
#define VSENSE_VMOT_SAMPLETIME ADC_SAMPLETIME_24CYCLES_5
const float VSENSE_VMOT_MULTIPLIER = (43.0f + 1.0f) / 1.0f;

// INFO : comment out for drivers without 12V sensing
#define HAVE_12V_SENSE
// PC4, 4.7K-1K divider
// source impedance = 4.7K // 1K = 825Ohm
// minimum sampling cycle: 24.5, 612.50ns @40MHz
const gpio::PinConfig VSENSE_12V_PIN = {GPIOA, gpio::Pin::PIN3, gpio::AF::NONE};
#define VSENSE_12V_ADC ADC1
#define VSENSE_12V_CHANNEL ADC_CHANNEL_4
#define VSENSE_12V_SAMPLETIME ADC_SAMPLETIME_24CYCLES_5
const float VSENSE_12V_MULTIPLIER = (4.7f + 1.0f) / 1.0f;

// INFO : comment out for drivers without 5V sensing
// #define HAVE_5V_SENSE

// INFO : CURRENT SENSING CONFIGS

const float ISENSE_SHUNT = 1e-3f;  // 1mOhm shunt
const float ISENSE_GAIN = 60.4f;   // 60.4x gain pre-amplification
const float ISENSE_CURRENT_PER_VOLT = 1.0f / (ISENSE_SHUNT * ISENSE_GAIN);
// if read voltage is more than ISENSE_HALF_GAIN_THRESHOLD * max_voltage, half
// PGA gain
// WARN : ISENSE_HALF_GAIN_THRESHOLD / 2 > ISENSE_DOUBLE_GAIN_THRESHOLD
const float ISENSE_HALF_GAIN_THRESHOLD_CONF =
    0.8f;  // halved value at 0.4 (0.5 - 0.1)
const uint16_t ISENSE_HALF_GAIN_THRESHOLD =
    ADC_RANGE * ISENSE_HALF_GAIN_THRESHOLD_CONF;
// if read voltage is less than ISENSE_DOUBLE_GAIN_THRESHOLD * max_voltage,
// double PGA gain
// WARN : ISENSE_DOUBLE_GAIN_THRESHOLD * 2 < ISENSE_HALF_GAIN_THRESHOLD
const float ISENSE_DOUBLE_GAIN_THRESHOLD_CONF =
    0.3f;  // doubled value at 0.6 (0.5 + 0.1)
const uint16_t ISENSE_DOUBLE_GAIN_THRESHOLD =
    ADC_RANGE * ISENSE_DOUBLE_GAIN_THRESHOLD_CONF;

const gpio::PinConfig ISENSE_U_PHASE_PIN = {GPIOA, gpio::Pin::PIN7,
                                            gpio::AF::NONE};
#define ISENSE_U_PHASE_OPAMP OPAMP1
#define ISENSE_U_PHASE_ADC ADC1
#define ISENSE_U_PHASE_CHANNEL ADC_CHANNEL_13
#define ISENSE_U_PHASE_SAMPLETIME ADC_SAMPLETIME_6CYCLES_5
#define ISENSE_U_PHASE_OPAMP_POWERMODE OPAMP_POWERMODE_HIGHSPEED
#define ISENSE_U_PHASE_OPAMP_INPUT_NUM OPAMP_NONINVERTINGINPUT_IO2

const gpio::PinConfig ISENSE_V_PHASE_PIN = {GPIOB, gpio::Pin::PIN14,
                                            gpio::AF::NONE};
#define ISENSE_V_PHASE_OPAMP OPAMP2
#define ISENSE_V_PHASE_ADC ADC2
#define ISENSE_V_PHASE_CHANNEL ADC_CHANNEL_16
#define ISENSE_V_PHASE_SAMPLETIME ADC_SAMPLETIME_6CYCLES_5
#define ISENSE_V_PHASE_OPAMP_POWERMODE OPAMP_POWERMODE_HIGHSPEED
#define ISENSE_V_PHASE_OPAMP_INPUT_NUM OPAMP_NONINVERTINGINPUT_IO1

const gpio::PinConfig ISENSE_W_PHASE_PIN = {GPIOB, gpio::Pin::PIN13,
                                            gpio::AF::NONE};
#define ISENSE_W_PHASE_OPAMP OPAMP3
#define ISENSE_W_PHASE_ADC ADC2
#define ISENSE_W_PHASE_CHANNEL ADC_CHANNEL_18
#define ISENSE_W_PHASE_SAMPLETIME ADC_SAMPLETIME_6CYCLES_5
#define ISENSE_W_PHASE_OPAMP_POWERMODE OPAMP_POWERMODE_HIGHSPEED
#define ISENSE_W_PHASE_OPAMP_INPUT_NUM OPAMP_NONINVERTINGINPUT_IO1

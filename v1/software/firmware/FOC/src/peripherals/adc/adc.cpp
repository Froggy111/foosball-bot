#include "adc.hpp"

#include <cmsis_os2.h>
#include <stm32g4xx.h>
#include <stm32g4xx_ll_opamp.h>

#include "config.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "gpio.hpp"

void opamp_init(void);
void adc_init(void);
void gpio_init(void);

void opamp_instance_init(OPAMP_HandleTypeDef* handle, OPAMP_TypeDef* instance,
                         uint32_t input_num, uint32_t power_mode);

uint8_t adc_instance_to_idx(ADC_TypeDef* instance);
ADC_HandleTypeDef* adc_instance_init(ADC_TypeDef* instance);

void start_ADC_read(ADC_HandleTypeDef* handle, ADC_ChannelConfTypeDef* channel);
uint32_t read_ADC(ADC_HandleTypeDef* handle, uint32_t timeout);

enum class PGAGain : uint8_t {
    GAIN2 = 2,
    GAIN4 = 4,
    GAIN8 = 8,
    GAIN16 = 16,
    GAIN32 = 32,
    GAIN64 = 64,
};

PGAGain double_gain(PGAGain gain) {
    uint8_t current_gain = (uint8_t)gain;
    if (current_gain >= (uint8_t)PGAGain::GAIN64) {
        return PGAGain::GAIN64;
    } else
        return (PGAGain)((uint8_t)gain * 2);
}
PGAGain half_gain(PGAGain gain) {
    uint8_t current_gain = (uint8_t)gain;
    if (current_gain <= (uint8_t)PGAGain::GAIN2) {
        return PGAGain::GAIN2;
    } else
        return (PGAGain)((uint8_t)gain / 2);
}

void set_PGA(OPAMP_HandleTypeDef* handle, PGAGain gain);
float read_PGA(OPAMP_HandleTypeDef* opamp, ADC_HandleTypeDef* adc,
               ADC_ChannelConfTypeDef* channel, PGAGain* gain,
               uint32_t timeout);

PGAGain U_phase_gain = PGAGain::GAIN2;
PGAGain V_phase_gain = PGAGain::GAIN2;
PGAGain W_phase_gain = PGAGain::GAIN2;

OPAMP_HandleTypeDef U_phase_opamp;
OPAMP_HandleTypeDef V_phase_opamp;
OPAMP_HandleTypeDef W_phase_opamp;

ADC_HandleTypeDef ADC_1;
ADC_HandleTypeDef ADC_2;
ADC_HandleTypeDef ADC_3;
ADC_HandleTypeDef ADC_4;
ADC_HandleTypeDef ADC_5;

ADC_HandleTypeDef* VSENSE_VMOT_ADC_handle = NULL;
#ifdef HAVE_12V_SENSE
ADC_HandleTypeDef* VSENSE_12V_ADC_handle = NULL;
#endif
#ifdef HAVE_5V_SENSE
ADC_HandleTypeDef* VSENSE_5V_ADC_handle = NULL;
#endif
ADC_HandleTypeDef* ISENSE_U_PHASE_ADC_handle = NULL;
ADC_HandleTypeDef* ISENSE_V_PHASE_ADC_handle = NULL;
ADC_HandleTypeDef* ISENSE_W_PHASE_ADC_handle = NULL;

ADC_ChannelConfTypeDef VSENSE_VMOT_ADC_channel_config = {
    VSENSE_VMOT_CHANNEL,      ADC_REGULAR_RANK_1,
    VSENSE_VMOT_SAMPLETIME,   ADC_SINGLE_ENDED,
    ADC_OFFSET_NONE,          0,
    ADC_OFFSET_SIGN_POSITIVE, DISABLE};
#ifdef HAVE_12V_SENSE
ADC_ChannelConfTypeDef VSENSE_12V_ADC_channel_config = {
    VSENSE_12V_CHANNEL,       ADC_REGULAR_RANK_1,
    VSENSE_12V_SAMPLETIME,    ADC_SINGLE_ENDED,
    ADC_OFFSET_NONE,          0,
    ADC_OFFSET_SIGN_POSITIVE, DISABLE};
#endif
#ifdef HAVE_5V_SENSE
ADC_ChannelConfTypeDef VSENSE_5V_ADC_channel_config = {
    VSENSE_5V_CHANNEL,        ADC_REGULAR_RANK_1,
    VSENSE_5V_SAMPLETIME,     ADC_SINGLE_ENDED,
    ADC_OFFSET_NONE,          0,
    ADC_OFFSET_SIGN_POSITIVE, DISABLE};
#endif
ADC_ChannelConfTypeDef ISENSE_U_PHASE_ADC_channel_config = {
    ISENSE_U_PHASE_CHANNEL,    ADC_REGULAR_RANK_1,
    ISENSE_U_PHASE_SAMPLETIME, ADC_SINGLE_ENDED,
    ADC_OFFSET_NONE,           0,
    ADC_OFFSET_SIGN_POSITIVE,  DISABLE};
ADC_ChannelConfTypeDef ISENSE_V_PHASE_ADC_channel_config = {
    ISENSE_V_PHASE_CHANNEL,    ADC_REGULAR_RANK_1,
    ISENSE_V_PHASE_SAMPLETIME, ADC_SINGLE_ENDED,
    ADC_OFFSET_NONE,           0,
    ADC_OFFSET_SIGN_POSITIVE,  DISABLE};
ADC_ChannelConfTypeDef ISENSE_W_PHASE_ADC_channel_config = {
    ISENSE_W_PHASE_CHANNEL,    ADC_REGULAR_RANK_1,
    ISENSE_W_PHASE_SAMPLETIME, ADC_SINGLE_ENDED,
    ADC_OFFSET_NONE,           0,
    ADC_OFFSET_SIGN_POSITIVE,  DISABLE};

struct ADCData {
    ADC_HandleTypeDef handle;
    bool inited = false;
};

static ADCData adc_data[5];

void adc::init(void) {
    gpio_init();
    adc_init();
    opamp_init();
}

void adc::start_VMOT_read(void) {
    start_ADC_read(VSENSE_VMOT_ADC_handle, &VSENSE_VMOT_ADC_channel_config);
    return;
}
float adc::read_VMOT(void) {
    uint32_t val = read_ADC(VSENSE_VMOT_ADC_handle, ADC_POLL_TIMEOUT);
    float voltage =
        (float)val * ADC_VOLTAGE_MULTIPLIER * VSENSE_VMOT_MULTIPLIER;
    return voltage;
}

#ifdef HAVE_12V_SENSE
void adc::start_12V_read(void) {
    start_ADC_read(VSENSE_12V_ADC_handle, &VSENSE_12V_ADC_channel_config);
    return;
}
float adc::read_12V(void) {
    uint32_t val = read_ADC(VSENSE_12V_ADC_handle, ADC_POLL_TIMEOUT);
    float voltage = (float)val * ADC_VOLTAGE_MULTIPLIER * VSENSE_12V_MULTIPLIER;
    return voltage;
}
#endif

#ifdef HAVE_5V_SENSE
void adc::start_5V_read(void) {
    start_ADC_read(VSENSE_5V_ADC_handle, &VSENSE_5V_ADC_channel_config);
    return;
}
float adc::read_5V(void) {
    uint32_t val = read_ADC(VSENSE_5V_ADC_handle, ADC_POLL_TIMEOUT);
    float voltage = (float)val * ADC_VOLTAGE_MULTIPLIER * VSENSE_5V_MULTIPLIER;
    return voltage;
}
#endif

float adc::read_U_current(void) {
    volatile float voltage = read_PGA(&U_phase_opamp, ISENSE_U_PHASE_ADC_handle,
                                      &ISENSE_U_PHASE_ADC_channel_config,
                                      &U_phase_gain, ADC_POLL_TIMEOUT);
    volatile float current = voltage * ISENSE_CURRENT_PER_VOLT;
    return current;
}

float adc::read_V_current(void) {
    volatile float voltage = 0;
    if (!direction_reversed) {
        voltage = read_PGA(&V_phase_opamp, ISENSE_V_PHASE_ADC_handle,
                           &ISENSE_V_PHASE_ADC_channel_config, &V_phase_gain,
                           ADC_POLL_TIMEOUT);
    } else {
        voltage = read_PGA(&W_phase_opamp, ISENSE_W_PHASE_ADC_handle,
                           &ISENSE_W_PHASE_ADC_channel_config, &W_phase_gain,
                           ADC_POLL_TIMEOUT);
    }
    volatile float current = voltage * ISENSE_CURRENT_PER_VOLT;
    return current;
}

float adc::read_W_current(void) {
    volatile float voltage = 0;
    if (!direction_reversed) {
        voltage = read_PGA(&W_phase_opamp, ISENSE_W_PHASE_ADC_handle,
                           &ISENSE_W_PHASE_ADC_channel_config, &W_phase_gain,
                           ADC_POLL_TIMEOUT);
    } else {
        voltage = read_PGA(&V_phase_opamp, ISENSE_V_PHASE_ADC_handle,
                           &ISENSE_V_PHASE_ADC_channel_config, &V_phase_gain,
                           ADC_POLL_TIMEOUT);
    }
    volatile float current = voltage * ISENSE_CURRENT_PER_VOLT;
    return current;
}

void start_ADC_read(ADC_HandleTypeDef* handle,
                    ADC_ChannelConfTypeDef* channel) {
    if (HAL_ADC_ConfigChannel(handle, channel) != HAL_OK) {
        debug::error("Failed to configure ADC channel");
        error::handler();
    }
    if (HAL_ADC_Start(handle) != HAL_OK) {
        debug::error("Failed to start ADC");
        error::handler();
    }
    return;
}

uint32_t read_ADC(ADC_HandleTypeDef* handle, uint32_t timeout) {
    if (HAL_ADC_PollForConversion(handle, timeout) != HAL_OK) {
        debug::error("Failed to poll for conversion for ADC");
        error::handler();
    }
    uint32_t val = HAL_ADC_GetValue(handle);
    if (HAL_ADC_Stop(handle) != HAL_OK) {
        debug::error("Failed to stop ADC");
        error::handler();
    }
    return val;
}

void set_PGA(OPAMP_HandleTypeDef* handle, PGAGain gain) {
    uint32_t gain_reg = 0;
    switch (gain) {
        case PGAGain::GAIN2:
            gain_reg = OPAMP_PGA_GAIN_2_OR_MINUS_1;
            break;
        case PGAGain::GAIN4:
            gain_reg = OPAMP_PGA_GAIN_4_OR_MINUS_3;
            break;
        case PGAGain::GAIN8:
            gain_reg = OPAMP_PGA_GAIN_8_OR_MINUS_7;
            break;
        case PGAGain::GAIN16:
            gain_reg = OPAMP_PGA_GAIN_16_OR_MINUS_15;
            break;
        case PGAGain::GAIN32:
            gain_reg = OPAMP_PGA_GAIN_32_OR_MINUS_31;
            break;
        case PGAGain::GAIN64:
            gain_reg = OPAMP_PGA_GAIN_64_OR_MINUS_63;
            break;
    }
    LL_OPAMP_SetPGAGain(handle->Instance, gain_reg);
    return;
}

float read_PGA(OPAMP_HandleTypeDef* opamp, ADC_HandleTypeDef* adc,
               ADC_ChannelConfTypeDef* channel, PGAGain* gain,
               uint32_t timeout) {
    start_ADC_read(adc, channel);
    uint32_t raw_reading = read_ADC(adc, timeout);
    debug::trace("read_PGA raw reading: %u, gain: %u", raw_reading,
                 (uint8_t)*gain);
    volatile float voltage =
        raw_reading * ADC_VOLTAGE_MULTIPLIER / (uint8_t)*gain;
    // adjust gain if needed
    if (raw_reading > ISENSE_HALF_GAIN_THRESHOLD && *gain != PGAGain::GAIN2) {
        *gain = half_gain(*gain);
        set_PGA(opamp, *gain);
    } else if (raw_reading < ISENSE_DOUBLE_GAIN_THRESHOLD &&
               *gain != PGAGain::GAIN64) {
        *gain = double_gain(*gain);
        set_PGA(opamp, *gain);
    }
    return voltage;
}

void opamp_init(void) {
    opamp_instance_init(&U_phase_opamp, ISENSE_U_PHASE_OPAMP,
                        ISENSE_U_PHASE_OPAMP_INPUT_NUM,
                        ISENSE_U_PHASE_OPAMP_POWERMODE);
    opamp_instance_init(&V_phase_opamp, ISENSE_V_PHASE_OPAMP,
                        ISENSE_V_PHASE_OPAMP_INPUT_NUM,
                        ISENSE_V_PHASE_OPAMP_POWERMODE);
    opamp_instance_init(&W_phase_opamp, ISENSE_W_PHASE_OPAMP,
                        ISENSE_W_PHASE_OPAMP_INPUT_NUM,
                        ISENSE_W_PHASE_OPAMP_POWERMODE);
    return;
}

void adc_init(void) {
    // ADC12 peripheral clock source
    RCC_PeriphCLKInitTypeDef ADC12_clk_init = {};
    ADC12_clk_init.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    ADC12_clk_init.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&ADC12_clk_init) != HAL_OK) {
        debug::error("Failed to configure ADC12 peripheral clock source");
    }

    VSENSE_VMOT_ADC_handle = adc_instance_init(VSENSE_VMOT_ADC);
#ifdef HAVE_12V_SENSE
    VSENSE_12V_ADC_handle = adc_instance_init(VSENSE_12V_ADC);
#endif
#ifdef HAVE_5V_SENSE
    VSENSE_5V_ADC_handle = adc_instance_init(VSENSE_5V_ADC);
#endif

    ISENSE_U_PHASE_ADC_handle = adc_instance_init(ISENSE_U_PHASE_ADC);
    ISENSE_V_PHASE_ADC_handle = adc_instance_init(ISENSE_V_PHASE_ADC);
    ISENSE_W_PHASE_ADC_handle = adc_instance_init(ISENSE_W_PHASE_ADC);
    return;
}

void opamp_instance_init(OPAMP_HandleTypeDef* handle, OPAMP_TypeDef* instance,
                         uint32_t input_num, uint32_t power_mode) {
    handle->Instance = instance;
    handle->Init.PowerMode = power_mode;
    handle->Init.Mode = OPAMP_PGA_MODE;
    handle->Init.NonInvertingInput = input_num;
    handle->Init.InternalOutput = ENABLE;
    handle->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    handle->Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
    handle->Init.PgaGain = OPAMP_PGA_GAIN_2_OR_MINUS_1;
    handle->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

    if (HAL_OPAMP_Init(handle) != HAL_OK) {
        uint8_t instance_num = 0;
        if (instance == OPAMP1) {
            instance_num = 0;
        } else if (instance == OPAMP2) {
            instance_num = 1;
        } else if (instance == OPAMP3) {
            instance_num = 2;
        }
        debug::error("Failed to initialise OPAMP %u", instance_num + 1);
        error::handler();
    }
    if (HAL_OPAMP_Start(handle) != HAL_OK) {
        uint8_t instance_num = 0;
        if (instance == OPAMP1) {
            instance_num = 0;
        } else if (instance == OPAMP2) {
            instance_num = 1;
        } else if (instance == OPAMP3) {
            instance_num = 2;
        }
        debug::error("Failed to start OPAMP %u", instance_num + 1);
        error::handler();
    }
    return;
}

void gpio_init(void) {
    gpio::init(VSENSE_VMOT_PIN, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::MEDIUM);
#ifdef HAVE_12V_SENSE
    gpio::init(VSENSE_12V_PIN, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::MEDIUM);
#endif
#ifdef HAVE_5V_SENSE
    gpio::init(VSENSE_5V_PIN, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::MEDIUM);
#endif
    gpio::init(ISENSE_U_PHASE_PIN, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::MEDIUM);
    gpio::init(ISENSE_V_PHASE_PIN, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::MEDIUM);
    gpio::init(ISENSE_W_PHASE_PIN, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::MEDIUM);
    return;
}

ADC_HandleTypeDef* adc_instance_init(ADC_TypeDef* instance) {
    uint8_t instance_idx = adc_instance_to_idx(instance);
    ADC_HandleTypeDef* handle = &(adc_data[instance_idx].handle);
    if (adc_data[instance_idx].inited) {
        return handle;
    }

    if (instance_idx == 0 || instance_idx == 1) {
        __HAL_RCC_ADC12_CLK_ENABLE();
    } else if (instance_idx == 2 || instance_idx == 3 || instance_idx == 4) {
    }

    handle->Instance = instance;
    handle->Init.ClockPrescaler = ADC_CLK_PRESCALER;
    handle->Init.Resolution = ADC_RESOLUTION_12B;
    handle->Init.DataAlign = ADC_DATAALIGN_RIGHT;  // little endian
    handle->Init.GainCompensation = 0;  // no way to calibrate for now
    handle->Init.ScanConvMode = ADC_SCAN_DISABLE;
    handle->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    handle->Init.LowPowerAutoWait = DISABLE;
    handle->Init.ContinuousConvMode = DISABLE;
    handle->Init.NbrOfConversion = 1;
    handle->Init.DiscontinuousConvMode = DISABLE;
    handle->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    handle->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    handle->Init.DMAContinuousRequests = DISABLE;
    handle->Init.Overrun = ADC_OVR_DATA_PRESERVED;
    handle->Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(handle) != HAL_OK) {
        debug::error("Failed to initialise ADC %u", instance_idx + 1);
        error::handler();
    }

    if (HAL_ADCEx_Calibration_Start(handle, ADC_SINGLE_ENDED) != HAL_OK) {
        debug::error("Failed to calibrate ADC %u", instance_idx + 1);
        error::handler();
    }

    adc_data[instance_idx].inited = true;
    return handle;
}

uint8_t adc_instance_to_idx(ADC_TypeDef* instance) {
    if (instance == ADC1) {
        return 0;
    } else if (instance == ADC2) {
        return 1;
    }
    return 255;
}

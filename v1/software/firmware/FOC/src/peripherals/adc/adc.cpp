#include "adc.hpp"

#include <stm32g4xx.h>
#include <stm32g4xx_ll_opamp.h>

#include "config.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "gpio.hpp"

void opamp_init(void);
void adc_init(void);
void gpio_init(void);
void dma_init(void);

void opamp_instance_init(OPAMP_HandleTypeDef* handle, OPAMP_TypeDef* instance,
                         uint32_t input_num, uint32_t power_mode);

uint8_t adc_instance_to_idx(ADC_TypeDef* instance);
ADC_HandleTypeDef* adc_instance_init(ADC_TypeDef* instance);

uint16_t rank_idx_to_val(uint8_t rank_idx);

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
        return (PGAGain)((uint8_t)gain << 1);
}
PGAGain half_gain(PGAGain gain) {
    uint8_t current_gain = (uint8_t)gain;
    if (current_gain <= (uint8_t)PGAGain::GAIN2) {
        return PGAGain::GAIN2;
    } else
        return (PGAGain)((uint8_t)gain >> 1);
}

void set_PGA(OPAMP_HandleTypeDef* handle, PGAGain gain);
float read_PGA(OPAMP_HandleTypeDef* opamp, ADC_HandleTypeDef* adc,
               ADC_ChannelConfTypeDef* channel, PGAGain* gain,
               uint32_t timeout);

static PGAGain U_phase_gain = PGAGain::GAIN2;
static PGAGain V_phase_gain = PGAGain::GAIN2;
static PGAGain W_phase_gain = PGAGain::GAIN2;

static DMA_HandleTypeDef DMA_ADC1;
static DMA_HandleTypeDef DMA_ADC2;
static volatile uint16_t DMA_BUFFERS[2][6] = {
    {0}};  // DMA_ADC1 and DMA_ADC2 buffers
static volatile bool ADC1_conversion_complete = false,
                     ADC2_conversion_complete = false;

static OPAMP_HandleTypeDef U_phase_opamp;
static OPAMP_HandleTypeDef V_phase_opamp;
static OPAMP_HandleTypeDef W_phase_opamp;

static ADC_HandleTypeDef ADC_1;
static ADC_HandleTypeDef ADC_2;

struct ADCChannelData {
    ADC_HandleTypeDef* handle = NULL;
    uint8_t dma_buf_idx1 = 0;
    uint8_t dma_buf_idx2 = 0;
};

static ADCChannelData VSENSE_VMOT_ADC_data;
#ifdef HAVE_12V_SENSE
static ADCChannelData VSENSE_12V_ADC_data;
#endif
#ifdef HAVE_5V_SENSE
static ADCChannelData VSENSE_5V_ADC_data;
#endif
static ADCChannelData ISENSE_U_PHASE_ADC_data;
static ADCChannelData ISENSE_V_PHASE_ADC_data;
static ADCChannelData ISENSE_W_PHASE_ADC_data;

struct ADCData {
    ADC_HandleTypeDef handle;
    bool inited = false;
};

static ADCData adc_data[2];

static uint8_t ADC1_num_conversions = 0, ADC2_num_conversions = 0;

void adc::init(void) {
    gpio_init();
    dma_init();
    opamp_init();
    adc_init();
}

void adc::start_conversions(void) {
    debug::debug("starting ADC conversions");
    if (ADC1_num_conversions > 0) {
        if (HAL_ADC_Start_DMA(&(adc_data[0].handle),
                              (uint32_t*)(&(DMA_BUFFERS[0][0])),
                              ADC1_num_conversions) != HAL_OK) {
            debug::error("Failed to start ADC1 read");
            error::handler();
        }
    }
    if (ADC2_num_conversions > 0) {
        if (HAL_ADC_Start_DMA(&(adc_data[1].handle),
                              (uint32_t*)(&(DMA_BUFFERS[1][0])),
                              ADC2_num_conversions) != HAL_OK) {
            debug::error("Failed to start ADC2 read");
            error::handler();
        }
    }
}

adc::Values adc::read(void) {
    debug::debug("reading from ADC");
    if (ADC1_num_conversions > 0) {
        while (!ADC1_conversion_complete);
        ADC1_conversion_complete = false;
    }
    debug::debug("completed poll from ADC1");
    if (ADC2_num_conversions > 0) {
        while (!ADC2_conversion_complete);
        ADC2_conversion_complete = false;
    }
    debug::debug("completed poll from ADC2");

    // get raw readings
    float VSENSE_VMOT_raw_reading =
        (float)DMA_BUFFERS[VSENSE_VMOT_ADC_data.dma_buf_idx1]
                          [VSENSE_VMOT_ADC_data.dma_buf_idx2];
#ifdef HAVE_12V_SENSE
    float VSENSE_12V_raw_reading =
        (float)DMA_BUFFERS[VSENSE_12V_ADC_data.dma_buf_idx1]
                          [VSENSE_12V_ADC_data.dma_buf_idx2];
#endif
#ifdef HAVE_5V_SENSE
    float VSENSE_5V_raw_reading =
        (float)DMA_BUFFERS[VSENSE_5V_ADC_data.dma_buf_idx1]
                          [VSENSE_5V_ADC_data.dma_buf_idx2];
#endif
    float ISENSE_U_PHASE_raw_reading =
        (float)DMA_BUFFERS[ISENSE_U_PHASE_ADC_data.dma_buf_idx1]
                          [ISENSE_U_PHASE_ADC_data.dma_buf_idx2];
    float ISENSE_V_PHASE_raw_reading =
        (float)DMA_BUFFERS[ISENSE_V_PHASE_ADC_data.dma_buf_idx1]
                          [ISENSE_V_PHASE_ADC_data.dma_buf_idx2];
    float ISENSE_W_PHASE_raw_reading =
        (float)DMA_BUFFERS[ISENSE_W_PHASE_ADC_data.dma_buf_idx1]
                          [ISENSE_W_PHASE_ADC_data.dma_buf_idx2];

    debug::debug("VMOT raw: %f, 12V raw: %f, U raw: %f, V raw: %f, W raw: %f",
                 VSENSE_VMOT_raw_reading, VSENSE_12V_raw_reading,
                 ISENSE_U_PHASE_raw_reading, ISENSE_V_PHASE_raw_reading,
                 ISENSE_W_PHASE_raw_reading);

    Values values;

    values.voltage_VMOT = VSENSE_VMOT_raw_reading * ADC_VOLTAGE_MULTIPLIER *
                          VSENSE_VMOT_MULTIPLIER;
#ifdef HAVE_12V_SENSE
    values.voltage_12V =
        VSENSE_12V_raw_reading * ADC_VOLTAGE_MULTIPLIER * VSENSE_12V_MULTIPLIER;
#endif
#ifdef HAVE_5V_SENSE
    values.voltage_5V =
        VSENSE_5V_raw_reading * ADC_VOLTAGE_MULTIPLIER * VSENSE_5V_MULTIPLIER;
#endif
    values.current_U = ISENSE_U_PHASE_raw_reading * ADC_VOLTAGE_MULTIPLIER /
                       (uint8_t)U_phase_gain * ISENSE_CURRENT_PER_VOLT;
    if (direction_reversed) {
        values.current_V = ISENSE_W_PHASE_raw_reading * ADC_VOLTAGE_MULTIPLIER /
                           (uint8_t)W_phase_gain * ISENSE_CURRENT_PER_VOLT;
        values.current_W = ISENSE_V_PHASE_raw_reading * ADC_VOLTAGE_MULTIPLIER /
                           (uint8_t)V_phase_gain * ISENSE_CURRENT_PER_VOLT;
    } else {
        values.current_V = ISENSE_V_PHASE_raw_reading * ADC_VOLTAGE_MULTIPLIER /
                           (uint8_t)V_phase_gain * ISENSE_CURRENT_PER_VOLT;
        values.current_W = ISENSE_W_PHASE_raw_reading * ADC_VOLTAGE_MULTIPLIER /
                           (uint8_t)W_phase_gain * ISENSE_CURRENT_PER_VOLT;
    }

    // adjust PGA gain if needed
    if (ISENSE_U_PHASE_raw_reading > ISENSE_HALF_GAIN_THRESHOLD &&
        U_phase_gain != PGAGain::GAIN2) {
        U_phase_gain = half_gain(U_phase_gain);
        set_PGA(&U_phase_opamp, U_phase_gain);
    } else if (ISENSE_U_PHASE_raw_reading < ISENSE_DOUBLE_GAIN_THRESHOLD &&
               U_phase_gain != PGAGain::GAIN64) {
        U_phase_gain = double_gain(U_phase_gain);
        set_PGA(&U_phase_opamp, U_phase_gain);
    }
    if (ISENSE_V_PHASE_raw_reading > ISENSE_HALF_GAIN_THRESHOLD &&
        V_phase_gain != PGAGain::GAIN2) {
        V_phase_gain = half_gain(V_phase_gain);
        set_PGA(&V_phase_opamp, V_phase_gain);
    } else if (ISENSE_V_PHASE_raw_reading < ISENSE_DOUBLE_GAIN_THRESHOLD &&
               V_phase_gain != PGAGain::GAIN64) {
        V_phase_gain = double_gain(V_phase_gain);
        set_PGA(&V_phase_opamp, V_phase_gain);
    }
    if (ISENSE_W_PHASE_raw_reading > ISENSE_HALF_GAIN_THRESHOLD &&
        W_phase_gain != PGAGain::GAIN2) {
        W_phase_gain = half_gain(W_phase_gain);
        set_PGA(&W_phase_opamp, W_phase_gain);
    } else if (ISENSE_W_PHASE_raw_reading < ISENSE_DOUBLE_GAIN_THRESHOLD &&
               W_phase_gain != PGAGain::GAIN64) {
        W_phase_gain = double_gain(W_phase_gain);
        set_PGA(&W_phase_opamp, W_phase_gain);
    }

    return values;
}

void adc::DMA_ADC1_handler(void) {
    HAL_DMA_IRQHandler(&DMA_ADC1);
    return;
}
void adc::DMA_ADC2_handler(void) {
    HAL_DMA_IRQHandler(&DMA_ADC2);
    return;
}
void adc::ADC1_conversion_complete_callback(void) {
    ADC1_conversion_complete = true;
    return;
}
void adc::ADC2_conversion_complete_callback(void) {
    ADC2_conversion_complete = true;
    return;
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

    // calculate the ADC configuration
#define CALCULATE_ADC_CHANNEL_CONFIG(peripheral_name)                       \
    if (peripheral_name##_ADC == ADC1) {                                    \
        ADC1_num_conversions++;                                             \
        peripheral_name##_ADC_data.dma_buf_idx1 = 0;                        \
        peripheral_name##_ADC_data.dma_buf_idx2 = ADC1_num_conversions - 1; \
    } else if (peripheral_name##_ADC == ADC2) {                             \
        ADC2_num_conversions++;                                             \
        peripheral_name##_ADC_data.dma_buf_idx1 = 1;                        \
        peripheral_name##_ADC_data.dma_buf_idx2 = ADC2_num_conversions - 1; \
    }
    CALCULATE_ADC_CHANNEL_CONFIG(VSENSE_VMOT);
#ifdef HAVE_12V_SENSE
    CALCULATE_ADC_CHANNEL_CONFIG(VSENSE_12V);
#endif
#ifdef HAVE_5V_SENSE
    CALCULATE_ADC_CHANNEL_CONFIG(VSENSE_5V);
#endif
    CALCULATE_ADC_CHANNEL_CONFIG(ISENSE_U_PHASE);
    CALCULATE_ADC_CHANNEL_CONFIG(ISENSE_V_PHASE);
    CALCULATE_ADC_CHANNEL_CONFIG(ISENSE_W_PHASE);
#undef CALCULATE_ADC_CHANNEL_CONFIG

    // initialise ADCs
    VSENSE_VMOT_ADC_data.handle = adc_instance_init(VSENSE_VMOT_ADC);
#ifdef HAVE_12V_SENSE
    VSENSE_12V_ADC_data.handle = adc_instance_init(VSENSE_12V_ADC);
#endif
#ifdef HAVE_5V_SENSE
    VSENSE_5V_ADC_data.handle = VSENSE_5V_ADC_handle =
        adc_instance_init(VSENSE_5V_ADC);
#endif
    ISENSE_U_PHASE_ADC_data.handle = adc_instance_init(ISENSE_U_PHASE_ADC);
    ISENSE_V_PHASE_ADC_data.handle = adc_instance_init(ISENSE_V_PHASE_ADC);
    ISENSE_W_PHASE_ADC_data.handle = adc_instance_init(ISENSE_W_PHASE_ADC);

    // configure channels
    ADC_ChannelConfTypeDef channel_config = {0};
#define CONFIGURE_ADC_CHANNEL(peripheral_name)                                \
    channel_config.Channel = peripheral_name##_CHANNEL;                       \
    channel_config.Rank =                                                     \
        rank_idx_to_val(peripheral_name##_ADC_data.dma_buf_idx2);             \
    channel_config.SamplingTime = peripheral_name##_SAMPLETIME;               \
    channel_config.SingleDiff = ADC_SINGLE_ENDED;                             \
    channel_config.OffsetNumber = ADC_OFFSET_NONE;                            \
    channel_config.Offset = 0;                                                \
    if (HAL_ADC_ConfigChannel(peripheral_name##_ADC_data.handle,              \
                              &channel_config) != HAL_OK) {                   \
        debug::error("Failed to configure " #peripheral_name " ADC channel"); \
        error::handler();                                                     \
    }
    CONFIGURE_ADC_CHANNEL(VSENSE_VMOT);
#ifdef HAVE_12V_SENSE
    CONFIGURE_ADC_CHANNEL(VSENSE_12V);
#endif
#ifdef HAVE_5V_SENSE
    CONFIGURE_ADC_CHANNEL(VSENSE_5V);
#endif
    CONFIGURE_ADC_CHANNEL(ISENSE_U_PHASE);
    CONFIGURE_ADC_CHANNEL(ISENSE_V_PHASE);
    CONFIGURE_ADC_CHANNEL(ISENSE_W_PHASE);
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
    __HAL_RCC_ADC12_CLK_ENABLE();

    handle->Instance = instance;
    handle->Init.ClockPrescaler = ADC_CLK_PRESCALER;
    handle->Init.Resolution = ADC_RESOLUTION_12B;
    handle->Init.DataAlign = ADC_DATAALIGN_RIGHT;  // little endian
    handle->Init.GainCompensation = 0;  // no way to calibrate for now
    handle->Init.ScanConvMode = ADC_SCAN_ENABLE;
    handle->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    handle->Init.LowPowerAutoWait = DISABLE;
    handle->Init.ContinuousConvMode = DISABLE;
    if (instance_idx == 0) {
        handle->Init.NbrOfConversion = ADC1_num_conversions;
    } else if (instance_idx == 1) {
        handle->Init.NbrOfConversion = ADC2_num_conversions;
    }
    handle->Init.DiscontinuousConvMode = DISABLE;
    handle->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    handle->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    handle->Init.DMAContinuousRequests = DISABLE;
    handle->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    handle->Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(handle) != HAL_OK) {
        debug::error("Failed to initialise ADC %u", instance_idx + 1);
        error::handler();
    }

    if (HAL_ADCEx_Calibration_Start(handle, ADC_SINGLE_ENDED) != HAL_OK) {
        debug::error("Failed to calibrate ADC %u", instance_idx + 1);
        error::handler();
    }

    if (instance_idx == 0) {
        __HAL_LINKDMA(handle, DMA_Handle, DMA_ADC1);
    } else if (instance_idx == 1) {
        __HAL_LINKDMA(handle, DMA_Handle, DMA_ADC2);
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

void dma_init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    DMA_ADC1.Instance = ADC1_DMA_INSTANCE;
    DMA_ADC1.Init.Request = DMA_REQUEST_ADC1;
    DMA_ADC1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DMA_ADC1.Init.PeriphInc = DMA_PINC_DISABLE;
    DMA_ADC1.Init.MemInc = DMA_MINC_ENABLE;
    DMA_ADC1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    DMA_ADC1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    DMA_ADC1.Init.Mode = DMA_NORMAL;
    DMA_ADC1.Init.Priority = DMA_PRIORITY_HIGH;

    DMA_ADC2.Instance = ADC2_DMA_INSTANCE;
    DMA_ADC2.Init.Request = DMA_REQUEST_ADC2;
    DMA_ADC2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DMA_ADC2.Init.PeriphInc = DMA_PINC_DISABLE;
    DMA_ADC2.Init.MemInc = DMA_MINC_ENABLE;
    DMA_ADC2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    DMA_ADC2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    DMA_ADC2.Init.Mode = DMA_NORMAL;
    DMA_ADC2.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&DMA_ADC1) != HAL_OK) {
        debug::error("Failed to init DMA for ADC1");
        error::handler();
    }
    if (HAL_DMA_Init(&DMA_ADC2) != HAL_OK) {
        debug::error("Failed to init DMA for ADC2");
        error::handler();
    }

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    return;
}

uint16_t rank_idx_to_val(uint8_t rank_idx) {
    // only 6 ranks are needed right now
    switch (rank_idx) {
        case 0:
            return ADC_REGULAR_RANK_1;
            break;
        case 1:
            return ADC_REGULAR_RANK_2;
            break;
        case 2:
            return ADC_REGULAR_RANK_3;
            break;
        case 3:
            return ADC_REGULAR_RANK_4;
            break;
        case 4:
            return ADC_REGULAR_RANK_5;
            break;
        case 5:
            return ADC_REGULAR_RANK_6;
            break;
    }
}

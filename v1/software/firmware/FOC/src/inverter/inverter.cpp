#include "inverter.hpp"

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_tim.h>
#include <stm32g4xx_hal_tim_ex.h>

#include <cmath>

#include "config.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "gpio.hpp"

TIM_HandleTypeDef timer;

static uint16_t resolution = 0;

static bool direction_reversed = false;

struct PWMFreqParams {
    uint16_t prescaler = 0;
    uint16_t period = 0;
    uint32_t frequency = 0;
};

enum class TargetSector : uint8_t {
    U_UV = 0,
    UV_V = 1,
    V_VW = 2,
    VW_W = 3,
    W_WU = 4,
    WU_U = 5,
};

void timer_init(uint32_t pwm_freq);
void gpio_init(void);
PWMFreqParams calculate_frequency_parameters(uint32_t target_frequency,
                                             uint32_t clock_source_frequency);
void inverter::init(uint32_t pwm_freq) {
    timer_init(pwm_freq);
    gpio_init();
}

void inverter::reverse_direction(void) {
    direction_reversed = !direction_reversed;
}

void inverter::set(float u, float v, float w) {
    // clamp amplitudes
    u = std::fmax(0.0f, std::fmin(MAX_PWM_ONTIME, u));
    v = std::fmax(0.0f, std::fmin(MAX_PWM_ONTIME, v));
    w = std::fmax(0.0f, std::fmin(MAX_PWM_ONTIME, w));

    // reverse direction if needed
    if (direction_reversed) {
        float tmp_v = v;
        v = w;
        w = tmp_v;
    }

    // convert amplitudes to duty cycles
    uint16_t u_count = u * resolution;
    uint16_t v_count = v * resolution;
    uint16_t w_count = w * resolution;

    // set duty cycles
    __HAL_TIM_SET_COMPARE(&timer, U_PHASE_CHANNEL, u_count);
    __HAL_TIM_SET_COMPARE(&timer, V_PHASE_CHANNEL, v_count);
    __HAL_TIM_SET_COMPARE(&timer, W_PHASE_CHANNEL, w_count);
}

void inverter::svpwm_set(float theta, float V_target, float V_dc) {
    // clamp V_target
    V_target =
        std::fmax(0.0f, std::fmin(V_target, V_dc * std::cosf(M_PI / 6.0f)));
    debug::trace("Inverter SVPWM: V_target: %f", V_target);

    TargetSector sector = (TargetSector)std::floorf(theta / (M_PI / 3.0f));
    float sector_theta = theta - (float)(uint8_t)sector * (M_PI / 3.0f);
    debug::trace("Inverter SVPWM: sector_theta: %f", sector_theta);

    float V_x = V_target * std::cosf(sector_theta);
    float V_y = V_target * std::sinf(sector_theta);
    debug::trace("Inverter SVPWM: V_x: %f, V_y: %f", V_x, V_y);

    // V_2_y = V_y
    // V_2_x / V_2_y = tan(30deg)
    float V_2_x = V_y * std::tanf(M_PI / 6.0f);
    // V_2_y / V_2 = cos(30deg)
    // V_2 = V_2_y / cos(30deg)
    float V_2 = V_y / std::cosf(M_PI / 6.0f);
    // V_1 = V_1_x = V_x - V_2_x
    float V_1 = V_x - V_2_x;

    float duty_1 = V_1 / V_dc;
    float duty_2 = V_2 / V_dc;
    float duty_zero = (1.0f - duty_1 - duty_2) / 2;

    float duty_U = 0, duty_V = 0, duty_W = 0;

    switch (sector) {
        case TargetSector::U_UV:
            duty_U = duty_zero + duty_1 + duty_2;
            duty_V = duty_zero + duty_2;
            duty_W = duty_zero;
            debug::trace("Inverter SVPWM: Sector U_UV");
            break;
        case TargetSector::UV_V:
            duty_U = duty_zero + duty_1;
            duty_V = duty_zero + duty_1 + duty_2;
            duty_W = duty_zero;
            debug::trace("Inverter SVPWM: Sector UV_V");
            break;
        case TargetSector::V_VW:
            duty_U = duty_zero;
            duty_V = duty_zero + duty_1 + duty_2;
            duty_W = duty_zero + duty_2;
            debug::trace("Inverter SVPWM: Sector V_VW");
            break;
        case TargetSector::VW_W:
            duty_U = duty_zero;
            duty_V = duty_zero + duty_1;
            duty_W = duty_zero + duty_1 + duty_2;
            debug::trace("Inverter SVPWM: Sector VW_W");
            break;
        case TargetSector::W_WU:
            duty_U = duty_zero + duty_2;
            duty_V = duty_zero;
            duty_W = duty_zero + duty_1 + duty_2;
            debug::trace("Inverter SVPWM: Sector W_WU");
            break;
        case TargetSector::WU_U:
            duty_U = duty_zero + duty_1 + duty_2;
            duty_V = duty_zero;
            duty_W = duty_zero + duty_1;
            debug::trace("Inverter SVPWM: Sector WU_U");
            break;
    }

    debug::trace("duty_U: %f, duty_V: %f, duty_W: %f", duty_U, duty_V, duty_W);
    set(duty_U, duty_V, duty_W);
}

void timer_init(uint32_t pwm_freq) {
    // init drive timer
    timer.Instance = DRIVE_PHASE_TIMER;
    // assuming APB prescaler = 1
#if DRIVE_PHASE_CLOCK == PCLK1
    uint32_t clk_freq = HAL_RCC_GetPCLK1Freq();
#elif DRIVE_PHASE_CLOCK == PCLK2
    uint32_t clk_freq = HAL_RCC_GetPCLK2Freq();
#else
#error "Invalid clock source for drive phase timer"
#endif
    PWMFreqParams freq_params =
        calculate_frequency_parameters(pwm_freq, clk_freq);
    timer.Init.Prescaler = freq_params.prescaler;
    timer.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    timer.Init.Period = freq_params.period;
    timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer.Init.RepetitionCounter = 0;  // interrupt every PWM cycle
    // changes to duty cycle will only apply after the current cycle completes
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    resolution = freq_params.period;

    if (HAL_TIM_PWM_Init(&timer) != HAL_OK) {
        debug::fatal("Inverter: drive timer PWM init failed.");
        error::handler();
    }

    // configure U, V, W channels
    TIM_OC_InitTypeDef oc_config = {};
    oc_config.OCMode = TIM_OCMODE_PWM1;          // duty cycle = pulse / period
    oc_config.Pulse = 0;                         // 0 duty cycle initially
    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;  // high == on
    oc_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;   // high == on
    oc_config.OCFastMode = TIM_OCFAST_DISABLE;      // not used for PWM
    oc_config.OCIdleState = TIM_OCIDLESTATE_RESET;  // high side off by default
    oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;  // low side off by default
    if (HAL_TIM_PWM_ConfigChannel(&timer, &oc_config, U_PHASE_CHANNEL)) {
        debug::fatal("Inverter: U phase PWM channel config failed.");
        error::handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&timer, &oc_config, V_PHASE_CHANNEL)) {
        debug::fatal("Inverter: V phase PWM channel config failed.");
        error::handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&timer, &oc_config, W_PHASE_CHANNEL)) {
        debug::fatal("Inverter: W phase PWM channel config failed.");
        error::handler();
    }
    // enable preload for changing duty cycle
    __HAL_TIM_ENABLE_OCxPRELOAD(&timer, U_PHASE_CHANNEL);
    __HAL_TIM_ENABLE_OCxPRELOAD(&timer, V_PHASE_CHANNEL);
    __HAL_TIM_ENABLE_OCxPRELOAD(&timer, W_PHASE_CHANNEL);

    // configure break time
    TIM_BreakDeadTimeConfigTypeDef break_deadtime_config = {};
    break_deadtime_config.OffStateRunMode = TIM_OSSR_DISABLE;  // off by default
    break_deadtime_config.OffStateIDLEMode =
        TIM_OSSI_DISABLE;                                 // off by default
    break_deadtime_config.LockLevel = TIM_LOCKLEVEL_OFF;  // dont lock registers
    break_deadtime_config.DeadTime =
        (uint32_t)ceilf((float)DRIVE_BREAKTIME / (float)1e9 /
                        (float)clk_freq);  // number of cycles to achieve
                                           // deadtime, rounded up for safety
    // break settings
    break_deadtime_config.BreakState = TIM_BREAK_DISABLE;  // off on break fault
    break_deadtime_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;  // high == on
    break_deadtime_config.BreakFilter =
        0;  // trigger break immediately on break signal, without filter
    break_deadtime_config.BreakAFMode =
        TIM_BREAK_AFMODE_INPUT;  // break acts as an input
    break_deadtime_config.Break2State = TIM_BREAK2_DISABLE;
    break_deadtime_config.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    break_deadtime_config.Break2Filter = 0;
    break_deadtime_config.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    // dont automatically restart output on break
    break_deadtime_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&timer, &break_deadtime_config)) {
        debug::fatal("Inverter: drive timer break and deadtime config failed.");
        error::handler();
    }

    // start PWM
    if (HAL_TIM_PWM_Start(&timer, U_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: failed to start U phase PWM");
        error::handler();
    }
    if (HAL_TIMEx_PWMN_Start(&timer, U_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: failed to start U_N phase PWM");
        error::handler();
    }
    if (HAL_TIM_PWM_Start(&timer, V_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: failed to start V phase PWM");
        error::handler();
    }
    if (HAL_TIMEx_PWMN_Start(&timer, V_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: failed to start V_N phase PWM");
        error::handler();
    }
    if (HAL_TIM_PWM_Start(&timer, W_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: failed to start W phase PWM");
        error::handler();
    }
    if (HAL_TIMEx_PWMN_Start(&timer, W_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: failed to start W_N phase PWM");
        error::handler();
    }
}

void gpio_init(void) {
    gpio::init(INVERTER_U, gpio::GPIOMode::AF_PP, gpio::GPIOPull::NOPULL,
               gpio::GPIOSpeed::HIGH);
    gpio::init(INVERTER_U_N, gpio::GPIOMode::AF_PP, gpio::GPIOPull::NOPULL,
               gpio::GPIOSpeed::HIGH);
    gpio::init(INVERTER_V, gpio::GPIOMode::AF_PP, gpio::GPIOPull::NOPULL,
               gpio::GPIOSpeed::HIGH);
    gpio::init(INVERTER_V_N, gpio::GPIOMode::AF_PP, gpio::GPIOPull::NOPULL,
               gpio::GPIOSpeed::HIGH);
    gpio::init(INVERTER_W, gpio::GPIOMode::AF_PP, gpio::GPIOPull::NOPULL,
               gpio::GPIOSpeed::HIGH);
    gpio::init(INVERTER_W_N, gpio::GPIOMode::AF_PP, gpio::GPIOPull::NOPULL,
               gpio::GPIOSpeed::HIGH);
}

/**
 * @brief Calculate the prescaler and period parameters for achieving a
 * target PWM frequency. Maximises ARR for resolution
 * @param target_frequency: Target frequency that will be achieved as closely as
 * possible
 * @param clock_source_frequency: Frequency of the clock source, either PCLK1 or
 * PCLK2 if APB prescalers are 1. TIM2..7 are on PCLK1, TIM1,8,20,15,16,17 are
 * on PCLK2
 * @returns prescaler, period and real frequency values
 */
PWMFreqParams calculate_frequency_parameters(uint32_t target_frequency,
                                             uint32_t clock_source_frequency) {
    // Formula: Freq = Clk / ((PSC + 1) * (ARR + 1))
    // (PSC + 1) * (ARR + 1) = Clk / Freq
    // PSC is the prescaler, ARR is the period (autoreload)
    uint32_t divisor = clock_source_frequency / target_frequency;
    PWMFreqParams params = {0, 0, 0};
    // maximum resolution is 16 bits
    // check from highest resolution down
    for (uint32_t i = 16; i >= PWM_MIN_RESOLUTION; i--) {
        uint32_t period = 1 << i;
        // compute prescaler
        uint32_t prescaler = divisor / period;
        // compute actual frequency
        uint32_t actual_frequency =
            clock_source_frequency / (period * prescaler);
        float deviation =
            fabsf(((float)actual_frequency - (float)target_frequency)) /
            target_frequency;
        // set the current match
        params.period = period - 1;
        params.prescaler = prescaler - 1;
        params.frequency = actual_frequency;
        debug::debug(
            "PWM frequency calculation: Trying %u bits of resolution, obtained "
            "prescaler %u, and actual frequency %uHz deviating %f from target "
            "%uHz",
            i, prescaler, actual_frequency, deviation, target_frequency);

        // accept this combination
        if (deviation < PWM_MAX_FREQ_DEVIATION) {
            return params;
        }
    }

    debug::warn(
        "PWM frequency generated %uHz is more than %f deviation from target "
        "%uHz.",
        params.frequency, PWM_MAX_FREQ_DEVIATION, target_frequency);
    // accept the latest combination (period = 1 << PWM_MIN_RESOLUTION - 1)
    return params;
}

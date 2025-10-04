#include "inverter.hpp"

#include <arm_math.h>
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_tim.h>
#include <stm32g4xx_hal_tim_ex.h>

#include <cmath>

#include "config.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "gpio.hpp"

static TIM_HandleTypeDef timer;

static uint16_t resolution = 0;

struct PWMFreqParams {
    uint16_t prescaler = 0;
    uint16_t period = 0;
    uint32_t frequency = 0;
};

static PWMFreqParams timer_init(uint32_t pwm_freq);
static void gpio_init(void);
static PWMFreqParams calculate_frequency_parameters(
    uint32_t target_frequency, uint32_t clock_source_frequency);
uint32_t inverter::init(uint32_t pwm_freq) {
    PWMFreqParams pwm_freq_params = timer_init(pwm_freq);
    gpio_init();
    set(0, 0, 0);  // all pull to ground
    return pwm_freq_params.frequency;
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

    debug::trace(
        "Inverter PWM: u_count: %u, v_count: %u, w_count: %u, resolution: %u",
        u_count, v_count, w_count, resolution);

    // set duty cycles
    __HAL_TIM_SET_COMPARE(&timer, U_PHASE_CHANNEL, u_count);
    __HAL_TIM_SET_COMPARE(&timer, V_PHASE_CHANNEL, v_count);
    __HAL_TIM_SET_COMPARE(&timer, W_PHASE_CHANNEL, w_count);
}

const float COS_PI_6 = 0.86602540378f;  // cos(PI/6)
const float TAN_PI_6 = 0.57735026919f;  // tan(PI/6)
const float PI_DIV_3 = M_PI / 3;

const float COS_LOOKUP[6] = {
    1.0f,   // cos(0)
    0.5f,   // cos(-60)
    -0.5f,  // cos(-120)
    -1.0f,  // cos(-180)
    -0.5f,  // cos(-240)
    0.5f    // cos(-300)
};
const float SIN_LOOKUP[6] = {
    0.0f,         // sin(0)
    -0.8660254f,  // sin(-60)
    -0.8660254f,  // sin(-120)
    0.0f,         // sin(-180)
    0.8660254f,   // sin(-240)
    0.8660254f    // sin(-300)
};

const uint8_t SECTOR_MAP[8] = {
    0, 1, 5, 0, 3, 2, 4, 0,
};

inverter::TargetSector inverter::svpwm_set(float theta, float V_d, float V_q,
                                           float V_dc) {
    return svpwm_set(std::sinf(theta), std::cosf(theta), V_d, V_q, V_dc);
}

inverter::TargetSector inverter::svpwm_set(float sin_theta, float cos_theta,
                                           float V_d, float V_q, float V_dc) {
    // inverse Park transformation
    float V_alpha = V_d * cos_theta - V_q * sin_theta;
    float V_beta = V_d * sin_theta + V_q * cos_theta;

    // clamp V magnitude
    float V_mag = 0;
    arm_sqrt_f32(V_alpha * V_alpha + V_beta * V_beta, &V_mag);
    float V_mag_clamped = std::fmin(V_mag, V_dc * COS_PI_6);
    float scale_factor = (V_mag > 0.0f) ? (V_mag_clamped / V_mag) : 0.0f;
    V_alpha *= scale_factor;
    V_beta *= scale_factor;
    debug::trace(
        "Inverter SVPWM: V_d: %f, V_q: %f, V_alpha: %f, V_beta = %f, V_mag: %f",
        V_d, V_q, V_alpha, V_beta, V_mag);

    // float target_angle = std::atan2f(V_beta, V_alpha);
    // if (target_angle < 0.0f) {
    //     target_angle += 2.0f * M_PI;
    // }
    // uint8_t sector = std::floorf(target_angle / PI_DIV_3);

    // faster sector finding
    // transform to 3 phase
    float U1 = V_beta;
    float U2 = -0.5f * V_beta + COS_PI_6 * V_alpha;
    float U3 = -0.5f * V_beta - COS_PI_6 * V_alpha;
    // map to sector
    uint8_t n = 0;
    if (U1 > 0.0f) n |= 1;
    if (U2 > 0.0f) n |= 2;
    if (U3 > 0.0f) n |= 4;
    uint8_t sector = SECTOR_MAP[n];

    // rotate V_alpha and V_beta to sectors
    float cos_rot = COS_LOOKUP[sector];
    float sin_rot = SIN_LOOKUP[sector];
    float V_x = V_alpha * cos_rot - V_beta * sin_rot;
    float V_y = V_alpha * sin_rot + V_beta * cos_rot;
    debug::trace("Inverter SVPWM: V_x: %f, V_y: %f", V_x, V_y);

    // calculate component voltages
    float V_2 = V_y / COS_PI_6;
    float V_1 = V_x - V_y * TAN_PI_6;
    // calculate duty cycles
    float inv_V_dc = 1.0f / V_dc;
    float duty_1 = V_1 * inv_V_dc;
    float duty_2 = V_2 * inv_V_dc;
    float duty_zero = (1.0f - duty_1 - duty_2) / 2;

    float duty_U = 0, duty_V = 0, duty_W = 0;

    TargetSector sector_enum = (TargetSector)sector;

    switch (sector_enum) {
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

    return sector_enum;
}

static PWMFreqParams timer_init(uint32_t pwm_freq) {
    // init timer
    timer.Instance = INVERTER_TIMER;
    // assuming APB prescaler = 1
#if INVERTER_CLOCK == PCLK1
    uint32_t clk_freq = HAL_RCC_GetPCLK1Freq();
#elif INVERTER_CLOCK == PCLK2
    uint32_t clk_freq = HAL_RCC_GetPCLK2Freq();
#else
#error "Invalid clock source for inverter timer"
#endif
    debug::trace("Inverter: Timer PLL frequency: %u", clk_freq);
    // enable timer
    if (INVERTER_TIMER == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
    } else if (INVERTER_TIMER == TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
    } else if (INVERTER_TIMER == TIM15) {
        __HAL_RCC_TIM15_CLK_ENABLE();
    }

    PWMFreqParams freq_params =
        calculate_frequency_parameters(pwm_freq, clk_freq);
    timer.Init.Prescaler = freq_params.prescaler;
    timer.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    timer.Init.Period = freq_params.period;
    timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer.Init.RepetitionCounter =
        1;  // interrupt at trough of every PWM cycle (counter underflow)
    // changes to period will only apply after the current cycle completes
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    resolution = freq_params.period;

    if (HAL_TIM_PWM_Init(&timer) != HAL_OK) {
        debug::fatal("Inverter: Timer PWM init failed.");
        error::handler();
    }

    TIM_MasterConfigTypeDef master_config = {};

    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    master_config.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&timer, &master_config) !=
        HAL_OK) {
        debug::fatal("Inverter: Master config synchronization failed.");
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
    break_deadtime_config.DeadTime = (uint32_t)ceilf(
        (float)INVERTER_BREAKTIME *
        ((float)1e9 / (float)clk_freq));  // number of cycles to achieve
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
        debug::fatal("Inverter: Timer break and deadtime config failed.");
        error::handler();
    }

    // start PWM
    if (HAL_TIM_PWM_Start(&timer, U_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: Failed to start U phase PWM");
        error::handler();
    }
#ifdef INVERTER_COMPLEMENTARY_PWM
    if (HAL_TIMEx_PWMN_Start(&timer, U_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: Failed to start U_N phase PWM");
        error::handler();
    }
#endif
    if (HAL_TIM_PWM_Start(&timer, V_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: Failed to start V phase PWM");
        error::handler();
    }
#ifdef INVERTER_COMPLEMENTARY_PWM
    if (HAL_TIMEx_PWMN_Start(&timer, V_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: Failed to start V_N phase PWM");
        error::handler();
    }
#endif
    if (HAL_TIM_PWM_Start(&timer, W_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: Failed to start W phase PWM");
        error::handler();
    }
#ifdef INVERTER_COMPLEMENTARY_PWM
    if (HAL_TIMEx_PWMN_Start(&timer, W_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: Failed to start W_N phase PWM");
        error::handler();
    }
#endif

    // enable update interrupt
    if (HAL_TIM_Base_Start_IT(&timer) != HAL_OK) {
        debug::fatal("Inverter: Failed to start timer update interrupt");
        error::handler();
    }
    // set lowest preempt priority possible, so other calls can preempt it
    if (INVERTER_TIMER == TIM1) {
        HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    } else if (INVERTER_TIMER == TIM8) {
        HAL_NVIC_SetPriority(TIM8_UP_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);
    }

    return freq_params;
}

static void gpio_init(void) {
    gpio::init(INVERTER_U, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
#ifdef INVERTER_COMPLEMENTARY_PWM
    gpio::init(INVERTER_U_N, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
#endif
    gpio::init(INVERTER_V, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
#ifdef INVERTER_COMPLEMENTARY_PWM
    gpio::init(INVERTER_V_N, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
#endif
    gpio::init(INVERTER_W, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
#ifdef INVERTER_COMPLEMENTARY_PWM
    gpio::init(INVERTER_W_N, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
#endif

#ifdef INVERTER_USE_DRIVER_MODE
    gpio::init(INVERTER_DRIVER_MODE, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(INVERTER_DRIVER_MODE, INVERTER_DRIVER_MODE_STATE);
#endif

#ifdef INVERTER_USE_PHASE_RESET
    gpio::init(INVERTER_U_RST, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(INVERTER_U_RST, INVERTER_RST_STATE);
    gpio::init(INVERTER_V_RST, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(INVERTER_V_RST, INVERTER_RST_STATE);
    gpio::init(INVERTER_W_RST, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(INVERTER_W_RST, INVERTER_RST_STATE);
#endif
}

/**
 * @brief Calculate the prescaler and period parameters for achieving a
 * target PWM frequency for center-aligned PWM. Maximises ARR for resolution
 * @param target_frequency: Target frequency that will be achieved as closely as
 * possible
 * @param clock_source_frequency: Frequency of the clock source, either PCLK1 or
 * PCLK2 if APB prescalers are 1. TIM2..7 are on PCLK1, TIM1,8,20,15,16,17 are
 * on PCLK2
 * @returns prescaler, period and real frequency values
 */
static PWMFreqParams calculate_frequency_parameters(
    uint32_t target_frequency, uint32_t clock_source_frequency) {
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
        uint32_t prescaler = divisor / (period * 2);
        // compute actual frequency
        uint32_t actual_frequency =
            clock_source_frequency / (period * 2 * prescaler);
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

void inverter::timer_irq(void) { HAL_TIM_IRQHandler(&timer); }

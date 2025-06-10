#include "pwm.hpp"

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_tim.h>
#include <stm32g4xx_hal_tim_ex.h>

#include <cmath>

#include "config.hpp"
#include "debug.hpp"
#include "error.hpp"

TIM_HandleTypeDef drive_timer;

struct PWMFreqParams {
    uint16_t prescaler = 0;
    uint16_t period = 0;
    uint32_t frequency = 0;
};

PWMFreqParams calculate_frequency_parameters(uint32_t target_frequency,
                                             uint32_t clock_source_frequency);

void pwm::init(uint32_t drive_phase_pwm_frequency) {
    pwm::init_drive_phase(drive_phase_pwm_frequency);
}

/**
 * @brief Initialises motor PWM timer hardware.
 * @param motor_pwm_frequency: Target PWM frequency of
 * @note Modify as needed to suit hardware. Configurations are in
 * src/include/config
 */
void pwm::init_drive_phase(uint32_t pwm_freq) {
    // init drive timer
    drive_timer.Instance = DRIVE_PHASE_TIMER;
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
    drive_timer.Init.Prescaler = freq_params.prescaler;
    drive_timer.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    drive_timer.Init.Period = freq_params.period;
    drive_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    drive_timer.Init.RepetitionCounter = 0;  // interrupt every PWM cycle
    // changes to duty cycle will only apply after the current cycle completes
    drive_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&drive_timer) != HAL_OK) {
        debug::fatal("drive timer PWM init failed.");
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
    if (HAL_TIM_PWM_ConfigChannel(&drive_timer, &oc_config, U_PHASE_CHANNEL)) {
        debug::fatal("U phase PWM channel config failed.");
        error::handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&drive_timer, &oc_config, V_PHASE_CHANNEL)) {
        debug::fatal("V phase PWM channel config failed.");
        error::handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&drive_timer, &oc_config, W_PHASE_CHANNEL)) {
        debug::fatal("W phase PWM channel config failed.");
        error::handler();
    }
    // enable preload for changing duty cycle
    __HAL_TIM_ENABLE_OCxPRELOAD(&drive_timer, U_PHASE_CHANNEL);
    __HAL_TIM_ENABLE_OCxPRELOAD(&drive_timer, V_PHASE_CHANNEL);
    __HAL_TIM_ENABLE_OCxPRELOAD(&drive_timer, W_PHASE_CHANNEL);

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
    if (HAL_TIMEx_ConfigBreakDeadTime(&drive_timer, &break_deadtime_config)) {
        debug::fatal("drive timer break and deadtime config failed.");
        error::handler();
    }
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

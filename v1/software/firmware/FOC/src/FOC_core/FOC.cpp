/**
 * WARN : It is assumed that the encoder direction is equal to inverter
 * WARN : direction is equal to linear direction. Change polarities in configs
 * WARN : if not the case.
 */
#include "FOC.hpp"

#include <cmsis_os2.h>
#include <math.h>

#include <cstdlib>

#include "adc.hpp"
#include "configs.hpp"
#include "encoder.hpp"
#include "endstop.hpp"
#include "inverter.hpp"

volatile static float coil_resistance = -1;
volatile static float speed_constant = -1;
volatile static float torque_constant = -1;
volatile static float torque_Kp = -1;
volatile static float torque_Ki = -1;
volatile static float velocity_Kp = -1;
volatile static float velocity_Ki = -1;
volatile static float position_Kp = -1;
volatile static float position_Kd = -1;

volatile static float target_torque = 0;
volatile static float target_angular_velocity = 0;
volatile static float target_linear_velocity = 0;
volatile static float target_angular_position = 0;
volatile static float target_linear_position = 0;

volatile static float previous_torque = 0;
volatile static float previous_angular_velocity = 0;
volatile static float previous_linear_velocity = 0;
// this is based off endstop = 0;
volatile static float previous_angular_position = 0;
volatile static float previous_linear_position = 0;

volatile static float torque_err_I = 0;
volatile static float velocity_err_I = 0;
volatile static float position_err_I = 0;

volatile static bool run_IRQ = false;
// this is based off theta = 0
volatile static int64_t encoder_position;

volatile static bool any_endstop_triggered = false;
volatile static endstop::Endstop triggered_endstop;

// encoder position + offset = zeroed position
volatile static float zero_position_angular_offset = 0;
volatile static float zero_position_linear_offset = 0;

void zero_encoder(void);
#ifdef USE_ENDSTOP
void zero_position(void);
void endstop_irq(endstop::Endstop endstop, endstop::State state, void* args);
#endif

void FOC::init(Parameters parameters) {
    coil_resistance = parameters.coil_resistance;
    speed_constant = parameters.speed_constant;
    torque_constant = 1.0f / speed_constant;
    torque_Kp = parameters.torque_Kp;
    torque_Ki = parameters.torque_Ki;
    velocity_Kp = parameters.velocity_Kp;
    velocity_Ki = parameters.velocity_Ki;
    position_Kp = parameters.position_Kp;
    position_Kd = parameters.position_Kd;

    adc::init();
    encoder::init();
    inverter::init(PWM_FREQUENCY);
#ifdef USE_ENDSTOP
    endstop::init(endstop_irq);
#endif

    // zero encoder
    zero_encoder();
    zero_position_angular_offset = 0.0f;
    zero_position_linear_offset = 0.0f;
    previous_angular_position = 0.0f;  // will be changed by homing later
    previous_linear_position = 0.0f;
    // start FOC loop
    run_IRQ = true;

#ifdef USE_ENDSTOP
    zero_position();
#endif
    return;
}

void zero_encoder(void) {
    encoder_position = encoder::get_count();
    adc::start_VMOT_read();
    float voltage = adc::read_VMOT();
    inverter::svpwm_set(0.0f, ZERO_ENCODER_VOLTAGE / voltage, voltage);

    // keep checking if it has either:
    // 1. hit an endstop
    // 2. stopped moving
    // NOTE : not accounting for overflow, as this is ran at the start
    uint32_t last_encoder_pulse_tick = osKernelGetTickCount();
    while (true) {
        osDelay(1);

#ifdef USE_ENDSTOP
        // check for endstop crash
        if (any_endstop_triggered) {
            // reverse and recover
            if (triggered_endstop == endstop::Endstop::END_POS) {
                // crashed into the end, go back by one set (i.e go until theta
                // < -(3/2)PI)
                float current_theta = -(M_PI / 2.0f);
                while (current_theta > -(3.0f / 2.0f) * M_PI) {
                    inverter::svpwm_set(current_theta, ZERO_ENCODER_VOLTAGE,
                                        voltage);
                    current_theta -= ZERO_ENCODER_CRASH_REVERSE_THETA_INCREMENT;
                    osDelay(1);
                }
            } else if (triggered_endstop == endstop::Endstop::ZERO) {
                // crashed into the start, go forward by one set (i.e go until
                // theta > (3/2)PI)
                float current_theta = M_PI / 2.0f;
                while (current_theta < (3.0f / 2.0f) * M_PI) {
                    inverter::svpwm_set(current_theta, ZERO_ENCODER_VOLTAGE,
                                        voltage);
                    current_theta += ZERO_ENCODER_CRASH_REVERSE_THETA_INCREMENT;
                    osDelay(1);
                }
            }
        }
#endif

        // check for stopped moving
        int64_t new_encoder_position = encoder::get_count();
        uint32_t tick = osKernelGetTickCount();
        if (new_encoder_position != encoder_position) {
            encoder_position = new_encoder_position;
            last_encoder_pulse_tick = tick;
        }
        if ((tick - last_encoder_pulse_tick) >
            ZERO_ENCODER_MAX_TICKS_PER_PULSE) {
            encoder::set_count(0);
            // turn off motor
            inverter::svpwm_set(0.0f, 0.0f, 0.0f);
            encoder_position = encoder::get_count();
            break;
        }
    }

    return;
}

#ifdef USE_ENDSTOP

void zero_position(void) {
    FOC::set_angular_velocity(-ZERO_POSITION_ANGULAR_VELOCITY);
    while (!any_endstop_triggered) {
        osDelay(1);
    }
    FOC::set_linear_velocity(0);
    FOC::set_linear_position(0);

    return;
}

void endstop_irq(endstop::Endstop endstop, endstop::State state,
                 [[maybe_unused]] void* args) {
    if (state == endstop::State::TRIGGERED) {
        any_endstop_triggered = true;
        triggered_endstop = endstop;
        if (endstop == endstop::Endstop::ZERO) {
            float angular_position = ZERO_ENDSTOP_ANGULAR_POSITION;
            float encoder_angular_position = FOC::get_angular_position();

            // offsets
            zero_position_angular_offset =
                angular_position - encoder_angular_position;
            zero_position_angular_offset =
                zero_position_linear_offset / DISTANCE_PER_RADIAN;
        }
    }
}
#endif

void FOC::handler(void) {
    if (!run_IRQ) {
        return;
    }
    // measure currents
    float U_current = adc::read_U_current();
    float V_current = adc::read_V_current();
    float W_current = adc::read_W_current();
}

float FOC::get_angular_position(void) {
    return (encoder::get_count() / ENCODER_RADIANS_PER_PULSE) +
           zero_position_angular_offset;
}
float FOC::get_linear_position(void) {
    return (encoder::get_count() / ENCODER_RADIANS_PER_PULSE *
            DISTANCE_PER_RADIAN) +
           zero_position_linear_offset;
}

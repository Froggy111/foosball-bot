/**
 * WARN : It is assumed that the encoder direction is equal to inverter
 * WARN : direction is equal to linear direction. Change polarities in configs
 * WARN : if not the case.
 */
#include "FOC.hpp"

#include <cmsis_os2.h>
#include <math.h>

#include <cstdlib>

#include "PID.hpp"
#include "adc.hpp"
#include "config.hpp"
#include "encoder.hpp"
#include "endstop.hpp"
#include "inverter.hpp"

volatile static float coil_resistance = COIL_RESISTANCE;
volatile static float speed_constant = -1;
volatile static float torque_constant = -1;
volatile static uint32_t num_winding_sets = -1;

FOC::PID current_PID(CURRENT_KP, CURRENT_KI, 0);
FOC::PID velocity_PID(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD);
FOC::PID position_PID(POSITION_KP, POSITION_KI, POSITION_KD);

volatile static float U_current = 0;
volatile static float V_current = 0;
volatile static float W_current = 0;
volatile static float VMOT_voltage = 0;

volatile static bool run_IRQ = false;
// this is based off theta = 0
volatile static int64_t encoder_position;

volatile static bool any_endstop_triggered = false;
volatile static endstop::Endstop triggered_endstop;

// encoder position + offset = zeroed position
volatile static float zero_position_angular_offset = 0;
volatile static float zero_position_linear_offset = 0;

volatile static uint32_t handler_counter = 0;

void zero_encoder(void);
#ifdef USE_ENDSTOP
void zero_position(void);
void endstop_irq(endstop::Endstop endstop, endstop::State state, void* args);
#endif

void FOC::init(Parameters parameters) {
    coil_resistance = parameters.coil_resistance;
    speed_constant = parameters.speed_constant;
    torque_constant = 1.0f / speed_constant;

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

    // start FOC loop
    adc::start_VMOT_read();
    VMOT_voltage = adc::read_VMOT();
    run_IRQ = true;

#ifdef USE_ENDSTOP
    zero_position();
#endif
    return;
}

void FOC::set_PID(PIDParams pid_parameters) {
    current_PID.set_params(pid_parameters.current_Kp, pid_parameters.current_Ki,
                           0);
    velocity_PID.set_params(pid_parameters.velocity_Kp,
                            pid_parameters.velocity_Ki,
                            pid_parameters.velocity_Kd);
    position_PID.set_params(pid_parameters.position_Kp,
                            pid_parameters.position_Ki,
                            pid_parameters.position_Kd);
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
    return;
}
#endif

void FOC::handler(void) {
    if (!run_IRQ) {
        return;
    }
    // measure currents
    U_current = adc::read_U_current();
    V_current = adc::read_V_current();
    W_current = adc::read_W_current();

    // start voltage measurement
    adc::start_VMOT_read();

    float angular_position = get_angular_position();
    float electrical_angle = (angular_position * NUM_WINDING_SETS);
    float target_torque = velocity_PID.get();
    float target_current = target_torque / torque_constant;

    // transform target current to phase currents

    if (handler_counter % FOC_CYCLES_PER_VELOCITY_LOOP) {
        // run velocity control
    }

    if (handler_counter % FOC_CYCLES_PER_POSITION_LOOP) {
        // run position control
    }

    // read VMOT (end of handler)
    VMOT_voltage = adc::read_VMOT();
    handler_counter++;
    return;
}

float FOC::get_angular_position(void) {
    encoder_position = encoder::get_count();
    return (encoder_position / ENCODER_RADIANS_PER_PULSE) +
           zero_position_angular_offset;
}
float FOC::get_linear_position(void) {
    encoder_position = encoder::get_count();
    return (encoder_position / ENCODER_RADIANS_PER_PULSE *
            DISTANCE_PER_RADIAN) +
           zero_position_linear_offset;
}

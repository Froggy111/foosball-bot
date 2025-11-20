/**
 * WARN : It is assumed that the encoder direction is equal to inverter
 * WARN : direction is equal to linear direction. Change polarities in configs
 * WARN : if not the case.
 */
#include "FOC.hpp"

#include <FreeRTOS.h>
#include <arm_math.h>
#include <task.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "PID.hpp"
#include "adc.hpp"
#include "alpha_beta_gamma_filter.hpp"
#include "config.hpp"
#include "cordic.hpp"
#include "debug.hpp"
#include "encoder.hpp"
#include "endstop.hpp"
#include "inverter.hpp"
#include "lowpass_filter.hpp"
#include "moving_average.hpp"
#include "uart.hpp"

gpio::PinConfig DEBUG1 = {GPIOA, gpio::Pin::PIN2, gpio::AF::NONE};
gpio::PinConfig DEBUG2 = {GPIOA, gpio::Pin::PIN1, gpio::AF::NONE};

volatile static float coil_resistance = COIL_RESISTANCE;
volatile static float speed_constant = -1;
volatile static float torque_constant = -1;
volatile static uint32_t num_winding_sets = -1;

FOC::PID I_d_PID(CURRENT_D_KP, CURRENT_D_KI, 0, 1.0f);
FOC::PID I_q_PID(CURRENT_Q_KP, CURRENT_Q_KI, 0, 1.0f);
FOC::PID velocity_PID(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD, 1.0f);
FOC::PID position_PID(POSITION_KP, POSITION_KI, POSITION_KD, 1.0f);

FOC::LowPassFilter velocity_lowpass_filter(VELOCITY_LOWPASS_ALPHA);
FOC::MovingAverage<float, VELOCITY_MOVING_AVERAGE_SIZE> velocity_moving_average;
FOC::AlphaBetaGammaFilter abg_filter(VELOCITY_ABG_ALPHA, VELOCITY_ABG_BETA,
                                     VELOCITY_ABG_GAMMA,
                                     VELOCITY_SAMPLING_PERIOD);

// CONTROL SETTINGS
volatile static float position_target = 0;
volatile static float velocity_target = 0;
volatile static float torque_target = 0;
enum class TargetMode { position, velocity, torque };
volatile static TargetMode target_mode = TargetMode::torque;
volatile static bool run_IRQ = false;
volatile static bool tuning_mode = false;

volatile static float current_limit = 0;
volatile static float torque_limit = 0;
volatile static float velocity_limit = 0;
volatile static float acceleration_limit = 0;
volatile static float jerk_limit = 0;

volatile static float ramped_velocity_target = 0;
volatile static float current_acceleration = 0;

// ANALOG SENSING VALUES
volatile static float U_current = 0;
volatile static float V_current = 0;
volatile static float W_current = 0;
volatile static float VMOT_voltage = 0;
volatile static inverter::TargetSector sector = inverter::TargetSector::U;
volatile static float U_on_time = 0;
volatile static float V_on_time = 0;
volatile static float W_on_time = 0;

// VELOCITY CALCULATION
volatile static float velocity = 0;
volatile static uint32_t actual_pwm_frequency = 0;
volatile static float time_between_velocity_cycles = 0;
volatile static uint32_t cycles_between_last_position_change = 1;
volatile static float velocity_multiplier = 0;
volatile static float angular_position = 0;
volatile static float past_angular_position = 0;

// PID frequency corrections
float current_cycle_frequency = 1.0f;
float velocity_cycle_frequency = 1.0f;
float position_cycle_frequency = 1.0f;

// this is based off theta = 0
volatile static int64_t encoder_position = 0;

volatile static bool any_endstop_triggered = false;
volatile static endstop::Endstop triggered_endstop;

// encoder position + offset = zeroed position
volatile static float zero_position_angular_offset = 0;
#ifdef USE_LINEAR_MOTION
volatile static float zero_position_linear_offset = 0;
#endif

volatile static uint32_t handler_counter = 0;

void zero_encoder(void);
#ifdef USE_ENDSTOP
void zero_position(void);
void endstop_irq(endstop::Endstop endstop, endstop::State state, void* args);
#endif

struct TuningData {
    // int64_t encoder_count = 0;
    // int32_t encoder_timer_count = 0;
    // int64_t rollover_count = 0;
    float torque_target = 0;
    float velocity_target = 0, velocity = 0;
    float position_target = 0, position = 0, angular_position = 0;
    float I_d_target = 0, I_d = 0;
    float I_q_target = 0, I_q = 0;
    float U_current = 0, V_current = 0, W_current = 0;
    float VMOT_voltage = 0;
} __attribute__((packed));
volatile static TuningData tuning_data;
void tuning_task(void* args);

void FOC::init(Parameters parameters) {
    coil_resistance = parameters.coil_resistance;
    speed_constant = parameters.speed_constant;
    torque_constant = 1.0f / speed_constant;

    gpio::init(DEBUG1, gpio::Mode::OUTPUT_PP_, gpio::Pull::NOPULL,
               gpio::Speed::HIGH);
    gpio::init(DEBUG2, gpio::Mode::OUTPUT_PP_, gpio::Pull::NOPULL,
               gpio::Speed::HIGH);
    gpio::write(DEBUG1, 1);
    gpio::write(DEBUG2, 1);

    BaseType_t task_creation_status =
        xTaskCreate(tuning_task, "tuning task", 256, NULL, 16, NULL);

    debug::debug("FOC: initialising CORDIC");
    cordic::init();
    debug::debug("FOC: initialising ADC");
    adc::init();
    debug::debug("FOC: initialised ADC, initialising encoder");
    encoder::init();
    debug::debug("FOC: initialised encoder, initialising inverter");
    actual_pwm_frequency = inverter::init(PWM_FREQUENCY);
    debug::debug("FOC: initialised inverter, initialising UART");
    uart::init(921600);
    debug::debug("FOC: initialised UART");
    current_cycle_frequency = (float)actual_pwm_frequency;
    velocity_cycle_frequency =
        (float)actual_pwm_frequency / (float)FOC_CYCLES_PER_VELOCITY_LOOP;
    position_cycle_frequency =
        (float)actual_pwm_frequency / (float)FOC_CYCLES_PER_POSITION_LOOP;
    time_between_velocity_cycles = (1.0f / velocity_cycle_frequency);
    velocity_multiplier = current_cycle_frequency;

    // PID frequency correction
    I_q_PID.set_params(CURRENT_Q_KP, CURRENT_Q_KI, 0.0f,
                       current_cycle_frequency);
    I_d_PID.set_params(CURRENT_D_KP, CURRENT_D_KI, 0.0f,
                       current_cycle_frequency);
    velocity_PID.set_params(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD,
                            velocity_cycle_frequency);
    position_PID.set_params(POSITION_KP, POSITION_KI, POSITION_KD,
                            position_cycle_frequency);

#ifdef USE_ENDSTOP
    endstop::init(endstop_irq);
#endif

    adc::start_conversions();
    adc::Values readings = adc::read();
    VMOT_voltage = readings.voltage_VMOT;
    debug::debug("VMOT: %fV", VMOT_voltage);

    // zero encoder
    zero_encoder();
    zero_position_angular_offset = 0.0f;
#ifdef USE_LINEAR_MOTION
    zero_position_linear_offset = 0.0f;
#endif

#ifdef USE_ENDSTOP
    // start FOC loop
    enable();
    zero_position();
#endif

    angular_position = get_angular_position();
    past_angular_position = angular_position;
    abg_filter.reset(angular_position);
    return;
}

void FOC::set_PID(PIDParams pid_parameters) {
    I_q_PID.set_params(pid_parameters.current_q_Kp, pid_parameters.current_q_Ki,
                       0.0f, current_cycle_frequency);
    I_d_PID.set_params(pid_parameters.current_d_Kp, pid_parameters.current_d_Ki,
                       0.0f, current_cycle_frequency);
    velocity_PID.set_params(
        pid_parameters.velocity_Kp, pid_parameters.velocity_Ki,
        pid_parameters.velocity_Kd, velocity_cycle_frequency);
    position_PID.set_params(
        pid_parameters.position_Kp, pid_parameters.position_Ki,
        pid_parameters.position_Kd, position_cycle_frequency);

    I_q_PID.set(0);
    I_d_PID.set(0);
    velocity_PID.set(0);
    position_PID.set(0);
    return;
}

void zero_encoder(void) {
    encoder_position = encoder::get_count();

    float sweep_theta = 0.0f;

    // TODO : implement endstop crash check
    for (int i = 0; i <= ZERO_ENCODER_SWEEP_STEPS; i++) {
        inverter::svpwm_set(sweep_theta, ZERO_ENCODER_SWEEP_VOLTAGE, 0.0f,
                            VMOT_voltage);
        sweep_theta += ZERO_ENCODER_SWEEP_INCREMENT;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    inverter::svpwm_set(0.0f, ZERO_ENCODER_HOLD_VOLTAGE, 0.0f, VMOT_voltage);

    // keep checking if it has either:
    // 1. hit an endstop
    // 2. stopped moving
    // NOTE : not accounting for overflow, as this is ran at the start
    uint32_t last_encoder_pulse_tick = xTaskGetTickCount();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1));

#ifdef USE_ENDSTOP
        // check for endstop crash
        if (any_endstop_triggered) {
            // reverse and recover
            if (triggered_endstop == endstop::Endstop::END_POS) {
                // crashed into the end, go back by one set (i.e go until theta
                // < -(3/2)PI)
                float current_theta = -(M_PI / 2.0f);
                while (current_theta > -(3.0f / 2.0f) * M_PI) {
                    inverter::svpwm_set(current_theta, 0.0f,
                                        ZERO_ENCODER_VOLTAGE, VMOT_voltage);
                    current_theta -= ZERO_ENCODER_CRASH_REVERSE_THETA_INCREMENT;
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            } else if (triggered_endstop == endstop::Endstop::ZERO) {
                // crashed into the start, go forward by one set (i.e go until
                // theta > (3/2)PI)
                float current_theta = M_PI / 2.0f;
                while (curre12t_theta < (3.0f / 2.0f) * M_PI) {
                    inverter::svpwm_set(current_theta, 0.0f,
                                        ZERO_ENCODER_VOLTAGE, VMOT_voltage);
                    current_theta += ZERO_ENCODER_CRASH_REVERSE_THETA_INCREMENT;
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            }
        }
#endif

        // check for stopped moving
        int64_t new_encoder_position = encoder::get_count();
        uint32_t tick = xTaskGetTickCount();
        if (new_encoder_position != encoder_position) {
            encoder_position = new_encoder_position;
            last_encoder_pulse_tick = tick;
        }
        if ((tick - last_encoder_pulse_tick) >
            ZERO_ENCODER_MAX_TICKS_PER_PULSE) {
            encoder::set_count(0);
            // turn off motor
            inverter::svpwm_set(0.0f, 0.0f, 0.0f, VMOT_voltage);
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
        vTaskDelay(pdMS_TO_TICKS(1));
    }
#ifdef USE_LINEAR_MOTION
    FOC::set_linear_velocity(0);
    FOC::set_linear_position(0);
#endif

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
#ifdef USE_LINEAR_MOTION
            zero_position_linear_offset =
                zero_position_linear_offset / DISTANCE_PER_RADIAN;
#endif
        }
    }
    return;
}
#endif

const float TWO_THIRDS = 2.0f / 3.0f;
const float ONE_THIRD = 1.0f / 3.0f;
const float ONE_OVER_SQRT3 = 1.0f / std::sqrtf(3.0f);

void FOC::handler(void) {
    if (!run_IRQ) {
        return;
    }

    // NOTE: t=0 us
    gpio::write(DEBUG1, 1);
    adc::start_conversions();
    // measure currents
    // NOTE: t=6us
    gpio::write(DEBUG1, 0);

    angular_position = get_angular_position();
    float electrical_angle =
        std::fmod((angular_position * NUM_WINDING_SETS), 2 * M_PI);
    if (electrical_angle < 0.0f) {
        electrical_angle += 2.0f * M_PI;
    }

    // NOTE: t=14us
    gpio::write(DEBUG2, 0);

    // calculate velocity with lowpass filter
    float velocity_tmp =
        (angular_position - past_angular_position) * velocity_multiplier;
    velocity = velocity_tmp;
    // // velocity = velocity_lowpass_filter.process(velocity_tmp);
    // velocity_moving_average.add(velocity_tmp);
    // velocity = velocity_moving_average.get();

    // abg_filter.update(angular_position);
    // float velocity_tmp = abg_filter.get_velocity();
    // velocity_moving_average.add(velocity_tmp);
    // velocity = velocity_moving_average.get();

    // run position control (only if in position mode)
    if (handler_counter % FOC_CYCLES_PER_POSITION_LOOP == 0 &&
        target_mode == TargetMode::position) {
        // run position control
        position_PID.set(position_target);
        position_PID.update(abg_filter.get_position());
        velocity_target = position_PID.get();
        if (velocity_target > velocity_limit) {
            velocity_target = velocity_limit;
        } else if (velocity_target < -velocity_limit) {
            velocity_target = -velocity_limit;
        }
    }

    // NOTE: t=14us
    gpio::write(DEBUG2, 1);

    // run velocity control (only if in velocity or position mode)
    if (handler_counter % FOC_CYCLES_PER_VELOCITY_LOOP == 0 &&
        (target_mode == TargetMode::velocity ||
         target_mode == TargetMode::position)) {
        // jerk limit
        float target_acceleration = (velocity_target - ramped_velocity_target) *
                                    velocity_cycle_frequency;
        if (target_acceleration > acceleration_limit) {
            target_acceleration = acceleration_limit;
        } else if (target_acceleration < -acceleration_limit) {
            target_acceleration = -acceleration_limit;
        }
        float acceleration_change = target_acceleration - current_acceleration;
        float max_acceleration_change =
            jerk_limit * time_between_velocity_cycles;
        if (acceleration_change > max_acceleration_change) {
            current_acceleration += max_acceleration_change;
        } else if (acceleration_change < -max_acceleration_change) {
            current_acceleration -= max_acceleration_change;
        } else {
            current_acceleration = target_acceleration;
        }

        // acceleration limit
        float velocity_change = velocity_target - ramped_velocity_target;
        float max_velocity_change =
            std::fabs(current_acceleration) * time_between_velocity_cycles;
        if (velocity_change > max_velocity_change) {
            ramped_velocity_target += max_velocity_change;
        } else if (velocity_change < -max_velocity_change) {
            ramped_velocity_target -= max_velocity_change;
        } else {
            ramped_velocity_target = velocity_target;
        }
        // run velocity control
        velocity_PID.set(ramped_velocity_target);
        velocity_PID.update(velocity);
        torque_target = velocity_PID.get();
        if (torque_target > torque_limit) {
            torque_target = torque_limit;
        } else if (torque_target < -torque_limit) {
            torque_target = -torque_limit;
        }
    }
    past_angular_position = angular_position;

    // NOTE: t=16us
    gpio::write(DEBUG2, 0);

    float I_target = torque_target / torque_constant;

    if (I_target > current_limit) {
        I_target = current_limit;
    } else if (I_target < -current_limit) {
        I_target = -current_limit;
    }

    // replace with CORDIC if needed
    // cordic::SinCosVal sin_cos_values = cordic::sincos(electrical_angle);
    // float sin_theta = sin_cos_values.sin;
    // float cos_theta = sin_cos_values.cos;
    float sin_theta, cos_theta;
    arm_sin_cos_f32(electrical_angle * (180.0f / M_PI), &sin_theta, &cos_theta);

    // NOTE: t=19us
    gpio::write(DEBUG2, 1);

    adc::Values readings = adc::read();
    U_current = -readings.current_U;
    V_current = -readings.current_V;
    W_current = -readings.current_W;
    VMOT_voltage = readings.voltage_VMOT;

    // ISSUE : I screwed up and the sensors only sense negative current
    // ISSUE : As a workaround, positive values are interpolated
    // figure out which 2 current measurements to trust
    float divisor = 0;
    switch (sector) {
        case inverter::TargetSector::U:
            // current going into coil U, exiting V and W
            U_current = -(V_current + W_current);
            break;
        case inverter::TargetSector::UV:
            // current going into coils U and V, exiting W
            divisor = U_on_time + V_on_time;
            if (divisor == 0) {
                U_current = 0, V_current = 0;
                break;
            }
            U_current = (-W_current) * (U_on_time / divisor);
            V_current = (-W_current) * (V_on_time / divisor);
            break;
        case inverter::TargetSector::V:
            // current going into coil V, exiting U and W
            V_current = -(U_current + W_current);
            break;
        case inverter::TargetSector::VW:
            // current going into coils V and W, exiting U
            divisor = V_on_time + W_on_time;
            if (divisor == 0) {
                V_current = 0, W_current = 0;
                break;
            }
            V_current = (-U_current) * (V_on_time / divisor);
            W_current = (-U_current) * (W_on_time / divisor);
            break;
        case inverter::TargetSector::W:
            // current going into coil W, exiting U and V
            W_current = -(U_current + V_current);
            break;
        case inverter::TargetSector::WU:
            // current going into coils U and W, exiting V
            divisor = U_on_time + W_on_time;
            if (divisor == 0) {
                U_current = 0, W_current = 0;
                break;
            }
            U_current = (-V_current) * (U_on_time / divisor);
            W_current = (-V_current) * (W_on_time / divisor);
            break;
    }

    // transform target current to phase currents
    // clarke transformation
    float I_alpha = U_current;
    float I_beta = ONE_OVER_SQRT3 * (V_current - W_current);

    float I_d = I_beta * sin_theta + I_alpha * cos_theta;
    float I_q = I_beta * cos_theta - I_alpha * sin_theta;

    // target I_d = 0, target I_q = I_target
    I_d_PID.update(I_d);
    I_q_PID.set(I_target);
    I_q_PID.update(I_q);

    float V_limit =
        current_limit * COIL_RESISTANCE + velocity * BACK_EMF_CONSTANT;
    // float V_d = I_d_PID.get() * COIL_RESISTANCE;
    float V_d = 0.0f * COIL_RESISTANCE;
    if (V_d > V_limit) {
        V_d = V_limit;
    } else if (V_d < -V_limit) {
        V_d = -V_limit;
    }
    // float V_q = I_q_PID.get() * COIL_RESISTANCE + velocity *
    // BACK_EMF_CONSTANT;
    float V_q = I_target * COIL_RESISTANCE + velocity * BACK_EMF_CONSTANT;
    if (V_q > V_limit) {
        V_q = V_limit;
    } else if (V_q < -V_limit) {
        V_q = -V_limit;
    }

    // NOTE: t=24us
    gpio::write(DEBUG2, 0);

    // sector = inverter::svpwm_set(sin_theta, cos_theta, V_d, V_q,
    // VMOT_voltage);
    inverter::SVPWMData svpwm_data =
        inverter::svpwm_set(sin_theta, cos_theta, V_d, -3.0f, VMOT_voltage);
    sector = svpwm_data.sector;
    U_on_time = svpwm_data.U_on_time;
    V_on_time = svpwm_data.V_on_time;
    W_on_time = svpwm_data.W_on_time;

    // NOTE: t=30us
    gpio::write(DEBUG2, 1);
    handler_counter++;
    return;
}

void FOC::enable(void) {
    run_IRQ = true;
    return;
}

void FOC::disable(void) {
    run_IRQ = false;
    return;
}

void FOC::enable_tuning_mode(void) {
    tuning_mode = true;
    return;
}

void FOC::disable_tuning_mode(void) {
    tuning_mode = false;
    return;
}

uint32_t FOC::get_handler_counter(void) { return handler_counter; }

float FOC::get_V_d_target(void) { return I_d_PID.get() * COIL_RESISTANCE; }
float FOC::get_V_q_target(void) { return I_q_PID.get() * COIL_RESISTANCE; }
float FOC::get_I_d_target(void) { return I_d_PID.get_target(); }
float FOC::get_I_q_target(void) { return I_q_PID.get_target(); }
float FOC::get_torque_target(void) { return torque_target; }
float FOC::get_velocity_target(void) { return velocity_target; }
float FOC::get_position_target(void) { return position_target; }
float FOC::get_I_d(void) { return I_d_PID.get_actual(); }
float FOC::get_I_q(void) { return I_q_PID.get_actual(); }
float FOC::get_torque(void) { return I_q_PID.get_actual() * torque_constant; }
float FOC::get_velocity(void) { return velocity_PID.get_actual(); }
float FOC::get_position(void) { return position_PID.get_actual(); }

float FOC::get_angular_position(void) {
    encoder_position = encoder::get_count();
    return (encoder_position * ENCODER_RADIANS_PER_PULSE) +
           zero_position_angular_offset;
}

#ifdef USE_LINEAR_MOTION
float FOC::get_linear_position(void) {
    encoder_position = encoder::get_count();
    return (encoder_position * ENCODER_RADIANS_PER_PULSE *
            DISTANCE_PER_RADIAN) +
           zero_position_linear_offset;
}
#endif

void FOC::set_max_current(float max_current) { current_limit = max_current; }
void FOC::set_torque(float torque) {
    target_mode = TargetMode::torque;
    torque_target = torque;
    return;
}
void FOC::set_max_torque(float torque) {
    torque_limit = torque;
    return;
}
void FOC::set_angular_velocity(float angular_velocity) {
    target_mode = TargetMode::velocity;
    velocity_target = angular_velocity;
    return;
}
void FOC::set_max_angular_velocity(float max_angular_velocity) {
    velocity_limit = max_angular_velocity;
    return;
}
void FOC::set_max_angular_acceleration(float max_angular_acceleration) {
    acceleration_limit = max_angular_acceleration;
    return;
}
void FOC::set_angular_jerk(float angular_jerk) {
    jerk_limit = angular_jerk;
    return;
}
void FOC::set_angular_position(float angular_position) {
    target_mode = TargetMode::position;
    position_target = angular_position;
    return;
}
#ifdef USE_LINEAR_MOTION
void FOC::set_linear_velocity(float linear_velocity) {
    set_angular_velocity(linear_velocity * RADIANS_PER_DISTANCE);
    return;
}
void FOC::set_max_linear_velocity(float max_linear_velocity) {
    set_max_angular_velocity(max_linear_velocity * RADIANS_PER_DISTANCE);
    return;
}
void FOC::set_max_linear_acceleration(float max_linear_acceleration) {
    set_max_angular_acceleration(max_linear_acceleration *
                                 RADIANS_PER_DISTANCE);
    return;
}
void FOC::set_linear_jerk(float linear_jerk) {
    set_angular_jerk(linear_jerk * RADIANS_PER_DISTANCE);
    return;
}
void FOC::set_linear_position(float linear_position) {
    set_angular_position(linear_position * RADIANS_PER_DISTANCE);
    return;
}
#endif

void FOC::set_parameters(Parameters parameters) {
    coil_resistance = parameters.coil_resistance;
    speed_constant = parameters.speed_constant;
    torque_constant = 1.0f / speed_constant;
    return;
}

void tuning_task([[maybe_unused]] void* args) {
    for (;;) {
        if (tuning_mode) {
            tuning_data.torque_target = torque_target;
            tuning_data.velocity_target = velocity_PID.get_target();
            tuning_data.velocity = velocity;
            tuning_data.position_target = position_PID.get_target();
            tuning_data.position = position_PID.get_actual();
            tuning_data.angular_position = angular_position;
            tuning_data.I_d_target = I_d_PID.get_target();
            tuning_data.I_d = I_d_PID.get_actual();
            tuning_data.I_q_target = I_q_PID.get_target();
            tuning_data.I_q = I_q_PID.get_actual();
            tuning_data.U_current = U_current;
            tuning_data.V_current = V_current;
            tuning_data.W_current = W_current;
            tuning_data.VMOT_voltage = VMOT_voltage;

            // tuning_data.encoder_count = encoder_position;
            // tuning_data.encoder_timer_count = encoder::get_timer_count();
            // tuning_data.rollover_count = encoder::get_rollovers();
            uart::transmit(0xFFFE, (uint8_t*)&tuning_data, sizeof(tuning_data));
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

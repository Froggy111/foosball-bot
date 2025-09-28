#pragma once

#include "config.hpp"

namespace FOC {
struct Parameters {
    float coil_resistance = COIL_RESISTANCE;  // ohms
    // torque constant is computed as 1 / speed constant
    float speed_constant = MOTOR_KV;  // radians per second per volt
    float num_winding_sets = NUM_WINDING_SETS;
};

struct PIDParams {
    float current_d_Kp = CURRENT_D_KP;
    float current_d_Ki = CURRENT_D_KI;
    float current_q_Kp = CURRENT_Q_KP;
    float current_q_Ki = CURRENT_Q_KI;
    float velocity_Kp = VELOCITY_KP;
    float velocity_Ki = VELOCITY_KI;
    float velocity_Kd = VELOCITY_KD;
    float position_Kp = POSITION_KP;
    float position_Ki = POSITION_KI;
    float position_Kd = POSITION_KD;
};

/**
 * @brief Init FOC loop, run this in task space
 */
void init(Parameters parameters);

/**
 * @brief Set FOC PID parameters
 */
void set_PID(PIDParams pid_parameters);

/**
 * @brief called in the FOC loop (usually PWM interrupt)
 */
void handler(void);

/**
 * @brief set target torque, and sets the algorithm to torque mode.
 * Torque is in newton-meters
 */
void set_torque(float torque);
void set_max_torque(float torque);
/**
 * @brief set target velocity, and sets the algorithm to velocity mode.
 * Angular velocity is in radians per second.
 * Linear velocity is in millimeters per second.
 */
void set_angular_velocity(float angular_velocity);
void set_linear_velocity(float linear_velocity);
void set_max_angular_velocity(float max_angular_velocity);
void set_max_linear_velocity(float max_linear_velocity);
void set_max_angular_acceleration(float max_angular_acceleration);
void set_max_linear_acceleration(float max_linear_acceleration);
void set_angular_jerk(float angular_jerk);
void set_linear_jerk(float linear_jerk);
/**
 * @brief set target position, and sets the algorithm to position mode.
 * Angular position is in radians.
 * Linear position is in millimeters.
 */
void set_angular_position(float angular_position);
void set_linear_position(float linear_position);
float get_angular_position(void);
float get_linear_position(void);

void set_parameters(Parameters parameters);

}  // namespace FOC

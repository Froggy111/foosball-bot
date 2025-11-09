#pragma once

#include <math.h>
#include <stdint.h>

#include "configs/encoder.hpp"

// #define USE_ENDSTOP

const bool direction_reversed = false;

const uint32_t PWM_FREQUENCY = 24000;
const uint32_t NUM_WINDING_SETS = 2;
const float COIL_TO_COIL_RESISTANCE = 1.1;                            // in ohms
const float COIL_RESISTANCE = COIL_TO_COIL_RESISTANCE * 3.0f / 4.0f;  // in ohms
const float MOTOR_KV = 61 * (2 * M_PI / 60);  // radians per second per volt

const uint8_t FOC_CYCLES_PER_VELOCITY_LOOP = 2;
const uint8_t FOC_CYCLES_PER_POSITION_LOOP = 8;

const float ENCODER_RADIANS_PER_PULSE = (2.0f * M_PI) / ENCODER_RESOLUTION;

const float ZERO_ENCODER_VOLTAGE = 1.0f;
const float ZERO_ENCODER_MIN_RPM = 0.1;
const float ZERO_ENCODER_CRASH_REVERSE_THETA_INCREMENT = M_PI / 180.0f;
const uint32_t ZERO_ENCODER_MAX_TICKS_PER_PULSE =
    1.0f / ((ZERO_ENCODER_MIN_RPM / (60.0f * 1000.0f)) * ENCODER_RESOLUTION);

// linear parameters
// NOTE : comment this out to not use linear motion
// #define USE_LINEAR_MOTION
// const float ROTATION_DISTANCE = 20.0f;  // 20mm per rotation
// const float DISTANCE_PER_RADIAN = ROTATION_DISTANCE / (2.0f * M_PI);
// const float RADIANS_PER_DISTANCE = 1 / DISTANCE_PER_RADIAN;
//
// // mm/s
// const float ZERO_POSITION_LINEAR_VELOCITY = 100;
// const float ZERO_POSITION_ANGULAR_VELOCITY =
//     ZERO_POSITION_LINEAR_VELOCITY / DISTANCE_PER_RADIAN;
// const float ZERO_ENDSTOP_LINEAR_POSITION = -5;
// const float ZERO_ENDSTOP_ANGULAR_POSITION =
//     ZERO_ENDSTOP_LINEAR_POSITION / DISTANCE_PER_RADIAN;

// PID
// current is PI
// direct-axis current (magnetic flux)
const float CURRENT_D_KP = 1;
const float CURRENT_D_KI = 0.1;
// quadrature-axis current (torque generating)
const float CURRENT_Q_KP = 1;
const float CURRENT_Q_KI = 0.1;
// velocity is PI/PID
const float VELOCITY_KP = 1;
const float VELOCITY_KI = 0;
const float VELOCITY_KD = 0;
// position is P/PI/PID
const float POSITION_KP = 0;
const float POSITION_KI = 0;
const float POSITION_KD = 0;

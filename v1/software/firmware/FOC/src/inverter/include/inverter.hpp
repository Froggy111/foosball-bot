#include <stdint.h>

namespace inverter {

/**
 * @brief Initialises inverter PWM timer hardware.
 * @param motor_pwm_frequency: Target PWM frequency of
 * @note Modify as needed to suit hardware. Configurations are in
 * src/configs
 */
void init(uint32_t pwm_frequency);

/**
 * @brief Reverse direction of the inverter (swaps V and W phases)
 */
void reverse_direction(void);

/**
 * @brief Set phase on-times
 * @param u: on-time of phase u
 * @param v: on-time of phase v
 * @param w: on-time of phase w
 */
void set(float u, float v, float w);

/**
 * @brief Set SVPWM angle and duty cycle
 * @param theta: angle of target voltage vector
 * @param V_target: magnitude of target voltage vector (line-to-line)
 * @param V_dc: inverter DC bus voltage
 */
void svpwm_set(float theta, float V_target, float V_dc);

}  // namespace inverter

#include <stdint.h>

namespace inverter {

/**
 * @brief Initialises inverter PWM timer hardware.
 * @param motor_pwm_frequency: Target PWM frequency of
 * @note Modify as needed to suit hardware. Configurations are in
 * src/include/config
 */
void init(uint32_t pwm_frequency);

/**
 * @brief Reverse direction of the inverter (swaps V and W phases)
 */
void reverse_direction(void);

/**
 * @brief Set SVPWM angle and duty cycle
 * @param electrical_angle: electrical angle in radians
 * @param duty_cycle: 0-65535, scaled to actual resolution automatically
 */
void svpwm_set(float electrical_angle, uint16_t duty_cycle);

}  // namespace inverter

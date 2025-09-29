#include <stdint.h>

namespace inverter {

/**
 * @brief Initialises inverter PWM timer hardware.
 * @param motor_pwm_frequency: Target PWM frequency of
 * @note Modify as needed to suit hardware. Configurations are in
 * src/configs
 * @returns actual PWM frequency
 */
uint32_t init(uint32_t pwm_frequency);

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

struct SVPWMDuty {
    volatile float u = 0, v = 0, w = 0;
};

/**
 * @brief Set SVPWM angle and duty cycle
 * @param theta: angle of target voltage vector
 * @param V_d: magnitude of target voltage d vector (line-to-line) (magnetic
 * flux)
 * @param V_q: magnitude of target voltage q vector (line-to-line) (torque)
 * @param V_dc: inverter DC bus voltage
 */
SVPWMDuty svpwm_set(float theta, float V_d, float V_q, float V_dc);
// same function but taking in sin and cos theta instead
SVPWMDuty svpwm_set(float sin_theta, float cos_theta, float V_d, float V_q,
                    float V_dc);

}  // namespace inverter

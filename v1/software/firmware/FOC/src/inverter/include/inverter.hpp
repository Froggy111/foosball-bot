#include <stdint.h>

namespace inverter {

enum class TargetSector : uint8_t {
    U = 0,
    UV = 1,
    V = 2,
    VW = 3,
    W = 4,
    WU = 5,
};

struct SVPWMData {
    TargetSector sector = TargetSector::U;
    float U_on_time = 0, V_on_time = 0, W_on_time = 0;
};

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

/**
 * @brief Set SVPWM angle and duty cycle
 * @param theta: angle of target voltage vector
 * @param V_d: magnitude of target voltage d vector (line-to-line) (magnetic
 * flux)
 * @param V_q: magnitude of target voltage q vector (line-to-line) (torque)
 * @param V_dc: inverter DC bus voltage
 */
SVPWMData svpwm_set(float theta, float V_d, float V_q, float V_dc);
// same function but taking in sin and cos theta instead
SVPWMData svpwm_set(float sin_theta, float cos_theta, float V_d, float V_q,
                    float V_dc);

void timer_irq(void);
}  // namespace inverter

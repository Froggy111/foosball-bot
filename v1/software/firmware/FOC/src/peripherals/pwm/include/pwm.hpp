#include <stdint.h>

namespace pwm {

void init(uint32_t drive_phase_pwm_frequency);
void init_drive_phase(uint32_t pwm_frequency);

}  // namespace pwm

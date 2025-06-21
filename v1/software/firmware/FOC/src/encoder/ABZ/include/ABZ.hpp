#include <stm32g4xx_hal.h>

#include "config.hpp"

namespace encoder {

void init(void);

uint16_t get_count(void);

}  // namespace encoder

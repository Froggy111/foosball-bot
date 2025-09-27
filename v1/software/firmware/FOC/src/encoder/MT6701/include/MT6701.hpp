#pragma once

#include "config.hpp"
#if ENCODER_TYPE == MT6701

#include <stm32g4xx_hal.h>

namespace encoder {

void init(void);

/**
 * @brief Get summed count
 * @returns summed count
 */
int64_t get_count(void);

/**
 * @brief Set summed count. Used for startup sequence when finding rotor angle.
 * @param count: summed count
 */
void set_count(int32_t count);

}  // namespace encoder

#endif

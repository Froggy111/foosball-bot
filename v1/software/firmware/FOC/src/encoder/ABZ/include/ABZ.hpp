#include <stm32g4xx_hal.h>

#include "config.hpp"

namespace encoder {

void init(void);

/**
 * @brief Get summed count (Z at multiples of encoder resolution)
 * @returns summed count
 */
int64_t get_count(void);

/**
 * @brief Set summed count (Z at multiples of encoder resolution). Used for
 * startup sequence when finding rotor angle.
 * @param count: summed count, following Z being at multiples of encoder
 * resolution
 */
void set_count(int64_t count);

/**
 * @brief This is only for core_interrupts.cpp. It should not be called anywhere
else.
 */
void timer_irq(void);

/**
 * @brief This is only for core_interrupts.cpp. It should not be called anywhere
else.
 */
void rollover_irq(void);

uint32_t get_z_pulses(void);

int32_t get_delta(void);

}  // namespace encoder

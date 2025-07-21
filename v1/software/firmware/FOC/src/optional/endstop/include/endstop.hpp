#pragma once

#include <stdint.h>

namespace endstop {

enum class State : bool {
    TRIGGERED = true,
    RELEASED = false,
};
enum class Endstop : uint8_t {
    ZERO,
    END_POS,
};

using InterruptFn = void (*)(Endstop, State, void*);

/**
 * @brief call in task space
 */
void init(InterruptFn irq);
}  // namespace endstop

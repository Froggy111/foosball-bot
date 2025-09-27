#pragma once

#include <stdint.h>

namespace swo {
void init(void);
void write(uint8_t* buf, uint16_t length);
}  // namespace swo

#pragma once

#include <cstdint>

namespace uart {

void init(uint32_t baud_rate);

void transmit(uint8_t* data, uint16_t len);
bool can_transmit(void);

// used in core_interrupts.cpp
void transmit_complete_callback(void);
void receive_complete_callback(void);

}  // namespace uart

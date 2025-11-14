#pragma once

#include <cstdint>

namespace uart {

void init(uint32_t baud_rate);

void transmit(uint8_t* data, uint16_t len);
bool can_transmit(void);

// used in core_interrupts.cpp
void DMA_TX_handler(void);
void DMA_RX_handler(void);
void UART_handler(void);
void transmit_complete_callback(void);
void receive_complete_callback(void);

}  // namespace uart

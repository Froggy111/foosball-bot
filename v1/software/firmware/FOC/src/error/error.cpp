#include "error.hpp"

void error::handler(void) {
  while (1) {
    // do nothing
    __asm__ __volatile__("nop");
  }
}

extern "C" {
void Error_Handler(void) { error::handler(); }
}

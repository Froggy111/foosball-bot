#include "error.hpp"
#include "timing.hpp"

void error::handler(void) {
  while (1) {
    // do nothing
    __asm__ __volatile__("nop");
  }
}

#include "usb.hpp"

#include <stm32g4xx_hal.h>

#include "config.hpp"
#include "error.hpp"

#if STDIO_TARGET == STDIO_USB

extern "C" {

#ifdef __GNUC__

int _write(int file, char *ptr, int len) {
  if (file != 1 && file != 2) {
    return -1;
  }

  usb::write((uint8_t *)ptr, len);
  return len;
}

#else

int fputc(int ch, FILE *f) {
  usb::write((uint8_t *)&ch, 1);
  return ch;
}

#endif
}

#endif

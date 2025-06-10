#include <stdint.h>

#include "error.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#ifndef PRINTF_USB
#define PRINTF_USB 1
#endif

namespace usb {
// must be called after RTOS is initialised
void init(void);
void write(uint8_t *buf, uint16_t length);
}  // namespace usb

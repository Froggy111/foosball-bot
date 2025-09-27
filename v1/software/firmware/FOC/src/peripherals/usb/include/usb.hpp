#include <stdint.h>

#include "usb_device.h"
#include "usbd_cdc_if.h"

namespace usb {
// must be called after RTOS is initialised
void init(void);
void write(uint8_t *buf, uint16_t length);
}  // namespace usb

#include <stdint.h>

#include "error.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#ifndef PRINTF_USB
#define PRINTF_USB 1
#endif

namespace usb {
inline void init(void) {
    if (MX_USB_Device_Init() != USBD_OK) {
        error::handler();
    }
}

inline void write(uint8_t *buf, uint16_t length) {
    CDC_Transmit_FS(buf, length);
};
}  // namespace usb

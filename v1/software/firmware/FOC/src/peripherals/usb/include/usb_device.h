#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "usbd_def.h"

/** USB Device initialization function. */
USBD_StatusTypeDef MX_USB_Device_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_DEVICE__H__ */

#include "usb_device.h"

#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_desc.h"

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef CDC_Desc;

/**
 * Init USB device Library, add supported class and start the library
 * @retval None
 */
USBD_StatusTypeDef MX_USB_Device_Init(void) {
    /* Init Device Library, add supported class and start the library. */
    if (USBD_Init(&hUsbDeviceFS, &CDC_Desc, DEVICE_FS) != USBD_OK) {
        return USBD_FAIL;
    }
    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK) {
        return USBD_FAIL;
    }
    if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) !=
        USBD_OK) {
        return USBD_FAIL;
    }
    if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
        return USBD_FAIL;
    }
    return USBD_OK;
}

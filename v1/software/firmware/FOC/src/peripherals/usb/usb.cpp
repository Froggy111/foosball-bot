#include "usb.hpp"

#include <cmsis_os2.h>
#include <stdio.h>
#include <stm32g4xx_hal.h>

#include "config.hpp"
#include "error.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"

uint8_t write_buffer[USB_TX_BUFFER_SIZE] = {0};
osMutexId_t write_buffer_mutex;
uint16_t write_buffer_idx = 0;
uint8_t transmit_buffer[USB_TX_BUFFER_SIZE] = {0};
osThreadId_t usb_flush_task_id = NULL;

const uint32_t TRANSMIT_COMPLETE = 1 << 0;
const uint32_t WRITE_REQUEST = 1 << 1;

void usb_flush_task(void *args);

void usb::init(void) {
    if (MX_USB_Device_Init() != USBD_OK) {
        error::handler();
    }
    const osMutexAttr_t write_buffer_mutex_attributes = {
        .name = "write buffer mutex"};
    write_buffer_mutex = osMutexNew(&write_buffer_mutex_attributes);
    const osThreadAttr_t usb_flush_task_attributes = {
        .name = "usb flush task",
        .stack_size = 256,
        .priority = osPriorityHigh};
    usb_flush_task_id =
        osThreadNew(usb_flush_task, NULL, &usb_flush_task_attributes);
    osThreadFlagsSet(usb_flush_task_id, TRANSMIT_COMPLETE);
}

void transmit_complete_callback(void) {
    if (usb_flush_task_id != NULL) {
        osThreadFlagsSet(usb_flush_task_id, TRANSMIT_COMPLETE);
    }
}

void usb::write(uint8_t *buf, uint16_t length) {
    uint16_t written = 0;
    while (written != length) {
        osMutexAcquire(write_buffer_mutex, osWaitForever);
        uint16_t free_space = USB_TX_BUFFER_SIZE - write_buffer_idx;
        uint16_t write_len = MIN(free_space, length - written);
        memcpy(&write_buffer[write_buffer_idx], &buf[written], write_len);
        write_buffer_idx += write_len;
        osMutexRelease(write_buffer_mutex);
        osThreadFlagsSet(usb_flush_task_id, WRITE_REQUEST);
        written += write_len;
        osDelay(1);
    }
}

void usb_flush_task([[maybe_unused]] void *args) {
    osDelay(1);
    for (;;) {
        osThreadFlagsWait(WRITE_REQUEST, osFlagsWaitAny, osWaitForever);
        osThreadFlagsWait(TRANSMIT_COMPLETE, osFlagsWaitAny, osWaitForever);
        osMutexAcquire(write_buffer_mutex, osWaitForever);
        memcpy(transmit_buffer, write_buffer, write_buffer_idx);
        CDC_Transmit_FS((uint8_t *)transmit_buffer, write_buffer_idx);
        memset(write_buffer, 0, USB_TX_BUFFER_SIZE);
        write_buffer_idx = 0;
        osMutexRelease(write_buffer_mutex);
    }
}

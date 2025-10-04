#include "usb.hpp"

#include <FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <stm32g4xx_hal.h>
#include <task.h>

#include <climits>

#include "config.hpp"
#include "error.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"

uint8_t write_buffer[USB_TX_BUFFER_SIZE] = {0};
SemaphoreHandle_t write_buffer_mutex;
uint16_t write_buffer_idx = 0;
uint8_t transmit_buffer[USB_TX_BUFFER_SIZE] = {0};
TaskHandle_t usb_flush_task_handle = NULL;

const uint32_t TRANSMIT_COMPLETE = 1 << 0;
const uint32_t WRITE_REQUEST = 1 << 1;

void usb_flush_task(void *args);

void usb::init(void) {
    if (MX_USB_Device_Init() != USBD_OK) {
        error::handler();
    }
    write_buffer_mutex = xSemaphoreCreateMutex();
    xTaskCreate(usb_flush_task, "usb flush task", 256, NULL, 4,
                &usb_flush_task_handle);
    xTaskNotify(usb_flush_task_handle, TRANSMIT_COMPLETE, eSetBits);
}

void transmit_complete_callback(void) {
    if (usb_flush_task_handle != NULL) {
        xTaskNotify(usb_flush_task_handle, TRANSMIT_COMPLETE, eSetBits);
    }
}

void usb::write(uint8_t *buf, uint16_t length) {
    uint16_t written = 0;
    while (written != length) {
        xSemaphoreTake(write_buffer_mutex, portMAX_DELAY);
        uint16_t free_space = USB_TX_BUFFER_SIZE - write_buffer_idx;
        uint16_t write_len = MIN(free_space, length - written);
        memcpy(&write_buffer[write_buffer_idx], &buf[written], write_len);
        write_buffer_idx += write_len;
        xSemaphoreGive(write_buffer_mutex);
        xTaskNotify(usb_flush_task_handle, WRITE_REQUEST, eSetBits);
        written += write_len;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void usb_flush_task([[maybe_unused]] void *args) {
    uint32_t notification_value = 0;
    vTaskDelay(pdMS_TO_TICKS(1));
    for (;;) {
        xTaskNotifyWait(0x00, ULONG_MAX, &notification_value, portMAX_DELAY);
        if (notification_value & WRITE_REQUEST) {
            xTaskNotifyWait(0x00, ULONG_MAX, &notification_value,
                            portMAX_DELAY);
            if (notification_value & TRANSMIT_COMPLETE) {
                xSemaphoreTake(write_buffer_mutex, portMAX_DELAY);
                memcpy(transmit_buffer, write_buffer, write_buffer_idx);
                CDC_Transmit_FS((uint8_t *)transmit_buffer, write_buffer_idx);
                memset(write_buffer, 0, USB_TX_BUFFER_SIZE);
                write_buffer_idx = 0;
                xSemaphoreGive(write_buffer_mutex);
            }
        }
    }
}

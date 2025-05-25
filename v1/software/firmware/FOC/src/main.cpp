#include "main.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <stm32g4xx_hal.h>

#include "clock.hpp"
#include "usb.hpp"

void usb_write_task(void *args);

int main(void) {
  HAL_Init();
  clock::init();

  osThreadId_t usb_write_task_handle;
  const osThreadAttr_t usb_write_task_attr = {
      .name = (char *)"USB write",
      .stack_size = 1024,
      .priority = (osPriority_t)osPriorityNormal,
  };

  osKernelInitialize();
  osThreadNew(usb_write_task, NULL, &usb_write_task_attr);
  osKernelStart();

  // should never reach here
  __disable_irq();
  while (1) {
    // should never reach here
  }
}

void usb_write_task(void *args) {
  usb::init();
  uint8_t str[] = "Hello World!\r\n";
  for (;;) {
    usb::write(str, sizeof(str));
    osDelay(1000);
  }
}

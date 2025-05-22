#include "main.hpp"
#include <FreeRTOS.h>
#include <cmsis_os.h>
#include <stm32g4xx_hal.h>

int main(void) {
  HAL_Init();

  osKernelStart();

  // should never reach here
  __disable_irq();
  while (1) {
    // should never reach here
  }
}

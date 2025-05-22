#include "main.hpp"

#include <FreeRTOS.h>
#include <cmsis_os.h>
#include <stm32g4xx_hal.h>

#include "clock.hpp"

int main(void) {
  HAL_Init();
  clock::init();

  osKernelStart();

  // should never reach here
  __disable_irq();
  while (1) {
    // should never reach here
  }
}

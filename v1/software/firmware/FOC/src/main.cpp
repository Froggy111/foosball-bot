#include "main.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <stm32g4xx_hal.h>

#include <cmath>

#include "clock.hpp"
#include "debug.hpp"
#include "inverter.hpp"
#include "usb.hpp"

void usb_write_task(void *args);

int main(void) {
    HAL_Init();
    clock::init();

    [[maybe_unused]] osThreadId_t usb_write_task_handle;
    const osThreadAttr_t usb_write_task_attr = {
        .name = (char *)"USB write",
        .stack_size = 2048,
        .priority = osPriorityNormal,
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

void usb_write_task([[maybe_unused]] void *args) {
    gpio::PinConfig led_config = {GPIOB, gpio::GPIOPin::PIN10,
                                  gpio::GPIOAF::NONE};
    gpio::init(led_config, gpio::GPIOMode::OUTPUT_PP, gpio::GPIOPull::NOPULL,
               gpio::GPIOSpeed::LOW);
    usb::init();
    osDelay(2000);
    inverter::init(20000);
    float theta = 0;
    for (;;) {
        theta += M_PI / 180;
        debug::log("Theta: %f", theta);
        if (theta > M_PI * 2) {
            theta -= M_PI * 2;
        }
        inverter::svpwm_set(theta, 4.0f, 24.0f);
        for (int i = 0; i < 16000; i++) {
            asm volatile("nop");
        }
        // osDelay(1);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
    }
}

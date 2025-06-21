#include "main.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <stm32g4xx_hal.h>

#include <cmath>

#include "clock.hpp"
#include "debug.hpp"
#include "encoder.hpp"
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
    gpio::PinConfig shifter_OE_config = {GPIOB, gpio::GPIOPin::PIN11,
                                         gpio::GPIOAF::NONE};
    gpio::init(shifter_OE_config, gpio::GPIOMode::OUTPUT_PP,
               gpio::GPIOPull::NOPULL, gpio::GPIOSpeed::LOW);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);

    gpio::PinConfig SPI1_SCS_config = {GPIOA, gpio::GPIOPin::PIN4,
                                       gpio::GPIOAF::NONE};
    gpio::PinConfig SPI1_MISO_config = {GPIOB, gpio::GPIOPin::PIN4,
                                        gpio::GPIOAF::NONE};
    gpio::PinConfig SPI1_MOSI_config = {GPIOB, gpio::GPIOPin::PIN5,
                                        gpio::GPIOAF::NONE};
    gpio::PinConfig I2C1_SDA_config = {GPIOB, gpio::GPIOPin::PIN9,
                                       gpio::GPIOAF::NONE};
    gpio::PinConfig I2C1_SCL_config = {GPIOA, gpio::GPIOPin::PIN15,
                                       gpio::GPIOAF::NONE};
    gpio::init(SPI1_SCS_config, gpio::GPIOMode::OUTPUT_PP,
               gpio::GPIOPull::NOPULL, gpio::GPIOSpeed::LOW);
    gpio::init(SPI1_MISO_config, gpio::GPIOMode::OUTPUT_PP,
               gpio::GPIOPull::NOPULL, gpio::GPIOSpeed::LOW);
    gpio::init(SPI1_MOSI_config, gpio::GPIOMode::OUTPUT_PP,
               gpio::GPIOPull::NOPULL, gpio::GPIOSpeed::LOW);
    gpio::init(I2C1_SDA_config, gpio::GPIOMode::OUTPUT_PP,
               gpio::GPIOPull::NOPULL, gpio::GPIOSpeed::LOW);
    gpio::init(I2C1_SCL_config, gpio::GPIOMode::OUTPUT_PP,
               gpio::GPIOPull::NOPULL, gpio::GPIOSpeed::LOW);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    usb::init();
    osDelay(2000);
    inverter::init(20000);
    encoder::init();
    float theta = 0;
    for (;;) {
        theta += M_PI / 45;
        if (theta > M_PI * 2) {
            theta -= M_PI * 2;
        }
        debug::log("Theta: %f", theta);
        uint16_t encoder_count = encoder::get_count();
        debug::log("Encoder count: %u", encoder_count);
        debug::log("Z pulses: %u", encoder::get_z_pulses());
        debug::log("Delta: %d", encoder::get_delta());
        inverter::svpwm_set(theta, 0.0f, 24.0f);
        osDelay(1);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
    }
}

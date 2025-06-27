#include "main.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <stm32g4xx_hal.h>

#include <cmath>

#include "clock.hpp"
#include "debug.hpp"
#include "encoder.hpp"
#include "gpio.hpp"
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
    gpio::PinConfig LED = {GPIOB, gpio::Pin::PIN10, gpio::AF::NONE};
    gpio::init(LED, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);

    gpio::PinConfig SPI1_SCS = {GPIOA, gpio::Pin::PIN4, gpio::AF::NONE};
    gpio::PinConfig SPI1_MISO = {GPIOB, gpio::Pin::PIN4, gpio::AF::NONE};
    gpio::PinConfig SPI1_MOSI = {GPIOB, gpio::Pin::PIN5, gpio::AF::NONE};
    gpio::PinConfig I2C1_SDA = {GPIOB, gpio::Pin::PIN9, gpio::AF::NONE};
    gpio::PinConfig I2C1_SCL = {GPIOA, gpio::Pin::PIN15, gpio::AF::NONE};
    gpio::init(SPI1_SCS, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::init(SPI1_MISO, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::init(SPI1_MOSI, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::init(I2C1_SDA, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::init(I2C1_SCL, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(SPI1_SCS, gpio::HIGH);
    gpio::write(SPI1_MISO, gpio::HIGH);
    gpio::write(SPI1_MOSI, gpio::HIGH);
    gpio::write(I2C1_SDA, gpio::HIGH);
    gpio::write(I2C1_SCL, gpio::HIGH);
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
        gpio::write(LED, !read(LED));
        osDelay(1);
    }
}

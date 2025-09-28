#include "main.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <stm32g4xx_hal.h>

#include <cmath>

#include "adc.hpp"
#include "clock.hpp"
#include "debug.hpp"
#include "encoder.hpp"
#include "gpio.hpp"
#include "inverter.hpp"
#include "spi.hpp"
#include "swo.hpp"

void main_task(void *args);

int main(void) {
    HAL_Init();
    clock::init();

    [[maybe_unused]] osThreadId_t main_task_handle;
    const osThreadAttr_t main_task_attr = {
        .name = (char *)"main task",
        .stack_size = 2048,
        .priority = osPriorityNormal,
    };

    osKernelInitialize();
    osThreadNew(main_task, NULL, &main_task_attr);
    osKernelStart();

    // should never reach here
    __disable_irq();
    while (1) {
        // should never reach here
    }
}

void main_task([[maybe_unused]] void *args) {
    gpio::PinConfig LED = {GPIOD, gpio::Pin::PIN2, gpio::AF::NONE};
    gpio::init(LED, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(LED, 1);

    swo::init();
    osDelay(1000);
    inverter::init(20000);
    inverter::set(0, 0, 0);
    encoder::init();
    encoder::set_count(0);
    float theta = 0;

    adc::init();
    for (;;) {
        // NOTE : debug test
        // debug::log("Hello World!");

        // NOTE : ADC test
        float U_current = adc::read_U_current();
        float V_current = adc::read_V_current();
        float W_current = adc::read_W_current();
        adc::start_VMOT_read();
        float VMOT = adc::read_VMOT();
        debug::log("VMOT: %fV, U: %fA, V: %fA, W: %fA", VMOT, U_current,
                   V_current, W_current);

        // NOTE : encoder test
        int64_t encoder_count = encoder::get_count();
        debug::log("Encoder count: %lld", encoder_count);
        debug::log("Z pulses: %u", encoder::get_z_pulses());
        debug::log("Rollover count: %lld", encoder::get_rollovers());
        debug::log("Delta: %d", encoder::get_delta());

        // NOTE : inverter test
        theta += M_PI / 45;
        if (theta > M_PI * 2) {
            theta -= M_PI * 2;
        }
        debug::log("Theta: %f", theta);
        inverter::svpwm_set(theta, 0.0f, 0.0f, VMOT);
        gpio::invert(LED);
        osDelay(100);
    }
}

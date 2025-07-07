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
#include "usb.hpp"

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
    gpio::PinConfig LED = {GPIOB, gpio::Pin::PIN10, gpio::AF::NONE};
    gpio::init(LED, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);

    usb::init();
    osDelay(2000);
    // inverter::init(20000);
    // encoder::init();
    adc::init();
    // float theta = 0;
    for (;;) {
        adc::start_5V_read();
        float vsense_5V_voltage = adc::read_5V();
        debug::log("5V rail voltage: %fV", vsense_5V_voltage);
        adc::start_12V_read();
        float vsense_12V_voltage = adc::read_12V();
        debug::log("12V rail voltage: %fV", vsense_12V_voltage);
        adc::start_VMOT_read();
        float vsense_VMOT_voltage = adc::read_VMOT();
        debug::log("VMOT rail voltage: %fV", vsense_VMOT_voltage);
        float U_current = adc::read_U_current();
        debug::log("U phase current: %fA", U_current);
        float V_current = adc::read_V_current();
        debug::log("V phase current: %fA", V_current);
        float W_current = adc::read_W_current();
        debug::log("W phase current: %fA", W_current);
        // theta += M_PI / 45;
        // if (theta > M_PI * 2) {
        //     theta -= M_PI * 2;
        // }
        // debug::log("Theta: %f", theta);
        // uint16_t encoder_count = encoder::get_count();
        // debug::log("Encoder count: %u", encoder_count);
        // debug::log("Z pulses: %u", encoder::get_z_pulses());
        // debug::log("Delta: %d", encoder::get_delta());
        // inverter::svpwm_set(theta, 0.0f, 24.0f);
        gpio::write(LED, !read(LED));
        osDelay(1);
    }
}

#include "main.hpp"

#include <FreeRTOS.h>
#include <cmsis_os2.h>
#include <stm32g4xx_hal.h>

#include <cmath>

#include "FOC.hpp"
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

    // swo::init();
    // osDelay(2000);
    // inverter::init(20000);
    // inverter::set(0, 0, 0);
    // encoder::init();
    // encoder::set_count(0);
    float theta = 0;
    //
    // adc::init();

    FOC::Parameters params = {COIL_RESISTANCE, MOTOR_KV, NUM_WINDING_SETS};

    FOC::init(params);
    debug::log("FOC initialisation complete");
    FOC::PIDParams PID_params = {
        CURRENT_D_KP, CURRENT_D_KI, CURRENT_Q_KP, CURRENT_Q_KI, VELOCITY_KP,
        VELOCITY_KI,  VELOCITY_KD,  POSITION_KP,  POSITION_KI,  POSITION_KD};
    FOC::set_PID(PID_params);
    FOC::set_max_current(1.0f);
    FOC::set_max_torque(0.1f);
    FOC::set_max_angular_velocity(10.0f);
    FOC::set_max_angular_acceleration(10.0f);
    FOC::set_angular_jerk(100.0f);
    FOC::enable();

    osDelay(100);

    uint32_t i = 0;
    for (;;) {
        debug::log("I_d_target: %f, I_q_target: %f, I_d: %f, I_q: %f",
                   FOC::get_I_d_target(), FOC::get_I_q_target(), FOC::get_I_d(),
                   FOC::get_I_q());
        debug::log("V_d: %f, V_q: %f", FOC::get_V_d_target(),
                   FOC::get_V_q_target());
        if ((i / 10) % 2 == 0) {
            // FOC::set_torque(0.05f);
            FOC::set_angular_velocity(-5.0f);
        } else {
            // FOC::set_torque(-0.05f);
            FOC::set_angular_velocity(-5.0f);
        }
        i++;
        // NOTE : debug test
        // debug::log("Hello World!");

        // NOTE : ADC test
        // float U_current = adc::read_U_current();
        // float V_current = adc::read_V_current();
        // float W_current = adc::read_W_current();
        // adc::start_VMOT_read();
        // float VMOT = adc::read_VMOT();
        // debug::log("VMOT: %fV, U: %fA, V: %fA, W: %fA", VMOT, U_current,
        //            V_current, W_current);

        // NOTE : encoder test
        // int64_t encoder_count = encoder::get_count();
        // debug::log("Encoder count: %lld", encoder_count);
        // debug::log("Z pulses: %u", encoder::get_z_pulses());
        // debug::log("Rollover count: %lld", encoder::get_rollovers());
        // debug::log("Delta: %d", encoder::get_delta());

        // NOTE : inverter test
        // theta -= M_PI / 45;
        // if (theta > M_PI * 2) {
        //     theta -= M_PI * 2;
        // }
        // debug::log("Theta: %f", theta);
        // inverter::svpwm_set(theta, 0.0f, 1.0f, VMOT);
        gpio::invert(LED);
        osDelay(100);
    }
}

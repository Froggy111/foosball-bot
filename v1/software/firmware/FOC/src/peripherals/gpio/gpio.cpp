#include "gpio.hpp"

#include <stm32g4xx_hal.h>

static gpio::InterruptFn callback_table[16] = {NULL};
static void* args_table[16] = {NULL};

uint8_t pin_to_line_number(gpio::Pin pin);

void gpio::init(const PinConfig& pin_config, Mode mode, Pull pull,
                Speed speed) {
    if (pin_config.port == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (pin_config.port == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (pin_config.port == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    } else if (pin_config.port == GPIOD) {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    } else if (pin_config.port == GPIOE) {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    } else if (pin_config.port == GPIOF) {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    } else if (pin_config.port == GPIOG) {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    GPIO_InitTypeDef init = {(uint32_t)pin_config.pin, (uint32_t)mode,
                             (uint32_t)pull, (uint32_t)speed,
                             (uint32_t)pin_config.alternate_function};
    HAL_GPIO_Init(pin_config.port, &init);
    return;
}

void gpio::attach_interrupt(const PinConfig& pin_config, Mode mode, Pull pull,
                            Speed speed, InterruptFn callback, void* args) {
    // not interrupt mode
    if (mode != Mode::INT_RISING && mode != Mode::INT_FALLING &&
        mode != Mode::INT_RISING_FALLING) {
        return;
    }

    gpio::init(pin_config, mode, pull, speed);
    uint8_t line_number = pin_to_line_number(pin_config.pin);
    if (line_number > 15) {  // invalid
        return;
    }

    callback_table[line_number] = callback;
    args_table[line_number] = args;

    IRQn_Type irq_number;
    if (line_number < 5) {
        irq_number = (IRQn_Type)(EXTI0_IRQn + line_number);
    } else if (line_number <= 9) {
        irq_number = EXTI9_5_IRQn;
    } else {
        irq_number = EXTI15_10_IRQn;
    }

    HAL_NVIC_SetPriority(irq_number, 5, 0);
    HAL_NVIC_EnableIRQ(irq_number);
}

uint8_t pin_to_line_number(gpio::Pin pin) {
    switch (pin) {
        case gpio::Pin::PIN0:
            return 0;
        case gpio::Pin::PIN1:
            return 1;
        case gpio::Pin::PIN2:
            return 2;
        case gpio::Pin::PIN3:
            return 3;
        case gpio::Pin::PIN4:
            return 4;
        case gpio::Pin::PIN5:
            return 5;
        case gpio::Pin::PIN6:
            return 6;
        case gpio::Pin::PIN7:
            return 7;
        case gpio::Pin::PIN8:
            return 8;
        case gpio::Pin::PIN9:
            return 9;
        case gpio::Pin::PIN10:
            return 10;
        case gpio::Pin::PIN11:
            return 11;
        case gpio::Pin::PIN12:
            return 12;
        case gpio::Pin::PIN13:
            return 13;
        case gpio::Pin::PIN14:
            return 14;
        case gpio::Pin::PIN15:
            return 15;
        case gpio::Pin::ALL:  // this is invalid
            return 255;
    }
}

extern "C" {

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    uint8_t line_number = pin_to_line_number((gpio::Pin)pin);
    if (line_number > 15) {
        return;
    }
    if (callback_table[line_number] != NULL) {
        callback_table[line_number](args_table[line_number]);
    }
}
}

#include "gpio.hpp"

#include <stm32g4xx_hal.h>

static gpio::InterruptFn callback_table[16] = {NULL};
static void* args_table[16] = {NULL};

uint8_t pin_to_line_number(gpio::GPIOPin pin);

void gpio::init(PinConfig pin_config, GPIOMode mode, GPIOPull pull,
                GPIOSpeed speed) {
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

void gpio::attach_interrupt(PinConfig pin_config, GPIOMode mode, GPIOPull pull,
                            GPIOSpeed speed, InterruptFn callback, void* args) {
    // not interrupt mode
    if (mode != GPIOMode::INT_RISING && mode != GPIOMode::INT_FALLING &&
        mode != GPIOMode::INT_RISING_FALLING) {
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

uint8_t pin_to_line_number(gpio::GPIOPin pin) {
    switch (pin) {
        case gpio::GPIOPin::PIN0:
            return 0;
        case gpio::GPIOPin::PIN1:
            return 1;
        case gpio::GPIOPin::PIN2:
            return 2;
        case gpio::GPIOPin::PIN3:
            return 3;
        case gpio::GPIOPin::PIN4:
            return 4;
        case gpio::GPIOPin::PIN5:
            return 5;
        case gpio::GPIOPin::PIN6:
            return 6;
        case gpio::GPIOPin::PIN7:
            return 7;
        case gpio::GPIOPin::PIN8:
            return 8;
        case gpio::GPIOPin::PIN9:
            return 9;
        case gpio::GPIOPin::PIN10:
            return 10;
        case gpio::GPIOPin::PIN11:
            return 11;
        case gpio::GPIOPin::PIN12:
            return 12;
        case gpio::GPIOPin::PIN13:
            return 13;
        case gpio::GPIOPin::PIN14:
            return 14;
        case gpio::GPIOPin::PIN15:
            return 15;
        case gpio::GPIOPin::ALL:  // this is invalid
            return 255;
    }
}

extern "C" {

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    uint8_t line_number = pin_to_line_number((gpio::GPIOPin)pin);
    if (line_number > 15) {
        return;
    }
    if (callback_table[line_number] != NULL) {
        callback_table[line_number](args_table[line_number]);
    }
}
}

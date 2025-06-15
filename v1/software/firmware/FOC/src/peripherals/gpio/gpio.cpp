#include "gpio.hpp"

#include <stm32g4xx_hal.h>

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

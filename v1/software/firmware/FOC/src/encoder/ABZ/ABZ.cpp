#include "ABZ.hpp"

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_gpio.h>

#include "config.hpp"

void gpio_init(void);

void encoder::init(void) { gpio_init(); }

void gpio_init(void) {
    if (ENCODER_A_PORT == GPIOA || ENCODER_B_PORT == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    if (ENCODER_A_PORT == GPIOB || ENCODER_B_PORT == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }

    GPIO_InitTypeDef A_gpio_init = {};
    A_gpio_init.Pin = ENCODER_A_PIN;
    A_gpio_init.Mode = GPIO_MODE_AF_PP;  // push pull
    A_gpio_init.Pull = GPIO_PULLUP;      // pull up
    A_gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    A_gpio_init.Alternate = ENCODER_A_AF;
    HAL_GPIO_Init(ENCODER_A_PORT, &A_gpio_init);

    GPIO_InitTypeDef B_gpio_init = {};
    B_gpio_init.Pin = ENCODER_B_PIN;
    B_gpio_init.Mode = GPIO_MODE_AF_PP;  // push pull
    B_gpio_init.Pull = GPIO_PULLUP;      // pull up
    B_gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    B_gpio_init.Alternate = ENCODER_B_AF;
    HAL_GPIO_Init(ENCODER_B_PORT, &B_gpio_init);
}

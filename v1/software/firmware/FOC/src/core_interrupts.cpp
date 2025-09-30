#include <stm32g4xx_hal.h>

#include "FOC.hpp"
#include "config.hpp"
#include "encoder.hpp"
#include "inverter.hpp"

extern "C" {

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim2;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles USB low priority interrupt remap.
 */
void USB_LP_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_FS); }

/**
 * @brief This function handles EXTI line0 interrupt.
 */
void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }

/**
 * @brief This function handles EXTI line1 interrupt.
 */
void EXTI1_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1); }

/**
 * @brief This function handles EXTI line2 interrupt.
 */
void EXTI2_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2); }

/**
 * @brief This function handles EXTI line3 interrupt.
 */
void EXTI3_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3); }

/**
 * @brief This function handles EXTI line4 interrupt.
 */
void EXTI4_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4); }

/**
 * @brief This function handles EXTI line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void) {
    // Note: The HAL handler will check for pins 5, 6, 7, 8, and 9
    // You can call it for each one, but it's more efficient to just check one
    // as the HAL function iterates through them anyway.
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void) {
    // Note: The HAL handler will check for pins 10 through 15.
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

void TIM3_IRQHandler(void) {
#if ENCODER_TYPE == ABZ
    if (ENCODER_TIMER == TIM3) {
        encoder::timer_irq();
    }
#endif
}

void TIM4_IRQHandler(void) {
#if ENCODER_TYPE == ABZ
    if (ENCODER_TIMER == TIM4) {
        encoder::timer_irq();
    }
#endif
}

void TIM1_UP_TIM16_IRQHandler(void) {
    if (INVERTER_TIMER == TIM1) {
        inverter::timer_irq();
    }
}

void TIM8_UP_IRQHandler(void) {
    if (INVERTER_TIMER == TIM8) {
        inverter::timer_irq();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* timer) {
#if ENCODER_TYPE == ABZ
    if (timer->Instance == ENCODER_TIMER) {
        encoder::rollover_irq();
    }
#endif
    if (timer->Instance == INVERTER_TIMER) {
        FOC::handler();
    }
}
}

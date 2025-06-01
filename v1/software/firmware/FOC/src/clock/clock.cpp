#include "clock.hpp"

#include <stm32g4xx_hal.h>

#include "error.hpp"

void clock::init() {
    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};

    // internal regulator boost mode for 170MHz
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) !=
        HAL_OK) {
        error::handler();
    }

    // HSE (using)
    osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
    osc_init.HSEState = RCC_HSE_ON;

    // HSI48
    osc_init.HSI48State = RCC_HSI48_ON;

    // HSI (not using)
    osc_init.HSIState = RCC_HSI_ON;
    osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

    // PLL
    osc_init.PLL.PLLState = RCC_PLL_ON;
    osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    /**
     * Target SYSCLK: 170MHz
     * HSE: 24MHz
     * SYSCLK = (HSE / PLLM_DIV) * PLLN_MUL / PLLR_DIV
     * VCO_CLK = (HSE / PLLM_DIV) * PLLN_MUL
     *
     * Needed clock rates:
     * SYSCLK <= 170MHz, as high as possible
     * PLLQ % 8MHz = 0
     *
     * PLLM_DIV = 6  => 24MHz / 6 = 4MHz (PLL input)
     * PLLN_MUL = 80 => 4MHz * 80 = 320MHz (VCO_CLK)
     * PLLR_DIV = 2  => 320MHz / 2 = 160MHz (SYSCLK)
     * PLLQ_DIV = 4  => 320MHz / 4 = 80MHz (FDCAN)
     * PLLP_DIV = 8  => 320MHz / 8 = 40MHz (ADCs)
     */
    osc_init.PLL.PLLM = RCC_PLLM_DIV6;
    osc_init.PLL.PLLN = 80;
    osc_init.PLL.PLLR = RCC_PLLR_DIV2;
    osc_init.PLL.PLLQ = RCC_PLLQ_DIV4;
    osc_init.PLL.PLLP = RCC_PLLP_DIV8;

    if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
        error::handler();
    }

    // CPU, AHB and APB buses clocks
    clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
    clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_4) != HAL_OK) {
        error::handler();
    }
}

extern "C" {

void SystemClock_Config(void) { clock::init(); }
}

cmake_minimum_required(VERSION 3.22)

project(STM32G4xx_HAL_Driver)
add_library(hal INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(hal INTERFACE
 USE_HAL_DRIVER
 STM32G474xx
 STM32_THREAD_SAFE_STRATEGY=4
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(hal INTERFACE
    Inc
    Inc/Legacy
)

target_sources(hal INTERFACE
    Src/stm32g4xx_hal_tim.c
    Src/stm32g4xx_hal_tim_ex.c
    Src/stm32g4xx_hal_adc.c
    Src/stm32g4xx_hal_adc_ex.c
    Src/stm32g4xx_ll_adc.c
    Src/stm32g4xx_hal.c
    Src/stm32g4xx_hal_rcc.c
    Src/stm32g4xx_hal_rcc_ex.c
    Src/stm32g4xx_hal_flash.c
    Src/stm32g4xx_hal_flash_ex.c
    Src/stm32g4xx_hal_flash_ramfunc.c
    Src/stm32g4xx_hal_gpio.c
    Src/stm32g4xx_hal_exti.c
    Src/stm32g4xx_hal_dma.c
    Src/stm32g4xx_hal_dma_ex.c
    Src/stm32g4xx_hal_pwr.c
    Src/stm32g4xx_hal_pwr_ex.c
    Src/stm32g4xx_hal_cortex.c
    Src/stm32g4xx_hal_fdcan.c
    Src/stm32g4xx_hal_i2c.c
    Src/stm32g4xx_hal_i2c_ex.c
    Src/stm32g4xx_hal_opamp.c
    Src/stm32g4xx_hal_opamp_ex.c
    Src/stm32g4xx_hal_spi.c
    Src/stm32g4xx_hal_spi_ex.c
    Src/stm32g4xx_hal_uart.c
    Src/stm32g4xx_hal_uart_ex.c
    Src/stm32g4xx_hal_pcd.c
    Src/stm32g4xx_hal_pcd_ex.c
    Src/stm32g4xx_ll_usb.c
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()



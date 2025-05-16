add_library(init)

target_compile_definitions(init INTERFACE
 USE_HAL_DRIVER
 STM32G474xx
 STM32_THREAD_SAFE_STRATEGY=4
    $<$<CONFIG:Debug>:DEBUG>
)
target_sources(init
PUBLIC
    include/adc.h
    include/fdcan.h
    include/gpio.h
    include/i2c.h
    include/opamp.h
    include/spi.h
    include/stm32g4xx_hal_conf.h
    include/stm32g4xx_it.h
    include/tim.h
    include/usart.h
    include/usb.h

PRIVATE
    adc.c
    fdcan.c
    gpio.c
    i2c.c
    opamp.c
    spi.c
    stm32g4xx_hal_msp.c
    stm32g4xx_hal_timebase_tim.c
    stm32g4xx_it.c
    tim.c
    usart.c
    usb.c
)

target_include_directories(init
PUBLIC
    include)

target_link_libraries(init
    hal
    freertos
    CMSIS
)

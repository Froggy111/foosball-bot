cmake_minimum_required(VERSION 3.22)

project(CMSIS)
add_library(CMSIS INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(CMSIS INTERFACE
 USE_HAL_DRIVER
 STM32G474xx
 STM32_THREAD_SAFE_STRATEGY=4
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(CMSIS INTERFACE
    Device/ST/STM32G4xx/Include
    Include
)

target_sources(CMSIS INTERFACE
    system_stm32g4xx.c
    startup_stm32g474xx.s
)

# Validate that CMSIS code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()



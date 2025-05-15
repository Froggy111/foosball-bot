cmake_minimum_required(VERSION 3.22)

project(freertos)
add_library(freertos INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(freertos INTERFACE
 USE_HAL_DRIVER
 STM32G474xx
 STM32_THREAD_SAFE_STRATEGY=4
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(freertos INTERFACE
    Source/include
    Source/CMSIS_RTOS_V2
    Source/portable/GCC/ARM_CM4F
)

target_sources(freertos INTERFACE
    Source/croutine.c
    Source/event_groups.c
    Source/list.c
    Source/queue.c
    Source/stream_buffer.c
    Source/tasks.c
    Source/timers.c
    Source/CMSIS_RTOS_V2/cmsis_os2.c
    Source/portable/MemMang/heap_4.c
    Source/portable/GCC/ARM_CM4F/port.c
)

# Validate that freertos code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()



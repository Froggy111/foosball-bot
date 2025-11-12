set(CUSTOM_LIBS
error
clock
usb
swo
msp
debug_
gpio
encoder
inverter
adc
spi
cordic
FOC_core
endstop
uart
)

add_subdirectory(configs)
list(APPEND COMMON_LIBS configs)

message(STATUS "COMMON_LIBS: ${COMMON_LIBS}")
message(STATUS "INCLUDE_DIRS: ${INCLUDE_DIRS}")
message(STATUS "CUSTOM_LIBS: ${CUSTOM_LIBS}")

add_subdirectory(clock)
add_subdirectory(debug)
add_subdirectory(error)
add_subdirectory(peripherals)
add_subdirectory(encoder)
add_subdirectory(inverter)
add_subdirectory(FOC_core)
add_subdirectory(optional)

set(TARGET_NAME "${PROJECT_NAME}-${MCU_SERIES}")
add_executable(${TARGET_NAME})

target_sources(${TARGET_NAME}
PUBLIC
    include/main.hpp
PRIVATE
    main.cpp
    core_interrupts.cpp
    syscalls.cpp
)

target_include_directories(${TARGET_NAME}
PRIVATE
    ${INCLUDE_DIRS}
    include/
)

message(STATUS "COMMON_LIBS: ${COMMON_LIBS}")

target_link_libraries(${TARGET_NAME}
PRIVATE
    ${COMMON_LIBS}
    ${CUSTOM_LIBS}
)

target_print_size(${TARGET_NAME})

stm32_generate_binary_file(${TARGET_NAME})
stm32_generate_hex_file(${TARGET_NAME})
generate_flash_target(
    TARGET ${TARGET_NAME}
    INTERFACE ${OPENOCD_INTERFACE_CFG}
    MCU ${OPENOCD_MCU_CFG}
)

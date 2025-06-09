set(CUSTOM_LIBS
error
clock
usb
msp
debug_
)

get_filename_component(CONFIG_DIR "include/" ABSOLUTE)
list(APPEND INCLUDE_DIRS "${CONFIG_DIR}")

message(STATUS "COMMON_LIBS: ${COMMON_LIBS}")
message(STATUS "INCLUDE_DIRS: ${INCLUDE_DIRS}")
message(STATUS "CUSTOM_LIBS: ${CUSTOM_LIBS}")

add_subdirectory(clock)
add_subdirectory(debug)
add_subdirectory(error)
add_subdirectory(peripherals)

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


stm32_print_size_of_target(${TARGET_NAME})
stm32_generate_binary_file(${TARGET_NAME})
stm32_generate_hex_file(${TARGET_NAME})

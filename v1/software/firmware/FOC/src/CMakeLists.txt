add_subdirectory(error)
add_subdirectory(clock)
add_subdirectory(peripherals)

set(CUSTOM_LIBS
error
clock
)

set(TARGET_NAME "${PROJECT_NAME}-${MCU_SERIES}")
add_executable(${TARGET_NAME})

target_sources(${TARGET_NAME}
PUBLIC
    main.hpp
PRIVATE
    main.cpp
)

target_include_directories(${TARGET_NAME}
PRIVATE
    ${INCLUDE_DIRS}
)


target_link_libraries(${TARGET_NAME}
PRIVATE
    ${COMMON_LIBS}
    ${CUSTOM_LIBS}
)


stm32_print_size_of_target(${TARGET_NAME})
stm32_generate_binary_file(${TARGET_NAME})
stm32_generate_hex_file(${TARGET_NAME})

add_library(configs)

target_sources(configs
PUBLIC
    config.hpp

    configs/debug.hpp

    configs/encoder.hpp
    configs/encoder/ABZ.hpp

    configs/peripherals.hpp
    configs/peripherals/usb.hpp

    configs/inverter.hpp
)

target_include_directories(configs
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(configs
PUBLIC
    ${COMMON_LIBS}
    gpio
)

set_target_properties(configs PROPERTIES LINKER_LANGUAGE CXX)

add_library(configs)

target_sources(configs
PUBLIC
    config.hpp

    configs/debug.hpp

    configs/encoder.hpp
    configs/encoder/ABZ.hpp

    configs/peripherals.hpp
    configs/peripherals/pwm.hpp
    configs/peripherals/usb.hpp
)

target_include_directories(configs
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(configs PROPERTIES LINKER_LANGUAGE CXX)

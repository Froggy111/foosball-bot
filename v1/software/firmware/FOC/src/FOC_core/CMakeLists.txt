add_library(FOC_core)

target_sources(FOC_core
PRIVATE
    FOC.cpp
PUBLIC
    include/FOC.hpp
)

target_include_directories(FOC_core
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(FOC_core
PUBLIC
    ${COMMON_LIBS}
    debug_
    inverter
    adc
    encoder
    endstop
)

add_library(adc)

target_sources(adc
PRIVATE
    adc.cpp
PUBLIC
    include/adc.hpp
)

target_include_directories(adc
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(adc
PUBLIC
    ${COMMON_LIBS}
    debug_
    error
    gpio
)

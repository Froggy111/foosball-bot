add_library(inverter)

target_sources(inverter
PRIVATE
    inverter.cpp
PUBLIC
    include/inverter.hpp
)

target_include_directories(inverter
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(inverter
PUBLIC
    ${COMMON_LIBS}
    gpio
    error
    debug_
)

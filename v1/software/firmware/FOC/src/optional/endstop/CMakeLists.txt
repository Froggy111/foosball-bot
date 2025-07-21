add_library(endstop)

target_sources(endstop
PRIVATE
    endstop.cpp
PUBLIC
    include/endstop.hpp
)

target_include_directories(endstop
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(endstop
PUBLIC
    ${COMMON_LIBS}
    debug_
    gpio
)

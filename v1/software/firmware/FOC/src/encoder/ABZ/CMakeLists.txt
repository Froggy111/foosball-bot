add_library(ABZ)

target_sources(ABZ
PRIVATE
    ABZ.cpp
PUBLIC
    include/ABZ.hpp
)

target_include_directories(ABZ
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(ABZ
PUBLIC
    ${COMMON_LIBS}
    gpio
    error
    debug_
)

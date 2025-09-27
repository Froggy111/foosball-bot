add_library(spi)

target_sources(spi
PRIVATE
    spi.cpp
PUBLIC
    include/spi.hpp
)

target_include_directories(spi
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(spi
PUBLIC
    ${COMMON_LIBS}
    debug_
    error
    gpio
)

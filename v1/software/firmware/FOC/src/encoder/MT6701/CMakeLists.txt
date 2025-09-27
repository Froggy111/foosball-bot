add_library(MT6701)

target_sources(MT6701
PRIVATE
    MT6701.cpp
PUBLIC
    include/MT6701.hpp
)

target_include_directories(MT6701
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(MT6701
PUBLIC
    ${COMMON_LIBS}
    gpio
    spi
    error
    debug_
)

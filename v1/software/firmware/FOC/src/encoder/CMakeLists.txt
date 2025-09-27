add_subdirectory(ABZ)
add_subdirectory(MT6701)

add_library(encoder)

target_sources(encoder
PRIVATE
    encoder.cpp
PUBLIC
    include/encoder.hpp
)

target_include_directories(encoder
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(encoder
PUBLIC
    ${COMMON_LIBS}
    gpio
    error
    debug_
    ABZ
    MT6701
)

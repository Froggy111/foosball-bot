add_library(gpio)

target_sources(gpio
PRIVATE
    gpio.cpp
PUBLIC
    include/gpio.hpp
)

target_include_directories(gpio
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(gpio
PUBLIC
    ${COMMON_LIBS}
)

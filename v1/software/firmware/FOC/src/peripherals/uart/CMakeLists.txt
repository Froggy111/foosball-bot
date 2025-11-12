add_library(uart)

target_sources(uart
PRIVATE
    uart.cpp
PUBLIC
    include/uart.hpp
)

target_include_directories(uart
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(uart
PUBLIC
    ${COMMON_LIBS}
)

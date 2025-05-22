add_library(clock)

target_sources(clock
PRIVATE
    clock.cpp
PUBLIC
    include/clock.hpp
)

target_include_directories(clock
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(clock
PUBLIC
    ${COMMON_LIBS}
    error
)

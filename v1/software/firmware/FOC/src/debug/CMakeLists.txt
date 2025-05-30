add_library(debug)

target_sources(debug
PRIVATE
    debug.cpp
PUBLIC
    debug.hpp
)

target_include_directories(debug
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(debug
PUBLIC
    ${COMMON_LIBS}
    error
)

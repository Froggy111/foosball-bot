add_library(debug_)

target_sources(debug_
PRIVATE
    debug.cpp
PUBLIC
    include/debug.hpp
)

target_include_directories(debug_
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(debug_
PUBLIC
    ${COMMON_LIBS}
    error
)

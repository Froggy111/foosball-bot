add_library(cordic)

target_sources(cordic
PRIVATE
    cordic.cpp
PUBLIC
    include/cordic.hpp
)

target_include_directories(cordic
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(cordic
PUBLIC
    ${COMMON_LIBS}
    debug_
    error
)

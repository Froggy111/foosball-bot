add_library(swo)

target_sources(swo
PRIVATE
    swo.cpp
PUBLIC
    include/swo.hpp
)

target_include_directories(swo
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(swo
PUBLIC
    ${COMMON_LIBS}
    gpio
)

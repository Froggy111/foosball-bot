add_library(error)

target_sources(error
PRIVATE
    error.cpp
PUBLIC
    include/error.hpp
)

target_include_directories(error
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(error
PUBLIC
    ${COMMON_LIBS}
)

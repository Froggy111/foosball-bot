add_library(pwm)

target_sources(pwm
PRIVATE
    pwm.cpp
PUBLIC
    include/pwm.hpp
)

target_include_directories(pwm
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(pwm
PUBLIC
    ${COMMON_LIBS}
    error
    debug_
)

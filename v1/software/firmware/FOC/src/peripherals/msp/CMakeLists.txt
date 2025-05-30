add_library(msp)
target_sources(msp
PRIVATE
    hal_msp_init.cpp
)

target_include_directories(msp
PRIVATE
    ${INCLUDE_DIRS}
)

target_link_libraries(msp
PUBLIC
    ${COMMON_LIBS}
)

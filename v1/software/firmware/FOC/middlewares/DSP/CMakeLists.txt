add_library(DSP)

target_include_directories(DSP
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    Include
)

file(GLOB_RECURSE DSP_SRC
    Source/*.c
)

target_sources(DSP PRIVATE
    ${DSP_SRC}
)

target_link_libraries(DSP
PUBLIC
    ${COMMON_LIBS}
)

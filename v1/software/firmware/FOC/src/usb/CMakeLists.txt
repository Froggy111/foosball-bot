add_library(usb)

target_sources(usb
PRIVATE
    usb.cpp
PUBLIC
    include/usb.hpp
)

target_include_directories(usb
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(usb
PUBLIC
    ${COMMON_LIBS}
)

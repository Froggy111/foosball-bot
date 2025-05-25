add_library(usb)

target_sources(usb
PRIVATE
    usb.cpp
    usb_device.c
    usbd_cdc_if.c
    usbd_desc.c
PUBLIC
    include/usb.hpp
    include/usb_device.h
    include/usbd_cdc_if.h
    include/usbd_desc.h
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
    error
    CDC
)

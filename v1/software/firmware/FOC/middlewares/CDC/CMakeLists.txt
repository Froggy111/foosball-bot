add_library(CDC)

target_sources(CDC
PRIVATE
    usbd_cdc.c
    usbd_conf.c
    usbd_core.c
    usbd_ctlreq.c
    usbd_ioreq.c
PUBLIC
    include/usbd_cdc.h
    include/usbd_conf.h
    include/usbd_core.h
    include/usbd_ctlreq.h
    include/usbd_def.h
    include/usbd_ioreq.h
)

target_include_directories(CDC
PRIVATE
    ${INCLUDE_DIRS}
PUBLIC
    include/
)

target_link_libraries(CDC
PUBLIC
    ${COMMON_LIBS}
)

px4_add_module(
    MODULE modules__cmos_sync
    MAIN cmos_sync
    SRCS
        cmos_sync_main.cpp
    DEPENDS
        px4_work_queue
        uorb
)


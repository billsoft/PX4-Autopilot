px4_add_module(
    MODULE board_status_leds
    MAIN board_status_leds
    SRCS BoardStatusLEDs.cpp
    MODULE_CONFIG
        module.yaml
)


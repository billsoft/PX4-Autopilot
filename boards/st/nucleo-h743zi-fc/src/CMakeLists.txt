add_library(drivers_board
    init.cpp
    spi.cpp
    i2c.cpp
    board_hw_info.c
    stm32_boardinitialize.c
)

target_link_libraries(drivers_board
    PRIVATE
        arch_io_pins
        arch_spi
        nuttx_arch
        nuttx_drivers
        px4_layer
)

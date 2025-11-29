#include <px4_platform_common/i2c.h>
#include <px4_arch/i2c_hw_description.h>

const px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] __attribute__((visibility("default"))) = {
    initI2CBusExternal(1),
};


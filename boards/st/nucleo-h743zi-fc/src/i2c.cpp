#include <px4_arch/i2c_hw_description.h>
#include <px4_platform_common/i2c.h>

#pragma GCC visibility push(default)
const px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
    initI2CBusExternal(1),
    initI2CBusExternal(2),
    initI2CBusExternal(3),
    initI2CBusInternal(4),
};
#pragma GCC visibility pop

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

const px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
    initSPIBus(SPI::Bus::SPI1, {
        initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortD, GPIO::Pin14}),
    }),
    initSPIBus(SPI::Bus::SPI3, {  /* Changed from SPI2 to SPI3 to avoid Ethernet RMII conflict */
        initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortA, GPIO::Pin4}),  /* PA4 = D24 */
    }),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);

/****************************************************************************
 * boards/st/nucleo-h743zi-fc/src/init.cpp
 *
 * Nucleo-H743ZI Flight Controller Board Initialization
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform/gpio.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include "board_config.h"
#include <px4_platform_common/i2c.h>
#include <px4_platform_common/init.h>
#include <syslog.h>
#include <errno.h>
#include <px4_platform/board_dma_alloc.h>
#include <drivers/drv_board_led.h>
#include <stm32_spi.h>
#include <sched.h>
#include <unistd.h>
extern "C" int board_status_leds_main(int argc, char *argv[]);

void stm32_spiinitialize(void);

static int px4_init_thread(int argc, char *argv[])
{
    syslog(LOG_INFO, "[InitThread] Starting px4_platform_init in 10s...\n");
    usleep(10000000);

    syslog(LOG_INFO, "[InitThread] Calling px4_platform_init\n");
    int ret = px4_platform_init();
    syslog(LOG_INFO, "[InitThread] px4_platform_init returned: %d\n", ret);
    int conf = px4_platform_configure();
    syslog(LOG_INFO, "[InitThread] px4_platform_configure returned: %d\n", conf);

    const char *argv_leds[] = {"board_status_leds", "start"};
    syslog(LOG_INFO, "[InitThread] starting board_status_leds\n");
    (void)board_status_leds_main(2, (char **)argv_leds);
    syslog(LOG_INFO, "[InitThread] board_status_leds started\n");
    return ret;
}

/****************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *   Reset external sensors
 ****************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	/* Reset all SPI sensors */
	board_control_spi_sensors_power(false, 0xffff);
	usleep(ms * 1000);
	board_control_spi_sensors_power(true, 0xffff);
}

/****************************************************************************
 * Name: board_control_spi_sensors_power
 *
 * Description:
 *   Control SPI sensors power (via CS pins)
 *   Standard PX4 interface
 *
 * Input Parameters:
 *   enable_power - true to enable power, false to disable
 *   bus_mask     - bitmask of buses to control (0xffff = all buses)
 *
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Board application initialization called after NuttX startup
 ****************************************************************************/
__EXPORT int board_app_initialize(uintptr_t arg)
{
	(void)arg;

	/* ========== 1. Configure LED GPIOs ========== */
	px4_arch_configgpio(GPIO_nLED_GREEN);
	px4_arch_configgpio(GPIO_nLED_YELLOW);
	px4_arch_configgpio(GPIO_nLED_RED);

	/* Initial state: all LEDs off (high level) */
	px4_arch_gpiowrite(GPIO_nLED_GREEN, BOARD_LED_OFF);
	px4_arch_gpiowrite(GPIO_nLED_YELLOW, BOARD_LED_OFF);
	px4_arch_gpiowrite(GPIO_nLED_RED, BOARD_LED_OFF);

	/* ========== 2. Configure SPI CS pins ========== */
	/* IMU1 CS (SPI1) */
	px4_arch_configgpio(GPIO_SPI1_CS_ICM42688P);
	px4_arch_gpiowrite(GPIO_SPI1_CS_ICM42688P, true);  /* High (deselected) */

    /* IMU2 CS (SPI3) */
    px4_arch_configgpio(GPIO_SPI3_CS_ICM42688P);
    px4_arch_gpiowrite(GPIO_SPI3_CS_ICM42688P, true);  /* High (deselected) */

	/* ========== 3. LED startup indication sequence ========== */
	/* Green LED blinks 3 times to indicate board initialization success */
	for (int i = 0; i < 3; i++) {
		px4_arch_gpiowrite(GPIO_nLED_GREEN, BOARD_LED_ON);
		usleep(100000);  /* 100ms */
		px4_arch_gpiowrite(GPIO_nLED_GREEN, BOARD_LED_OFF);
		usleep(100000);  /* 100ms */
	}

    /* ========== 4. Initialization complete (module controls LEDs thereafter) ========== */
    px4_arch_gpiowrite(GPIO_nLED_YELLOW, BOARD_LED_OFF);

	/* ========== 5. Initialize sensor power (all SPI buses) ========== */
	board_control_spi_sensors_power(true, 0xffff);

	/* Ensure px4_i2c_buses is linked in from i2c.cpp */
	(void)px4_i2c_buses;

	/* ========== 6. Initialize PX4 Platform (mounts ROMFS, starts uORB, etc.) ========== */

    /* SPI Init */
    stm32_spiinitialize();

    /* DMA Alloc Init */
    if (board_dma_alloc_init() < 0) {
        syslog(LOG_ERR, "[Init] DMA alloc init failed\n");
    }

    /* Start PX4 init in a separate task to avoid blocking NSH startup */
    int taskid = task_create("px4_init", 100, 4096, px4_init_thread, NULL);
    if (taskid < 0) {
        syslog(LOG_ERR, "[Init] Failed to start px4_init task: %d\n", errno);
    } else {
        syslog(LOG_INFO, "[Init] Started px4_init task (id=%d)\n", taskid);
    }

    return OK;
}

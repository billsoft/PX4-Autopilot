#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

#define GPIO_nLED_GREEN  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)
#define GPIO_nLED_YELLOW (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN1)
#define GPIO_nLED_RED    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN14)

/* LED states (active low, renamed to avoid conflict with drv_board_led.h) */
#define BOARD_LED_ON   0
#define BOARD_LED_OFF  1

/* SPI1 - IMU1 (CubeMX: PA5/PA6/PD7) */
#define PX4_SPI_BUS_SENSORS1  1
#define GPIO_SPI1_CS_ICM42688P  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN14)

/* SPI3 - IMU2 (CubeMX: PC10/PC11/PB2) */
#define PX4_SPI_BUS_SENSORS2  3
#define GPIO_SPI3_CS_ICM42688P  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN15)

#define PX4_I2C_BUS_EXPANSION  1

#define BOARD_NUMBER_I2C_BUSES 2

#define BOARD_CONSOLE_UART  3
#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NAME "Nucleo-H743ZI-FC"
#define BOARD_HAS_NO_BOOTLOADER  1

/* Hardware Version Info */
#define HW_INFO_INIT_PREFIX    "NUCH743FC"
#define BOARD_HAS_VERSIONING   1

#define BOARD_SPI_BUS_MAX_BUS_ITEMS 2

/* CMOS同步输入（行/帧）GPIO：按板文档切换至 PE3/PE4 EXTI */
#define GPIO_CMOS_SYNC_LINE   (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN3)
#define GPIO_CMOS_SYNC_FRAME  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN4)

/* GPIO initialization list for stm32_boardinitialize */
#define PX4_GPIO_INIT_LIST { \
	GPIO_nLED_GREEN, \
	GPIO_nLED_YELLOW, \
	GPIO_nLED_RED, \
	GPIO_SPI1_CS_ICM42688P, \
	GPIO_SPI3_CS_ICM42688P, \
	GPIO_CMOS_SYNC_LINE, \
	GPIO_CMOS_SYNC_FRAME, \
}

/* HRT (High Resolution Timer) configuration */
#define HRT_TIMER               5  /* use timer5 for the HRT (32-bit timer) */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

__BEGIN_DECLS

/* Board initialization */
extern int board_app_initialize(uintptr_t arg);

/* Peripheral reset */
extern void board_peripheral_reset(int ms);

extern void board_control_spi_sensors_power(bool enable_power, int bus_mask);

#include <px4_platform_common/board_common.h>
__END_DECLS

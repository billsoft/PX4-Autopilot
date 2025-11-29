/****************************************************************************
 * boards/st/nucleo-h743zi-fc/src/stm32_boardinitialize.c
 *
 * Nucleo-H743ZI Flight Controller Early Board Initialization
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "board_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <chip.h>
#include <stm32_uart.h>
#include <arch/board/board.h>
#include "arm_internal.h"

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/gpio.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have
 *   been initialized.
 *
 *   This is a minimal implementation for Nucleo-H743ZI-FC (no PWM outputs,
 *   no USB device mode, minimal power management).
 *
 ****************************************************************************/

__EXPORT void
stm32_boardinitialize(void)
{
	/* Configure LEDs (board-specific GPIOs) */
	board_autoled_initialize();

	/* Configure basic GPIO pins */
	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
	px4_gpio_init(gpio, arraySize(gpio));

	/* Note: USB initialization is skipped for minimal flight controller */
	/* Note: No power management rails on Nucleo board */
}

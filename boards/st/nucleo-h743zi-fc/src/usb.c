#include <nuttx/config.h>
#include <sys/types.h>
#include <stdbool.h>
#include <debug.h>
#include <px4_platform_common/px4_config.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <stm32.h>
#include "board_config.h"

/**
 * @brief Initialize USB OTG FS hardware
 *
 * Called by board_app_initialize() during system startup.
 * Configures GPIO pins for USB D+/D- and VBUS detection.
 */
__EXPORT void stm32_usbinitialize(void)
{
#ifdef GPIO_OTGFS_VBUS
	stm32_configgpio(GPIO_OTGFS_VBUS);
	uinfo("USB OTG FS initialized (VBUS detection enabled)\n");
#else
	uinfo("USB OTG FS initialized (no VBUS detection)\n");
#endif
}

/**
 * @brief Handle USB suspend/resume events
 *
 * Called by NuttX USB device stack when:
 * - USB suspend: Windows enters power save, or cable disconnected
 * - USB resume: Windows wakes up, or cable reconnected
 *
 * Strategy: On resume, re-initialize VBUS GPIO to ensure clean state.
 * The cdcacm_autostart module (if enabled) will handle reconnection.
 */
__EXPORT void stm32_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
	if (resume) {
		uinfo("USB resume detected, reinitializing VBUS GPIO\n");

		/* Re-configure VBUS GPIO to ensure stable detection */
#ifdef GPIO_OTGFS_VBUS
		stm32_configgpio(GPIO_OTGFS_VBUS);
#endif

		/*
		 * Note: Actual reconnection is handled by cdcacm_autostart module
		 * or startup script (sercon). We only ensure hardware is ready.
		 */
	} else {
		uinfo("USB suspend detected (host sleep or cable disconnect)\n");
		/*
		 * On suspend, NuttX USB stack will close endpoints.
		 * We don't force disconnect here - let the stack handle it.
		 * Windows COM port will disappear until resume.
		 */
	}
}

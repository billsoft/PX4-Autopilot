/****************************************************************************
 * boards/st/nucleo-h743zi-fc/src/board_hw_info.c
 *
 * Nucleo-H743ZI-FC Hardware Version Info (Stub Implementation)
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <stdint.h>
#include "board_config.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
static int hw_version = 0;
static int hw_revision = 0;
static char hw_type_name[] = HW_INFO_INIT_PREFIX "00";

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_get_hw_type_name
 *
 * Description:
 *   Returns board hardware type name
 *
 ****************************************************************************/
__EXPORT const char *board_get_hw_type_name(void)
{
	return (const char *)hw_type_name;
}

/****************************************************************************
 * Name: board_get_hw_version
 *
 * Description:
 *   Returns board hardware version
 *
 ****************************************************************************/
__EXPORT int board_get_hw_version(void)
{
	return hw_version;
}

/****************************************************************************
 * Name: board_get_hw_revision
 *
 * Description:
 *   Returns board hardware revision
 *
 ****************************************************************************/
__EXPORT int board_get_hw_revision(void)
{
	return hw_revision;
}

/****************************************************************************
 * Name: board_determine_hw_info
 *
 * Description:
 *   Stub implementation - no hardware version detection needed
 *
 ****************************************************************************/
__EXPORT int board_determine_hw_info(void)
{
	/* For Nucleo-H743ZI-FC, we don't have hardware version detection */
	hw_version = 0;
	hw_revision = 0;
	return 0;
}

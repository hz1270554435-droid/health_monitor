/*******************************************************************************
* File Name : app_cm55_display_bringup.h
*
* Description : Minimal CM55 display bring-up API.
*******************************************************************************/

#ifndef __APP_CM55_DISPLAY_BRINGUP_H__
#define __APP_CM55_DISPLAY_BRINGUP_H__

#include "cy_result.h"

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * Initialize the CM55 display bring-up task.
 *
 * Creates a FreeRTOS task that initializes GFXSS, I2C, panel, and
 * renders a test pattern. Follows the official LVGL demo sequence.
 *
 * @return CY_RSLT_SUCCESS on success
 */
cy_rslt_t app_cm55_display_bringup_init(void);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_CM55_DISPLAY_BRINGUP_H__ */

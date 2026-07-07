/**
 * @file lv_port_disp.h
 * LVGL display port for PSoC Edge E84 — Waveshare 4.3" DSI LCD (832×480 RGB565)
 *
 * Bridges LVGL flush callback to the existing GFXSS display controller via
 * double-buffered framebuffers with software 180° rotation in the flush path.
 */

#ifndef LV_PORT_DISP_H
#define LV_PORT_DISP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include "cy_graphics.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief Initialize LVGL display port.
 *
 * Must be called after GFXSS hardware init (Cy_GFXSS_Init, DC interrupt, panel).
 * Registers two full-screen draw buffers and the flush callback.
 *
 * @param dc_task_handle  FreeRTOS task handle for DC interrupt notification.
 * @param gfxss           Pointer to the GFXSS peripheral (e.g. GFXSS).
 * @param gfx_ctx         Pointer to the GFXSS runtime context.
 */
void lv_port_disp_init(TaskHandle_t dc_task_handle,
                       GFXSS_Type *gfxss,
                       cy_stc_gfx_context_t *gfx_ctx);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_PORT_DISP_H*/

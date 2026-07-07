/**
 * @file lv_port_disp.c
 * LVGL display port for PSoC Edge E84 — Waveshare 4.3" DSI LCD (832×480 RGB565)
 *
 * Bridges LVGL's flush callback to the GFXSS display controller using the
 * existing double-buffered framebuffers.  A software 180° rotation is applied
 * in the flush path so the physical panel orientation matches the UI.
 */

#include "lv_port_disp.h"
#include "app_cm55_display_rotation.h"
#include "lvgl.h"
#include "cy_graphics.h"
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------ */
/* Constants                                                          */
/* ------------------------------------------------------------------ */
#define DISP_HOR_RES        832U
#define DISP_VER_RES        480U
#define DISP_ACTUAL_HOR_RES 800U   /* visible area (832 for DSI alignment) */
#define BYTE_PER_PIXEL      2U     /* RGB565 */

/* From app_cm55_display_buffers.c — allocated in .cy_gpu_buf (gfx_mem) */
extern uint16_t frame_buffer1[DISP_HOR_RES * DISP_VER_RES];
extern uint16_t frame_buffer2[DISP_HOR_RES * DISP_VER_RES];

/* ------------------------------------------------------------------ */
/* State                                                              */
/* ------------------------------------------------------------------ */
static TaskHandle_t s_dc_task_handle;
static GFXSS_Type *s_gfxss;
static cy_stc_gfx_context_t *s_gfx_ctx;

/* ------------------------------------------------------------------ */
/* LVGL flush callback                                                */
/* ------------------------------------------------------------------ */
static void disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    (void)area;  /* full-frame mode — area covers the whole screen */

    uint16_t *fb = (uint16_t *)px_map;

    /* 180° full-frame rotation for physical panel orientation. */
    app_cm55_display_rotate_180_rgb565_inplace(fb, DISP_HOR_RES * DISP_VER_RES);

    /* Point the display controller at this buffer */
    Cy_GFXSS_Set_FrameBuffer(s_gfxss, (uint32_t *)fb, s_gfx_ctx);

    /* Wait for DC to finish scanning out (frame-complete interrupt) */
    if (s_dc_task_handle != NULL)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000U));
    }

    /* Tell LVGL we're done */
    lv_display_flush_ready(disp);
}

/* ------------------------------------------------------------------ */
/* Tick source — use FreeRTOS tick (1 kHz)                            */
/* ------------------------------------------------------------------ */
static uint32_t tick_get_cb(void)
{
    return xTaskGetTickCount();
}

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */
void lv_port_disp_init(TaskHandle_t dc_task_handle,
                       GFXSS_Type *gfxss,
                       cy_stc_gfx_context_t *gfx_ctx)
{
    s_dc_task_handle = dc_task_handle;
    s_gfxss = gfxss;
    s_gfx_ctx = gfx_ctx;

    /* Register LVGL tick source (1 ms resolution from FreeRTOS) */
    lv_tick_set_cb(tick_get_cb);

    /* Create display */
    lv_display_t *disp = lv_display_create(DISP_HOR_RES, DISP_VER_RES);
    lv_display_set_flush_cb(disp, disp_flush);

    /* Use FULL render mode: LVGL renders the entire screen into one buffer,
     * calls flush, then uses the other buffer for the next frame.
     * This prevents LVGL from reading back the flipped buffer, which would
     * cause flickering with the in-place 180° flip. */
    lv_display_set_buffers(disp,
                           frame_buffer1, frame_buffer2,
                           DISP_HOR_RES * DISP_VER_RES * BYTE_PER_PIXEL,
                           LV_DISPLAY_RENDER_MODE_FULL);

    /* NOTE: 180° rotation is handled manually in disp_flush() over the full
     * stride-aligned framebuffer.  We do NOT use lv_display_set_rotation()
     * because it requires a scratch buffer (~800 KB) that doesn't fit in
     * gfx_mem. */
}

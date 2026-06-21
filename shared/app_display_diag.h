/*******************************************************************************
* File Name : app_display_diag.h
*
* Description : Temporary CM33/CM55 display-only diagnostic markers.
*******************************************************************************/

#ifndef APP_DISPLAY_DIAG_H
#define APP_DISPLAY_DIAG_H

#include <stdint.h>

#include "cy_pdl.h"
#if defined(COMPONENT_CM55)
#include "cymem_CM55_0.h"
#else
#include "cymem_CM33_0.h"
#endif

#if defined(__cplusplus)
extern "C" {
#endif

#define APP_DISPLAY_DIAG_MAGIC             (0x44535044u) /* "DSPD" */
#define APP_DISPLAY_DIAG_VERSION           (1u)
#define APP_DISPLAY_DIAG_OFFSET_BYTES      (0x0003F000u)

#if defined(COMPONENT_CM55)
#define APP_DISPLAY_DIAG_BASE_ADDR         (CYMEM_CM55_0_m33_m55_shared_START)
#define APP_DISPLAY_DIAG_REGION_SIZE_BYTES (CYMEM_CM55_0_m33_m55_shared_SIZE)
#else
#define APP_DISPLAY_DIAG_BASE_ADDR         (CYMEM_CM33_0_m33_m55_shared_START)
#define APP_DISPLAY_DIAG_REGION_SIZE_BYTES (CYMEM_CM33_0_m33_m55_shared_SIZE)
#endif

#define APP_DISPLAY_DIAG_ADDR              (APP_DISPLAY_DIAG_BASE_ADDR + \
                                            APP_DISPLAY_DIAG_OFFSET_BYTES)

typedef enum
{
    APP_DISPLAY_DIAG_STAGE_EMPTY = 0,
    APP_DISPLAY_DIAG_STAGE_CM33_LAUNCHER_START,
    APP_DISPLAY_DIAG_STAGE_CM33_CM55_RELEASED,
    APP_DISPLAY_DIAG_STAGE_CM55_MAIN_ENTER,
    APP_DISPLAY_DIAG_STAGE_CM55_CYBSP_INIT_OK,
    APP_DISPLAY_DIAG_STAGE_CM55_CYBSP_INIT_FAIL,
    APP_DISPLAY_DIAG_STAGE_CM55_RETARGET_OK,
    APP_DISPLAY_DIAG_STAGE_CM55_RETARGET_FAIL,
    APP_DISPLAY_DIAG_STAGE_CM55_RETARGET_SKIP,
    APP_DISPLAY_DIAG_STAGE_DISPLAY_TASK_CREATE_OK,
    APP_DISPLAY_DIAG_STAGE_DISPLAY_TASK_CREATE_FAIL,
    APP_DISPLAY_DIAG_STAGE_DISPLAY_TASK_START,
    APP_DISPLAY_DIAG_STAGE_FRAMEBUFFER_DRAWN,
    APP_DISPLAY_DIAG_STAGE_GFXSS_INIT_OK,
    APP_DISPLAY_DIAG_STAGE_GFXSS_INIT_FAIL,
    APP_DISPLAY_DIAG_STAGE_IRQ_INIT_OK,
    APP_DISPLAY_DIAG_STAGE_IRQ_INIT_FAIL,
    APP_DISPLAY_DIAG_STAGE_I2C_INIT_OK,
    APP_DISPLAY_DIAG_STAGE_I2C_INIT_FAIL,
    APP_DISPLAY_DIAG_STAGE_PANEL_INIT_OK,
    APP_DISPLAY_DIAG_STAGE_PANEL_INIT_FAIL,
    APP_DISPLAY_DIAG_STAGE_PANEL_ID_REG_WRITE_OK,
    APP_DISPLAY_DIAG_STAGE_PANEL_ID_REG_WRITE_FAIL,
    APP_DISPLAY_DIAG_STAGE_PANEL_ID_READ_OK,
    APP_DISPLAY_DIAG_STAGE_PANEL_ID_READ_FAIL,
    APP_DISPLAY_DIAG_STAGE_PANEL_ID_SUPPORTED,
    APP_DISPLAY_DIAG_STAGE_PANEL_ID_UNSUPPORTED,
    APP_DISPLAY_DIAG_STAGE_PANEL_OFFICIAL_CMD_OK,
    APP_DISPLAY_DIAG_STAGE_PANEL_OFFICIAL_CMD_FAIL,
    APP_DISPLAY_DIAG_STAGE_PANEL_DIRECT_CMD_OK,
    APP_DISPLAY_DIAG_STAGE_PANEL_DIRECT_CMD_FAIL,
    APP_DISPLAY_DIAG_STAGE_PANEL_DIRECT_INIT_OK,
    APP_DISPLAY_DIAG_STAGE_PANEL_DIRECT_INIT_FAIL,
    APP_DISPLAY_DIAG_STAGE_PANEL_I2C_TRACE,
    APP_DISPLAY_DIAG_STAGE_PANEL_I2C_LOWLEVEL_OK,
    APP_DISPLAY_DIAG_STAGE_PANEL_I2C_LOWLEVEL_FAIL,
    APP_DISPLAY_DIAG_STAGE_VGLITE_INIT_OK,
    APP_DISPLAY_DIAG_STAGE_VGLITE_INIT_FAIL,
    APP_DISPLAY_DIAG_STAGE_FRAMEBUFFER_SET_OK,
    APP_DISPLAY_DIAG_STAGE_FRAMEBUFFER_SET_TIMEOUT,
    APP_DISPLAY_DIAG_STAGE_DISPLAY_READY
} app_display_diag_stage_t;

typedef struct
{
    volatile uint32_t magic;
    volatile uint32_t version;
    volatile uint32_t stage;
    volatile uint32_t result;
    volatile uint32_t line;
    volatile uint32_t heartbeat;
    volatile uint32_t flags;
    volatile uint32_t detail0;
    volatile uint32_t detail1;
    volatile uint32_t detail2;
    volatile uint32_t detail3;
    volatile uint32_t reserved[5];
} app_display_diag_region_t;

#define APP_DISPLAY_DIAG_REGION \
    ((volatile app_display_diag_region_t *)APP_DISPLAY_DIAG_ADDR)

typedef char app_display_diag_region_size_check[
    ((APP_DISPLAY_DIAG_OFFSET_BYTES + sizeof(app_display_diag_region_t)) <=
     APP_DISPLAY_DIAG_REGION_SIZE_BYTES) ? 1 : -1];

static inline void app_display_diag_reset(void)
{
    volatile app_display_diag_region_t *diag = APP_DISPLAY_DIAG_REGION;

    diag->magic = APP_DISPLAY_DIAG_MAGIC;
    diag->version = APP_DISPLAY_DIAG_VERSION;
    diag->stage = APP_DISPLAY_DIAG_STAGE_EMPTY;
    diag->result = 0u;
    diag->line = 0u;
    diag->heartbeat = 0u;
    diag->flags = 0u;
    diag->detail0 = 0u;
    diag->detail1 = 0u;
    diag->detail2 = 0u;
    diag->detail3 = 0u;
}

static inline void app_display_diag_mark(uint32_t stage,
                                         uint32_t result,
                                         uint32_t line,
                                         uint32_t flags)
{
    volatile app_display_diag_region_t *diag = APP_DISPLAY_DIAG_REGION;

    if ((APP_DISPLAY_DIAG_MAGIC != diag->magic) ||
        (APP_DISPLAY_DIAG_VERSION != diag->version))
    {
        app_display_diag_reset();
    }

    diag->result = result;
    diag->line = line;
    diag->flags = flags;
    diag->stage = stage;
    diag->heartbeat++;
}

static inline void app_display_diag_set_detail(uint32_t detail0,
                                               uint32_t detail1)
{
    volatile app_display_diag_region_t *diag = APP_DISPLAY_DIAG_REGION;

    diag->detail0 = detail0;
    diag->detail1 = detail1;
}

static inline void app_display_diag_set_extra_detail(uint32_t detail2,
                                                     uint32_t detail3)
{
    volatile app_display_diag_region_t *diag = APP_DISPLAY_DIAG_REGION;

    diag->detail2 = detail2;
    diag->detail3 = detail3;
}

static inline const char *app_display_diag_stage_name(uint32_t stage)
{
    switch (stage)
    {
        case APP_DISPLAY_DIAG_STAGE_EMPTY: return "EMPTY";
        case APP_DISPLAY_DIAG_STAGE_CM33_LAUNCHER_START: return "CM33_LAUNCHER_START";
        case APP_DISPLAY_DIAG_STAGE_CM33_CM55_RELEASED: return "CM33_CM55_RELEASED";
        case APP_DISPLAY_DIAG_STAGE_CM55_MAIN_ENTER: return "CM55_MAIN_ENTER";
        case APP_DISPLAY_DIAG_STAGE_CM55_CYBSP_INIT_OK: return "CM55_CYBSP_INIT_OK";
        case APP_DISPLAY_DIAG_STAGE_CM55_CYBSP_INIT_FAIL: return "CM55_CYBSP_INIT_FAIL";
        case APP_DISPLAY_DIAG_STAGE_CM55_RETARGET_OK: return "CM55_RETARGET_OK";
        case APP_DISPLAY_DIAG_STAGE_CM55_RETARGET_FAIL: return "CM55_RETARGET_FAIL";
        case APP_DISPLAY_DIAG_STAGE_CM55_RETARGET_SKIP: return "CM55_RETARGET_SKIP";
        case APP_DISPLAY_DIAG_STAGE_DISPLAY_TASK_CREATE_OK: return "DISPLAY_TASK_CREATE_OK";
        case APP_DISPLAY_DIAG_STAGE_DISPLAY_TASK_CREATE_FAIL: return "DISPLAY_TASK_CREATE_FAIL";
        case APP_DISPLAY_DIAG_STAGE_DISPLAY_TASK_START: return "DISPLAY_TASK_START";
        case APP_DISPLAY_DIAG_STAGE_FRAMEBUFFER_DRAWN: return "FRAMEBUFFER_DRAWN";
        case APP_DISPLAY_DIAG_STAGE_GFXSS_INIT_OK: return "GFXSS_INIT_OK";
        case APP_DISPLAY_DIAG_STAGE_GFXSS_INIT_FAIL: return "GFXSS_INIT_FAIL";
        case APP_DISPLAY_DIAG_STAGE_IRQ_INIT_OK: return "IRQ_INIT_OK";
        case APP_DISPLAY_DIAG_STAGE_IRQ_INIT_FAIL: return "IRQ_INIT_FAIL";
        case APP_DISPLAY_DIAG_STAGE_I2C_INIT_OK: return "I2C_INIT_OK";
        case APP_DISPLAY_DIAG_STAGE_I2C_INIT_FAIL: return "I2C_INIT_FAIL";
        case APP_DISPLAY_DIAG_STAGE_PANEL_INIT_OK: return "PANEL_INIT_OK";
        case APP_DISPLAY_DIAG_STAGE_PANEL_INIT_FAIL: return "PANEL_INIT_FAIL";
        case APP_DISPLAY_DIAG_STAGE_PANEL_ID_REG_WRITE_OK: return "PANEL_ID_REG_WRITE_OK";
        case APP_DISPLAY_DIAG_STAGE_PANEL_ID_REG_WRITE_FAIL: return "PANEL_ID_REG_WRITE_FAIL";
        case APP_DISPLAY_DIAG_STAGE_PANEL_ID_READ_OK: return "PANEL_ID_READ_OK";
        case APP_DISPLAY_DIAG_STAGE_PANEL_ID_READ_FAIL: return "PANEL_ID_READ_FAIL";
        case APP_DISPLAY_DIAG_STAGE_PANEL_ID_SUPPORTED: return "PANEL_ID_SUPPORTED";
        case APP_DISPLAY_DIAG_STAGE_PANEL_ID_UNSUPPORTED: return "PANEL_ID_UNSUPPORTED";
        case APP_DISPLAY_DIAG_STAGE_PANEL_OFFICIAL_CMD_OK: return "PANEL_OFFICIAL_CMD_OK";
        case APP_DISPLAY_DIAG_STAGE_PANEL_OFFICIAL_CMD_FAIL: return "PANEL_OFFICIAL_CMD_FAIL";
        case APP_DISPLAY_DIAG_STAGE_PANEL_DIRECT_CMD_OK: return "PANEL_DIRECT_CMD_OK";
        case APP_DISPLAY_DIAG_STAGE_PANEL_DIRECT_CMD_FAIL: return "PANEL_DIRECT_CMD_FAIL";
        case APP_DISPLAY_DIAG_STAGE_PANEL_DIRECT_INIT_OK: return "PANEL_DIRECT_INIT_OK";
        case APP_DISPLAY_DIAG_STAGE_PANEL_DIRECT_INIT_FAIL: return "PANEL_DIRECT_INIT_FAIL";
        case APP_DISPLAY_DIAG_STAGE_PANEL_I2C_TRACE: return "PANEL_I2C_TRACE";
        case APP_DISPLAY_DIAG_STAGE_PANEL_I2C_LOWLEVEL_OK: return "PANEL_I2C_LOWLEVEL_OK";
        case APP_DISPLAY_DIAG_STAGE_PANEL_I2C_LOWLEVEL_FAIL: return "PANEL_I2C_LOWLEVEL_FAIL";
        case APP_DISPLAY_DIAG_STAGE_VGLITE_INIT_OK: return "VGLITE_INIT_OK";
        case APP_DISPLAY_DIAG_STAGE_VGLITE_INIT_FAIL: return "VGLITE_INIT_FAIL";
        case APP_DISPLAY_DIAG_STAGE_FRAMEBUFFER_SET_OK: return "FRAMEBUFFER_SET_OK";
        case APP_DISPLAY_DIAG_STAGE_FRAMEBUFFER_SET_TIMEOUT: return "FRAMEBUFFER_SET_TIMEOUT";
        case APP_DISPLAY_DIAG_STAGE_DISPLAY_READY: return "DISPLAY_READY";
        default: return "UNKNOWN";
    }
}

#if defined(__cplusplus)
}
#endif

#endif /* APP_DISPLAY_DIAG_H */

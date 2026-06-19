/*******************************************************************************
* File Name : app_display_backend_lcd.c
*
* Description : LCD display backend for E84 health monitor.
*
* Renders an LCD smoke frame to a Waveshare 4.3-inch DSI LCD
* (832x480, RGB565) via the GFXSS hardware on the KIT_PSE84_EVAL_EPC2 board.
*
* Uses the official Infineon display-dsi-waveshare-4-3-lcd driver library.
*
* STATUS: SKELETON - requires:
* 1. Waveshare 4.3-inch DSI LCD connected to R-Pi MIPI-DSI connector
* 2. display-dsi-waveshare-4-3-lcd library added to project deps
* 3. Device-configurator: SCB I2C + GFXSS (832x480, RGB565)
* 4. Display pin aliases exported to cycfg_pins.h
*
* These types are local to CM33_NS presentation code. They are not a shared
* memory ABI and are not part of the BLE wire protocol.
*******************************************************************************/

#include "app_display.h"

#if (APP_DISPLAY_ENABLE && APP_DISPLAY_LCD_ENABLE)

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* GFXSS PDL headers - available in mtb-dsl-pse8xxgp */
#include "cy_graphics.h"
#include "cy_mipidsi.h"

/* BSP headers - provides CYBSP_I2C_CONTROLLER_HW, CY_MMIO_GFXSS_* */
#include "cybsp.h"
#include "cy_sysclk.h"
#include "cycfg_peripherals.h"
#include "cycfg_pins.h"

/* Display driver selection */
#if (APP_DISPLAY_LCD_DRIVER_SELECT == APP_DISPLAY_LCD_DRIVER_EK79007AD3)
#include "mtb_display_ek79007ad3.h"
#endif

/* Always include Waveshare driver for I2C panel control */
#include "mtb_disp_dsi_waveshare_4p3.h"
#include "cy_scb_i2c.h"

/*******************************************************************************
* Configuration - Waveshare 4.3-inch DSI LCD (832x480, RGB565)
*******************************************************************************/

/* Display resolution - Waveshare 4.3" (832x480) as confirmed by official demo */
#define APP_DISPLAY_LCD_WIDTH              (MTB_DISP_WAVESHARE_4P3_HOR_RES)  /* 832 */
#define APP_DISPLAY_LCD_HEIGHT             (MTB_DISP_WAVESHARE_4P3_VER_RES)  /* 480 */
#define APP_DISPLAY_LCD_PIXEL_BYTES        (2u)  /* RGB565 = 2 bytes per pixel */

/* Stride must be 128-byte aligned for GFXSS */
#define APP_DISPLAY_LCD_STRIDE             (APP_DISPLAY_LCD_WIDTH * APP_DISPLAY_LCD_PIXEL_BYTES)

/* Framebuffer - allocated in gfx_mem by linker */
/* Size: 832 * 480 * 2 = 798,720 bytes (~780 KB, fits in 1.77 MB gfx_mem) */
static uint16_t lcd_framebuffer[APP_DISPLAY_LCD_WIDTH * APP_DISPLAY_LCD_HEIGHT]
    __attribute__((section(".cy_gpu_buf")));

/* GFXSS context - runtime state */
static cy_stc_gfx_context_t gfx_context;

/* I2C context for panel control */
static cy_stc_scb_i2c_context_t i2c_context;

/* Initialization state */
static bool lcd_initialized = false;
static const char *lcd_last_fail_reason = "none";

/*******************************************************************************
* Forward Declarations
*******************************************************************************/

static cy_en_gfx_status_t app_display_backend_lcd_init_gfxss(void);
static cy_en_gfx_status_t app_display_backend_lcd_init_panel(void);
static cy_en_gfx_status_t app_display_backend_lcd_initial_transfer(void);
static cy_en_scb_i2c_status_t app_display_backend_lcd_init_i2c(void);
#if (APP_DISPLAY_LCD_DRIVER_SELECT != APP_DISPLAY_LCD_DRIVER_EK79007AD3)
static void app_display_backend_lcd_reset_panel(void);
#endif
static void app_display_backend_lcd_step_before(uint32_t step,
                                                const char *call);
static void app_display_backend_lcd_step_after(uint32_t step,
                                               const char *call,
                                               long result);
static cy_rslt_t app_display_backend_lcd_fail(const char *reason);
static void app_display_backend_lcd_render_frame(
    const e84_display_snapshot_t *snapshot);
static void app_display_backend_lcd_clear(uint16_t color);
static void app_display_backend_lcd_draw_text(
    uint16_t x, uint16_t y,
    const char *text,
    uint16_t fg_color,
    uint16_t bg_color);
static void app_display_backend_lcd_draw_rect(
    uint16_t x, uint16_t y,
    uint16_t w, uint16_t h,
    uint16_t color);

/*******************************************************************************
* Public API
*******************************************************************************/

cy_rslt_t app_display_backend_lcd_hw_init(void)
{
    cy_en_gfx_status_t gfx_status;
    cy_en_scb_i2c_status_t i2c_status;

    lcd_last_fail_reason = "none";

    printf("[DISPLAY_LCD] init begin\r\n");
    fflush(stdout);
    printf("[DISPLAY_LCD_INFO] w=%lu h=%lu\r\n",
           (unsigned long)APP_DISPLAY_LCD_WIDTH,
           (unsigned long)APP_DISPLAY_LCD_HEIGHT);
    fflush(stdout);

    app_display_backend_lcd_step_before(1u, "static_config_check");
    if (0u != (APP_DISPLAY_LCD_STRIDE % 128u))
    {
        app_display_backend_lcd_step_after(1u,
                                           "static_config_check",
                                           -1);
        return app_display_backend_lcd_fail("framebuffer_stride_not_128_aligned");
    }
    app_display_backend_lcd_step_after(1u, "static_config_check", 0);

    /* Step 2: GFXSS peripheral group clocks (must be before GFXSS init) */
    app_display_backend_lcd_step_before(2u, "GFXSS_PeriGroupSlaveInit");
    Cy_SysClk_PeriGroupSlaveInit(CY_MMIO_GFXSS_GPU_PERI_NR,
                                  CY_MMIO_GFXSS_GPU_GROUP_NR,
                                  CY_MMIO_GFXSS_GPU_SLAVE_NR,
                                  CY_MMIO_GFXSS_GPU_CLK_HF_NR);
    Cy_SysClk_PeriGroupSlaveInit(CY_MMIO_GFXSS_DC_PERI_NR,
                                  CY_MMIO_GFXSS_DC_GROUP_NR,
                                  CY_MMIO_GFXSS_DC_SLAVE_NR,
                                  CY_MMIO_GFXSS_DC_CLK_HF_NR);
    Cy_SysClk_PeriGroupSlaveInit(CY_MMIO_GFXSS_MIPIDSI_PERI_NR,
                                  CY_MMIO_GFXSS_MIPIDSI_GROUP_NR,
                                  CY_MMIO_GFXSS_MIPIDSI_SLAVE_NR,
                                  CY_MMIO_GFXSS_MIPIDSI_CLK_HF_NR);
    app_display_backend_lcd_step_after(2u, "GFXSS_PeriGroupSlaveInit", 0);

    /* Step 3: GFXSS init (before I2C, matching official demo order) */
#if (APP_DISPLAY_LCD_SKIP_GFXSS_INIT)
    printf("[DISPLAY_LCD_STEP] 03 bypass=Cy_GFXSS_Init\r\n");
    fflush(stdout);
    return app_display_backend_lcd_fail("gfxss_init_bypassed");
#else
    printf("[DISPLAY_LCD_GFXSS_CFG] dc_cfg=1 gpu_cfg=1 ctx=1\r\n");
    fflush(stdout);
    printf("[DISPLAY_LCD_GFXSS_CFG] w=%lu h=%lu stride=%lu\r\n",
           (unsigned long)APP_DISPLAY_LCD_WIDTH,
           (unsigned long)APP_DISPLAY_LCD_HEIGHT,
           (unsigned long)APP_DISPLAY_LCD_STRIDE);
    fflush(stdout);
    app_display_backend_lcd_step_before(3u, "Cy_GFXSS_Init");
    gfx_status = app_display_backend_lcd_init_gfxss();
    app_display_backend_lcd_step_after(3u, "Cy_GFXSS_Init", (long)gfx_status);
    if (CY_GFX_SUCCESS != gfx_status)
    {
        return app_display_backend_lcd_fail("gfxss_init_failed");
    }
#endif

    /* Step 4: I2C init (after GFXSS, matching official demo order) */
    app_display_backend_lcd_step_before(4u, "Cy_SCB_I2C_Init_Enable");
    i2c_status = app_display_backend_lcd_init_i2c();
    app_display_backend_lcd_step_after(4u, "Cy_SCB_I2C_Init_Enable",
                                       (long)i2c_status);
    if (CY_SCB_I2C_SUCCESS != i2c_status)
    {
        return app_display_backend_lcd_fail("i2c_init_failed");
    }

    /* Step 5: Panel init via I2C (500ms delay + Waveshare commands) */
    app_display_backend_lcd_step_before(5u, "panel_init_i2c");
    gfx_status = app_display_backend_lcd_init_panel();
    app_display_backend_lcd_step_after(5u, "panel_init_i2c", (long)gfx_status);
    if (CY_GFX_SUCCESS != gfx_status)
    {
        return app_display_backend_lcd_fail("panel_init_failed");
    }

    /* Step 6: Clear framebuffer and show initial screen */
    app_display_backend_lcd_step_before(6u, "framebuffer_clear_rgb565");
    app_display_backend_lcd_clear(0x0000); /* Black */
    __DSB();
    __ISB();
    app_display_backend_lcd_step_after(6u, "framebuffer_clear_rgb565", 0);

    app_display_backend_lcd_step_before(7u, "Cy_GFXSS_Transfer_Frame");
    gfx_status = app_display_backend_lcd_initial_transfer();
    app_display_backend_lcd_step_after(7u,
                                       "Cy_GFXSS_Transfer_Frame",
                                       (long)gfx_status);
    if (CY_GFX_SUCCESS != gfx_status)
    {
        return app_display_backend_lcd_fail("initial_frame_transfer_failed");
    }

    lcd_initialized = true;
    printf("[DISPLAY_LCD] OK: %lux%lu RGB565 framebuffer ready\r\n",
           (unsigned long)APP_DISPLAY_LCD_WIDTH,
           (unsigned long)APP_DISPLAY_LCD_HEIGHT);
    fflush(stdout);

    return CY_RSLT_SUCCESS;
}

const char *app_display_backend_lcd_last_fail_reason(void)
{
    return lcd_last_fail_reason;
}

cy_rslt_t app_display_backend_lcd_render_snapshot(
    const e84_display_snapshot_t *snapshot,
    bool force)
{
    cy_en_gfx_status_t gfx_status;

    (void)force;

    if (!lcd_initialized || (NULL == snapshot))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    /* Render the productized final screen to framebuffer */
    app_display_backend_lcd_render_frame(snapshot);
    __DSB();
    __ISB();

    /* Transfer framebuffer to display via DC */
    app_display_backend_lcd_step_before(8u, "Cy_GFXSS_Transfer_Frame_draw");
    gfx_status = Cy_GFXSS_Transfer_Frame(GFXSS, &gfx_context);
    app_display_backend_lcd_step_after(8u,
                                       "Cy_GFXSS_Transfer_Frame_draw",
                                       (long)gfx_status);
    if (CY_GFX_SUCCESS != gfx_status)
    {
        printf("[DISPLAY_LCD_DRAW] result=fail reason=frame_transfer_failed "
               "code=%ld\r\n",
               (long)gfx_status);
        fflush(stdout);
        return CY_RSLT_TYPE_ERROR;
    }

    return CY_RSLT_SUCCESS;
}

/*******************************************************************************
* GFXSS Initialization
*******************************************************************************/

static cy_en_gfx_status_t app_display_backend_lcd_init_gfxss(void)
{
    cy_en_gfx_status_t gfx_status;

    /*
     * GFXSS initialization following official demo pattern.
     *
     * The official demo uses BSP-generated GFXSS_config from cycfg_peripherals.c
     * (created by Device Configurator GFXSS personality). Since the E84 project's
     * BSP does not have the GFXSS personality configured, we create equivalent
     * structures manually here.
     *
     * Key differences from official demo:
     * - Official: uses BSP-generated global GFXSS_config
     * - Here: creates local config with same structure
     * - Official: CM55 core, W4P3INCH (832x480, 1 DSI lane)
     * - Here: CM33 core, EK79007AD3 (1024x600, 2 DSI lanes)
     */

    /* Graphics layer config - framebuffer in .cy_gpu_buf section */
    static cy_stc_gfx_layer_config_t gfx_layer_cfg = {
        .layer_type = GFX_LAYER_GRAPHICS,
        .buffer_address = (gctADDRESS *)lcd_framebuffer,
        .uv_buffer_address = (gctADDRESS *)lcd_framebuffer,
        .input_format_type = vivRGB565,
        .tiling_type = vivLINEAR,
        .pos_x = 0,
        .pos_y = 0,
        .width = APP_DISPLAY_LCD_WIDTH,
        .height = APP_DISPLAY_LCD_HEIGHT,
        .zorder = 0,
        .layer_enable = true,
        .visibility = true,
    };

    /* Display controller config - matches official demo structure */
    static cy_stc_gfx_dc_config_t dc_cfg = {
        .gfx_layer_config = &gfx_layer_cfg,
        .ovl0_layer_config = NULL,
        .ovl1_layer_config = NULL,
        .rlad_config = NULL,
        .cursor_config = NULL,
        .display_type = GFX_DISP_TYPE_DSI_DPI,
        .display_format = vivD24,
        .display_size = vivDISPLAY_CUSTOMIZED,
        .display_width = APP_DISPLAY_LCD_WIDTH,
        .display_height = APP_DISPLAY_LCD_HEIGHT,
        .interrupt_mask = GFXSS_DC_INTR_CORE_MASK,
    };

    /* GPU config - disabled for now (official demo enables it) */
    static cy_stc_gfx_gpu_cfg_t gpu_cfg = {
        .enable = false,
    };

    /* Top-level GFXSS config - uses Waveshare 4.3" DSI config (confirmed working) */
    static cy_stc_gfx_config_t gfx_cfg = {
        .dc_cfg = &dc_cfg,
        .gpu_cfg = &gpu_cfg,
        .mipi_dsi_cfg = &mtb_disp_waveshare_4p3_dsi_config,
        .display_update_type = GFX_SINGLE_BUFFER,
        .clockHz = 400000000u,  /* CLK_HF1 = 400 MHz */
    };

    printf("[DISPLAY_LCD_GFXSS] init start\r\n");
    fflush(stdout);

    gfx_status = Cy_GFXSS_Init(GFXSS, &gfx_cfg, &gfx_context);

    printf("[DISPLAY_LCD_GFXSS] init result=%ld\r\n", (long)gfx_status);
    fflush(stdout);

    if (CY_GFX_SUCCESS != gfx_status)
    {
        return gfx_status;
    }

    /* DC interrupt - deferred until proper handler is implemented
     * The official demo uses DC interrupt for frame completion notification.
     * For now, we use polling via Cy_GFXSS_Transfer_Frame(). */
    printf("[DISPLAY_LCD_GFXSS] dc_irq deferred=polling_mode\r\n");
    fflush(stdout);

    return CY_GFX_SUCCESS;
}

/*******************************************************************************
* Panel Initialization (DCS Commands)
*******************************************************************************/

/**
 * Panel init: send Waveshare 4.3" I2C commands directly, bypassing device ID check.
 *
 * The official demo uses mtb_disp_waveshare_4p3_init() which checks device ID
 * (0xC3/0xDE). Our board responds with 0x0e. Since the official demo works
 * on the same board, we send the same I2C commands without ID validation.
 *
 * I2C commands (from mtb_disp_dsi_waveshare_4p3.c):
 *   {0xAD, 0x00}  disable
 *   {0xAD, 0x01}  enable
 *   {0xC0, 0x01}  power on complete
 *   {0xAB, 0x00}  brightness full (inverted: 0x00 = max)
 *   {0xAA, 0x01}  final enable
 */
static cy_en_gfx_status_t app_display_backend_lcd_init_panel(void)
{
    static const uint8_t panel_cmds[][2] = {
        {0xADu, 0x00u},  /* disable */
        {0xADu, 0x01u},  /* enable */
        {0xC0u, 0x01u},  /* power on complete */
        {0xABu, 0x00u},  /* brightness full (inverted) */
        {0xAAu, 0x01u},  /* final enable */
    };
    cy_en_scb_i2c_status_t i2c_result;
    uint32_t cmd_idx;

    printf("[DISPLAY_LCD_DRIVER] selected=waveshare_4p3_direct\r\n");
    fflush(stdout);
    printf("[DISPLAY_LCD_I2C] hw=CYBSP_I2C_CONTROLLER addr=0x45 cmds=5\r\n");
    fflush(stdout);

    /* Wait for panel controller to stabilize (matches official demo's 500ms) */
    Cy_SysLib_Delay(500u);

    for (cmd_idx = 0; cmd_idx < 5u; cmd_idx++)
    {
        i2c_result = Cy_SCB_I2C_MasterWrite(CYBSP_I2C_CONTROLLER_HW,
            &(cy_stc_scb_i2c_master_xfer_config_t){
                .slaveAddress = 0x45u,
                .buffer = (uint8_t *)panel_cmds[cmd_idx],
                .bufferSize = 2u,
                .xferPending = false,
            },
            &i2c_context);
        if (CY_SCB_I2C_SUCCESS == i2c_result)
        {
            uint32_t timeout = 100u;
            while ((Cy_SCB_I2C_MasterGetStatus(CYBSP_I2C_CONTROLLER_HW,
                                               &i2c_context) &
                    CY_SCB_I2C_MASTER_BUSY) && timeout)
            {
                Cy_SysLib_Delay(1u);
                timeout--;
            }
            if (0u == timeout)
            {
                i2c_result = CY_SCB_I2C_MASTER_MANUAL_TIMEOUT;
            }
        }
        printf("[DISPLAY_LCD_PANEL] cmd[%lu] reg=0x%02x val=0x%02x result=%ld\r\n",
               (unsigned long)cmd_idx,
               panel_cmds[cmd_idx][0],
               panel_cmds[cmd_idx][1],
               (long)i2c_result);
        fflush(stdout);

        if (CY_SCB_I2C_SUCCESS != i2c_result)
        {
            printf("[DISPLAY_LCD_PANEL] init_fail cmd=%lu code=0x%08lx\r\n",
                   (unsigned long)cmd_idx, (unsigned long)i2c_result);
            fflush(stdout);
            return CY_GFX_BAD_PARAM;
        }
        Cy_SysLib_Delay(100u);
    }

    printf("[DISPLAY_LCD] Waveshare 4.3-inch panel initialized via I2C\r\n");
    fflush(stdout);
    return CY_GFX_SUCCESS;
}

static cy_en_gfx_status_t app_display_backend_lcd_initial_transfer(void)
{
    return Cy_GFXSS_Transfer_Frame(GFXSS, &gfx_context);
}

static cy_en_scb_i2c_status_t app_display_backend_lcd_init_i2c(void)
{
    cy_en_scb_i2c_status_t result;

    result = Cy_SCB_I2C_Init(CYBSP_I2C_CONTROLLER_HW,
                             &CYBSP_I2C_CONTROLLER_config,
                             &i2c_context);
    if (CY_SCB_I2C_SUCCESS == result)
    {
        Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_HW);
    }

    return result;
}

#if (APP_DISPLAY_LCD_DRIVER_SELECT != APP_DISPLAY_LCD_DRIVER_EK79007AD3)
static void app_display_backend_lcd_reset_panel(void)
{
#if (APP_DISPLAY_LCD_SKIP_PANEL_RESET)
    printf("[DISPLAY_LCD_PANEL] reset_gpio=bypassed\r\n");
    fflush(stdout);
    printf("[DISPLAY_LCD_STEP] 02.1 after=panel_reset_symbol_check result=0 bypass=1\r\n");
    fflush(stdout);
    printf("[DISPLAY_LCD_STEP] 02.2 after=panel_reset_gpio_low result=0 bypass=1\r\n");
    fflush(stdout);
    printf("[DISPLAY_LCD_STEP] 02.3 after=panel_reset_delay_low_ms result=0 bypass=1\r\n");
    fflush(stdout);
    printf("[DISPLAY_LCD_STEP] 02.4 after=panel_reset_gpio_high result=0 bypass=1\r\n");
    fflush(stdout);
    printf("[DISPLAY_LCD_STEP] 02.5 after=panel_reset_delay_high_ms result=0 bypass=1\r\n");
    fflush(stdout);
    printf("[DISPLAY_LCD_STEP] 02 after=panel_reset_gpio result=0 bypass=1\r\n");
    fflush(stdout);
#elif defined(CYBSP_DISP_RST_ENABLED) && (CYBSP_DISP_RST_ENABLED)
    printf("[DISPLAY_LCD_PANEL] rst_port=%lu rst_pin=%lu\r\n",
           (unsigned long)CYBSP_DISP_RST_PORT_NUM,
           (unsigned long)CYBSP_DISP_RST_PIN);
    fflush(stdout);

    /* 02.1: Verify symbol expansion */
    printf("[DISPLAY_LCD_STEP] 02.1 before=panel_reset_symbol_check\r\n");
    fflush(stdout);
    if ((void *)0 == (void *)CYBSP_DISP_RST_PORT)
    {
        printf("[DISPLAY_LCD_STEP] 02.1 after=panel_reset_symbol_check result=-1\r\n");
        fflush(stdout);
        printf("[DISPLAY_LCD_STEP] 02 after=panel_reset_gpio result=-1\r\n");
        fflush(stdout);
        return;
    }
    printf("[DISPLAY_LCD_STEP] 02.1 after=panel_reset_symbol_check result=0\r\n");
    fflush(stdout);

    /* 02.2: Assert RESET low */
    printf("[DISPLAY_LCD_STEP] 02.2 before=panel_reset_gpio_low\r\n");
    fflush(stdout);
    Cy_GPIO_Write(CYBSP_DISP_RST_PORT, CYBSP_DISP_RST_PIN, 0u);
    printf("[DISPLAY_LCD_STEP] 02.2 after=panel_reset_gpio_low result=0\r\n");
    fflush(stdout);

    /* 02.3: Hold low 10ms (use Cy_SysLib_Delay, not vTaskDelay) */
    printf("[DISPLAY_LCD_STEP] 02.3 before=panel_reset_delay_low_ms value=10\r\n");
    fflush(stdout);
    Cy_SysLib_Delay(10u);
    printf("[DISPLAY_LCD_STEP] 02.3 after=panel_reset_delay_low_ms result=0\r\n");
    fflush(stdout);

    /* 02.4: Release RESET high */
    printf("[DISPLAY_LCD_STEP] 02.4 before=panel_reset_gpio_high\r\n");
    fflush(stdout);
    Cy_GPIO_Write(CYBSP_DISP_RST_PORT, CYBSP_DISP_RST_PIN, 1u);
    printf("[DISPLAY_LCD_STEP] 02.4 after=panel_reset_gpio_high result=0\r\n");
    fflush(stdout);

    /* 02.5: Hold high (configurable, use Cy_SysLib_Delay, not vTaskDelay) */
    printf("[DISPLAY_LCD_STEP] 02.5 before=panel_reset_delay_high_ms value=%lu\r\n",
           (unsigned long)APP_DISPLAY_LCD_PANEL_POST_RESET_DELAY_MS);
    fflush(stdout);
    Cy_SysLib_Delay(APP_DISPLAY_LCD_PANEL_POST_RESET_DELAY_MS);
    printf("[DISPLAY_LCD_STEP] 02.5 after=panel_reset_delay_high_ms result=0\r\n");
    fflush(stdout);

    printf("[DISPLAY_LCD_STEP] 02 after=panel_reset_gpio result=0\r\n");
    fflush(stdout);
#else
    printf("[DISPLAY_LCD_PANEL] reset_gpio=not_generated\r\n");
    fflush(stdout);
    printf("[DISPLAY_LCD_STEP] 02 after=panel_reset_gpio result=0 skip=1\r\n");
    fflush(stdout);
#endif
}
#endif /* !EK79007AD3 */

static void app_display_backend_lcd_step_before(uint32_t step,
                                                const char *call)
{
    printf("[DISPLAY_LCD_STEP] %02lu before=%s\r\n",
           (unsigned long)step,
           call);
    fflush(stdout);
}

static void app_display_backend_lcd_step_after(uint32_t step,
                                               const char *call,
                                               long result)
{
    printf("[DISPLAY_LCD_STEP] %02lu after=%s result=%ld\r\n",
           (unsigned long)step,
           call,
           result);
    fflush(stdout);
}

static cy_rslt_t app_display_backend_lcd_fail(const char *reason)
{
    lcd_initialized = false;
    lcd_last_fail_reason = (NULL != reason) ? reason : "unknown";
    return CY_RSLT_TYPE_ERROR;
}

/*******************************************************************************
* Framebuffer Rendering
*******************************************************************************/

static void app_display_backend_lcd_clear(uint16_t color)
{
    for (uint32_t i = 0;
         i < (APP_DISPLAY_LCD_WIDTH * APP_DISPLAY_LCD_HEIGHT);
         i++)
    {
        lcd_framebuffer[i] = color;
    }
}

static void app_display_backend_lcd_render_frame(
    const e84_display_snapshot_t *snapshot)
{
    /*
     * TODO: Render the productized final screen layout to lcd_framebuffer.
     *
     * Layout (portrait 480x640):
     *
     *   +------------------------------------------+
     *   | E84 夜间呼吸与咳嗽健康伴侣               |  <- Header bar
     *   +------------------------------------------+
     *   | 状态: 监测中 | 数据源: 真实雷达联调       |  <- Status bar
     *   +------------------------------------------+
     *   | 雷达                                      |  <- Radar section
     *   |   来源: 真实雷达联调                      |
     *   |   人体: 有人                              |
     *   |   呼吸率: 16.8 bpm                        |
     *   |   心率: 71.0 bpm                          |
     *   |   距离: 1.20 m                            |
     *   |   质量: 良好                              |
     *   +------------------------------------------+
     *   | 咳嗽模型                                  |  <- Cough section
     *   |   状态: 模型未验证                        |
     *   |   说明: 模型未完成板级验证                |
     *   +------------------------------------------+
     *   | 用于趋势观察与竞赛演示，不作为医学诊断    |  <- Disclaimer
     *   +------------------------------------------+
     *
     * Use the same text mapping as app_display_backend_null.c:
     * - Main status: 监测中 / 数据暂不可用 / 雷达数据超时 / 信号较差 / 数据无效
     * - Source label: 真实雷达联调 / 雷达未连接 / 雷达数据超时 / 雷达信号较差 / 数据无效
     * - Presence: 有人 / 暂无数据 / 数据超时 / 未验证 / 信号较差
     * - Quality: 良好 / 信号较差 / Not Verified
     * - Cough: 模型未验证 (always)
     * - Disclaimer: 用于趋势观察与竞赛演示，不作为医学诊断
     *
     * For text rendering, either:
     * a) Use a bitmap font (simplest, no GPU needed)
     * b) Use VGLite GPU for vector text (requires GPU init)
     * c) Pre-render text to bitmap and blit
     */

    (void)snapshot; /* TODO: Remove when rendering is implemented */

    /* Placeholder: clear to dark blue */
    app_display_backend_lcd_clear(0x001F);
}

static void app_display_backend_lcd_draw_text(
    uint16_t x, uint16_t y,
    const char *text,
    uint16_t fg_color,
    uint16_t bg_color)
{
    /* TODO: Implement bitmap font rendering */
    (void)x;
    (void)y;
    (void)text;
    (void)fg_color;
    (void)bg_color;
}

static void app_display_backend_lcd_draw_rect(
    uint16_t x, uint16_t y,
    uint16_t w, uint16_t h,
    uint16_t color)
{
    /* TODO: Implement rectangle fill */
    (void)x;
    (void)y;
    (void)w;
    (void)h;
    (void)color;
}

static const char *app_display_lcd_decode_i2c_status(cy_en_scb_i2c_status_t code)
{
    switch (code)
    {
        case CY_SCB_I2C_SUCCESS:
            return "SUCCESS";
        case CY_SCB_I2C_BAD_PARAM:
            return "BAD_PARAM(device_id_mismatch_or_invalid_arg)";
        case CY_SCB_I2C_MASTER_NOT_READY:
            return "MASTER_NOT_READY";
        case CY_SCB_I2C_MASTER_MANUAL_TIMEOUT:
            return "MASTER_MANUAL_TIMEOUT";
        case CY_SCB_I2C_MASTER_MANUAL_ADDR_NAK:
            return "MASTER_MANUAL_ADDR_NAK(no_ack_from_device)";
        case CY_SCB_I2C_MASTER_MANUAL_NAK:
            return "MASTER_MANUAL_NAK";
        case CY_SCB_I2C_MASTER_MANUAL_ARB_LOST:
            return "MASTER_MANUAL_ARB_LOST";
        case CY_SCB_I2C_MASTER_MANUAL_BUS_ERR:
            return "MASTER_MANUAL_BUS_ERR";
        case CY_SCB_I2C_MASTER_MANUAL_ABORT_START:
            return "MASTER_MANUAL_ABORT_START";
        default:
            return "UNKNOWN";
    }
}

#endif /* APP_DISPLAY_ENABLE && APP_DISPLAY_LCD_ENABLE */

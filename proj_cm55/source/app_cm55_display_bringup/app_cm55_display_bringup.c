/*******************************************************************************
* File Name : app_cm55_display_bringup.c
*
* Description : Minimal CM55 display bring-up following official LVGL demo pattern.
*
* This file implements a minimal display initialization on CM55, following the
* same sequence as the PSOC_Edge_Graphics_LVGL_Demo. It is used for LCD
* hardware verification only -- not for production display.
*
* Reference: proj_cm55/main.c cm55_gfx_task() in the official demo.
*******************************************************************************/

#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cy_graphics.h"
#include "cy_mipidsi.h"
#include "cy_scb_i2c.h"
#include "mtb_disp_dsi_waveshare_4p3.h"

#include <stdio.h>
#include <string.h>

/*******************************************************************************
* Configuration
*******************************************************************************/

/* Display resolution -- Waveshare 4.3" */
#define CM55_DISP_HOR_RES       (832U)
#define CM55_DISP_VER_RES       (480U)
#define CM55_DISP_BYTE_PER_PIXEL (2U)  /* RGB565 */

/* Framebuffer size */
#define CM55_DISP_BUF_SIZE      (CM55_DISP_HOR_RES * CM55_DISP_VER_RES * \
                                 CM55_DISP_BYTE_PER_PIXEL)

/* Task configuration */
#define CM55_DISP_TASK_NAME     ("CM55 Disp")
#define CM55_DISP_TASK_STACK    (4096U)
#define CM55_DISP_TASK_PRIORITY (configMAX_PRIORITIES - 1U)

/* I2C panel controller address */
#define PANEL_I2C_ADDR          (0x45U)

/*******************************************************************************
* Framebuffers -- placed in .cy_gpu_buf (shared with CM33)
*******************************************************************************/

CY_SECTION(".cy_gpu_buf") static uint8_t disp_buf1[CM55_DISP_BUF_SIZE];
CY_SECTION(".cy_gpu_buf") static uint8_t disp_buf2[CM55_DISP_BUF_SIZE];

static cy_stc_gfx_context_t gfx_context;
static cy_stc_scb_i2c_context_t i2c_context;
static TaskHandle_t cm55_disp_task_handle;

/*******************************************************************************
* DC Interrupt Handler
*******************************************************************************/

static void dc_irq_handler(void)
{
    Cy_GFXSS_Clear_DC_Interrupt(GFXSS, &gfx_context);
    if (NULL != cm55_disp_task_handle)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(cm55_disp_task_handle,
                               &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*******************************************************************************
* Display Task -- follows official demo cm55_gfx_task() sequence
*******************************************************************************/

static void cm55_display_task(void *pvParameters)
{
    cy_en_gfx_status_t gfx_status;
    cy_en_scb_i2c_status_t i2c_status;
    cy_stc_sysint_t dc_irq_cfg;
    uint32_t tick_count = 0;

    (void)pvParameters;

    cm55_disp_task_handle = xTaskGetCurrentTaskHandle();

    printf("[CM55_DISP] task start\r\n");
    fflush(stdout);

    /* ----------------------------------------------------------------
     * Step 1: GFXSS peripheral group clocks
     * Reference: official demo does this inside Cy_GFXSS_Init
     * ---------------------------------------------------------------- */
    printf("[CM55_DISP] step01 gfxss_peri_clocks\r\n");
    fflush(stdout);
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

    /* ----------------------------------------------------------------
     * Step 2: GFXSS init
     * Reference: official demo proj_cm55/main.c line 521
     * Creates local GFXSS config (BSP doesn't have GFXSS personality)
     * ---------------------------------------------------------------- */
    printf("[CM55_DISP] step02 gfxss_init\r\n");
    fflush(stdout);

    /* Clear framebuffers */
    memset(disp_buf1, 0, sizeof(disp_buf1));
    memset(disp_buf2, 0, sizeof(disp_buf2));

    /* Create local GFXSS config structures */
    {
        static cy_stc_gfx_layer_config_t gfx_layer_cfg = {
            .layer_type = GFX_LAYER_GRAPHICS,
            .buffer_address = NULL,  /* set below */
            .uv_buffer_address = NULL,
            .input_format_type = vivRGB565,
            .tiling_type = vivLINEAR,
            .pos_x = 0,
            .pos_y = 0,
            .width = CM55_DISP_HOR_RES,
            .height = CM55_DISP_VER_RES,
            .zorder = 0,
            .layer_enable = true,
            .visibility = true,
        };
        static cy_stc_gfx_dc_config_t dc_cfg = {
            .gfx_layer_config = &gfx_layer_cfg,
            .ovl0_layer_config = NULL,
            .ovl1_layer_config = NULL,
            .rlad_config = NULL,
            .cursor_config = NULL,
            .display_type = GFX_DISP_TYPE_DSI_DPI,
            .display_format = vivD24,
            .display_size = vivDISPLAY_CUSTOMIZED,
            .display_width = CM55_DISP_HOR_RES,
            .display_height = CM55_DISP_VER_RES,
            .interrupt_mask = GFXSS_DC_INTR_CORE_MASK,
        };
        static cy_stc_gfx_gpu_cfg_t gpu_cfg = {
            .enable = false,
        };
        static cy_stc_gfx_config_t local_gfx_cfg = {
            .dc_cfg = &dc_cfg,
            .gpu_cfg = &gpu_cfg,
            .mipi_dsi_cfg = &mtb_disp_waveshare_4p3_dsi_config,
            .display_update_type = GFX_SINGLE_BUFFER,
            .clockHz = 400000000u,
        };

        /* Set framebuffer address */
        gfx_layer_cfg.buffer_address = (gctADDRESS *)disp_buf1;
        gfx_layer_cfg.uv_buffer_address = (gctADDRESS *)disp_buf1;

        gfx_status = Cy_GFXSS_Init(GFXSS, &local_gfx_cfg, &gfx_context);
    }
    printf("[CM55_DISP] step02 gfxss_init result=%ld\r\n", (long)gfx_status);
    fflush(stdout);
    if (CY_GFX_SUCCESS != gfx_status)
    {
        printf("[CM55_DISP] FAIL: gfxss_init\r\n");
        fflush(stdout);
        vTaskDelete(NULL);
        return;
    }

    /* ----------------------------------------------------------------
     * Step 3: DC interrupt
     * Reference: official demo proj_cm55/main.c lines 526-535
     * ---------------------------------------------------------------- */
    printf("[CM55_DISP] step03 dc_irq\r\n");
    fflush(stdout);
    dc_irq_cfg.intrSrc = gfxss_interrupt_dc_IRQn;
    dc_irq_cfg.intrPriority = 3U;
    Cy_SysInt_Init(&dc_irq_cfg, dc_irq_handler);
    NVIC_EnableIRQ(gfxss_interrupt_dc_IRQn);

    /* ----------------------------------------------------------------
     * Step 4: I2C init for display panel
     * Reference: official demo proj_cm55/main.c lines 553-577
     * ---------------------------------------------------------------- */
    printf("[CM55_DISP] step04 i2c_init\r\n");
    fflush(stdout);
    i2c_status = Cy_SCB_I2C_Init(CYBSP_I2C_CONTROLLER_HW,
                                  &CYBSP_I2C_CONTROLLER_config,
                                  &i2c_context);
    printf("[CM55_DISP] step04 i2c_init result=%ld\r\n", (long)i2c_status);
    fflush(stdout);
    if (CY_SCB_I2C_SUCCESS != i2c_status)
    {
        printf("[CM55_DISP] FAIL: i2c_init\r\n");
        fflush(stdout);
        vTaskDelete(NULL);
        return;
    }
    Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_HW);

    /* ----------------------------------------------------------------
     * Step 5: 500ms delay + panel init
     * Reference: official demo proj_cm55/main.c lines 579, 632-638
     * ---------------------------------------------------------------- */
    printf("[CM55_DISP] step05 delay_500ms\r\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(500));

    printf("[CM55_DISP] step05 panel_init addr=0x%02x\r\n", PANEL_I2C_ADDR);
    fflush(stdout);
    i2c_status = mtb_disp_waveshare_4p3_init(CYBSP_I2C_CONTROLLER_HW,
                                              &i2c_context);
    printf("[CM55_DISP] step05 panel_init result=%ld\r\n", (long)i2c_status);
    fflush(stdout);
    if (CY_SCB_I2C_SUCCESS != i2c_status)
    {
        printf("[CM55_DISP] WARN: panel_init failed (continuing)\r\n");
        fflush(stdout);
    }

    /* ----------------------------------------------------------------
     * Step 6: Draw test pattern
     * Fill framebuffer with a gradient + "E84 Display OK" text area
     * ---------------------------------------------------------------- */
    printf("[CM55_DISP] step06 draw_test\r\n");
    fflush(stdout);

    /* Draw a simple color gradient in the framebuffer */
    {
        uint16_t *fb = (uint16_t *)disp_buf1;
        for (uint32_t y = 0; y < CM55_DISP_VER_RES; y++)
        {
            for (uint32_t x = 0; x < CM55_DISP_HOR_RES; x++)
            {
                /* RGB565 gradient: blue to green */
                uint8_t r = 0;
                uint8_t g = (uint8_t)((y * 255U) / CM55_DISP_VER_RES);
                uint8_t b = (uint8_t)(255U - (y * 255U) / CM55_DISP_VER_RES);
                fb[y * CM55_DISP_HOR_RES + x] =
                    (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
            }
        }

        /* Draw a white rectangle in the center (text placeholder) */
        uint32_t box_x = CM55_DISP_HOR_RES / 4;
        uint32_t box_y = CM55_DISP_VER_RES / 3;
        uint32_t box_w = CM55_DISP_HOR_RES / 2;
        uint32_t box_h = CM55_DISP_VER_RES / 3;
        for (uint32_t y = box_y; y < box_y + box_h; y++)
        {
            for (uint32_t x = box_x; x < box_x + box_w; x++)
            {
                fb[y * CM55_DISP_HOR_RES + x] = 0xFFFF; /* white */
            }
        }
    }

    __DSB();
    __ISB();

    /* Transfer framebuffer to display */
    printf("[CM55_DISP] step06 transfer\r\n");
    fflush(stdout);
    gfx_status = Cy_GFXSS_Transfer_Frame(GFXSS, &gfx_context);
    printf("[CM55_DISP] step06 transfer result=%ld\r\n", (long)gfx_status);
    fflush(stdout);

    printf("[CM55_DISP] READY\r\n");
    fflush(stdout);

    /* ----------------------------------------------------------------
     * Main loop: periodic refresh
     * Reference: official demo proj_cm55/main.c lines 694-701
     * ---------------------------------------------------------------- */
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        tick_count++;

        /* Swap framebuffer and transfer */
        Cy_GFXSS_Set_FrameBuffer(GFXSS, (uint32_t *)disp_buf1, &gfx_context);

        if (tick_count % 10U == 0U)
        {
            printf("[CM55_DISP] tick=%lu\r\n", (unsigned long)tick_count);
            fflush(stdout);
        }
    }
}

/*******************************************************************************
* Public API
*******************************************************************************/

cy_rslt_t app_cm55_display_bringup_init(void)
{
    BaseType_t ret;

    printf("[CM55_DISP] init\r\n");
    fflush(stdout);

    ret = xTaskCreate(cm55_display_task,
                      CM55_DISP_TASK_NAME,
                      CM55_DISP_TASK_STACK,
                      NULL,
                      CM55_DISP_TASK_PRIORITY,
                      NULL);
    if (pdPASS != ret)
    {
        printf("[CM55_DISP] FAIL: task_create\r\n");
        fflush(stdout);
        return CY_RSLT_TYPE_ERROR;
    }

    return CY_RSLT_SUCCESS;
}

/* [] END OF FILE */

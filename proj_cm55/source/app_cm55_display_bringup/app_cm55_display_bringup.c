/*******************************************************************************
* File Name : app_cm55_display_bringup.c
*
* Description : CM55 official-like display proof path for Waveshare 4.3" DSI.
*******************************************************************************/

#include "app_cm55_display_bringup.h"

#include "app_display_diag.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cy_graphics.h"
#include "cy_scb_i2c.h"
#include "cycfg.h"
#include "mtb_disp_dsi_waveshare_4p3.h"
#include "vg_lite.h"
#include "vg_lite_platform.h"

#ifndef APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE
#define APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE (0u)
#endif

#if (APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE)
#include "app_display_cm55_shared.h"
#endif

#include <stdio.h>
#include <string.h>

#pragma GCC optimize ("no-tree-vectorize")

#ifndef APP_DISPLAY_DIAG_ENABLE
#define APP_DISPLAY_DIAG_ENABLE (0u)
#endif

#ifndef APP_DISPLAY_CM55_UART_LOG_ENABLE
#define APP_DISPLAY_CM55_UART_LOG_ENABLE (1u)
#endif

#ifndef APP_DISPLAY_PANEL_I2C_SUBDIAG_ENABLE
#define APP_DISPLAY_PANEL_I2C_SUBDIAG_ENABLE (0u)
#endif

#ifndef APP_DISPLAY_PANEL_I2C_BYPASS_ID_ENABLE
#define APP_DISPLAY_PANEL_I2C_BYPASS_ID_ENABLE (0u)
#endif

#ifndef APP_DISPLAY_PANEL_I2C_LOWLEVEL_PROBE_ENABLE
#define APP_DISPLAY_PANEL_I2C_LOWLEVEL_PROBE_ENABLE (0u)
#endif

#define CM55_DISP_HOR_RES               (832U)
#define CM55_DISP_VER_RES               (480U)
#define CM55_DISP_ACTUAL_HOR_RES        (800U)
#define CM55_DISP_GPU_INT_PRIORITY      (3U)
#define CM55_DISP_DC_INT_PRIORITY       (3U)
#define CM55_DISP_I2C_INT_PRIORITY      (2U)
#define CM55_DISP_TASK_NAME             ("CM55 Gfx Task")
#define CM55_DISP_TASK_STACK            (configMINIMAL_STACK_SIZE * 16U)
#define CM55_DISP_TASK_PRIORITY         (configMAX_PRIORITIES - 1U)
#define CM55_DISP_APP_BUFFER_COUNT      (2U)
#define CM55_DISP_GPU_CMD_BUFFER_SIZE   ((64U) * (1024U))
#define CM55_DISP_GPU_TESS_BUFFER_SIZE  (CM55_DISP_VER_RES * 128U)
#define CM55_DISP_VGLITE_HEAP_SIZE      \
    ((CM55_DISP_GPU_CMD_BUFFER_SIZE * CM55_DISP_APP_BUFFER_COUNT) + \
     (CM55_DISP_GPU_TESS_BUFFER_SIZE * CM55_DISP_APP_BUFFER_COUNT))
#define CM55_DISP_GPU_MEM_BASE          (0x0U)
#define CM55_DISP_VG_PARAMS_POS         (0U)
#define CM55_DISP_PANEL_I2C_RETRY_COUNT (5U)
#define CM55_DISP_PANEL_I2C_TIMEOUT_MS  (5U)
#define CM55_DISP_PANEL_I2C_DELAY_MS    (1U)
#define CM55_DISP_PANEL_I2C_ERROR_MASK  (CY_SCB_I2C_MASTER_DATA_NAK | \
                                         CY_SCB_I2C_MASTER_ADDR_NAK | \
                                         CY_SCB_I2C_MASTER_ARB_LOST | \
                                         CY_SCB_I2C_MASTER_ABORT_START | \
                                         CY_SCB_I2C_MASTER_BUS_ERR)
#define CM55_DISP_PANEL_FLAG(index_, reg_, val_) \
    ((((uint32_t)(index_) & 0xFFu) << 16) | \
     (((uint32_t)(reg_) & 0xFFu) << 8) | \
     ((uint32_t)(val_) & 0xFFu))

#define RGB565(r, g, b) \
    (uint16_t)((((uint16_t)(r) & 0xF8U) << 8) | \
               (((uint16_t)(g) & 0xFCU) << 3) | \
               (((uint16_t)(b) & 0xF8U) >> 3))

#if (APP_DISPLAY_CM55_UART_LOG_ENABLE)
#define CM55_DISP_LOG(...) \
    do { printf(__VA_ARGS__); fflush(stdout); } while (0)
#else
#define CM55_DISP_LOG(...) \
    do { } while (0)
#endif

#if (APP_DISPLAY_DIAG_ENABLE)
#define CM55_DISP_DIAG(stage_, result_, flags_) \
    app_display_diag_mark((stage_), (uint32_t)(result_), (uint32_t)__LINE__, (uint32_t)(flags_))
#else
#define CM55_DISP_DIAG(stage_, result_, flags_) \
    do { (void)(stage_); (void)(result_); (void)(flags_); } while (0)
#endif

extern uint16_t frame_buffer1[CM55_DISP_HOR_RES * CM55_DISP_VER_RES];
extern uint16_t frame_buffer2[CM55_DISP_HOR_RES * CM55_DISP_VER_RES];
extern uint8_t contiguous_mem[CM55_DISP_VGLITE_HEAP_SIZE];

static volatile void *vglite_heap_base = contiguous_mem;
static TaskHandle_t cm55_gfx_task_handle;
static cy_stc_gfx_context_t gfx_context;
static cy_stc_scb_i2c_context_t display_i2c_context;
static vg_module_parameters_t vg_params;

#if (APP_DISPLAY_PANEL_I2C_SUBDIAG_ENABLE)
static volatile uint32_t display_i2c_irq_count;
#endif

static const cy_stc_sysint_t dc_irq_cfg =
{
    .intrSrc = GFXSS_DC_IRQ,
    .intrPriority = CM55_DISP_DC_INT_PRIORITY
};

static const cy_stc_sysint_t gpu_irq_cfg =
{
    .intrSrc = GFXSS_GPU_IRQ,
    .intrPriority = CM55_DISP_GPU_INT_PRIORITY
};

static const cy_stc_sysint_t display_i2c_irq_cfg =
{
    .intrSrc = CYBSP_I2C_CONTROLLER_IRQ,
    .intrPriority = CM55_DISP_I2C_INT_PRIORITY
};

static void dc_irq_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    Cy_GFXSS_Clear_DC_Interrupt(GFXSS, &gfx_context);
    if (NULL != cm55_gfx_task_handle)
    {
        xTaskNotifyFromISR(cm55_gfx_task_handle,
                           1U,
                           eSetValueWithOverwrite,
                           &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void gpu_irq_handler(void)
{
    Cy_GFXSS_Clear_GPU_Interrupt(GFXSS, &gfx_context);
    vg_lite_IRQHandler();
}

static void display_i2c_irq_handler(void)
{
#if (APP_DISPLAY_PANEL_I2C_SUBDIAG_ENABLE)
    display_i2c_irq_count++;
#endif
    Cy_SCB_I2C_Interrupt(CYBSP_I2C_CONTROLLER_HW, &display_i2c_context);
}

static void fill_rect(uint16_t *fb,
                      uint32_t x,
                      uint32_t y,
                      uint32_t width,
                      uint32_t height,
                      uint16_t color)
{
    uint32_t x_end = x + width;
    uint32_t y_end = y + height;

    if (x_end > CM55_DISP_ACTUAL_HOR_RES)
    {
        x_end = CM55_DISP_ACTUAL_HOR_RES;
    }
    if (y_end > CM55_DISP_VER_RES)
    {
        y_end = CM55_DISP_VER_RES;
    }

    for (uint32_t yy = y; yy < y_end; ++yy)
    {
        for (uint32_t xx = x; xx < x_end; ++xx)
        {
            fb[(yy * CM55_DISP_HOR_RES) + xx] = color;
        }
    }
}

static void draw_segment_digit(uint16_t *fb,
                               uint32_t x,
                               uint32_t y,
                               uint8_t digit,
                               uint16_t color)
{
    static const uint8_t segments[10] =
    {
        0x3FU, 0x06U, 0x5BU, 0x4FU, 0x66U,
        0x6DU, 0x7DU, 0x07U, 0x7FU, 0x6FU
    };
    const uint32_t w = 58U;
    const uint32_t h = 92U;
    const uint32_t t = 10U;
    uint8_t mask = segments[digit % 10U];

    if (mask & 0x01U) { fill_rect(fb, x + t, y, w - (2U * t), t, color); }
    if (mask & 0x02U) { fill_rect(fb, x + w - t, y + t, t, (h / 2U) - t, color); }
    if (mask & 0x04U) { fill_rect(fb, x + w - t, y + (h / 2U), t, (h / 2U) - t, color); }
    if (mask & 0x08U) { fill_rect(fb, x + t, y + h - t, w - (2U * t), t, color); }
    if (mask & 0x10U) { fill_rect(fb, x, y + (h / 2U), t, (h / 2U) - t, color); }
    if (mask & 0x20U) { fill_rect(fb, x, y + t, t, (h / 2U) - t, color); }
    if (mask & 0x40U) { fill_rect(fb, x + t, y + (h / 2U) - (t / 2U), w - (2U * t), t, color); }
}

static void draw_block_e(uint16_t *fb, uint32_t x, uint32_t y, uint16_t color)
{
    fill_rect(fb, x, y, 14U, 92U, color);
    fill_rect(fb, x, y, 58U, 12U, color);
    fill_rect(fb, x, y + 40U, 50U, 12U, color);
    fill_rect(fb, x, y + 80U, 58U, 12U, color);
}

static void draw_block_o(uint16_t *fb, uint32_t x, uint32_t y, uint16_t color)
{
    fill_rect(fb, x, y, 58U, 12U, color);
    fill_rect(fb, x, y + 80U, 58U, 12U, color);
    fill_rect(fb, x, y, 14U, 92U, color);
    fill_rect(fb, x + 44U, y, 14U, 92U, color);
}

static void draw_block_k(uint16_t *fb, uint32_t x, uint32_t y, uint16_t color)
{
    fill_rect(fb, x, y, 14U, 92U, color);
    fill_rect(fb, x + 14U, y + 40U, 18U, 12U, color);
    fill_rect(fb, x + 30U, y + 26U, 14U, 14U, color);
    fill_rect(fb, x + 42U, y + 12U, 14U, 14U, color);
    fill_rect(fb, x + 30U, y + 52U, 14U, 14U, color);
    fill_rect(fb, x + 42U, y + 66U, 14U, 14U, color);
}

static void draw_proof_frame(uint16_t *fb, uint32_t phase)
{
    uint16_t bg_top = RGB565(16U, 24U, 32U);
    uint16_t bg_bottom = RGB565(8U, 52U, 64U);
    uint16_t white = RGB565(245U, 247U, 250U);
    uint16_t cyan = RGB565(0U, 179U, 164U);
    uint16_t yellow = RGB565(253U, 181U, 21U);
    uint16_t red = RGB565(240U, 82U, 82U);
    uint16_t accent = (0U == (phase & 1U)) ? cyan : yellow;

    for (uint32_t y = 0U; y < CM55_DISP_VER_RES; ++y)
    {
        uint8_t mix = (uint8_t)((y * 255U) / CM55_DISP_VER_RES);
        uint8_t r = (uint8_t)(16U - ((8U * mix) / 255U));
        uint8_t g = (uint8_t)(24U + ((28U * mix) / 255U));
        uint8_t b = (uint8_t)(32U + ((32U * mix) / 255U));
        uint16_t color = RGB565(r, g, b);
        for (uint32_t x = 0U; x < CM55_DISP_HOR_RES; ++x)
        {
            fb[(y * CM55_DISP_HOR_RES) + x] = color;
        }
    }

    fill_rect(fb, 0U, 0U, CM55_DISP_ACTUAL_HOR_RES, 8U, bg_bottom);
    fill_rect(fb, 0U, CM55_DISP_VER_RES - 8U, CM55_DISP_ACTUAL_HOR_RES, 8U, bg_top);
    fill_rect(fb, 118U, 96U, 564U, 188U, RGB565(22U, 32U, 44U));
    fill_rect(fb, 118U, 96U, 564U, 8U, accent);

    draw_block_e(fb, 184U, 142U, white);
    draw_segment_digit(fb, 264U, 142U, 8U, cyan);
    draw_segment_digit(fb, 344U, 142U, 4U, yellow);
    draw_block_o(fb, 464U, 142U, white);
    draw_block_k(fb, 544U, 142U, red);

    fill_rect(fb, 160U, 318U, 480U, 18U, cyan);
    fill_rect(fb, 160U, 350U, 480U, 18U, yellow);
    fill_rect(fb, 160U, 382U, 480U, 18U, red);
}

#if (APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE)
/* Minimal 5x7 ASCII font for live data display. Covers 0x20..0x7E. */
static const uint8_t font_5x7[][5] =
{
    {0x00,0x00,0x00,0x00,0x00}, /* space */
    {0x00,0x00,0x5F,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* $ */
    {0x23,0x13,0x08,0x64,0x62}, /* % */
    {0x36,0x49,0x55,0x22,0x50}, /* & */
    {0x00,0x05,0x03,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00}, /* ) */
    {0x08,0x2A,0x1C,0x2A,0x08}, /* * */
    {0x08,0x08,0x3E,0x08,0x08}, /* + */
    {0x00,0x50,0x30,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08}, /* - */
    {0x00,0x60,0x60,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00}, /* ; */
    {0x00,0x08,0x14,0x22,0x41}, /* < */
    {0x14,0x14,0x14,0x14,0x14}, /* = */
    {0x41,0x22,0x14,0x08,0x00}, /* > */
    {0x02,0x01,0x51,0x09,0x06}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x01,0x01}, /* F */
    {0x3E,0x41,0x41,0x51,0x32}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x04,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x7F,0x20,0x18,0x20,0x7F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x03,0x04,0x78,0x04,0x03}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}, /* Z */
    {0x00,0x00,0x7F,0x41,0x41}, /* [ */
    {0x02,0x04,0x08,0x10,0x20}, /* \ */
    {0x41,0x41,0x7F,0x00,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04}, /* ^ */
    {0x40,0x40,0x40,0x40,0x40}, /* _ */
    {0x00,0x01,0x02,0x04,0x00}, /* ` */
    {0x20,0x54,0x54,0x54,0x78}, /* a */
    {0x7F,0x48,0x44,0x44,0x38}, /* b */
    {0x38,0x44,0x44,0x44,0x20}, /* c */
    {0x38,0x44,0x44,0x48,0x7F}, /* d */
    {0x38,0x54,0x54,0x54,0x18}, /* e */
    {0x08,0x7E,0x09,0x01,0x02}, /* f */
    {0x08,0x14,0x54,0x54,0x3C}, /* g */
    {0x7F,0x08,0x04,0x04,0x78}, /* h */
    {0x00,0x44,0x7D,0x40,0x00}, /* i */
    {0x20,0x40,0x44,0x3D,0x00}, /* j */
    {0x00,0x7F,0x10,0x28,0x44}, /* k */
    {0x00,0x41,0x7F,0x40,0x00}, /* l */
    {0x7C,0x04,0x18,0x04,0x78}, /* m */
    {0x7C,0x08,0x04,0x04,0x78}, /* n */
    {0x38,0x44,0x44,0x44,0x38}, /* o */
    {0x7C,0x14,0x14,0x14,0x08}, /* p */
    {0x08,0x14,0x14,0x18,0x7C}, /* q */
    {0x7C,0x08,0x04,0x04,0x08}, /* r */
    {0x48,0x54,0x54,0x54,0x20}, /* s */
    {0x04,0x3F,0x44,0x40,0x20}, /* t */
    {0x3C,0x40,0x40,0x20,0x7C}, /* u */
    {0x1C,0x20,0x40,0x20,0x1C}, /* v */
    {0x3C,0x40,0x30,0x40,0x3C}, /* w */
    {0x44,0x28,0x10,0x28,0x44}, /* x */
    {0x0C,0x50,0x50,0x50,0x3C}, /* y */
    {0x44,0x64,0x54,0x4C,0x44}, /* z */
    {0x00,0x08,0x36,0x41,0x00}, /* { */
    {0x00,0x00,0x7F,0x00,0x00}, /* | */
    {0x00,0x41,0x36,0x08,0x00}, /* } */
    {0x08,0x08,0x2A,0x1C,0x08}, /* ~ */
};

static void draw_char(uint16_t *fb,
                      uint32_t x,
                      uint32_t y,
                      char ch,
                      uint16_t color,
                      uint32_t scale)
{
    if ((ch < 0x20) || (ch > 0x7E))
    {
        ch = ' ';
    }
    const uint8_t *glyph = font_5x7[(uint32_t)ch - 0x20U];
    for (uint32_t col = 0U; col < 5U; ++col)
    {
        uint8_t line = glyph[col];
        for (uint32_t row = 0U; row < 7U; ++row)
        {
            if (0U != (line & (1U << row)))
            {
                fill_rect(fb,
                          x + (col * scale),
                          y + (row * scale),
                          scale,
                          scale,
                          color);
            }
        }
    }
}

static void draw_text(uint16_t *fb,
                      uint32_t x,
                      uint32_t y,
                      const char *text,
                      uint16_t color,
                      uint32_t scale)
{
    while ('\0' != *text)
    {
        draw_char(fb, x, y, *text, color, scale);
        x += 6U * scale;
        ++text;
    }
}

static uint32_t draw_number(uint16_t *fb,
                            uint32_t x,
                            uint32_t y,
                            uint32_t value,
                            uint16_t color,
                            uint32_t scale)
{
    char buf[12];
    int len = 0;
    uint32_t tmp = value;

    if (0U == tmp)
    {
        buf[len++] = '0';
    }
    else
    {
        while (tmp > 0U)
        {
            buf[len++] = (char)('0' + (tmp % 10U));
            tmp /= 10U;
        }
    }

    for (int i = len - 1; i >= 0; --i)
    {
        draw_char(fb, x, y, buf[i], color, scale);
        x += 6U * scale;
    }

    return (uint32_t)len * 6U * scale;
}

static void draw_float_1dp(uint16_t *fb,
                           uint32_t x,
                           uint32_t y,
                           uint16_t value_x10,
                           uint16_t color,
                           uint32_t scale)
{
    uint32_t whole = value_x10 / 10U;
    uint32_t frac = value_x10 % 10U;

    x += draw_number(fb, x, y, whole, color, scale);
    draw_char(fb, x, y, '.', color, scale);
    x += 6U * scale;
    draw_char(fb, x, y, (char)('0' + frac), color, scale);
}

static void draw_live_frame(uint16_t *fb)
{
    volatile app_display_cm55_snapshot_t *snap = APP_DISPLAY_CM55_SNAPSHOT;
    uint32_t seq_end, seq_begin;
    uint16_t bg = RGB565(16U, 24U, 32U);
    uint16_t title_color = RGB565(0U, 200U, 255U);
    uint16_t label_color = RGB565(160U, 170U, 180U);
    uint16_t value_color = RGB565(245U, 247U, 250U);
    uint16_t warn_color = RGB565(253U, 181U, 21U);
    uint16_t error_color = RGB565(240U, 82U, 82U);
    uint16_t ok_color = RGB565(0U, 200U, 120U);
    uint16_t dim_color = RGB565(80U, 90U, 100U);
    uint32_t y;

    /* Clear framebuffer. */
    for (uint32_t i = 0U; i < (CM55_DISP_HOR_RES * CM55_DISP_VER_RES); ++i)
    {
        fb[i] = bg;
    }

    /* Torn-read check: read seq_end, then data, then seq_begin. */
    APP_DISPLAY_CM55_INVALIDATE_CACHE(
        (uint32_t)snap, sizeof(app_display_cm55_snapshot_t));
    seq_end = snap->seq_end;
    __DMB();

    /* Title bar. */
    fill_rect(fb, 0U, 0U, CM55_DISP_ACTUAL_HOR_RES, 40U,
              RGB565(22U, 32U, 44U));
    draw_text(fb, 16U, 10U, "E84 Health Monitor", title_color, 2U);

    /* Status indicator: top-right. */
    {
        uint16_t status_color;
        const char *status_text;
        uint8_t health = snap->health_state;

        if (health == DISPLAY_CM55_HEALTH_NORMAL)
        {
            status_color = ok_color;
            status_text = "NORMAL";
        }
        else if (health == DISPLAY_CM55_HEALTH_ATTENTION)
        {
            status_color = warn_color;
            status_text = "ATTENTION";
        }
        else if (health == DISPLAY_CM55_HEALTH_WARNING)
        {
            status_color = error_color;
            status_text = "WARNING";
        }
        else if (health == DISPLAY_CM55_HEALTH_SENSOR_LOST)
        {
            status_color = warn_color;
            status_text = "SENSOR LOST";
        }
        else if (health == DISPLAY_CM55_HEALTH_ERROR)
        {
            status_color = error_color;
            status_text = "ERROR";
        }
        else
        {
            status_color = dim_color;
            status_text = "INIT";
        }
        draw_text(fb, 560U, 12U, status_text, status_color, 2U);
    }

    /* Separator line. */
    fill_rect(fb, 0U, 42U, CM55_DISP_ACTUAL_HOR_RES, 2U,
              RGB565(40U, 60U, 80U));

    /* === MIC Audio Section === */
    y = 56U;
    draw_text(fb, 24U, y, "-- MIC Audio", label_color, 2U);
    y += 24U;

    /* Cough probability. */
    draw_text(fb, 40U, y, "Cough Prob:", label_color, 2U);
    {
        uint16_t prob_x1000 = snap->mic_cough_prob_x1000;
        uint16_t prob_color = (prob_x1000 > 500U) ? error_color :
                              (prob_x1000 > 300U) ? warn_color :
                              value_color;
        draw_float_1dp(fb, 280U, y, prob_x1000, prob_color, 2U);
    }
    y += 22U;

    /* Cough model verification status. */
    if (0U != snap->cough_model_not_verified)
    {
        draw_text(fb, 40U, y, "Model: NOT VERIFIED", warn_color, 2U);
    }
    else
    {
        draw_text(fb, 40U, y, "Model: Verified", ok_color, 2U);
    }
    y += 22U;

    /* Cough counts. */
    draw_text(fb, 40U, y, "Coughs 1m:", label_color, 2U);
    draw_number(fb, 280U, y, snap->cough_count_1min, value_color, 2U);
    y += 22U;

    draw_text(fb, 40U, y, "Coughs 5m:", label_color, 2U);
    draw_number(fb, 280U, y, snap->cough_count_5min, value_color, 2U);
    y += 22U;

    /* Audio quality. */
    draw_text(fb, 40U, y, "Audio Qual:", label_color, 2U);
    draw_number(fb, 280U, y, snap->audio_quality, value_color, 2U);
    draw_text(fb, 320U, y, "%", dim_color, 2U);

    /* Separator. */
    y += 30U;
    fill_rect(fb, 16U, y, CM55_DISP_ACTUAL_HOR_RES - 32U, 1U,
              RGB565(40U, 60U, 80U));
    y += 12U;

    /* === Radar Section === */
    draw_text(fb, 24U, y, "-- Radar", label_color, 2U);
    y += 24U;

    {
        uint8_t radar_src = snap->radar_source;
        uint16_t radar_color;
        const char *radar_text;

        switch (radar_src)
        {
            case DISPLAY_CM55_RADAR_NORMAL:
                radar_color = ok_color;
                radar_text = "Source: NORMAL";
                break;
            case DISPLAY_CM55_RADAR_UNAVAILABLE:
                radar_color = dim_color;
                radar_text = "Source: N/A";
                break;
            case DISPLAY_CM55_RADAR_STALE:
                radar_color = warn_color;
                radar_text = "Source: STALE";
                break;
            case DISPLAY_CM55_RADAR_INVALID:
                radar_color = error_color;
                radar_text = "Source: INVALID";
                break;
            case DISPLAY_CM55_RADAR_LOW_QUALITY:
                radar_color = warn_color;
                radar_text = "Source: LOW QUALITY";
                break;
            default:
                radar_color = dim_color;
                radar_text = "Source: UNKNOWN";
                break;
        }
        draw_text(fb, 40U, y, radar_text, radar_color, 2U);
    }
    y += 24U;

    /* Presence. */
    draw_text(fb, 40U, y, "Presence:", label_color, 2U);
    if (0U != snap->radar_presence)
    {
        draw_text(fb, 280U, y, "YES", ok_color, 2U);
    }
    else
    {
        draw_text(fb, 280U, y, "NO", dim_color, 2U);
    }
    y += 22U;

    /* Breath rate. */
    draw_text(fb, 40U, y, "Breath RR:", label_color, 2U);
    if (0U != snap->rr_bpm_x10)
    {
        draw_float_1dp(fb, 280U, y, snap->rr_bpm_x10, value_color, 2U);
        draw_text(fb, 380U, y, "bpm", dim_color, 2U);
    }
    else
    {
        draw_text(fb, 280U, y, "N/A", dim_color, 2U);
    }
    y += 22U;

    /* Heart rate. */
    draw_text(fb, 40U, y, "Heart HR:", label_color, 2U);
    if (0U != snap->hr_bpm_x10)
    {
        draw_float_1dp(fb, 280U, y, snap->hr_bpm_x10, value_color, 2U);
        draw_text(fb, 380U, y, "bpm", dim_color, 2U);
    }
    else
    {
        draw_text(fb, 280U, y, "N/A", dim_color, 2U);
    }
    y += 22U;

    /* Distance. */
    draw_text(fb, 40U, y, "Distance:", label_color, 2U);
    if (0U != snap->distance_cm)
    {
        draw_number(fb, 280U, y, snap->distance_cm, value_color, 2U);
        draw_text(fb, 340U, y, "cm", dim_color, 2U);
    }
    else
    {
        draw_text(fb, 280U, y, "N/A", dim_color, 2U);
    }
    y += 22U;

    /* Radar quality. */
    draw_text(fb, 40U, y, "Quality:", label_color, 2U);
    draw_number(fb, 280U, y, snap->radar_quality, value_color, 2U);
    draw_text(fb, 320U, y, "/100", dim_color, 2U);

    /* Separator. */
    y += 30U;
    fill_rect(fb, 16U, y, CM55_DISP_ACTUAL_HOR_RES - 32U, 1U,
              RGB565(40U, 60U, 80U));
    y += 12U;

    /* === Fusion Section === */
    draw_text(fb, 24U, y, "-- Fusion", label_color, 2U);
    y += 24U;

    draw_text(fb, 40U, y, "Confidence:", label_color, 2U);
    draw_number(fb, 280U, y, snap->fusion_confidence, value_color, 2U);
    draw_text(fb, 320U, y, "%", dim_color, 2U);

    /* Footer: disclaimer. */
    fill_rect(fb, 0U, CM55_DISP_VER_RES - 28U,
              CM55_DISP_ACTUAL_HOR_RES, 28U, RGB565(12U, 18U, 24U));
    draw_text(fb, 16U, CM55_DISP_VER_RES - 20U,
              "E84 Health Monitor v1.0", dim_color, 1U);

    /* Seq begin check: reject torn read if CM33 was mid-publish. */
    __DMB();
    seq_begin = snap->seq_begin;
    if (seq_begin != seq_end)
    {
        /* CM33 mid-publish: skip this frame to avoid rendering torn data.
         * The previous framebuffer remains displayed (no swap happened). */
        return;
    }
}
#endif /* APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE */

static uint32_t set_framebuffer(uint16_t *fb)
{
    Cy_GFXSS_Set_FrameBuffer(GFXSS, (uint32_t *)fb, &gfx_context);
    if (0U == ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000U)))
    {
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_FRAMEBUFFER_SET_TIMEOUT, 0u, 0u);
        return 0u;
    }

    CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_FRAMEBUFFER_SET_OK, 0u, 0u);
    return 1u;
}

#if (APP_DISPLAY_PANEL_I2C_SUBDIAG_ENABLE)
static uint32_t cm55_panel_i2c_line_flags(void)
{
    uint32_t flags = 0u;

    if (0u != Cy_GPIO_Read(CYBSP_I2C_SCL_PORT, CYBSP_I2C_SCL_PIN))
    {
        flags |= 0x1u;
    }
    if (0u != Cy_GPIO_Read(CYBSP_I2C_SDA_PORT, CYBSP_I2C_SDA_PIN))
    {
        flags |= 0x2u;
    }
    if (0u != NVIC_GetPendingIRQ(CYBSP_I2C_CONTROLLER_IRQ))
    {
        flags |= 0x10u;
    }
    if (0u != NVIC_GetActive(CYBSP_I2C_CONTROLLER_IRQ))
    {
        flags |= 0x20u;
    }

    return flags;
}

static uint32_t cm55_panel_i2c_hw_snapshot(void)
{
    uint32_t snapshot = 0u;

    snapshot |= ((CYBSP_I2C_CONTROLLER_HW->INTR_M_MASKED & 0xFFu) << 24);
    snapshot |= ((CYBSP_I2C_CONTROLLER_HW->INTR_TX_MASKED & 0xFFu) << 16);
    snapshot |= ((CYBSP_I2C_CONTROLLER_HW->I2C_STATUS & 0xFFu) << 8);
    snapshot |= (CYBSP_I2C_CONTROLLER_HW->INTR_CAUSE & 0xFFu);

    return snapshot;
}

static cy_en_scb_i2c_status_t cm55_panel_i2c_transfer(uint8_t *buffer,
                                                      uint32_t buffer_size,
                                                      uint32_t is_read)
{
    cy_stc_scb_i2c_master_xfer_config_t transfer_config;
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_MASTER_MANUAL_TIMEOUT;
    uint8_t retry_count = CM55_DISP_PANEL_I2C_RETRY_COUNT;
    uint32_t last_controller_status = 0u;
    uint32_t last_transfer_count = 0u;
    uint32_t start_irq_count = display_i2c_irq_count;
    uint32_t last_hw_snapshot = 0u;
    uint32_t last_line_flags = cm55_panel_i2c_line_flags();

    transfer_config.slaveAddress = MTB_DISP_WAVESHARE_4P3_I2C_ADDR;
    transfer_config.buffer = buffer;
    transfer_config.bufferSize = buffer_size;
    transfer_config.xferPending = false;

    do
    {
        uint32_t controller_status = 0u;
        uint32_t timeout_count = CM55_DISP_PANEL_I2C_TIMEOUT_MS;

        if (0u != is_read)
        {
            i2c_status = Cy_SCB_I2C_MasterRead(CYBSP_I2C_CONTROLLER_HW,
                                               &transfer_config,
                                               &display_i2c_context);
        }
        else
        {
            i2c_status = Cy_SCB_I2C_MasterWrite(CYBSP_I2C_CONTROLLER_HW,
                                                &transfer_config,
                                                &display_i2c_context);
        }

        if (CY_SCB_I2C_SUCCESS == i2c_status)
        {
            do
            {
                controller_status =
                    Cy_SCB_I2C_MasterGetStatus(CYBSP_I2C_CONTROLLER_HW,
                                               &display_i2c_context);
                last_controller_status = controller_status;
                last_hw_snapshot = cm55_panel_i2c_hw_snapshot();
                last_line_flags = cm55_panel_i2c_line_flags();
                Cy_SysLib_Delay(CM55_DISP_PANEL_I2C_DELAY_MS);
                timeout_count--;
            } while (((controller_status & CY_SCB_I2C_MASTER_BUSY) != 0u) &&
                     (timeout_count > 0u));

            last_transfer_count =
                Cy_SCB_I2C_MasterGetTransferCount(CYBSP_I2C_CONTROLLER_HW,
                                                  &display_i2c_context);

            if ((controller_status & CY_SCB_I2C_MASTER_BUSY) != 0u)
            {
                i2c_status = CY_SCB_I2C_MASTER_MANUAL_TIMEOUT;
            }
            else if ((controller_status & CM55_DISP_PANEL_I2C_ERROR_MASK) != 0u)
            {
                i2c_status = CY_SCB_I2C_BAD_PARAM;
            }
            else if (Cy_SCB_I2C_MasterGetTransferCount(CYBSP_I2C_CONTROLLER_HW,
                                                       &display_i2c_context) !=
                     buffer_size)
            {
                i2c_status = CY_SCB_I2C_BAD_PARAM;
            }
            else
            {
                retry_count = 0u;
            }
        }

        if (CY_SCB_I2C_SUCCESS != i2c_status)
        {
            Cy_SCB_I2C_Disable(CYBSP_I2C_CONTROLLER_HW, &display_i2c_context);
            Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_HW);
            retry_count--;
        }
    } while ((retry_count > 0u) && (CY_SCB_I2C_SUCCESS != i2c_status));

    app_display_diag_set_detail(last_controller_status, last_transfer_count);
    app_display_diag_set_extra_detail(last_hw_snapshot,
                                      (((display_i2c_irq_count - start_irq_count) & 0xFFFFu) << 16) |
                                      ((last_line_flags & 0xFFu) << 8) |
                                      (retry_count & 0xFFu));
    CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_I2C_TRACE,
                   last_hw_snapshot,
                   (((display_i2c_irq_count - start_irq_count) & 0xFFFFu) << 16) |
                   ((last_line_flags & 0xFFu) << 8) |
                   (retry_count & 0xFFu));

    return i2c_status;
}

static cy_en_scb_i2c_status_t cm55_panel_i2c_write(uint8_t *buffer,
                                                   uint32_t buffer_size)
{
    return cm55_panel_i2c_transfer(buffer, buffer_size, 0u);
}

static cy_en_scb_i2c_status_t cm55_panel_i2c_read(uint8_t *buffer,
                                                  uint32_t buffer_size)
{
    return cm55_panel_i2c_transfer(buffer, buffer_size, 1u);
}

static cy_en_scb_i2c_status_t cm55_panel_i2c_lowlevel_write(uint8_t reg,
                                                            uint8_t value)
{
    cy_en_scb_i2c_status_t status;
    uint32_t start_irq_count = display_i2c_irq_count;
    uint32_t step = 0u;

    Cy_SCB_I2C_Disable(CYBSP_I2C_CONTROLLER_HW, &display_i2c_context);
    Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_HW);

    status = Cy_SCB_I2C_MasterSendStart(CYBSP_I2C_CONTROLLER_HW,
                                        MTB_DISP_WAVESHARE_4P3_I2C_ADDR,
                                        CY_SCB_I2C_WRITE_XFER,
                                        CM55_DISP_PANEL_I2C_TIMEOUT_MS,
                                        &display_i2c_context);
    step = 1u;
    if (CY_SCB_I2C_SUCCESS == status)
    {
        status = Cy_SCB_I2C_MasterWriteByte(CYBSP_I2C_CONTROLLER_HW,
                                            reg,
                                            CM55_DISP_PANEL_I2C_TIMEOUT_MS,
                                            &display_i2c_context);
        step = 2u;
    }
    if (CY_SCB_I2C_SUCCESS == status)
    {
        status = Cy_SCB_I2C_MasterWriteByte(CYBSP_I2C_CONTROLLER_HW,
                                            value,
                                            CM55_DISP_PANEL_I2C_TIMEOUT_MS,
                                            &display_i2c_context);
        step = 3u;
    }

    (void)Cy_SCB_I2C_MasterSendStop(CYBSP_I2C_CONTROLLER_HW,
                                    CM55_DISP_PANEL_I2C_TIMEOUT_MS,
                                    &display_i2c_context);

    app_display_diag_set_detail(
        Cy_SCB_I2C_MasterGetStatus(CYBSP_I2C_CONTROLLER_HW,
                                   &display_i2c_context),
        Cy_SCB_I2C_MasterGetTransferCount(CYBSP_I2C_CONTROLLER_HW,
                                          &display_i2c_context));
    app_display_diag_set_extra_detail(
        cm55_panel_i2c_hw_snapshot(),
        (((display_i2c_irq_count - start_irq_count) & 0xFFFFu) << 16) |
        ((cm55_panel_i2c_line_flags() & 0xFFu) << 8) |
        (step & 0xFFu));

    if (CY_SCB_I2C_SUCCESS == status)
    {
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_I2C_LOWLEVEL_OK,
                       (uint32_t)status,
                       CM55_DISP_PANEL_FLAG(0u, reg, value));
    }
    else
    {
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_I2C_LOWLEVEL_FAIL,
                       (uint32_t)status,
                       CM55_DISP_PANEL_FLAG(step, reg, value));
    }

    return status;
}

static cy_en_scb_i2c_status_t cm55_panel_i2c_init_subdiag(void)
{
    static uint8_t official_cmds[][MTB_DISP_WAVESHARE_4P3_I2C_PKT_SIZE] =
    {
        { MTB_DISP_WAVESHARE_4P3_CTRL_REG, MTB_DISP_WAVESHARE_4P3_DISABLE_CMD },
        { MTB_DISP_WAVESHARE_4P3_CTRL_REG, MTB_DISP_WAVESHARE_4P3_ENABLE_CMD },
        { MTB_DISP_WAVESHARE_4P3_POWERON_REG,
          MTB_DISP_WAVESHARE_4P3_POWERON_CMPLT_CMD },
        { MTB_DISP_WAVESHARE_4P3_BRIGHTNESS_CTRL_REG, 0xFFu },
    };
    static uint8_t local_direct_cmds[][MTB_DISP_WAVESHARE_4P3_I2C_PKT_SIZE] =
    {
        { 0xADu, 0x00u },
        { 0xADu, 0x01u },
        { 0xC0u, 0x01u },
        { 0xABu, 0x00u },
        { 0xAAu, 0x01u },
    };
    uint8_t cmd = MTB_DISP_WAVESHARE_4P3_ID_REG;
    uint8_t device_id = 0u;
    cy_en_scb_i2c_status_t status;

    Cy_SysLib_Delay(100u);

#if (APP_DISPLAY_PANEL_I2C_LOWLEVEL_PROBE_ENABLE)
    status = cm55_panel_i2c_lowlevel_write(0xADu, 0x00u);
    if (CY_SCB_I2C_SUCCESS != status)
    {
        return status;
    }
#endif

    status = cm55_panel_i2c_write(&cmd, sizeof(cmd));
    if (CY_SCB_I2C_SUCCESS != status)
    {
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_ID_REG_WRITE_FAIL,
                       (uint32_t)status,
                       (uint32_t)MTB_DISP_WAVESHARE_4P3_ID_REG);
#if !(APP_DISPLAY_PANEL_I2C_BYPASS_ID_ENABLE)
        return status;
#endif
    }
    else
    {
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_ID_REG_WRITE_OK,
                       (uint32_t)status,
                       (uint32_t)MTB_DISP_WAVESHARE_4P3_ID_REG);

        status = cm55_panel_i2c_read(&device_id,
                                     MTB_DISP_WAVESHARE_4P3_NUM_ID_BYTES);
        if (CY_SCB_I2C_SUCCESS != status)
        {
            CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_ID_READ_FAIL,
                           (uint32_t)status,
                           (uint32_t)device_id);
#if !(APP_DISPLAY_PANEL_I2C_BYPASS_ID_ENABLE)
            return status;
#endif
        }
        else
        {
            CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_ID_READ_OK,
                           (uint32_t)device_id,
                           0u);

            if ((MTB_DISP_WAVESHARE_4P3_ID_1 == device_id) ||
                (MTB_DISP_WAVESHARE_4P3_ID_2 == device_id))
            {
                CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_ID_SUPPORTED,
                               (uint32_t)device_id,
                               0u);

                for (uint32_t index = 0u; index < CY_ARRAY_SIZE(official_cmds); ++index)
                {
                    status = cm55_panel_i2c_write(official_cmds[index],
                                                  MTB_DISP_WAVESHARE_4P3_I2C_PKT_SIZE);
                    if (CY_SCB_I2C_SUCCESS != status)
                    {
                        CM55_DISP_DIAG(
                            APP_DISPLAY_DIAG_STAGE_PANEL_OFFICIAL_CMD_FAIL,
                            (uint32_t)status,
                            CM55_DISP_PANEL_FLAG(index,
                                                 official_cmds[index][0],
                                                 official_cmds[index][1]));
                        return status;
                    }
                    CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_OFFICIAL_CMD_OK,
                                   (uint32_t)status,
                                   CM55_DISP_PANEL_FLAG(index,
                                                        official_cmds[index][0],
                                                        official_cmds[index][1]));
                    Cy_SysLib_Delay(100u);
                }

                return CY_SCB_I2C_SUCCESS;
            }

            CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_ID_UNSUPPORTED,
                           (uint32_t)device_id,
                           (((uint32_t)MTB_DISP_WAVESHARE_4P3_ID_1 << 8) |
                            (uint32_t)MTB_DISP_WAVESHARE_4P3_ID_2));
        }
    }

#if (APP_DISPLAY_PANEL_I2C_BYPASS_ID_ENABLE)
    for (uint32_t index = 0u; index < CY_ARRAY_SIZE(local_direct_cmds); ++index)
    {
        status = cm55_panel_i2c_write(local_direct_cmds[index],
                                      MTB_DISP_WAVESHARE_4P3_I2C_PKT_SIZE);
        if (CY_SCB_I2C_SUCCESS != status)
        {
            uint32_t cmd_flags = CM55_DISP_PANEL_FLAG(index,
                                                      local_direct_cmds[index][0],
                                                      local_direct_cmds[index][1]);
            CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_DIRECT_CMD_FAIL,
                           (uint32_t)status,
                           cmd_flags);
            CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_DIRECT_INIT_FAIL,
                           (uint32_t)status,
                           cmd_flags);
            return status;
        }
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_DIRECT_CMD_OK,
                       (uint32_t)status,
                       CM55_DISP_PANEL_FLAG(index,
                                            local_direct_cmds[index][0],
                                            local_direct_cmds[index][1]));
        Cy_SysLib_Delay(100u);
    }

    CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_DIRECT_INIT_OK,
                   (uint32_t)device_id,
                   (uint32_t)CY_ARRAY_SIZE(local_direct_cmds));
    return CY_SCB_I2C_SUCCESS;
#else
    return CY_SCB_I2C_BAD_PARAM;
#endif
}
#endif /* APP_DISPLAY_PANEL_I2C_SUBDIAG_ENABLE */

static void cm55_gfx_task(void *arg)
{
    uint32_t tick_count = 0U;
    uint16_t *active_fb = frame_buffer1;
    cy_en_sysint_status_t sysint_status;
    cy_en_gfx_status_t gfx_status;
    cy_en_scb_i2c_status_t i2c_status;
    vg_lite_error_t vglite_status;

    (void)arg;

    cm55_gfx_task_handle = xTaskGetCurrentTaskHandle();

    CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_DISPLAY_TASK_START, 0u, 0u);
    CM55_DISP_LOG("[CM55_DISP] task start\r\n");

    memset(frame_buffer1, 0, sizeof(frame_buffer1));
    memset(frame_buffer2, 0, sizeof(frame_buffer2));
#if (APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE)
    draw_live_frame(frame_buffer1);
    draw_live_frame(frame_buffer2);
#else
    draw_proof_frame(frame_buffer1, 0U);
    draw_proof_frame(frame_buffer2, 1U);
#endif
    CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_FRAMEBUFFER_DRAWN, 0u, 0u);

    GFXSS_config.mipi_dsi_cfg = &mtb_disp_waveshare_4p3_dsi_config;
    GFXSS_config.dc_cfg->gfx_layer_config->width = CM55_DISP_HOR_RES;
    GFXSS_config.dc_cfg->gfx_layer_config->height = CM55_DISP_VER_RES;
    GFXSS_config.dc_cfg->display_width = CM55_DISP_HOR_RES;
    GFXSS_config.dc_cfg->display_height = CM55_DISP_VER_RES;
    GFXSS_config.dc_cfg->gfx_layer_config->buffer_address = (gctADDRESS *)frame_buffer1;
    GFXSS_config.dc_cfg->gfx_layer_config->uv_buffer_address = (gctADDRESS *)frame_buffer1;

    CM55_DISP_LOG("[CM55_DISP] step01 gfxss_init\r\n");
    gfx_status = Cy_GFXSS_Init(GFXSS, &GFXSS_config, &gfx_context);
    CM55_DISP_LOG("[CM55_DISP] step01 gfxss_init result=%ld\r\n",
                  (long)gfx_status);
    if (CY_GFX_SUCCESS != gfx_status)
    {
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_GFXSS_INIT_FAIL,
                       (uint32_t)gfx_status,
                       0u);
        CY_ASSERT(0);
    }
    CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_GFXSS_INIT_OK,
                   (uint32_t)gfx_status,
                   0u);

    CM55_DISP_LOG("[CM55_DISP] step02 irq_init\r\n");
    sysint_status = Cy_SysInt_Init(&dc_irq_cfg, dc_irq_handler);
    if (CY_SYSINT_SUCCESS != sysint_status)
    {
        CM55_DISP_LOG("[CM55_DISP] FAIL: dc_irq result=%ld\r\n",
                      (long)sysint_status);
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_IRQ_INIT_FAIL,
                       (uint32_t)sysint_status,
                       1u);
        CY_ASSERT(0);
    }
    NVIC_EnableIRQ(GFXSS_DC_IRQ);

    sysint_status = Cy_SysInt_Init(&gpu_irq_cfg, gpu_irq_handler);
    if (CY_SYSINT_SUCCESS != sysint_status)
    {
        CM55_DISP_LOG("[CM55_DISP] FAIL: gpu_irq result=%ld\r\n",
                      (long)sysint_status);
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_IRQ_INIT_FAIL,
                       (uint32_t)sysint_status,
                       2u);
        CY_ASSERT(0);
    }
    Cy_GFXSS_Enable_GPU_Interrupt(GFXSS);
    NVIC_EnableIRQ(GFXSS_GPU_IRQ);
    CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_IRQ_INIT_OK, 0u, 0u);

    CM55_DISP_LOG("[CM55_DISP] step03 i2c_init\r\n");
    i2c_status = Cy_SCB_I2C_Init(CYBSP_I2C_CONTROLLER_HW,
                                 &CYBSP_I2C_CONTROLLER_config,
                                 &display_i2c_context);
    CM55_DISP_LOG("[CM55_DISP] step03 i2c_init result=%ld\r\n",
                  (long)i2c_status);
    if (CY_SCB_I2C_SUCCESS != i2c_status)
    {
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_I2C_INIT_FAIL,
                       (uint32_t)i2c_status,
                       0u);
        CY_ASSERT(0);
    }
    CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_I2C_INIT_OK,
                   (uint32_t)i2c_status,
                   0u);

    sysint_status = Cy_SysInt_Init(&display_i2c_irq_cfg,
                                   display_i2c_irq_handler);
    if (CY_SYSINT_SUCCESS != sysint_status)
    {
        CM55_DISP_LOG("[CM55_DISP] FAIL: i2c_irq result=%ld\r\n",
                      (long)sysint_status);
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_IRQ_INIT_FAIL,
                       (uint32_t)sysint_status,
                       3u);
        CY_ASSERT(0);
    }
    NVIC_EnableIRQ(CYBSP_I2C_CONTROLLER_IRQ);
    Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_HW);

    CM55_DISP_LOG("[CM55_DISP] step04 panel_delay_500ms\r\n");
    vTaskDelay(pdMS_TO_TICKS(500U));

    CM55_DISP_LOG("[CM55_DISP] step05 panel_init\r\n");
#if (APP_DISPLAY_PANEL_I2C_SUBDIAG_ENABLE)
    i2c_status = cm55_panel_i2c_init_subdiag();
#else
    i2c_status = mtb_disp_waveshare_4p3_init(CYBSP_I2C_CONTROLLER_HW,
                                             &display_i2c_context);
#endif
    CM55_DISP_LOG("[CM55_DISP] step05 panel_init result=%ld\r\n",
                  (long)i2c_status);
    if (CY_SCB_I2C_SUCCESS != i2c_status)
    {
#if !(APP_DISPLAY_PANEL_I2C_SUBDIAG_ENABLE)
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_INIT_FAIL,
                       (uint32_t)i2c_status,
                       0u);
#endif
        CY_ASSERT(0);
    }
    CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_PANEL_INIT_OK,
                   (uint32_t)i2c_status,
                   0u);

    CM55_DISP_LOG("[CM55_DISP] step06 vglite_init\r\n");
    vg_params.register_mem_base = (uint32_t)GFXSS_GFXSS_GPU_GCNANO;
    vg_params.gpu_mem_base[CM55_DISP_VG_PARAMS_POS] = CM55_DISP_GPU_MEM_BASE;
    vg_params.contiguous_mem_base[CM55_DISP_VG_PARAMS_POS] =
        (void *)vglite_heap_base;
    vg_params.contiguous_mem_size[CM55_DISP_VG_PARAMS_POS] =
        CM55_DISP_VGLITE_HEAP_SIZE;
    vg_lite_init_mem(&vg_params);
    vglite_status = vg_lite_init(CM55_DISP_HOR_RES, CM55_DISP_VER_RES);
    CM55_DISP_LOG("[CM55_DISP] step06 vglite_init result=%ld\r\n",
                  (long)vglite_status);
    if (VG_LITE_SUCCESS != vglite_status)
    {
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_VGLITE_INIT_FAIL,
                       (uint32_t)vglite_status,
                       0u);
        CY_ASSERT(0);
    }
    CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_VGLITE_INIT_OK,
                   (uint32_t)vglite_status,
                   0u);

    if (0U != set_framebuffer(active_fb))
    {
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_DISPLAY_READY, 0u, 0u);
    }
#if (APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE)
    CM55_DISP_LOG("[CM55_DISP] READY bridge_mode=1\r\n");
#else
    CM55_DISP_LOG("[CM55_DISP] READY proof_framebuffer=1\r\n");
#endif

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000U));
        ++tick_count;
        active_fb = (active_fb == frame_buffer1) ? frame_buffer2 : frame_buffer1;

#if (APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE)
        /* Read bridge and render live data. */
        draw_live_frame(active_fb);
#else
        /* Proof mode: alternate between two proof frames. */
        draw_proof_frame(active_fb, (tick_count & 1U));
#endif

        if (0U != set_framebuffer(active_fb))
        {
            CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_DISPLAY_READY, 0u, 0u);
        }

        if ((tick_count % 10U) == 0U)
        {
            CM55_DISP_LOG("[CM55_DISP] tick=%lu\r\n",
                          (unsigned long)tick_count);
        }
    }
}

cy_rslt_t app_cm55_display_bringup_init(void)
{
    BaseType_t ret;

    CM55_DISP_LOG("[CM55_DISP] init display_only_task\r\n");

    ret = xTaskCreate(cm55_gfx_task,
                      CM55_DISP_TASK_NAME,
                      CM55_DISP_TASK_STACK,
                      NULL,
                      CM55_DISP_TASK_PRIORITY,
                      &cm55_gfx_task_handle);
    if (pdPASS != ret)
    {
        CM55_DISP_LOG("[CM55_DISP] FAIL: task_create\r\n");
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_DISPLAY_TASK_CREATE_FAIL,
                       (uint32_t)ret,
                       0u);
        return CY_RSLT_TYPE_ERROR;
    }

    CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_DISPLAY_TASK_CREATE_OK,
                   (uint32_t)ret,
                   0u);
    return CY_RSLT_SUCCESS;
}

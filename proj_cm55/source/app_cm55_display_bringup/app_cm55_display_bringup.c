/*******************************************************************************
* File Name : app_cm55_display_bringup.c
*
* Description : CM55 official-like display proof path for Waveshare 4.3" DSI.
*******************************************************************************/

#include "app_cm55_display_bringup.h"

#include "app_build_config.h"
#include "app_display_diag.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cy_graphics.h"
#include "cy_scb_i2c.h"
#include "cycfg.h"
#include "mtb_disp_dsi_waveshare_4p3.h"
#include "vg_lite.h"
#include "vg_lite_platform.h"

#if (APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE) || (APP_DISPLAY_LVGL_ENABLE)
#include "app_display_cm55_shared.h"
#endif

#if (APP_DISPLAY_LVGL_ENABLE)
#include "lvgl.h"
#include "lv_port_disp.h"
#include "ui_health_dashboard.h"
#endif

#include <stdio.h>
#include <string.h>

#pragma GCC optimize ("no-tree-vectorize")

#ifndef APP_DISPLAY_LVGL_DATA_UPDATE_TICKS
#define APP_DISPLAY_LVGL_DATA_UPDATE_TICKS (30U)
#endif

#if (APP_DISPLAY_LVGL_DATA_UPDATE_TICKS < 1)
#error "APP_DISPLAY_LVGL_DATA_UPDATE_TICKS must be >= 1"
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

/* === 16x16 Chinese bitmap font === */
/* Minimal font for health-monitor UI. Each glyph is 32 bytes (16 rows x 2 bytes). */
typedef struct { uint16_t code; const uint8_t bitmap[32]; } cn_glyph_t;

static const cn_glyph_t cn_font[] =
{
{0x5065, {0x10,0x20,0x18,0xFC,0x2E,0x24,0x22,0x24,0x24,0xFE,0x64,0x24,0x67,0xFC,0x22,0x20,0x22,0xFC,0x2A,0x20,0x26,0xFE,0x26,0x20,0x27,0x20,0x28,0xFE,0x00,0x00,0x00,0x00}},
    {0x6CE2, {0x00,0x00,0x00,0x20,0x30,0x20,0x1B,0xFE,0x02,0x24,0x42,0x24,0x72,0x20,0x13,0xFC,0x02,0x88,0x1A,0x88,0x12,0xD8,0x34,0x50,0x64,0x20,0x4C,0xD8,0x0B,0x8E,0x02,0x00}},
    {0x4E0D, {0x00,0x00,0x7F,0xFE,0x00,0x80,0x01,0x00,0x03,0x00,0x07,0x60,0x05,0x30,0x19,0x18,0x31,0x0C,0x61,0x04,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00}},
    {0x6D4B, {0x00,0x00,0x00,0x06,0x77,0xC6,0x14,0x56,0x04,0x56,0x45,0x56,0x65,0x56,0x25,0x56,0x05,0x56,0x15,0x56,0x15,0x56,0x23,0x96,0x22,0x86,0x46,0x46,0x4C,0x4E,0x00,0x00}},
    {0x5E38, {0x00,0x00,0x01,0x80,0x09,0x90,0x09,0x90,0x7F,0xFE,0x60,0x02,0x6F,0xF2,0x08,0x10,0x0F,0xF0,0x01,0x80,0x3F,0xFC,0x3F,0xFC,0x31,0x84,0x31,0x8C,0x31,0x98,0x01,0x80}},
    {0x521D, {0x18,0x00,0x09,0xFE,0x01,0xFE,0x7C,0x26,0x04,0x66,0x08,0x64,0x1B,0x64,0x1E,0x44,0x3E,0x44,0x5A,0x44,0x18,0xC4,0x18,0x84,0x19,0x84,0x1B,0x1C,0x18,0x00,0x00,0x00}},
    {0x8FBE, {0x20,0x60,0x30,0x60,0x18,0x40,0x00,0x40,0x07,0xFE,0x00,0x40,0x70,0x40,0x10,0xD0,0x10,0x98,0x11,0x8C,0x13,0x04,0x1A,0x04,0x2C,0x00,0x43,0xFE,0x00,0x00,0x00,0x00}},
    {0x5F85, {0x08,0x60,0x18,0x60,0x13,0xFC,0x20,0x60,0x4C,0x60,0x0B,0xFE,0x10,0x08,0x30,0x08,0x77,0xFE,0x50,0x08,0x11,0x88,0x10,0x88,0x10,0x18,0x10,0x38,0x10,0x00,0x00,0x00}},
    {0x5F53, {0x00,0x00,0x01,0x80,0x11,0x84,0x19,0x8C,0x09,0x98,0x09,0x90,0x01,0x80,0x3F,0xFC,0x00,0x04,0x00,0x04,0x3F,0xFC,0x00,0x04,0x00,0x04,0x3F,0xFC,0x00,0x04,0x00,0x04}},
    {0x5230, {0x00,0x00,0x00,0x06,0x7F,0x86,0x08,0x26,0x19,0x26,0x11,0x26,0x67,0xA6,0x7C,0xA6,0x0C,0x26,0x0C,0x26,0x7F,0xA6,0x0C,0x26,0x0C,0x06,0x7F,0x86,0x60,0x1C,0x00,0x08}},
    {0x7B49, {0x10,0x20,0x3F,0x7E,0x64,0xC8,0x45,0x8C,0x1F,0xF8,0x01,0x80,0x01,0x80,0xFF,0xFE,0x00,0x30,0x00,0x30,0x3F,0xFC,0x08,0x30,0x04,0x30,0x00,0x70,0x00,0x00,0x00,0x00}},
    {0x52A8, {0x00,0x00,0x00,0x30,0x00,0x30,0x3E,0x30,0x00,0x30,0x00,0xFE,0x7F,0x32,0x7F,0x22,0x10,0x22,0x16,0x22,0x22,0x22,0x23,0x62,0x7D,0x46,0x00,0xC6,0x01,0x9C,0x00,0x00}},
    {0x65AD, {0x00,0x00,0x04,0x00,0x74,0xFE,0x75,0x20,0x7D,0x20,0x64,0x20,0x7F,0xBE,0x64,0x24,0x6D,0x24,0x6D,0xA4,0x74,0xE4,0x64,0x44,0x60,0x44,0x7F,0x44,0x00,0x84,0x00,0x00}},
    {0x5206, {0x00,0x00,0x04,0x40,0x04,0x60,0x0C,0x20,0x08,0x30,0x18,0x18,0x30,0x0E,0x6F,0xFE,0x02,0x10,0x02,0x10,0x02,0x10,0x06,0x10,0x04,0x10,0x18,0x30,0x30,0x60,0x20,0x00}},
    {0x53F7, {0x00,0x00,0x0F,0xF8,0x08,0x18,0x08,0x18,0x0F,0xF8,0x00,0x00,0x7F,0xFE,0x04,0x00,0x04,0x00,0x0F,0xF8,0x00,0x18,0x00,0x18,0x00,0x10,0x00,0xF0,0x00,0x80,0x00,0x00}},
    {0x547C, {0x00,0x00,0x00,0x04,0x7B,0xFE,0x48,0x20,0x49,0x24,0x49,0x24,0x49,0x24,0x49,0x24,0x48,0x20,0x7B,0xFE,0x78,0x20,0x48,0x20,0x40,0x20,0x00,0x20,0x00,0xE0,0x00,0x80}},
    {0x59CB, {0x10,0x60,0x10,0x40,0x10,0x40,0x7C,0x88,0x25,0x84,0x25,0xFE,0x24,0x00,0x24,0x00,0x69,0xFC,0x39,0x84,0x19,0x84,0x15,0x84,0x31,0xFC,0x61,0x84,0x01,0x84,0x00,0x00}},
    {0x5316, {0x00,0x00,0x08,0xC0,0x0C,0xC0,0x08,0xC4,0x18,0xC4,0x18,0xCC,0x38,0xD8,0x78,0xE0,0x18,0xC0,0x19,0xC0,0x1F,0xC0,0x18,0xC0,0x18,0xC2,0x18,0xC2,0x18,0x7E,0x00,0x00}},
    {0x95F4, {0x10,0x00,0x19,0xFC,0x08,0x04,0x60,0x04,0x67,0xE4,0x64,0x24,0x64,0x24,0x67,0xE4,0x64,0x24,0x64,0x24,0x67,0xE4,0x60,0x04,0x60,0x04,0x60,0x0C,0x00,0x00,0x00,0x00}},
    {0x68C0, {0x00,0x00,0x10,0x60,0x10,0x60,0x10,0xF0,0x7C,0x90,0x11,0x0C,0x13,0xF6,0x32,0x00,0x3D,0x44,0x77,0x6C,0x51,0x28,0x51,0xA8,0x10,0x10,0x14,0x10,0x17,0xFE,0x10,0x00}},
    {0x4EF6, {0x08,0x20,0x08,0x20,0x13,0x20,0x32,0x20,0x33,0xFC,0x76,0x20,0x54,0x20,0x10,0x20,0x17,0xFE,0x10,0x20,0x10,0x20,0x10,0x20,0x10,0x20,0x10,0x20,0x00,0x00,0x00,0x00}},
    {0x4ECA, {0x01,0x80,0x01,0x80,0x02,0x40,0x06,0x60,0x0C,0x30,0x19,0x18,0x31,0x8E,0x40,0x82,0x1F,0xF8,0x00,0x18,0x00,0x30,0x00,0x20,0x00,0x40,0x00,0xC0,0x00,0x80,0x00,0x00}},
    {0x8FDB, {0x00,0x00,0x00,0x90,0x30,0x90,0x10,0x90,0x07,0xFE,0x00,0x90,0x70,0x90,0x10,0x90,0x17,0xFE,0x11,0x10,0x11,0x10,0x13,0x10,0x12,0x10,0x7E,0x00,0x43,0xFE,0x00,0x00}},
    {0x636E, {0x00,0x00,0x10,0x00,0x11,0xFC,0x11,0x04,0x11,0x04,0x7D,0xFC,0x11,0x10,0x15,0x10,0x19,0xFE,0x71,0x10,0x53,0xFC,0x12,0x84,0x12,0x84,0x12,0x84,0x34,0xFC,0x00,0x80}},
    {0x5361, {0x00,0x00,0x01,0x00,0x01,0x00,0x01,0xFC,0x01,0x04,0x01,0x00,0x01,0x00,0x7F,0xFE,0x01,0x00,0x01,0x40,0x01,0x70,0x01,0x18,0x01,0x08,0x01,0x00,0x01,0x00,0x01,0x00}},
    {0x54B3, {0x00,0x60,0x00,0x20,0x7B,0xFE,0x48,0x40,0x48,0xC0,0x48,0x98,0x4B,0xF0,0x49,0x34,0x48,0x64,0x78,0xCC,0x4B,0x98,0x40,0x38,0x00,0xE4,0x03,0x82,0x02,0x00,0x00,0x00}},
    {0x53EF, {0x00,0x00,0x00,0x00,0x7F,0xFE,0x00,0x18,0x00,0x18,0x1F,0x98,0x10,0x98,0x10,0x98,0x10,0x98,0x10,0x98,0x1F,0x98,0x10,0x98,0x10,0x18,0x00,0x18,0x00,0x78,0x00,0x60}},
    {0x76D1, {0x02,0x20,0x32,0x60,0x32,0x60,0x32,0x7E,0x32,0xD0,0x32,0x98,0x32,0x8C,0x02,0x00,0x00,0x00,0x3F,0xFC,0x32,0x44,0x32,0x44,0x32,0x44,0x7F,0xFE,0x00,0x00,0x00,0x00}},
    {0x96F7, {0x00,0x00,0x3F,0xFC,0x01,0x80,0x7F,0xFE,0x41,0x82,0x5D,0xBA,0x01,0x80,0x1D,0xB8,0x00,0x00,0x1F,0xF8,0x11,0x88,0x1F,0xF8,0x11,0x88,0x11,0x88,0x1F,0xF8,0x10,0x08}},
    {0x7387, {0x00,0x00,0x01,0x00,0x01,0x80,0x3F,0xFE,0x23,0x04,0x32,0x6C,0x17,0x80,0x01,0x00,0x1A,0x6C,0x67,0xF4,0x01,0x80,0x7F,0xFE,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80}},
    {0x7720, {0x00,0x00,0x03,0xFC,0x7B,0x04,0x4B,0x04,0x4B,0x04,0x7B,0xFC,0x4B,0x20,0x4B,0x20,0x7B,0xFE,0x4B,0x22,0x4B,0x10,0x4B,0x10,0x7B,0x50,0x4B,0xCA,0x03,0x0E,0x00,0x00}},
    {0x660E, {0x00,0x00,0x7C,0xFC,0x64,0xC4,0x64,0xC4,0x64,0xFC,0x7C,0xC4,0x64,0xC4,0x64,0xC4,0x64,0xFC,0x7C,0x84,0x64,0x84,0x60,0x84,0x01,0x84,0x03,0x0C,0x00,0x00,0x00,0x00}},
    {0x6A21, {0x10,0xD8,0x13,0xFE,0x10,0xD8,0x7C,0x00,0x11,0xFC,0x11,0x04,0x39,0xFC,0x35,0x04,0x75,0xFC,0x50,0x20,0x13,0xFE,0x10,0x50,0x10,0xC8,0x17,0x07,0x00,0x02,0x00,0x00}},
    {0x5185, {0x00,0x00,0x01,0x80,0x01,0x80,0x01,0x80,0x3F,0xFC,0x21,0x04,0x21,0x04,0x21,0x84,0x23,0x64,0x26,0x34,0x2C,0x14,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x1C,0x00,0x00}},
    {0x5E73, {0x00,0x00,0x3F,0xFC,0x01,0x80,0x11,0x88,0x19,0x98,0x0D,0x90,0x05,0xA0,0x01,0x80,0x7F,0xFE,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x00,0x00}},
    {0x52BF, {0x00,0x00,0x08,0x40,0x08,0x40,0x7D,0xF8,0x08,0x48,0x1D,0xC8,0x68,0xE8,0x09,0x8A,0x3B,0x0E,0x21,0x04,0x3F,0xF8,0x03,0x08,0x02,0x08,0x0C,0x08,0x78,0x70,0x00,0x00}},
    {0x524D, {0x0C,0x10,0x06,0x20,0x7F,0xFE,0x00,0x00,0x00,0x04,0x3F,0x64,0x21,0x64,0x3F,0x64,0x21,0x64,0x21,0x64,0x3F,0x64,0x21,0x64,0x21,0x04,0x27,0x1C,0x20,0x10,0x00,0x00}},
    {0x8F7B, {0x08,0x00,0x18,0xFC,0x7E,0x08,0x10,0x10,0x10,0x38,0x2C,0x6C,0x2D,0xC2,0x7E,0x00,0x0C,0xFC,0x0C,0x10,0x3E,0x10,0x6C,0x10,0x0C,0x10,0x0D,0xFE,0x0C,0x00,0x00,0x00}},
    {0x8D8B, {0x08,0x20,0x08,0x60,0x7E,0x7C,0x08,0x88,0x09,0x88,0x08,0x7E,0x7F,0x06,0x08,0x06,0x28,0x7E,0x2E,0x06,0x28,0x06,0x38,0xFE,0x4C,0x00,0x43,0xFE,0x00,0x00,0x00,0x00}},
    {0x4EBA, {0x00,0x00,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x03,0xC0,0x02,0x40,0x06,0x60,0x04,0x30,0x08,0x18,0x30,0x0C,0x60,0x06,0x00,0x00}},
    {0x65F6, {0x00,0x08,0x00,0x08,0x7C,0x08,0x65,0xFE,0x65,0xFE,0x64,0x08,0x7C,0x88,0x64,0xC8,0x64,0x48,0x64,0x08,0x7C,0x08,0x64,0x08,0x60,0x08,0x00,0x38,0x00,0x00,0x00,0x00}},
    {0x5B9E, {0x01,0x00,0x01,0x80,0x3F,0xFC,0x20,0x04,0x20,0x44,0x06,0x40,0x10,0x40,0x18,0x40,0x00,0x40,0x7F,0xFE,0x00,0x80,0x01,0x20,0x06,0x38,0x7C,0x0E,0x20,0x00,0x00,0x00}},
    {0x4E8B, {0x00,0x00,0x01,0x80,0x7F,0xFE,0x01,0x80,0x1F,0xF8,0x11,0x88,0x1F,0xF8,0x01,0x80,0x3F,0xF8,0x01,0x88,0x7F,0xFE,0x01,0x88,0x3F,0xF8,0x01,0x88,0x03,0x00,0x02,0x00}},
    {0x6570, {0x00,0x00,0x0C,0x20,0x2D,0x20,0x0F,0x60,0x7F,0xFE,0x1C,0x44,0x3E,0x44,0x6D,0xCC,0x08,0xC8,0x7F,0x28,0x11,0x28,0x32,0x38,0x0E,0x10,0x0D,0x68,0x30,0xC6,0x40,0x00}},
    {0x7761, {0x00,0x00,0x00,0x04,0x7B,0xFE,0x48,0x20,0x4B,0xFE,0x79,0x2C,0x79,0x2C,0x49,0x2C,0x4F,0xFE,0x79,0x2C,0x49,0x2C,0x4B,0xFE,0x78,0x20,0x48,0x20,0x43,0xFE,0x00,0x00}},
    {0x55FD, {0x01,0x18,0x71,0x10,0x57,0xD0,0x51,0x1E,0x57,0xF6,0x55,0x60,0x55,0x48,0x57,0xC8,0x51,0x08,0x73,0x98,0x57,0x58,0x45,0x14,0x09,0x34,0x01,0x62,0x00,0x00,0x00,0x00}},
    {0x6001, {0x01,0x00,0x01,0x00,0x7F,0xFE,0x02,0x40,0x06,0x60,0x04,0x20,0x1B,0x18,0x31,0x0E,0x60,0x00,0x00,0x80,0x24,0x8C,0x24,0x06,0x64,0x12,0x47,0xF0,0x00,0x00,0x00,0x00}},
    {0x4F53, {0x00,0x00,0x00,0x40,0x18,0x40,0x10,0x40,0x17,0xFE,0x30,0xE0,0x30,0xD0,0x51,0xD0,0x51,0x58,0x13,0x48,0x16,0x46,0x1D,0xFA,0x10,0x40,0x10,0x40,0x10,0x40,0x00,0x00}},
    {0x5FAE, {0x00,0x00,0x11,0x18,0x35,0x50,0x65,0x50,0x45,0x5E,0x17,0xE4,0x30,0x24,0x2F,0xA4,0x60,0x14,0x67,0x94,0x24,0x98,0x24,0xD8,0x24,0xD8,0x2D,0x3C,0x20,0xC6,0x00,0x00}},
    {0x665A, {0x00,0xC0,0x78,0x80,0x49,0xF8,0x4A,0x10,0x4A,0x30,0x4B,0xFC,0x7B,0x24,0x4B,0x24,0x4B,0xFC,0x48,0x50,0x78,0xD0,0x48,0x90,0x41,0x12,0x06,0x1E,0x00,0x00,0x00,0x00}},
    {0x7A33, {0x00,0xC0,0x7C,0xF8,0x11,0x18,0x13,0x10,0x12,0xFC,0x7C,0x04,0x31,0xFC,0x38,0x04,0x35,0xFC,0x54,0x00,0x50,0xA0,0x12,0xA6,0x12,0x8A,0x10,0xF8,0x10,0x00,0x00,0x00}},
    {0x65E0, {0x00,0x00,0x00,0x00,0x1F,0xF8,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x7F,0xFE,0x03,0x40,0x02,0x40,0x02,0x40,0x06,0x40,0x0C,0x42,0x18,0x46,0x70,0x7C,0x00,0x00}},
    {0x5438, {0x00,0x00,0x7F,0xF8,0x64,0x88,0x64,0x88,0x64,0x90,0x64,0x9C,0x65,0xC4,0x65,0x44,0x65,0x48,0x7D,0x28,0x65,0x38,0x62,0x10,0x66,0x6C,0x0C,0xC6,0x00,0x80,0x00,0x00}},
    {0x663E, {0x00,0x00,0x1F,0xF8,0x10,0x08,0x1F,0xF8,0x1F,0xF8,0x10,0x08,0x1F,0xF8,0x06,0x40,0x26,0x40,0x36,0x44,0x16,0x48,0x1E,0x58,0x06,0x40,0x7F,0xFE,0x00,0x00,0x00,0x00}},
    {0x5FC3, {0x01,0x00,0x01,0x80,0x00,0x80,0x00,0x80,0x06,0x00,0x36,0x08,0x26,0x0C,0x26,0x06,0x66,0x02,0x66,0x00,0x46,0x00,0x06,0x08,0x06,0x18,0x03,0xF0,0x00,0x00,0x00,0x00}},
    {0x4FE1, {0x00,0x00,0x08,0x40,0x18,0x60,0x17,0xFE,0x10,0x00,0x33,0xFC,0x30,0x00,0x50,0x00,0x53,0xFC,0x10,0x00,0x13,0xFC,0x13,0x04,0x13,0x04,0x13,0xFC,0x13,0x04,0x10,0x00}},
    {0x578B, {0x00,0x00,0x00,0x04,0x3F,0xA4,0x09,0x24,0x09,0x24,0x7F,0xE4,0x7F,0xE4,0x11,0x24,0x31,0x04,0x61,0x0C,0x01,0x80,0x1F,0xFC,0x01,0x80,0x01,0x80,0x7F,0xFE,0x00,0x00}},
    {0x884C, {0x18,0x00,0x11,0xFC,0x30,0x00,0x60,0x00,0x4C,0x00,0x1B,0xFE,0x10,0x10,0x30,0x10,0x70,0x10,0x50,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x70,0x10,0x20,0x00,0x00}},
    {0x591C, {0x01,0x80,0x01,0x80,0x7F,0xFE,0x08,0x80,0x09,0x80,0x19,0xFC,0x11,0x08,0x32,0x48,0x77,0x78,0x51,0x90,0x10,0xB0,0x10,0xE0,0x10,0xF0,0x17,0x1E,0x10,0x00,0x00,0x00}},
    {0x5DF2, {0x00,0x00,0x00,0x00,0x3F,0xF8,0x00,0x08,0x00,0x08,0x30,0x08,0x30,0x08,0x3F,0xF8,0x30,0x08,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x06,0x30,0x06,0x1F,0xFC,0x00,0x00}},
    {0x5F02, {0x00,0x00,0x1F,0xF8,0x10,0x08,0x10,0x08,0x1F,0xF8,0x10,0x00,0x10,0x04,0x1F,0xFC,0x00,0x00,0x04,0x20,0x7F,0xFE,0x04,0x20,0x0C,0x20,0x18,0x20,0x70,0x20,0x00,0x00}},
    {0x7528, {0x00,0x00,0x1F,0xFC,0x11,0x8C,0x11,0x8C,0x1F,0xFC,0x11,0x8C,0x11,0x8C,0x11,0x8C,0x1F,0xFC,0x31,0x8C,0x31,0x8C,0x21,0x8C,0x21,0x8C,0x61,0x98,0x00,0x00,0x00,0x00}},
    {0x6682, {0x00,0x00,0x08,0x04,0x08,0x7C,0x37,0x40,0x34,0x7C,0x3F,0x58,0x04,0x58,0x7F,0xD8,0x04,0x98,0x00,0x00,0x1F,0xF8,0x10,0x08,0x1F,0xF8,0x10,0x08,0x1F,0xF8,0x00,0x00}},
    {0x6574, {0x00,0x00,0x08,0x30,0x7F,0xA0,0x08,0x3E,0x7F,0x44,0x69,0x6C,0x7F,0x38,0x1E,0x38,0x6A,0x66,0x40,0x00,0x3F,0xFC,0x01,0x80,0x09,0xF8,0x09,0x80,0x7F,0xFE,0x00,0x00}},
    {0x6B63, {0x00,0x00,0x00,0x00,0x3F,0xFC,0x3F,0xFC,0x00,0x80,0x00,0x80,0x18,0x80,0x18,0xFC,0x18,0xFC,0x18,0x80,0x18,0x80,0x18,0x80,0x18,0x80,0x18,0x80,0x7F,0xFE,0x00,0x00}},
    {0x4E2D, {0x00,0x00,0x01,0x80,0x01,0x80,0x01,0x80,0x3F,0xFC,0x21,0x84,0x21,0x84,0x21,0x84,0x21,0x84,0x3F,0xFC,0x21,0x84,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80}},
    {0x949F, {0x00,0x00,0x10,0x20,0x10,0x20,0x30,0x20,0x3D,0xFE,0x61,0x26,0x7D,0x26,0x11,0x26,0x11,0x26,0x7D,0x26,0x11,0xFE,0x11,0x26,0x14,0x20,0x1C,0x20,0x10,0x20,0x00,0x30}},
    {0x72B6, {0x00,0x00,0x0C,0x60,0x0C,0x28,0x4C,0x2C,0x6C,0x24,0x2C,0x20,0x0F,0xFE,0x0C,0x60,0x1C,0x60,0x3C,0x70,0x6C,0x50,0x4C,0xD0,0x0C,0x88,0x0D,0x8C,0x0F,0x06,0x0C,0x00}},
    {0x5EB7, {0x00,0x80,0x1F,0xFE,0x10,0x80,0x10,0x80,0x17,0xF8,0x10,0x88,0x1F,0xFE,0x10,0x88,0x37,0xF8,0x3C,0xCC,0x27,0xB8,0x22,0x90,0x6C,0x8C,0x49,0x86,0x00,0x00,0x00,0x00}}, /* 康 */
    {0x672A, {0x01,0x00,0x01,0x00,0x01,0x00,0x3F,0xF8,0x01,0x00,0x01,0x00,0x7F,0xFE,0x03,0x80,0x07,0x40,0x0D,0x60,0x19,0x30,0x31,0x1C,0x61,0x06,0x01,0x00,0x00,0x00,0x00,0x00}}, /* 未 */
    {0x8FD1, {0x00,0x00,0x00,0x04,0x31,0xFC,0x19,0x00,0x09,0x00,0x01,0x00,0x01,0xFE,0x71,0x10,0x11,0x10,0x11,0x10,0x13,0x10,0x12,0x10,0x16,0x10,0x3C,0x00,0x63,0xFE,0x00,0x00}}, /* 近 */
    {0x534A, {0x00,0x00,0x01,0x80,0x11,0x88,0x19,0x98,0x19,0x90,0x09,0x90,0x01,0x80,0x3F,0xFC,0x01,0x80,0x01,0x80,0x7F,0xFE,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x00,0x00}}, /* 半 */
    {0x6709, {0x01,0x00,0x01,0x00,0x3F,0xFE,0x02,0x00,0x06,0x00,0x07,0xFC,0x0C,0x0C,0x14,0x0C,0x67,0xFC,0x44,0x0C,0x07,0xFC,0x04,0x0C,0x04,0x0C,0x04,0x18,0x00,0x00,0x00,0x00}}, /* 有 */
    {0x5C0F, {0x00,0x00,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x09,0x90,0x19,0x98,0x11,0x88,0x11,0x8C,0x31,0x84,0x21,0x84,0x61,0x84,0x01,0x80,0x01,0x80,0x03,0x80,0x00,0x00}}, /* 小 */
    {0x6B21, {0x00,0x00,0x00,0x80,0x21,0x80,0x21,0x00,0x31,0xFC,0x1A,0x04,0x02,0x4C,0x04,0x48,0x00,0x40,0x18,0x40,0x10,0xE0,0x30,0xA0,0x61,0x10,0x43,0x0C,0x0C,0x06,0x00,0x00}}, /* 次 */
    {0x63D0, {0x00,0x00,0x11,0xFC,0x11,0xFC,0x11,0x04,0x7D,0xFC,0x11,0x04,0x11,0xFC,0x10,0x00,0x1B,0xFE,0xF0,0x20,0x11,0x20,0x11,0x3C,0x13,0x20,0x12,0xE0,0x76,0x7E,0x04,0x00}}, /* 提 */
    {0x9192, {0x00,0x00,0x00,0x7E,0x7F,0x42,0x14,0x42,0x14,0x7E,0x7E,0x42,0x5E,0x7E,0x5E,0x10,0x5E,0x50,0x66,0xFE,0x42,0x90,0x7E,0xFE,0x42,0x10,0x7E,0x10,0x42,0xFE,0x42,0x00}}, /* 醒 */
    {0x8B66, {0x12,0x30,0x7F,0xBE,0x12,0x44,0x3F,0xA8,0x7C,0xB8,0x27,0x1C,0x3C,0x66,0x7F,0xFE,0x00,0x00,0x1F,0xF8,0x1F,0xF8,0x00,0x00,0x1F,0xF8,0x1F,0xF8,0x10,0x08,0x00,0x00}}, /* 警 */
    {0x544A, {0x00,0x00,0x00,0x80,0x08,0x80,0x10,0x80,0x3F,0xFC,0x20,0x80,0x00,0x80,0x7F,0xFE,0x00,0x00,0x1F,0xF8,0x10,0x08,0x10,0x08,0x10,0x08,0x1F,0xF8,0x10,0x08,0x00,0x00}}, /* 告 */
};

#define CN_FONT_COUNT (sizeof(cn_font) / sizeof(cn_font[0]))

static void draw_cn_char(uint16_t *fb,
                         uint32_t x,
                         uint32_t y,
                         uint16_t code,
                         uint16_t color)
{
    for (uint32_t i = 0U; i < CN_FONT_COUNT; ++i)
    {
        if (cn_font[i].code == code)
        {
            const uint8_t *bmp = cn_font[i].bitmap;
            for (uint32_t row = 0U; row < 16U; ++row)
            {
                uint16_t row_bits =
                    ((uint16_t)bmp[row * 2U] << 8) | bmp[(row * 2U) + 1U];
                for (uint32_t col = 0U; col < 16U; ++col)
                {
                    if (0U != (row_bits & (0x8000U >> col)))
                    {
                        fb[((y + row) * CM55_DISP_HOR_RES) + x + col] = color;
                    }
                }
            }
            return;
        }
    }
    /* Fallback: draw '?' if glyph not found */
    draw_char(fb, x, y, '?', color, 2U);
}

/* Draw a Chinese character scaled by factor (1x=16px, 2x=32px) */
static void draw_cn_char_scaled(uint16_t *fb,
                                uint32_t x,
                                uint32_t y,
                                uint16_t code,
                                uint16_t color,
                                uint32_t scale)
{
    for (uint32_t i = 0U; i < CN_FONT_COUNT; ++i)
    {
        if (cn_font[i].code == code)
        {
            const uint8_t *bmp = cn_font[i].bitmap;
            for (uint32_t row = 0U; row < 16U; ++row)
            {
                uint16_t row_bits =
                    ((uint16_t)bmp[row * 2U] << 8) | bmp[(row * 2U) + 1U];
                for (uint32_t col = 0U; col < 16U; ++col)
                {
                    if (0U != (row_bits & (0x8000U >> col)))
                    {
                        fill_rect(fb,
                                  x + (col * scale),
                                  y + (row * scale),
                                  scale, scale, color);
                    }
                }
            }
            return;
        }
    }
    draw_char(fb, x, y, '?', color, scale);
}

static void draw_float_1dp_scaled(uint16_t *fb,
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

/* Draw a card with soft gradient alert glow on the top edge */
static void draw_card_with_alert(uint16_t *fb,
                                 uint32_t cx, uint32_t cy,
                                 uint32_t cw, uint32_t ch,
                                 uint16_t accent_color,
                                 uint32_t alert_start_ms,
                                 uint32_t now_ms,
                                 uint32_t alert_duration_ms)
{
    fill_rect(fb, cx, cy, cw, ch, RGB565(255U, 255U, 255U));
    fill_rect(fb, cx, cy, 5U, ch, accent_color);

    if (0U != alert_start_ms)
    {
        uint32_t elapsed = now_ms - alert_start_ms;
        if (elapsed < alert_duration_ms)
        {
            /* Soft gradient: 6 layers of decreasing intensity */
            uint32_t progress_x256 = (elapsed * 256U) / alert_duration_ms;
            uint32_t layers = 6U;
            for (uint32_t layer = 0U; layer < layers; ++layer)
            {
                uint32_t intensity = 256U - progress_x256;
                uint32_t layer_fade = intensity * (layers - layer) / layers;
                uint16_t r = (uint16_t)((layer_fade * 240U) / 256U);
                uint16_t g = (uint16_t)((layer_fade * 82U) / 256U);
                uint16_t b = (uint16_t)((layer_fade * 82U) / 256U);
                uint16_t glow = RGB565(r, g, b);
                fill_rect(fb, cx + layer, cy + layer, cw - (2U * layer), 2U, glow);
            }
            /* Bottom edge glow (thinner) */
            for (uint32_t layer = 0U; layer < 3U; ++layer)
            {
                uint32_t intensity = 256U - progress_x256;
                uint32_t layer_fade = intensity * (3U - layer) / 3U;
                uint16_t r = (uint16_t)((layer_fade * 200U) / 256U);
                uint16_t g = (uint16_t)((layer_fade * 60U) / 256U);
                uint16_t b = (uint16_t)((layer_fade * 60U) / 256U);
                uint16_t glow = RGB565(r, g, b);
                fill_rect(fb, cx + layer, cy + ch - 2U - layer, cw - (2U * layer), 2U, glow);
            }
        }
    }
}

/* Render fading alert text inside a card */
static void draw_alert_in_card(uint16_t *fb,
                               uint32_t cx, uint32_t cy, uint32_t cw, uint32_t ch,
                               uint32_t alert_start_ms, uint32_t now_ms,
                               uint32_t alert_duration_ms,
                               uint16_t code1, uint16_t code2, uint16_t code3,
                               uint16_t code4, uint16_t code5)
{
    if (0U == alert_start_ms) return;
    uint32_t elapsed = now_ms - alert_start_ms;
    if (elapsed >= alert_duration_ms) return;

    uint32_t progress = (elapsed * 256U) / alert_duration_ms;
    uint32_t intensity = 256U - progress;
    uint16_t r = (uint16_t)((intensity * 240U) / 256U);
    uint16_t g = (uint16_t)((intensity * 60U) / 256U);
    uint16_t b = (uint16_t)((intensity * 60U) / 256U);
    if (r < 40U) r = 40U;
    if (g < 10U) g = 10U;
    if (b < 10U) b = 10U;
    uint16_t alert_color = RGB565(r, g, b);

    uint32_t text_y = cy + ch - 36U;
    uint32_t tx = cx + 20U;
    if (0xFFFFU != code1) { draw_cn_char_scaled(fb, tx, text_y, code1, alert_color, 2U); tx += 36U; }
    if (0xFFFFU != code2) { draw_cn_char_scaled(fb, tx, text_y, code2, alert_color, 2U); tx += 36U; }
    if (0xFFFFU != code3) { draw_cn_char_scaled(fb, tx, text_y, code3, alert_color, 2U); tx += 36U; }
    if (0xFFFFU != code4) { draw_cn_char_scaled(fb, tx, text_y, code4, alert_color, 2U); tx += 36U; }
    if (0xFFFFU != code5) { draw_cn_char_scaled(fb, tx, text_y, code5, alert_color, 2U); }
}

static void draw_live_frame(uint16_t *fb)
{
    volatile app_display_cm55_snapshot_t *snap = APP_DISPLAY_CM55_SNAPSHOT;
    uint32_t seq_end, seq_begin;
    uint8_t health;
    uint16_t status_color;

    /* Alert state: per-card */
    static uint32_t cough_alert_ms;
    static uint32_t rr_alert_ms;
    static uint32_t hr_alert_ms;
    static uint32_t radar_alert_ms;
    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    uint32_t alert_dur = 15000U;

    /* Colors */
    uint16_t c_bg       = RGB565(242U, 243U, 245U);
    uint16_t c_label    = RGB565(100U, 106U, 115U);
    uint16_t c_value    = RGB565(34U,  34U,  34U);
    uint16_t c_unit     = RGB565(150U, 156U, 165U);
    uint16_t c_teal     = RGB565(0U,   179U, 164U);
    uint16_t c_green    = RGB565(76U,  175U, 80U);
    uint16_t c_yellow   = RGB565(253U, 181U, 21U);
    uint16_t c_red      = RGB565(240U, 82U,  82U);
    uint16_t c_gray     = RGB565(170U, 176U, 185U);
    uint16_t c_header_bg = RGB565(255U, 255U, 255U);
    uint16_t c_border   = RGB565(220U, 222U, 226U);
    uint16_t c_bar_bg   = RGB565(248U, 249U, 250U);

    /* Layout: x_off=20, cards 376x160, header 60px, bar 44px */
    uint32_t x_off = 20U;
    uint32_t card_w = 376U;
    uint32_t card_h = 160U;
    uint32_t card_x1 = 20U + x_off;
    uint32_t card_x2 = 404U + x_off;
    uint32_t card_y1 = 68U;
    uint32_t card_y2 = 238U;
    uint32_t bar_y = CM55_DISP_VER_RES - 44U;
    uint32_t cs = 2U; /* Chinese scale */
    uint32_t ns = 4U; /* Number scale */

    /* Clear */
    for (uint32_t i = 0U; i < (CM55_DISP_HOR_RES * CM55_DISP_VER_RES); ++i)
        fb[i] = c_bg;

    /* Torn-read check */
    APP_DISPLAY_CM55_INVALIDATE_CACHE(
        (uint32_t)snap, sizeof(app_display_cm55_snapshot_t));
    seq_end = snap->seq_end;
    __DMB();

    health = snap->health_state;

    /* === Alert state machine === */
    /* Cough: reset if new cough detected while alert active */
    if (snap->cough_count_5min > 0U)
    {
        if (0U == cough_alert_ms || (now_ms - cough_alert_ms) > alert_dur)
            cough_alert_ms = now_ms;
    }
    else
    {
        cough_alert_ms = 0U;
    }

    /* RR: abnormal if >25 or <8 bpm (and data present) */
    if (0U != snap->rr_bpm_x10 && (snap->rr_bpm_x10 > 250U || snap->rr_bpm_x10 < 80U))
    {
        if (0U == rr_alert_ms || (now_ms - rr_alert_ms) > alert_dur)
            rr_alert_ms = now_ms;
    }
    else
    {
        rr_alert_ms = 0U;
    }

    /* HR: abnormal if >100 or <50 bpm (and data present) */
    if (0U != snap->hr_bpm_x10 && (snap->hr_bpm_x10 > 1000U || snap->hr_bpm_x10 < 500U))
    {
        if (0U == hr_alert_ms || (now_ms - hr_alert_ms) > alert_dur)
            hr_alert_ms = now_ms;
    }
    else
    {
        hr_alert_ms = 0U;
    }

    /* === Header (60px) === */
    fill_rect(fb, 0U, 0U, CM55_DISP_ACTUAL_HOR_RES, 60U, c_header_bg);
    fill_rect(fb, 0U, 59U, CM55_DISP_ACTUAL_HOR_RES, 2U, c_border);

    {
        uint32_t tx = 24U + x_off;
        draw_cn_char_scaled(fb, tx, 8U, 0x591C, c_teal, cs); tx += 18U * cs;
        draw_cn_char_scaled(fb, tx, 8U, 0x95F4, c_teal, cs); tx += 18U * cs;
        draw_cn_char_scaled(fb, tx, 8U, 0x5065, c_teal, cs); tx += 18U * cs;
        draw_cn_char_scaled(fb, tx, 8U, 0x5EB7, c_teal, cs); tx += 18U * cs;
        draw_cn_char_scaled(fb, tx, 8U, 0x76D1, c_teal, cs); tx += 18U * cs;
        draw_cn_char_scaled(fb, tx, 8U, 0x6D4B, c_teal, cs);
    }

    /* Status dot + text */
    if (health == DISPLAY_CM55_HEALTH_NORMAL) status_color = c_green;
    else if (health == DISPLAY_CM55_HEALTH_ATTENTION) status_color = c_yellow;
    else if (health == DISPLAY_CM55_HEALTH_WARNING) status_color = c_red;
    else if (health == DISPLAY_CM55_HEALTH_SENSOR_LOST) status_color = c_yellow;
    else if (health == DISPLAY_CM55_HEALTH_ERROR) status_color = c_red;
    else status_color = c_gray;

    fill_rect(fb, 560U + x_off, 14U, 14U, 14U, status_color);
    {
        uint32_t sx = 580U + x_off;
        draw_cn_char_scaled(fb, sx, 8U, 0x5B9E, status_color, cs); sx += 18U * cs;
        draw_cn_char_scaled(fb, sx, 8U, 0x65F6, status_color, cs); sx += 18U * cs;
        draw_cn_char_scaled(fb, sx, 8U, 0x76D1, status_color, cs); sx += 18U * cs;
        draw_cn_char_scaled(fb, sx, 8U, 0x6D4B, status_color, cs); sx += 18U * cs;
        draw_cn_char_scaled(fb, sx, 8U, 0x4E2D, status_color, cs);
    }

    /* === Card 1: 呼吸率 === */
    draw_card_with_alert(fb, card_x1, card_y1, card_w, card_h,
                         c_teal, rr_alert_ms, now_ms, alert_dur);
    {
        uint32_t cx = card_x1 + 20U;
        draw_cn_char_scaled(fb, cx, card_y1 + 10U, 0x547C, c_label, cs); cx += 18U * cs;
        draw_cn_char_scaled(fb, cx, card_y1 + 10U, 0x5438, c_label, cs); cx += 18U * cs;
        draw_cn_char_scaled(fb, cx, card_y1 + 10U, 0x7387, c_label, cs);
    }
    if (0U != snap->rr_bpm_x10)
    {
        draw_float_1dp_scaled(fb, card_x1 + 20U, card_y1 + 48U,
                              snap->rr_bpm_x10, c_value, ns);
        draw_text(fb, card_x1 + 220U, card_y1 + 64U, "bpm", c_unit, 2U);
        draw_cn_char_scaled(fb, card_x1 + 20U, card_y1 + 120U, 0x5E73, c_green, cs);
        draw_cn_char_scaled(fb, card_x1 + 20U + 18U * cs, card_y1 + 120U, 0x7A33, c_green, cs);
    }
    else
    {
        draw_text(fb, card_x1 + 20U, card_y1 + 56U, "--", c_gray, ns);
        draw_cn_char_scaled(fb, card_x1 + 20U, card_y1 + 120U, 0x6682, c_gray, cs);
        draw_cn_char_scaled(fb, card_x1 + 20U + 18U * cs, card_y1 + 120U, 0x65E0, c_gray, cs);
        draw_cn_char_scaled(fb, card_x1 + 20U + 36U * cs, card_y1 + 120U, 0xCAFD, c_gray, cs);
        draw_cn_char_scaled(fb, card_x1 + 20U + 54U * cs, card_y1 + 120U, 0x636E, c_gray, cs);
    }
    /* RR alert text inside card */
    draw_alert_in_card(fb, card_x1, card_y1, card_w, card_h,
                       rr_alert_ms, now_ms, alert_dur,
                       0x547C, 0x5438, 0x7387, 0x5F02, 0x5E38);

    /* === Card 2: 心率 === */
    draw_card_with_alert(fb, card_x2, card_y1, card_w, card_h,
                         c_red, hr_alert_ms, now_ms, alert_dur);
    {
        uint32_t cx = card_x2 + 20U;
        draw_cn_char_scaled(fb, cx, card_y1 + 10U, 0x5FC3, c_label, cs); cx += 18U * cs;
        draw_cn_char_scaled(fb, cx, card_y1 + 10U, 0x7387, c_label, cs);
    }
    if (0U != snap->hr_bpm_x10)
    {
        draw_float_1dp_scaled(fb, card_x2 + 20U, card_y1 + 48U,
                              snap->hr_bpm_x10, c_value, ns);
        draw_text(fb, card_x2 + 220U, card_y1 + 64U, "bpm", c_unit, 2U);
        draw_cn_char_scaled(fb, card_x2 + 20U, card_y1 + 120U, 0x6B63, c_green, cs);
        draw_cn_char_scaled(fb, card_x2 + 20U + 18U * cs, card_y1 + 120U, 0x5E38, c_green, cs);
    }
    else
    {
        draw_text(fb, card_x2 + 20U, card_y1 + 56U, "--", c_gray, ns);
        draw_cn_char_scaled(fb, card_x2 + 20U, card_y1 + 120U, 0x6682, c_gray, cs);
        draw_cn_char_scaled(fb, card_x2 + 20U + 18U * cs, card_y1 + 120U, 0x65E0, c_gray, cs);
        draw_cn_char_scaled(fb, card_x2 + 20U + 36U * cs, card_y1 + 120U, 0xCAFD, c_gray, cs);
        draw_cn_char_scaled(fb, card_x2 + 20U + 54U * cs, card_y1 + 120U, 0x636E, c_gray, cs);
    }
    /* HR alert text inside card */
    draw_alert_in_card(fb, card_x2, card_y1, card_w, card_h,
                       hr_alert_ms, now_ms, alert_dur,
                       0x5FC3, 0x7387, 0x5F02, 0x5E38, 0xFFFF);

    /* === Card 3: 咳嗽事件 === */
    draw_card_with_alert(fb, card_x1, card_y2, card_w, card_h,
                         c_yellow, cough_alert_ms, now_ms, alert_dur);
    {
        uint32_t cx = card_x1 + 20U;
        draw_cn_char_scaled(fb, cx, card_y2 + 10U, 0x54B3, c_label, cs); cx += 18U * cs;
        draw_cn_char_scaled(fb, cx, card_y2 + 10U, 0x55FD, c_label, cs); cx += 18U * cs;
        draw_cn_char_scaled(fb, cx, card_y2 + 10U, 0x4E8B, c_label, cs); cx += 18U * cs;
        draw_cn_char_scaled(fb, cx, card_y2 + 10U, 0x4EF6, c_label, cs);
    }
    {
        uint16_t c5 = snap->cough_count_5min;
        uint32_t c_total = snap->cough_event_count_total;
        uint32_t lx;

        /* 整晚 X 次 */
        lx = card_x1 + 20U;
        draw_cn_char_scaled(fb, lx, card_y2 + 48U, 0x6574, c_label, cs); lx += 18U * cs;
        draw_cn_char_scaled(fb, lx, card_y2 + 48U, 0x665A, c_label, cs); lx += 18U * cs;
        draw_number(fb, lx, card_y2 + 48U, c_total, c_value, ns); lx += 60U;
        draw_cn_char_scaled(fb, lx, card_y2 + 60U, 0x6B21, c_unit, cs);

        /* 近5分钟 X 次 */
        lx = card_x1 + 20U;
        draw_cn_char_scaled(fb, lx, card_y2 + 92U, 0x8FD1, c_label, cs); lx += 18U * cs;
        draw_number(fb, lx, card_y2 + 92U, 5U, c_label, 3U); lx += 30U;
        draw_cn_char_scaled(fb, lx, card_y2 + 92U, 0x5206, c_label, cs); lx += 18U * cs;
        draw_cn_char_scaled(fb, lx, card_y2 + 92U, 0x949F, c_label, cs); lx += 18U * cs;
        draw_number(fb, lx, card_y2 + 92U, c5, c_value, 3U); lx += 54U;
        draw_cn_char_scaled(fb, lx, card_y2 + 100U, 0x6B21, c_unit, cs);

        /* 当前状态 */
        lx = card_x1 + 20U;
        draw_cn_char_scaled(fb, lx, card_y2 + 130U, 0x5F53, c_label, cs); lx += 18U * cs;
        draw_cn_char_scaled(fb, lx, card_y2 + 130U, 0x524D, c_label, cs); lx += 18U * cs;
        if (c5 > 0U)
        {
            draw_cn_char_scaled(fb, lx, card_y2 + 130U, 0x6709, c_yellow, cs); lx += 18U * cs;
            draw_cn_char_scaled(fb, lx, card_y2 + 130U, 0x54B3, c_yellow, cs); lx += 18U * cs;
            draw_cn_char_scaled(fb, lx, card_y2 + 130U, 0x55FD, c_yellow, cs);
        }
        else
        {
            draw_cn_char_scaled(fb, lx, card_y2 + 130U, 0x65E0, c_green, cs); lx += 18U * cs;
            draw_cn_char_scaled(fb, lx, card_y2 + 130U, 0x54B3, c_green, cs); lx += 18U * cs;
            draw_cn_char_scaled(fb, lx, card_y2 + 130U, 0x55FD, c_green, cs);
        }
    }
    /* Cough alert text inside card */
    draw_alert_in_card(fb, card_x1, card_y2, card_w, card_h,
                       cough_alert_ms, now_ms, alert_dur,
                       0x68C0, 0x6D4B, 0x5230, 0x54B3, 0x55FD);

    /* === Card 4: 雷达 === */
    draw_card_with_alert(fb, card_x2, card_y2, card_w, card_h,
                         c_green, radar_alert_ms, now_ms, alert_dur);
    {
        uint32_t cx = card_x2 + 20U;
        draw_cn_char_scaled(fb, cx, card_y2 + 10U, 0x96F7, c_label, cs); cx += 18U * cs;
        draw_cn_char_scaled(fb, cx, card_y2 + 10U, 0x8FBE, c_label, cs);
    }
    {
        uint8_t radar_src = snap->radar_source;
        if (DISPLAY_CM55_RADAR_NORMAL == radar_src)
        {
            if (0U != snap->radar_presence)
            {
                uint32_t cx = card_x2 + 20U;
                draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x5DF2, c_green, cs); cx += 18U * cs;
                draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x68C0, c_green, cs); cx += 18U * cs;
                draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x6D4B, c_green, cs); cx += 18U * cs;
                draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x5230, c_green, cs); cx += 18U * cs;
                draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x4EBA, c_green, cs); cx += 18U * cs;
                draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x4F53, c_green, cs);
                if (0U != snap->distance_cm)
                {
                    draw_number(fb, card_x2 + 20U, card_y2 + 92U, snap->distance_cm, c_value, ns);
                    draw_text(fb, card_x2 + 220U, card_y2 + 108U, "cm", c_unit, 2U);
                }
            }
            else
            {
                uint32_t cx = card_x2 + 20U;
                draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x672A, c_gray, cs); cx += 18U * cs;
                draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x68C0, c_gray, cs); cx += 18U * cs;
                draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x6D4B, c_gray, cs); cx += 18U * cs;
                draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x5230, c_gray, cs); cx += 18U * cs;
                draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x4EBA, c_gray, cs); cx += 18U * cs;
                draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x4F53, c_gray, cs);
            }
        }
        else if (DISPLAY_CM55_RADAR_UNAVAILABLE == radar_src)
        {
            uint32_t cx = card_x2 + 20U;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x6570, c_gray, cs); cx += 18U * cs;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x636E, c_gray, cs); cx += 18U * cs;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x6682, c_gray, cs); cx += 18U * cs;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x4E0D, c_gray, cs); cx += 18U * cs;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x53EF, c_gray, cs); cx += 18U * cs;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x7528, c_gray, cs);
        }
        else if (DISPLAY_CM55_RADAR_STALE == radar_src)
        {
            uint32_t cx = card_x2 + 20U;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x4FE1, c_yellow, cs); cx += 18U * cs;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x53F7, c_yellow, cs); cx += 18U * cs;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x5DF2, c_yellow, cs); cx += 18U * cs;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x65AD, c_yellow, cs);
        }
        else
        {
            uint32_t cx = card_x2 + 20U;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x96F7, c_red, cs); cx += 18U * cs;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x8FBE, c_red, cs); cx += 18U * cs;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x5F02, c_red, cs); cx += 18U * cs;
            draw_cn_char_scaled(fb, cx, card_y2 + 48U, 0x5E38, c_red, cs);
        }
    }

    /* === Status Bar === */
    fill_rect(fb, 0U, bar_y, CM55_DISP_ACTUAL_HOR_RES, 44U, c_bar_bg);
    fill_rect(fb, 0U, bar_y, CM55_DISP_ACTUAL_HOR_RES, 1U, c_border);

    {
        uint32_t bx = 24U + x_off;
        if (health == DISPLAY_CM55_HEALTH_NORMAL)
        {
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x7761, c_label, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x7720, c_label, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x547C, c_label, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x5438, c_label, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x8D8B, c_label, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x52BF, c_label, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x5E73, c_label, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x7A33, c_label, cs);
        }
        else if (health == DISPLAY_CM55_HEALTH_ATTENTION)
        {
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x547C, c_yellow, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x5438, c_yellow, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x8F7B, c_yellow, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x5FAE, c_yellow, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x6CE2, c_yellow, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x52A8, c_yellow, cs);
        }
        else if (health == DISPLAY_CM55_HEALTH_WARNING)
        {
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x5F02, c_red, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x5E38, c_red, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x547C, c_red, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x5438, c_red, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x68C0, c_red, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x6D4B, c_red, cs);
        }
        else
        {
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x76D1, c_gray, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x6D4B, c_gray, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x8FDB, c_gray, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x884C, c_gray, cs); bx += 18U * cs;
            draw_cn_char_scaled(fb, bx, bar_y + 12U, 0x4E2D, c_gray, cs);
        }

        fill_rect(fb, 520U + x_off, bar_y + 16U, 10U, 10U,
                  (snap->audio_quality > 50U) ? c_green : c_red);
        draw_text(fb, 534U + x_off, bar_y + 12U, "MIC", c_label, 2U);

        fill_rect(fb, 610U + x_off, bar_y + 16U, 10U, 10U,
                  (DISPLAY_CM55_RADAR_NORMAL == snap->radar_source) ? c_green : c_gray);
        draw_text(fb, 624U + x_off, bar_y + 12U, "Radar", c_label, 2U);

        fill_rect(fb, 710U + x_off, bar_y + 16U, 10U, 10U,
                  (0U != snap->ble_connected) ? c_green : c_gray);
        draw_text(fb, 724U + x_off, bar_y + 12U, "BLE", c_label, 2U);
    }

    __DMB();
    seq_begin = snap->seq_begin;
    if (seq_begin != seq_end) return;
}

#endif /* APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE */

static void flip_framebuffer_180(uint16_t *fb)
{
    uint32_t total = CM55_DISP_HOR_RES * CM55_DISP_VER_RES;
    uint32_t i = 0U;
    uint32_t j = total - 1U;

    while (i < j)
    {
        uint16_t tmp = fb[i];
        fb[i] = fb[j];
        fb[j] = tmp;
        ++i;
        --j;
    }
}

static uint32_t set_framebuffer(uint16_t *fb)
{
    flip_framebuffer_180(fb);
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
#if !APP_DISPLAY_LVGL_ENABLE
    uint16_t *active_fb = frame_buffer1;
#endif
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
#if (APP_DISPLAY_LVGL_ENABLE)
    /* LVGL will render into the framebuffers — skip CPU drawing */
    CM55_DISP_LOG("[CM55_DISP] LVGL mode: skipping CPU frame draw\r\n");
#elif (APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE)
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

#if (APP_DISPLAY_LVGL_ENABLE)
    /* Initialize LVGL and the dashboard UI */
    CM55_DISP_LOG("[CM55_DISP] step07 lvgl_init\r\n");
    lv_init();
    lv_port_disp_init(cm55_gfx_task_handle, GFXSS, &gfx_context);
    ui_health_dashboard_init();
    CM55_DISP_LOG("[CM55_DISP] step07 lvgl_init OK\r\n");
    CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_DISPLAY_READY, 0u, 0u);
    CM55_DISP_LOG("[CM55_DISP] READY lvgl_mode=1\r\n");
#else
    if (0U != set_framebuffer(active_fb))
    {
        CM55_DISP_DIAG(APP_DISPLAY_DIAG_STAGE_DISPLAY_READY, 0u, 0u);
    }
#if (APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE)
    CM55_DISP_LOG("[CM55_DISP] READY bridge_mode=1\r\n");
#else
    CM55_DISP_LOG("[CM55_DISP] READY proof_framebuffer=1\r\n");
#endif
#endif /* APP_DISPLAY_LVGL_ENABLE */

    for (;;)
    {
#if (APP_DISPLAY_LVGL_ENABLE)
        /* LVGL mode: ~30 FPS timer handler + configurable data update. */
        lv_timer_handler();
        if ((tick_count % APP_DISPLAY_LVGL_DATA_UPDATE_TICKS) == 0U)
        {
            ui_health_dashboard_update();
        }
        ++tick_count;
        vTaskDelay(pdMS_TO_TICKS(33U));
#else
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
#endif /* APP_DISPLAY_LVGL_ENABLE */
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

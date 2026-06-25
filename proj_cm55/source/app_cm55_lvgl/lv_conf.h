/**
 * @file lv_conf.h
 * LVGL v9.2.0 configuration for E84 Health Monitor — Waveshare 4.3" DSI LCD
 *
 * Copy of lv_conf_template.h with project-specific overrides.
 * Placed next to the lvgl folder (via include path) so LV_CONF_INCLUDE_SIMPLE
 * picks it up.
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/
/*Color depth: 16 (RGB565) — matches the Waveshare 4.3" DSI panel*/
#define LV_COLOR_DEPTH 16

/*=========================
   STDLIB WRAPPER SETTINGS
 *=========================*/
/*Use CLIB malloc (FreeRTOS heap_3 wraps C library malloc)*/
#define LV_USE_STDLIB_MALLOC    LV_STDLIB_CLIB
#define LV_USE_STDLIB_STRING    LV_STDLIB_CLIB
#define LV_USE_STDLIB_SPRINTF   LV_STDLIB_CLIB

/*====================
   HAL SETTINGS
 *====================*/
/*33ms refresh period → ~30 FPS*/
#define LV_DEF_REFR_PERIOD  33
#define LV_DPI_DEF 130

/*=================
 * OPERATING SYSTEM
 *=================*/
#define LV_USE_OS   LV_OS_FREERTOS

/*========================
 * RENDERING CONFIGURATION
 *========================*/
#define LV_DRAW_BUF_STRIDE_ALIGN                1
#define LV_DRAW_BUF_ALIGN                       4
#define LV_DRAW_TRANSFORM_USE_MATRIX            0
#define LV_DRAW_LAYER_SIMPLE_BUF_SIZE    (16 * 1024)
#define LV_DRAW_THREAD_STACK_SIZE    (4 * 1024)

#define LV_USE_DRAW_SW 1
#if LV_USE_DRAW_SW == 1
    #define LV_DRAW_SW_SUPPORT_RGB565       1
    #define LV_DRAW_SW_SUPPORT_RGB565A8     1
    #define LV_DRAW_SW_SUPPORT_RGB888       0
    #define LV_DRAW_SW_SUPPORT_XRGB8888    0
    #define LV_DRAW_SW_SUPPORT_ARGB8888    0
    #define LV_DRAW_SW_SUPPORT_L8          0
    #define LV_DRAW_SW_SUPPORT_AL88        0
    #define LV_DRAW_SW_SUPPORT_A8          1
    #define LV_DRAW_SW_SUPPORT_I1          0

    #define LV_DRAW_SW_DRAW_UNIT_CNT    1
    #define LV_USE_DRAW_ARM2D_SYNC      0
    #define LV_USE_NATIVE_HELIUM_ASM    0
    #define LV_DRAW_SW_COMPLEX          1

    #if LV_DRAW_SW_COMPLEX == 1
        #define LV_DRAW_SW_SHADOW_CACHE_SIZE 0
        #define LV_DRAW_SW_CIRCLE_CACHE_SIZE 4
    #endif

    #define  LV_USE_DRAW_SW_ASM     LV_DRAW_SW_ASM_NONE
    #define LV_USE_DRAW_SW_COMPLEX_GRADIENTS    0
#endif

/*Disable VG-Lite GPU draw for V1 (CPU SW render only)*/
#define LV_USE_DRAW_VGLITE 0

/*Disable other GPU backends*/
#define LV_USE_DRAW_PXP 0
#define LV_USE_DRAW_DAVE2D 0

/*=======================
 * FEATURE CONFIGURATION
 *=======================*/
#define LV_USE_ANIMIMG    0
#define LV_USE_ARC        1
#define LV_USE_BAR        0
#define LV_USE_BUTTON     0
#define LV_USE_BUTTONMATRIX 0
#define LV_USE_CALENDAR   0
#define LV_USE_CANVAS     0
#define LV_USE_CHART      0
#define LV_USE_CHECKBOX   0
#define LV_USE_DROPDOWN   0
#define LV_USE_IMAGE      0
#define LV_USE_IMAGEBUTTON 0
#define LV_USE_KEYBOARD   0
#define LV_USE_LABEL      1
#if LV_USE_LABEL
    #define LV_LABEL_TEXT_SELECTION 0
    #define LV_LABEL_LONG_TXT_HINT 1
    #define LV_LABEL_WAIT_CHAR_COUNT 3
#endif
#define LV_USE_LED        1
#define LV_USE_LINE       1
#define LV_USE_LIST       0
#define LV_USE_LOTTIE     0
#define LV_USE_MENU       0
#define LV_USE_MSGBOX     0
#define LV_USE_ROLLER     0
#define LV_USE_SCALE      0
#define LV_USE_SLIDER     0
#define LV_USE_SPAN       0
#define LV_USE_SPINBOX    0
#define LV_USE_SPINNER    0
#define LV_USE_SWITCH     0
#define LV_USE_TABLE      0
#define LV_USE_TABVIEW    0
#define LV_USE_TEXTAREA   0
#define LV_USE_TILEVIEW   0
#define LV_USE_WIN        0

/*==================
 * THEMES
 *==================*/
#define LV_USE_THEME_DEFAULT 1
#if LV_USE_THEME_DEFAULT
    #define LV_THEME_DEFAULT_DARK 1
    #define LV_THEME_DEFAULT_GROW 0
    #define LV_THEME_DEFAULT_TRANSITION_TIME 0
#endif
#define LV_USE_THEME_SIMPLE 0
#define LV_USE_THEME_MONO   0

/*==================
 * LAYOUTS
 *==================*/
#define LV_USE_FLEX  1
#define LV_USE_GRID  1

/*====================
 * LOGGING
 *====================*/
#define LV_USE_LOG 0

/*====================
 * ASSERTS
 *====================*/
#define LV_USE_ASSERT_NULL          1
#define LV_USE_ASSERT_MALLOC        1
#define LV_USE_ASSERT_STYLE         0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ           0

#define LV_ASSERT_HANDLER_INCLUDE <stdint.h>
#define LV_ASSERT_HANDLER while(1);

/*====================
 * DEBUG
 *====================*/
#define LV_USE_REFR_DEBUG 0
#define LV_USE_LAYER_DEBUG 0
#define LV_USE_PARALLEL_DRAW_DEBUG 0

/*====================
 * OTHERS
 *====================*/
#define LV_ENABLE_GLOBAL_CUSTOM 0
#define LV_CACHE_DEF_SIZE       0
#define LV_IMAGE_HEADER_CACHE_DEF_CNT 0
#define LV_GRADIENT_MAX_STOPS   2
#define LV_COLOR_MIX_ROUND_OFS  0
#define LV_OBJ_STYLE_CACHE      0
#define LV_USE_OBJ_ID           0
#define LV_OBJ_ID_AUTO_ASSIGN   0
#define LV_USE_OBJ_ID_BUILTIN   0
#define LV_USE_OBJ_PROPERTY     0
#define LV_USE_OBJ_PROPERTY_NAME 0
#define LV_USE_VG_LITE_THORVG   0

/*=====================
 * COMPILER SETTINGS
 *====================*/
#define LV_BIG_ENDIAN_SYSTEM 0
#define LV_ATTRIBUTE_TICK_INC
#define LV_ATTRIBUTE_TIMER_HANDLER
#define LV_ATTRIBUTE_FLUSH_READY
#define LV_ATTRIBUTE_MEM_ALIGN_SIZE 1
#define LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_LARGE_CONST
#define LV_ATTRIBUTE_LARGE_RAM_ARRAY
#define LV_ATTRIBUTE_FAST_MEM
#define LV_EXPORT_CONST_INT(int_value) struct _silence_gcc_warning
#define LV_ATTRIBUTE_EXTERN_DATA
#define LV_USE_FLOAT            0
#define LV_USE_MATRIX           0
#define LV_USE_PRIVATE_API      0

/*==================
 * FONT USAGE
 *===================*/
/*Montserrat ASCII fonts — enable sizes used by the dashboard*/
#define LV_FONT_MONTSERRAT_8  0
#define LV_FONT_MONTSERRAT_10 0
#define LV_FONT_MONTSERRAT_12 0
#define LV_FONT_MONTSERRAT_14 1   /*default UI text*/
#define LV_FONT_MONTSERRAT_16 0
#define LV_FONT_MONTSERRAT_18 1   /*card labels*/
#define LV_FONT_MONTSERRAT_20 0
#define LV_FONT_MONTSERRAT_22 0
#define LV_FONT_MONTSERRAT_24 0
#define LV_FONT_MONTSERRAT_26 0
#define LV_FONT_MONTSERRAT_28 1   /*card values*/
#define LV_FONT_MONTSERRAT_30 0
#define LV_FONT_MONTSERRAT_32 0
#define LV_FONT_MONTSERRAT_34 0
#define LV_FONT_MONTSERRAT_36 0
#define LV_FONT_MONTSERRAT_38 0
#define LV_FONT_MONTSERRAT_40 0
#define LV_FONT_MONTSERRAT_42 0
#define LV_FONT_MONTSERRAT_44 0
#define LV_FONT_MONTSERRAT_46 0
#define LV_FONT_MONTSERRAT_48 1   /*large bpm digits*/

#define LV_FONT_MONTSERRAT_28_COMPRESSED 0
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW 0
#define LV_FONT_SIMSUN_14_CJK            0
#define LV_FONT_SIMSUN_16_CJK            0

#define LV_FONT_UNSCII_8  0
#define LV_FONT_UNSCII_16 0

/*Custom CJK fonts for E84 dashboard — NSimSun 16/20/28px with ASCII + dashboard CJK*/
#define LV_FONT_CUSTOM_DECLARE \
    LV_FONT_DECLARE(lv_font_simsun_16_e84) \
    LV_FONT_DECLARE(lv_font_simsun_20_e84) \
    LV_FONT_DECLARE(lv_font_simsun_28_e84)

#define LV_FONT_DEFAULT &lv_font_montserrat_14

#define LV_FONT_FMT_TXT_LARGE 0
#define LV_USE_FONT_COMPRESSED 0
#define LV_USE_FONT_PLACEHOLDER 1

/*=================
 * TEXT SETTINGS
 *=================*/
#define LV_TXT_ENC LV_TXT_ENC_UTF8
#define LV_TXT_BREAK_CHARS " ,.;:-_)]}"
#define LV_TXT_LINE_BREAK_LONG_LEN 0
#define LV_TXT_LINE_BREAK_LONG_PRE_MIN_LEN 3
#define LV_TXT_LINE_BREAK_LONG_POST_MIN_LEN 3
#define LV_USE_BIDI 0
#define LV_USE_ARABIC_PERSIAN_CHARS 0

#endif /*LV_CONF_H*/

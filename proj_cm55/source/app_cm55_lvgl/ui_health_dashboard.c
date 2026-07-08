/**
 * @file ui_health_dashboard.c
 * Health monitor dashboard UI — LVGL v2 polished product screen.
 *
 * Layout (800 × 480 visible, 832 frame, 32px HOFFSET):
 *   ┌─────────────────────────────────────────────────────────┐
 *   │  (outer rounded container 14..818, 14..466)              │
 *   │  [ 起夜0次 ]                      [● 实时监测中]         │
 *   ├───────────────────────┬─────────────────────────────────┤
 *   │  ◉ 呼吸率              │  ◉ 心率                         │
 *   │      17  bpm           │      112  bpm                   │
 *   │    [ 平稳 ]            │    [ 正常 ]                      │
 *   ├───────────────────────┼─────────────────────────────────┤
 *   │  ◉ 咳嗽事件            │  ◉ 雷达                         │
 *   │  整晚 1 次             │      46  cm                     │
 *   │  最近30min: 1 次       │  已检测到人体                    │
 *   │  [ 无明显咳嗽 ]        │                                 │
 *   ├───────────────────────┴─────────────────────────────────┤
 *   │  [✓] 睡眠呼吸趋势平稳           MIC ●  Radar ●  BLE     │
 *   └─────────────────────────────────────────────────────────┘
 */

#include "ui_health_dashboard.h"
#include "lvgl.h"
#include "app_build_config.h"
#include "app_display_cm55_shared.h"
#if (APP_DISPLAY_LVGL_TOUCH_ENABLE)
#include "lv_port_indev.h"
#endif
#include "fonts/lv_font_simsun_16_e84.h"
#include "fonts/lv_font_simsun_20_e84.h"
#include "fonts/lv_font_simsun_28_e84.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifndef APP_DISPLAY_LVGL_TIMEZONE_OFFSET_S
#define APP_DISPLAY_LVGL_TIMEZONE_OFFSET_S (8 * 60 * 60)
#endif

#ifndef APP_DISPLAY_LVGL_COUGH_EVENT_HOLD_MS
#define APP_DISPLAY_LVGL_COUGH_EVENT_HOLD_MS (3000U)
#endif

#ifndef APP_DISPLAY_LVGL_TOUCH_ENABLE
#define APP_DISPLAY_LVGL_TOUCH_ENABLE (0U)
#endif

#ifndef APP_DISPLAY_LVGL_TOUCH_DIAG_ENABLE
#define APP_DISPLAY_LVGL_TOUCH_DIAG_ENABLE (0U)
#endif

#if (APP_DISPLAY_CM55_UART_LOG_ENABLE)
#define LVGL_DASH_LOG(...)       \
    do                           \
    {                            \
        printf(__VA_ARGS__);     \
        fflush(stdout);          \
    } while (0)
#else
#define LVGL_DASH_LOG(...) \
    do                     \
    {                      \
    } while (0)
#endif

/* ------------------------------------------------------------------ */
/* Layout constants — fixed coordinates                               */
/* ------------------------------------------------------------------ */
#define HOFF               32U
#define OUTER_X            (14U + HOFF)
#define OUTER_Y            14U
#define OUTER_W            804U
#define OUTER_H            452U
#define OUTER_R            14U

#define TITLE_X            (OUTER_X + 32U)
#define TITLE_Y            (OUTER_Y + 20U)
#define WAKE_CAPSULE_W     176U
#define WAKE_CAPSULE_H     46U
#define WAKE_CAPSULE_R     8U

#define CAPSULE_W          190U
#define CAPSULE_H          46U
#define CAPSULE_X          (OUTER_X + OUTER_W - 32U - CAPSULE_W)
#define CAPSULE_Y          (OUTER_Y + 18U)

#define CARD_TOP_Y         120U
#define CARD_BOT_Y         296U
#define CARD_GAP_X         22U
#define CARD_GAP_Y         16U
#define CARD_W             ((OUTER_W - 64U - CARD_GAP_X) / 2U)
#define CARD_H_L           160U
#define CARD_H_R           160U
#define CARD_X0            (OUTER_X + 32U)
#define CARD_X1            (CARD_X0 + CARD_W + CARD_GAP_X)

#define FOOTER_X           (OUTER_X + 32U)
#define FOOTER_Y           (OUTER_Y + OUTER_H - 52U)
#define FOOTER_W           (OUTER_W - 64U)
#define FOOTER_H           42U

#define HISTORY_SAMPLE_PERIOD_MS (30U * 1000U)
#define HISTORY_SAMPLE_COUNT     60U
#define COUGH_RECENT_WINDOW_MS   (30U * 60U * 1000U)
#define COUGH_EVENT_RING_COUNT   128U
#define COUGH_NIGHT_BUCKET_COUNT 12U
#define COUGH_NIGHT_BUCKET_MS    (60U * 60U * 1000U)

#define DETAIL_TITLE_X      32
#define DETAIL_TITLE_Y      24
#define DETAIL_CURRENT_X    34
#define DETAIL_CURRENT_Y    68
#define DETAIL_CHART_X      32
#define DETAIL_CHART_Y      154
#define DETAIL_CHART_W      740
#define DETAIL_CHART_H      200
#define DETAIL_STAT_Y       374
#define DETAIL_PAGE_DOT_Y   424

/* ------------------------------------------------------------------ */
/* Fonts                                                              */
/* ------------------------------------------------------------------ */
#define FONT_16   (&lv_font_simsun_16_e84)
#define FONT_20   (&lv_font_simsun_20_e84)
#define FONT_28   (&lv_font_simsun_28_e84)
#define FONT_48   (&lv_font_montserrat_48)
#define FONT_56   (&lv_font_montserrat_48)  /* use 48 as 56 substitute */
#define FONT_UNIT (&lv_font_montserrat_28)

/* ------------------------------------------------------------------ */
/* Colors                                                             */
/* ------------------------------------------------------------------ */
#define C_BG_DEEP       lv_color_hex(0x050A10)
#define C_OUTER_BORDER  lv_color_hex(0x1A2535)
#define C_CARD_BG       lv_color_hex(0x0B1520)
#define C_CARD_BORDER   lv_color_hex(0x1D2B3A)
#define C_WHITE         lv_color_hex(0xFFFFFF)
#define C_TEXT          lv_color_hex(0xECF0F5)
#define C_TEXT_DIM      lv_color_hex(0x6B7A8D)
#define C_CYAN          lv_color_hex(0x00D4C8)
#define C_CYAN_DIM      lv_color_hex(0x0A3A3A)
#define C_ROSE          lv_color_hex(0xFF5A5A)
#define C_ROSE_DIM      lv_color_hex(0x3A1515)
#define C_PURPLE        lv_color_hex(0xA87FFF)
#define C_PURPLE_DIM    lv_color_hex(0x2A1A3A)
#define C_AMBER         lv_color_hex(0xFFB84D)
#define C_AMBER_DIM     lv_color_hex(0x3A2A10)
#define C_GREEN         lv_color_hex(0x2EE87A)
#define C_GREEN_DIM     lv_color_hex(0x0A3A1A)
#define C_CAPSULE_BG    lv_color_hex(0x0A2A1A)
#define C_FOOTER_BG     lv_color_hex(0x080E18)

/* ------------------------------------------------------------------ */
/* Widget handles                                                     */
/* ------------------------------------------------------------------ */
static lv_obj_t *lbl_subtitle;
static lv_obj_t *lbl_clock;
static lv_obj_t *capsule_status;
static lv_obj_t *lbl_status;

/* RR card */
static lv_obj_t *lbl_rr_value;
static lv_obj_t *lbl_rr_unit;
static lv_obj_t *lbl_rr_badge;

/* HR card */
static lv_obj_t *lbl_hr_value;
static lv_obj_t *lbl_hr_unit;
static lv_obj_t *lbl_hr_badge;

/* Cough card */
static lv_obj_t *lbl_cough_all;
static lv_obj_t *lbl_cough_half;
static lv_obj_t *lbl_cough_badge;

/* Radar card */
static lv_obj_t *lbl_radar_value;
static lv_obj_t *lbl_radar_unit;
static lv_obj_t *lbl_radar_presence;

/* Footer */
static lv_obj_t *lbl_trend;
static lv_obj_t *lbl_diag;
static lv_obj_t *dot_mic, *dot_radar, *dot_ble;

typedef enum
{
    UI_PAGE_HOME = 0,
    UI_PAGE_RR_DETAIL,
    UI_PAGE_HR_DETAIL,
    UI_PAGE_NIGHT,
    UI_PAGE_COUNT
} ui_page_t;

typedef struct
{
    uint16_t values[HISTORY_SAMPLE_COUNT];
    uint8_t count;
    uint8_t head;
    bool last_sample_valid;
    uint32_t last_sample_ms;
} metric_history_t;

typedef struct
{
    lv_obj_t *root;
    lv_obj_t *lbl_current;
    lv_obj_t *lbl_state;
    lv_obj_t *lbl_min;
    lv_obj_t *lbl_max;
    lv_obj_t *lbl_avg;
    lv_obj_t *line;
    lv_obj_t *empty;
    lv_point_precise_t points[HISTORY_SAMPLE_COUNT];
} metric_detail_widgets_t;

static lv_obj_t *page_home;
static lv_obj_t *page_rr_detail;
static lv_obj_t *page_hr_detail;
static lv_obj_t *page_night;
static ui_page_t s_active_page;

static metric_history_t s_rr_history;
static metric_history_t s_hr_history;
static metric_detail_widgets_t s_rr_detail;
static metric_detail_widgets_t s_hr_detail;

static lv_obj_t *lbl_night_total;
static lv_obj_t *lbl_night_total_unit;
static lv_obj_t *lbl_night_recent;
static lv_obj_t *lbl_night_recent_unit;
static lv_obj_t *lbl_night_empty;
static lv_obj_t *night_bars[COUGH_NIGHT_BUCKET_COUNT];

static uint32_t s_cough_recent_ts[COUGH_EVENT_RING_COUNT];
static uint8_t s_cough_recent_count;
static uint8_t s_cough_recent_head;
static uint32_t s_cough_night_buckets[COUGH_NIGHT_BUCKET_COUNT];
static bool s_cough_session_start_valid;
static uint32_t s_cough_session_start_ms;

/* Display-local state only: demo wake count is RAM-only and resets on reboot. */
static bool s_cough_event_total_valid;
static uint32_t s_last_cough_event_total;
static bool s_cough_event_id_valid;
static uint32_t s_last_cough_event_id;
static uint32_t s_cough_event_hold_until_ms;

#define WAKE_FAR_DELTA_CM          (50U)
#define WAKE_WINDOW_MS             (30U * 1000U)
#define WAKE_ABSENT_CONFIRM_HITS   (2U)

typedef enum
{
    WAKE_STATE_WAIT_PRESENT = 0,
    WAKE_STATE_PRESENT,
    WAKE_STATE_FAR_WINDOW,
    WAKE_STATE_ABSENT_COUNTED
} wake_state_t;

static wake_state_t s_wake_state;
static uint32_t s_wake_count;
static uint32_t s_wake_window_start_ms;
static uint16_t s_wake_near_distance_cm;
static uint8_t s_wake_absent_hits;

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */
static lv_obj_t *make_card(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                           lv_coord_t w, lv_coord_t h, lv_color_t border)
{
    lv_obj_t *c = lv_obj_create(parent);
    lv_obj_set_size(c, w, h);
    lv_obj_set_pos(c, x, y);
    lv_obj_set_style_bg_color(c, C_CARD_BG, 0);
    lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(c, 8, 0);
    lv_obj_set_style_border_color(c, border, 0);
    lv_obj_set_style_border_width(c, 1, 0);
    lv_obj_set_style_pad_all(c, 14, 0);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
    return c;
}

static lv_obj_t *make_icon_badge(lv_obj_t *parent, lv_color_t bg, lv_color_t fg,
                                 const char *symbol)
{
    lv_obj_t *badge = lv_obj_create(parent);
    lv_obj_set_size(badge, 52, 52);
    lv_obj_set_style_bg_color(badge, bg, 0);
    lv_obj_set_style_bg_opa(badge, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(badge, 26, 0);
    lv_obj_set_style_border_width(badge, 0, 0);
    lv_obj_set_style_pad_all(badge, 0, 0);
    lv_obj_clear_flag(badge, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl = lv_label_create(badge);
    lv_label_set_text(lbl, symbol);
    lv_obj_set_style_text_color(lbl, fg, 0);
    lv_obj_set_style_text_font(lbl, FONT_28, 0);
    lv_obj_center(lbl);
    return badge;
}

static lv_obj_t *make_badge_pill(lv_obj_t *parent, const char *text,
                                 lv_color_t bg, lv_color_t fg)
{
    lv_obj_t *pill = lv_obj_create(parent);
    lv_obj_set_height(pill, 30);
    lv_obj_set_style_bg_color(pill, bg, 0);
    lv_obj_set_style_bg_opa(pill, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(pill, 15, 0);
    lv_obj_set_style_border_width(pill, 0, 0);
    lv_obj_set_style_pad_hor(pill, 12, 0);
    lv_obj_set_style_pad_ver(pill, 3, 0);
    lv_obj_clear_flag(pill, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl = lv_label_create(pill);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, fg, 0);
    lv_obj_set_style_text_font(lbl, FONT_20, 0);
    lv_obj_center(lbl);
    lv_obj_set_width(pill, LV_SIZE_CONTENT);
    return pill;
}

static void set_badge_pill(lv_obj_t *pill, const char *text,
                           lv_color_t bg, lv_color_t fg)
{
    lv_obj_t *lbl = lv_obj_get_child(pill, 0);

    lv_obj_set_style_bg_color(pill, bg, 0);
    if (NULL != lbl)
    {
        lv_label_set_text(lbl, text);
        lv_obj_set_style_text_color(lbl, fg, 0);
        lv_obj_center(lbl);
    }
    lv_obj_set_width(pill, LV_SIZE_CONTENT);
}

static void show_page(ui_page_t page);
static void page_gesture_cb(lv_event_t *e);
#if (APP_DISPLAY_LVGL_TOUCH_ENABLE)
static void touch_swipe_cb(int32_t dir);
#endif

static void add_gesture_bubble_to_children(lv_obj_t *parent)
{
    uint32_t child_count = lv_obj_get_child_count(parent);

    for (uint32_t i = 0; i < child_count; ++i)
    {
        lv_obj_t *child = lv_obj_get_child(parent, (int32_t)i);

        if (NULL != child)
        {
            lv_obj_add_flag(child, LV_OBJ_FLAG_GESTURE_BUBBLE);
            add_gesture_bubble_to_children(child);
        }
    }
}

static lv_obj_t *make_page_root(lv_obj_t *scr)
{
    lv_obj_t *root = lv_obj_create(scr);

    lv_obj_set_size(root, OUTER_W, OUTER_H);
    lv_obj_set_pos(root, OUTER_X, OUTER_Y);
    lv_obj_set_style_bg_color(root, C_BG_DEEP, 0);
    lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(root, OUTER_R, 0);
    lv_obj_set_style_border_color(root, C_OUTER_BORDER, 0);
    lv_obj_set_style_border_width(root, 1, 0);
    lv_obj_set_style_pad_all(root, 0, 0);
    lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(root, page_gesture_cb, LV_EVENT_GESTURE, NULL);
    return root;
}

static void set_page_hidden(lv_obj_t *page, bool hidden)
{
    if (NULL == page)
    {
        return;
    }

    if (hidden)
    {
        lv_obj_add_flag(page, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_remove_flag(page, LV_OBJ_FLAG_HIDDEN);
    }
}

static void show_page(ui_page_t page)
{
    if (page >= UI_PAGE_COUNT)
    {
        page = UI_PAGE_HOME;
    }

    s_active_page = page;
    set_page_hidden(page_home, UI_PAGE_HOME != page);
    set_page_hidden(page_rr_detail, UI_PAGE_RR_DETAIL != page);
    set_page_hidden(page_hr_detail, UI_PAGE_HR_DETAIL != page);
    set_page_hidden(page_night, UI_PAGE_NIGHT != page);
}

static void switch_page(int32_t delta)
{
    int32_t page = (int32_t)s_active_page + delta;

    while (page < 0)
    {
        page += (int32_t)UI_PAGE_COUNT;
    }
    while (page >= (int32_t)UI_PAGE_COUNT)
    {
        page -= (int32_t)UI_PAGE_COUNT;
    }

    show_page((ui_page_t)page);
}

static void page_gesture_cb(lv_event_t *e)
{
    lv_indev_t *indev;
    lv_dir_t dir;

    if (LV_EVENT_GESTURE != lv_event_get_code(e))
    {
        return;
    }

    indev = lv_event_get_indev(e);
    if (NULL == indev)
    {
        indev = lv_indev_active();
    }
    if (NULL == indev)
    {
        return;
    }

    dir = lv_indev_get_gesture_dir(indev);
    if (LV_DIR_LEFT == dir)
    {
        switch_page(1);
    }
    else if (LV_DIR_RIGHT == dir)
    {
        switch_page(-1);
    }
}

#if (APP_DISPLAY_LVGL_TOUCH_ENABLE)
static void touch_swipe_cb(int32_t dir)
{
    if (dir < 0)
    {
        switch_page(1);
    }
    else if (dir > 0)
    {
        switch_page(-1);
    }
}
#endif

static lv_obj_t *make_simple_label(lv_obj_t *parent,
                                   const char *text,
                                   const lv_font_t *font,
                                   lv_color_t color,
                                   lv_coord_t x,
                                   lv_coord_t y)
{
    lv_obj_t *lbl = lv_label_create(parent);

    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, color, 0);
    lv_obj_set_style_text_font(lbl, font, 0);
    lv_obj_set_pos(lbl, x, y);
    return lbl;
}

static void make_page_indicator(lv_obj_t *parent, const char *text)
{
    lv_obj_t *lbl = make_simple_label(parent, text, FONT_20, C_TEXT_DIM,
                                      356, DETAIL_PAGE_DOT_Y);

    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(lbl, 96);
}

static lv_obj_t *make_chart_box(lv_obj_t *parent)
{
    lv_obj_t *box = lv_obj_create(parent);

    lv_obj_set_size(box, DETAIL_CHART_W, DETAIL_CHART_H);
    lv_obj_set_pos(box, DETAIL_CHART_X, DETAIL_CHART_Y);
    lv_obj_set_style_bg_color(box, C_CARD_BG, 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(box, 8, 0);
    lv_obj_set_style_border_color(box, C_CARD_BORDER, 0);
    lv_obj_set_style_border_width(box, 1, 0);
    lv_obj_set_style_pad_all(box, 0, 0);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    return box;
}

static void make_chart_grid(lv_obj_t *box)
{
    for (uint32_t i = 1; i < 4; ++i)
    {
        lv_obj_t *line = lv_obj_create(box);
        lv_coord_t y = (lv_coord_t)((DETAIL_CHART_H * i) / 4U);

        lv_obj_set_size(line, DETAIL_CHART_W - 36, 1);
        lv_obj_set_pos(line, 18, y);
        lv_obj_set_style_bg_color(line, C_CARD_BORDER, 0);
        lv_obj_set_style_bg_opa(line, LV_OPA_60, 0);
        lv_obj_set_style_border_width(line, 0, 0);
        lv_obj_set_style_pad_all(line, 0, 0);
        lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE);
    }

    for (uint32_t i = 1; i < 4; ++i)
    {
        lv_obj_t *line = lv_obj_create(box);
        lv_coord_t x = (lv_coord_t)((DETAIL_CHART_W * i) / 4U);

        lv_obj_set_size(line, 1, DETAIL_CHART_H - 28);
        lv_obj_set_pos(line, x, 14);
        lv_obj_set_style_bg_color(line, C_CARD_BORDER, 0);
        lv_obj_set_style_bg_opa(line, LV_OPA_40, 0);
        lv_obj_set_style_border_width(line, 0, 0);
        lv_obj_set_style_pad_all(line, 0, 0);
        lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE);
    }
}

static void create_metric_detail_page(lv_obj_t *scr,
                                      metric_detail_widgets_t *page,
                                      const char *title,
                                      const char *page_mark,
                                      lv_color_t accent)
{
    lv_obj_t *chart_box;

    page->root = make_page_root(scr);
    make_simple_label(page->root, title, FONT_28, C_TEXT,
                      DETAIL_TITLE_X, DETAIL_TITLE_Y);
    make_simple_label(page->root, "当前", FONT_20, C_TEXT_DIM,
                      DETAIL_CURRENT_X, DETAIL_CURRENT_Y + 10);
    page->lbl_current = make_simple_label(page->root, "--",
                                          FONT_48, accent,
                                          DETAIL_CURRENT_X + 76,
                                          DETAIL_CURRENT_Y);
    make_simple_label(page->root, "bpm", FONT_UNIT, C_TEXT,
                      DETAIL_CURRENT_X + 214, DETAIL_CURRENT_Y + 18);
    page->lbl_state = make_simple_label(page->root, "等待数据",
                                        FONT_20, C_TEXT_DIM,
                                        DETAIL_CURRENT_X,
                                        DETAIL_CURRENT_Y + 66);

    chart_box = make_chart_box(page->root);
    make_chart_grid(chart_box);
    page->line = lv_line_create(chart_box);
    lv_obj_set_size(page->line, DETAIL_CHART_W - 32, DETAIL_CHART_H - 28);
    lv_obj_set_pos(page->line, 16, 12);
    lv_obj_set_style_line_color(page->line, accent, 0);
    lv_obj_set_style_line_width(page->line, 4, 0);
    lv_obj_set_style_line_rounded(page->line, true, 0);

    page->empty = make_simple_label(chart_box, "等待数据", FONT_20,
                                    C_TEXT_DIM, 316, 92);

    page->lbl_min = make_simple_label(page->root, "Min --", FONT_28,
                                      C_TEXT, 42, DETAIL_STAT_Y);
    page->lbl_max = make_simple_label(page->root, "Max --", FONT_28,
                                      C_TEXT, 302, DETAIL_STAT_Y);
    page->lbl_avg = make_simple_label(page->root, "Avg --", FONT_28,
                                      C_TEXT, 562, DETAIL_STAT_Y);
    make_page_indicator(page->root, page_mark);

    add_gesture_bubble_to_children(page->root);
}

static void create_night_page(lv_obj_t *scr)
{
    lv_obj_t *chart_box;
    const lv_coord_t gap = 10;
    const lv_coord_t bar_w =
        (DETAIL_CHART_W - 32 - ((lv_coord_t)COUGH_NIGHT_BUCKET_COUNT - 1) * gap) /
        (lv_coord_t)COUGH_NIGHT_BUCKET_COUNT;

    page_night = make_page_root(scr);
    make_simple_label(page_night, "整夜", FONT_28, C_TEXT,
                      DETAIL_TITLE_X, DETAIL_TITLE_Y);
    make_simple_label(page_night, "整夜咳嗽", FONT_20, C_TEXT_DIM,
                      DETAIL_CURRENT_X, DETAIL_CURRENT_Y + 4);
    lbl_night_total = make_simple_label(page_night, "--",
                                        FONT_48, C_AMBER,
                                        DETAIL_CURRENT_X,
                                        DETAIL_CURRENT_Y + 32);
    lbl_night_total_unit = make_simple_label(page_night, "次", FONT_28,
                                             C_TEXT,
                                             DETAIL_CURRENT_X + 126,
                                             DETAIL_CURRENT_Y + 48);

    make_simple_label(page_night, "最近30min", FONT_20, C_TEXT_DIM,
                      450, DETAIL_CURRENT_Y + 4);
    lbl_night_recent = make_simple_label(page_night, "--",
                                         FONT_48, C_TEXT,
                                         450, DETAIL_CURRENT_Y + 32);
    lbl_night_recent_unit = make_simple_label(page_night, "次", FONT_28,
                                              C_TEXT,
                                              576, DETAIL_CURRENT_Y + 48);

    chart_box = make_chart_box(page_night);
    make_chart_grid(chart_box);
    make_simple_label(chart_box, "整夜咳嗽", FONT_20, C_TEXT_DIM, 18, 12);
    for (uint32_t i = 0; i < COUGH_NIGHT_BUCKET_COUNT; ++i)
    {
        night_bars[i] = lv_obj_create(chart_box);
        lv_obj_set_size(night_bars[i], bar_w, 2);
        lv_obj_set_pos(night_bars[i],
                       16 + (lv_coord_t)i * (bar_w + gap),
                       DETAIL_CHART_H - 16);
        lv_obj_set_style_bg_color(night_bars[i], C_AMBER, 0);
        lv_obj_set_style_bg_opa(night_bars[i], LV_OPA_COVER, 0);
        lv_obj_set_style_radius(night_bars[i], 4, 0);
        lv_obj_set_style_border_width(night_bars[i], 0, 0);
        lv_obj_set_style_pad_all(night_bars[i], 0, 0);
        lv_obj_clear_flag(night_bars[i], LV_OBJ_FLAG_SCROLLABLE);
    }

    lbl_night_empty = make_simple_label(chart_box, "暂无咳嗽事件", FONT_20,
                                        C_TEXT_DIM, 292, 92);
    make_simple_label(page_night, "0h", FONT_16, C_TEXT_DIM,
                      DETAIL_CHART_X + 16, DETAIL_STAT_Y);
    make_simple_label(page_night, "6h", FONT_16, C_TEXT_DIM,
                      DETAIL_CHART_X + 360, DETAIL_STAT_Y);
    make_simple_label(page_night, "12h", FONT_16, C_TEXT_DIM,
                      DETAIL_CHART_X + 700, DETAIL_STAT_Y);
    make_page_indicator(page_night, "4/4");

    add_gesture_bubble_to_children(page_night);
}

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t then_ms)
{
    return (uint32_t)(now_ms - then_ms);
}

static void metric_history_push(metric_history_t *history, uint16_t value_x10)
{
    history->values[history->head] = value_x10;
    history->head = (uint8_t)((history->head + 1U) % HISTORY_SAMPLE_COUNT);
    if (history->count < HISTORY_SAMPLE_COUNT)
    {
        history->count++;
    }
}

static void metric_history_update(metric_history_t *history,
                                  uint16_t value_x10,
                                  uint32_t now_ms)
{
    if (0U == value_x10)
    {
        return;
    }

    if ((!history->last_sample_valid) ||
        (elapsed_ms(now_ms, history->last_sample_ms) >= HISTORY_SAMPLE_PERIOD_MS))
    {
        metric_history_push(history, value_x10);
        history->last_sample_ms = now_ms;
        history->last_sample_valid = true;
    }
}

static uint8_t metric_history_values(const metric_history_t *history,
                                     uint16_t *out,
                                     uint8_t out_count)
{
    uint8_t count = history->count;
    uint8_t start;

    if (count > out_count)
    {
        count = out_count;
    }

    start = (uint8_t)((history->head + HISTORY_SAMPLE_COUNT - count) %
                      HISTORY_SAMPLE_COUNT);
    for (uint8_t i = 0; i < count; ++i)
    {
        out[i] = history->values[(start + i) % HISTORY_SAMPLE_COUNT];
    }

    return count;
}

static bool metric_history_stats(const metric_history_t *history,
                                 uint16_t *min_x10,
                                 uint16_t *max_x10,
                                 uint16_t *avg_x10)
{
    uint16_t values[HISTORY_SAMPLE_COUNT];
    uint32_t sum = 0;
    uint8_t count = metric_history_values(history, values, HISTORY_SAMPLE_COUNT);

    if (0U == count)
    {
        return false;
    }

    *min_x10 = values[0];
    *max_x10 = values[0];
    for (uint8_t i = 0; i < count; ++i)
    {
        if (values[i] < *min_x10)
        {
            *min_x10 = values[i];
        }
        if (values[i] > *max_x10)
        {
            *max_x10 = values[i];
        }
        sum += values[i];
    }

    *avg_x10 = (uint16_t)((sum + ((uint32_t)count / 2U)) / count);
    return true;
}

static void update_metric_detail(metric_detail_widgets_t *page,
                                 const metric_history_t *history,
                                 uint16_t current_x10)
{
    uint16_t values[HISTORY_SAMPLE_COUNT];
    uint8_t count;
    uint16_t min_x10 = 0;
    uint16_t max_x10 = 0;
    uint16_t avg_x10 = 0;
    char buf[40];

    if (0U != current_x10)
    {
        snprintf(buf, sizeof(buf), "%u",
                 (unsigned int)((current_x10 + 5U) / 10U));
        lv_label_set_text(page->lbl_state, "实时中");
    }
    else
    {
        snprintf(buf, sizeof(buf), "--");
        lv_label_set_text(page->lbl_state, "等待数据");
    }
    lv_label_set_text(page->lbl_current, buf);

    if (metric_history_stats(history, &min_x10, &max_x10, &avg_x10))
    {
        snprintf(buf, sizeof(buf), "Min %u",
                 (unsigned int)((min_x10 + 5U) / 10U));
        lv_label_set_text(page->lbl_min, buf);
        snprintf(buf, sizeof(buf), "Max %u",
                 (unsigned int)((max_x10 + 5U) / 10U));
        lv_label_set_text(page->lbl_max, buf);
        snprintf(buf, sizeof(buf), "Avg %u",
                 (unsigned int)((avg_x10 + 5U) / 10U));
        lv_label_set_text(page->lbl_avg, buf);
    }
    else
    {
        lv_label_set_text(page->lbl_min, "Min --");
        lv_label_set_text(page->lbl_max, "Max --");
        lv_label_set_text(page->lbl_avg, "Avg --");
    }

    count = metric_history_values(history, values, HISTORY_SAMPLE_COUNT);
    if (0U == count)
    {
        lv_obj_add_flag(page->line, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(page->empty, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if (1U == count)
    {
        values[1] = values[0];
        count = 2U;
    }

    if (max_x10 <= min_x10)
    {
        max_x10 = (uint16_t)(min_x10 + 10U);
    }

    for (uint8_t i = 0; i < count; ++i)
    {
        uint32_t x = ((uint32_t)i * (DETAIL_CHART_W - 34U)) /
                     (uint32_t)(count - 1U);
        uint32_t y = ((uint32_t)(max_x10 - values[i]) *
                      (DETAIL_CHART_H - 34U)) /
                     (uint32_t)(max_x10 - min_x10);

        page->points[i].x = (lv_value_precise_t)x;
        page->points[i].y = (lv_value_precise_t)y;
    }

    lv_line_set_points_mutable(page->line, page->points, count);
    lv_obj_remove_flag(page->line, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(page->empty, LV_OBJ_FLAG_HIDDEN);
}

static void cough_note_session_start(uint32_t now_ms)
{
    if (!s_cough_session_start_valid)
    {
        s_cough_session_start_ms = now_ms;
        s_cough_session_start_valid = true;
    }
}

static void cough_recent_push(uint32_t now_ms)
{
    s_cough_recent_ts[s_cough_recent_head] = now_ms;
    s_cough_recent_head =
        (uint8_t)((s_cough_recent_head + 1U) % COUGH_EVENT_RING_COUNT);
    if (s_cough_recent_count < COUGH_EVENT_RING_COUNT)
    {
        s_cough_recent_count++;
    }
}

static void cough_bucket_push(uint32_t now_ms)
{
    uint32_t bucket;

    cough_note_session_start(now_ms);
    bucket = elapsed_ms(now_ms, s_cough_session_start_ms) /
             COUGH_NIGHT_BUCKET_MS;
    if (bucket >= COUGH_NIGHT_BUCKET_COUNT)
    {
        bucket = COUGH_NIGHT_BUCKET_COUNT - 1U;
    }
    s_cough_night_buckets[bucket]++;
}

static void record_cough_events(uint32_t now_ms, uint32_t count)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        cough_recent_push(now_ms);
        cough_bucket_push(now_ms);
    }
}

static uint32_t cough_recent_count(uint32_t now_ms)
{
    uint32_t count = 0;
    uint8_t ring_count = s_cough_recent_count;
    uint8_t start =
        (uint8_t)((s_cough_recent_head + COUGH_EVENT_RING_COUNT - ring_count) %
                  COUGH_EVENT_RING_COUNT);

    for (uint8_t i = 0; i < ring_count; ++i)
    {
        uint32_t ts = s_cough_recent_ts[(start + i) % COUGH_EVENT_RING_COUNT];

        if (elapsed_ms(now_ms, ts) <= COUGH_RECENT_WINDOW_MS)
        {
            count++;
        }
    }

    return count;
}

static void update_night_page(uint32_t total, uint32_t recent_30min)
{
    uint32_t max_bucket = 0;
    const lv_coord_t gap = 10;
    const lv_coord_t bar_w =
        (DETAIL_CHART_W - 32 - ((lv_coord_t)COUGH_NIGHT_BUCKET_COUNT - 1) * gap) /
        (lv_coord_t)COUGH_NIGHT_BUCKET_COUNT;
    char buf[48];

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)total);
    lv_label_set_text(lbl_night_total, buf);
    lv_obj_align_to(lbl_night_total_unit, lbl_night_total,
                    LV_ALIGN_OUT_RIGHT_MID, 8, 4);

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)recent_30min);
    lv_label_set_text(lbl_night_recent, buf);
    lv_obj_align_to(lbl_night_recent_unit, lbl_night_recent,
                    LV_ALIGN_OUT_RIGHT_MID, 8, 4);

    for (uint32_t i = 0; i < COUGH_NIGHT_BUCKET_COUNT; ++i)
    {
        if (s_cough_night_buckets[i] > max_bucket)
        {
            max_bucket = s_cough_night_buckets[i];
        }
    }

    if (0U == max_bucket)
    {
        lv_obj_remove_flag(lbl_night_empty, LV_OBJ_FLAG_HIDDEN);
        max_bucket = 1U;
    }
    else
    {
        lv_obj_add_flag(lbl_night_empty, LV_OBJ_FLAG_HIDDEN);
    }

    for (uint32_t i = 0; i < COUGH_NIGHT_BUCKET_COUNT; ++i)
    {
        lv_coord_t h =
            (lv_coord_t)((s_cough_night_buckets[i] * (DETAIL_CHART_H - 34U)) /
                         max_bucket);
        if (h < 2)
        {
            h = 2;
        }

        lv_obj_set_size(night_bars[i], bar_w, h);
        lv_obj_set_pos(night_bars[i],
                       16 + (lv_coord_t)i * (bar_w + gap),
                       DETAIL_CHART_H - 16 - h);
        lv_obj_set_style_bg_opa(night_bars[i],
                                (s_cough_night_buckets[i] > 0U) ?
                                LV_OPA_COVER : LV_OPA_30,
                                0);
    }
}

static void set_wake_capsule(void)
{
    char buf[32];
    if (NULL == lbl_subtitle)
    {
        return;
    }

    snprintf(buf, sizeof(buf), "起夜%lu次", (unsigned long)s_wake_count);
    lv_label_set_text(lbl_subtitle, buf);
    lv_obj_set_size(lbl_subtitle, WAKE_CAPSULE_W, WAKE_CAPSULE_H);
    lv_obj_set_style_text_color(lbl_subtitle, C_WHITE, 0);
    lv_obj_set_style_bg_color(lbl_subtitle, C_CARD_BG, 0);
    lv_obj_set_style_bg_opa(lbl_subtitle, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(lbl_subtitle, C_CARD_BORDER, 0);
    lv_obj_set_style_border_width(lbl_subtitle, 1, 0);
    lv_obj_set_style_radius(lbl_subtitle, WAKE_CAPSULE_R, 0);
    lv_obj_set_style_pad_hor(lbl_subtitle, 18, 0);
    lv_obj_set_style_pad_ver(lbl_subtitle, 8, 0);
    lv_obj_set_style_text_align(lbl_subtitle, LV_TEXT_ALIGN_CENTER, 0);
}

static bool wake_radar_is_usable(const app_display_cm55_snapshot_t *snap)
{
    return (NULL != snap) && (DISPLAY_CM55_RADAR_NORMAL == snap->radar_source);
}

static bool wake_distance_is_far(uint16_t distance_cm)
{
    if ((0U == s_wake_near_distance_cm) || (0U == distance_cm))
    {
        return false;
    }

    return ((uint32_t)distance_cm >=
            ((uint32_t)s_wake_near_distance_cm + WAKE_FAR_DELTA_CM));
}

static void wake_note_present_distance(uint16_t distance_cm)
{
    if (0U == distance_cm)
    {
        return;
    }

    if ((0U == s_wake_near_distance_cm) ||
        (distance_cm < s_wake_near_distance_cm))
    {
        s_wake_near_distance_cm = distance_cm;
    }
}

static bool wake_window_expired(uint32_t now_ms)
{
    return ((uint32_t)(now_ms - s_wake_window_start_ms) > WAKE_WINDOW_MS);
}

static void wake_start_far_window(uint32_t now_ms)
{
    s_wake_state = WAKE_STATE_FAR_WINDOW;
    s_wake_window_start_ms = now_ms;
    s_wake_absent_hits = 0U;
}

static void update_wake_state(const app_display_cm55_snapshot_t *snap,
                              uint32_t now_ms)
{
    if (!wake_radar_is_usable(snap))
    {
        return;
    }

    if ((0U != snap->radar_presence) && (snap->distance_cm > 0U))
    {
        bool far_now;

        if ((WAKE_STATE_WAIT_PRESENT == s_wake_state) ||
            (WAKE_STATE_ABSENT_COUNTED == s_wake_state))
        {
            s_wake_state = WAKE_STATE_PRESENT;
            s_wake_near_distance_cm = snap->distance_cm;
            s_wake_window_start_ms = now_ms;
            s_wake_absent_hits = 0U;
            return;
        }

        wake_note_present_distance(snap->distance_cm);
        far_now = wake_distance_is_far(snap->distance_cm);
        if (far_now)
        {
            if ((WAKE_STATE_FAR_WINDOW != s_wake_state) ||
                wake_window_expired(now_ms))
            {
                wake_start_far_window(now_ms);
            }
        }
        else
        {
            s_wake_state = WAKE_STATE_PRESENT;
            s_wake_window_start_ms = now_ms;
            s_wake_absent_hits = 0U;
        }
        return;
    }

    if ((0U == snap->radar_presence) &&
        (WAKE_STATE_FAR_WINDOW == s_wake_state))
    {
        if (wake_window_expired(now_ms))
        {
            s_wake_state = WAKE_STATE_WAIT_PRESENT;
            s_wake_window_start_ms = now_ms;
            s_wake_absent_hits = 0U;
            return;
        }

        if (s_wake_absent_hits < WAKE_ABSENT_CONFIRM_HITS)
        {
            s_wake_absent_hits++;
        }
        if (s_wake_absent_hits >= WAKE_ABSENT_CONFIRM_HITS)
        {
            s_wake_count++;
            s_wake_state = WAKE_STATE_ABSENT_COUNTED;
            s_wake_absent_hits = 0U;
        }
    }
}

static void set_metric_value(lv_obj_t *value, lv_obj_t *unit,
                             const char *text, lv_coord_t unit_y_ofs)
{
    lv_label_set_text(value, text);
    lv_obj_update_layout(value);
    lv_obj_align_to(unit, value, LV_ALIGN_OUT_RIGHT_MID, 8, unit_y_ofs);
}

static void update_clock_label(uint32_t epoch_s, uint32_t time_flags)
{
    char buf[16];

    if ((0U == epoch_s) ||
        (0U == (time_flags & DISPLAY_CM55_TIME_FLAG_VALID)))
    {
        lv_label_set_text(lbl_clock, "--:--");
        lv_obj_set_style_text_color(lbl_clock, C_TEXT_DIM, 0);
        return;
    }

    uint32_t local_s = epoch_s + (uint32_t)APP_DISPLAY_LVGL_TIMEZONE_OFFSET_S;
    uint32_t day_s = local_s % (24U * 60U * 60U);
    uint32_t hour = day_s / (60U * 60U);
    uint32_t minute = (day_s / 60U) % 60U;

    snprintf(buf, sizeof(buf), "%02lu:%02lu",
             (unsigned long)hour,
             (unsigned long)minute);
    lv_label_set_text(lbl_clock, buf);

    if (0U != (time_flags & DISPLAY_CM55_TIME_FLAG_BLE_SYNCED))
    {
        lv_obj_set_style_text_color(lbl_clock, C_GREEN, 0);
    }
    else if (0U != (time_flags & DISPLAY_CM55_TIME_FLAG_NVM_LOADED))
    {
        lv_obj_set_style_text_color(lbl_clock, C_AMBER, 0);
    }
    else
    {
        lv_obj_set_style_text_color(lbl_clock, C_TEXT, 0);
    }
}

static bool ui_deadline_active(uint32_t deadline_ms, uint32_t now_ms)
{
    return ((int32_t)(deadline_ms - now_ms) > 0);
}

static bool update_cough_event_latch(uint32_t last_cough_event_id,
                                     uint32_t cough_event_count_total,
                                     bool snapshot_cough_event_active,
                                     uint32_t now_ms)
{
    bool new_event = false;
    uint32_t new_event_count = 0;

    if (!s_cough_event_total_valid)
    {
        s_last_cough_event_total = cough_event_count_total;
        s_cough_event_total_valid = true;
    }
    else if (cough_event_count_total > s_last_cough_event_total)
    {
        new_event_count = cough_event_count_total - s_last_cough_event_total;
        s_last_cough_event_total = cough_event_count_total;
    }
    else if (cough_event_count_total < s_last_cough_event_total)
    {
        s_last_cough_event_total = cough_event_count_total;
    }

    if (0u != last_cough_event_id)
    {
        if (!s_cough_event_id_valid)
        {
            s_last_cough_event_id = last_cough_event_id;
            s_cough_event_id_valid = true;
            new_event = snapshot_cough_event_active;
        }
        else if (last_cough_event_id != s_last_cough_event_id)
        {
            s_last_cough_event_id = last_cough_event_id;
            new_event = true;
        }
    }

    if ((0U == new_event_count) && new_event)
    {
        new_event_count = 1U;
    }

    if (0U != new_event_count)
    {
        new_event = true;
        record_cough_events(now_ms, new_event_count);
        s_cough_event_hold_until_ms =
            now_ms + (uint32_t)APP_DISPLAY_LVGL_COUGH_EVENT_HOLD_MS;
    }

    return snapshot_cough_event_active ||
           ui_deadline_active(s_cough_event_hold_until_ms, now_ms);
}

static void update_cough_labels(uint32_t cough_recent_30min,
                                uint32_t cough_event_count_total,
                                bool cough_event_active)
{
    char buf[48];

    snprintf(buf, sizeof(buf), "整夜: %lu 次",
             (unsigned long)cough_event_count_total);
    lv_label_set_text(lbl_cough_all, buf);
    snprintf(buf, sizeof(buf), "最近30min: %lu 次",
             (unsigned long)cough_recent_30min);
    lv_label_set_text(lbl_cough_half, buf);

    if (cough_event_active)
    {
        set_badge_pill(lbl_cough_badge, "检测到咳嗽", C_AMBER_DIM, C_AMBER);
    }
    else
    {
        set_badge_pill(lbl_cough_badge, "无明显咳嗽", C_GREEN_DIM, C_GREEN);
    }
}

static lv_obj_t *make_dot(lv_obj_t *parent, lv_color_t color)
{
    lv_obj_t *dot = lv_led_create(parent);
    lv_obj_set_size(dot, 10, 10);
    lv_led_set_color(dot, color);
    lv_led_on(dot);
    return dot;
}

/* ------------------------------------------------------------------ */
/* Snapshot read                                                      */
/* ------------------------------------------------------------------ */
static uint32_t s_snap_fail_count;
static uint32_t s_snap_ok_count;

static void copy_snapshot_ui_fields(app_display_cm55_snapshot_t *out,
                                    volatile app_display_cm55_snapshot_t *snap)
{
    out->magic = snap->magic;
    out->version = snap->version;
    out->heartbeat = snap->heartbeat;
    out->timestamp_ms = snap->timestamp_ms;
    out->health_state = snap->health_state;
    out->alert_code = snap->alert_code;
    out->radar_source = snap->radar_source;
    out->flags = snap->flags;
    out->mic_cough_prob_x1000 = snap->mic_cough_prob_x1000;
    out->audio_quality = snap->audio_quality;
    out->cough_model_not_verified = snap->cough_model_not_verified;
    out->rr_bpm_x10 = snap->rr_bpm_x10;
    out->hr_bpm_x10 = snap->hr_bpm_x10;
    out->radar_quality = snap->radar_quality;
    out->radar_presence = snap->radar_presence;
    out->distance_cm = snap->distance_cm;
    out->cough_count_1min = snap->cough_count_1min;
    out->cough_count_5min = snap->cough_count_5min;
    out->cough_event_count_total = snap->cough_event_count_total;
    out->last_cough_event_id = snap->last_cough_event_id;
    out->fusion_confidence = snap->fusion_confidence;
    out->ble_connected = snap->ble_connected;
    out->wall_epoch_s = snap->wall_epoch_s;
    out->wall_time_flags = snap->wall_time_flags;
}

static bool read_snapshot(app_display_cm55_snapshot_t *out)
{
    volatile app_display_cm55_snapshot_t *snap = APP_DISPLAY_CM55_SNAPSHOT;
    const uint32_t max_attempts = 4U;

    for (uint32_t attempt = 0U; attempt < max_attempts; ++attempt) {
        APP_DISPLAY_CM55_INVALIDATE_CACHE((uint32_t)snap, sizeof(*snap));

        out->magic = snap->magic;
        out->version = snap->version;
        out->seq_begin = snap->seq_begin;
        out->seq_end = snap->seq_end;
        out->heartbeat = snap->heartbeat;

        if (out->magic != APP_DISPLAY_CM55_SNAPSHOT_MAGIC) {
            s_snap_fail_count++;
            LVGL_DASH_LOG("[LVGL_DBG] snap magic=0x%08lX fail=%lu\r\n",
                          (unsigned long)out->magic,
                          (unsigned long)s_snap_fail_count);
            return false;
        }

        if (out->version != APP_DISPLAY_CM55_SNAPSHOT_VERSION) {
            s_snap_fail_count++;
            LVGL_DASH_LOG("[LVGL_DBG] snap version=%lu fail=%lu\r\n",
                          (unsigned long)out->version,
                          (unsigned long)s_snap_fail_count);
            return false;
        }

        /*
         * CM33 publishes complete display snapshots with seq_begin == seq_end.
         * The heartbeat value itself may be odd, so parity is not a valid
         * in-progress marker for this bridge.
         */
        uint32_t seq_end = snap->seq_end;
        __DMB();
        copy_snapshot_ui_fields(out, snap);
        __DMB();
        uint32_t seq_begin = snap->seq_begin;
        out->seq_begin = seq_begin;
        out->seq_end = seq_end;

        if (seq_begin == seq_end) {
            s_snap_ok_count++;
            return true;
        }
    }

    s_snap_fail_count++;
    LVGL_DASH_LOG("[LVGL_DBG] snap torn fail=%lu begin=%lu end=%lu\r\n",
                  (unsigned long)s_snap_fail_count,
                  (unsigned long)snap->seq_begin,
                  (unsigned long)snap->seq_end);
    return false;
}

static void set_snapshot_diag(const app_display_cm55_snapshot_t *snap)
{
    char buf[80];

    snprintf(buf, sizeof(buf), "LVGLDBG M=%08lX V=%lu B=%lu E=%lu H=%lu",
             (unsigned long)snap->magic,
             (unsigned long)snap->version,
             (unsigned long)snap->seq_begin,
             (unsigned long)snap->seq_end,
             (unsigned long)snap->heartbeat);
    lv_label_set_text(lbl_trend, buf);

    snprintf(buf, sizeof(buf), "RR=%u HR=%u D=%u P=%u OK=%lu F=%lu",
             (unsigned int)((snap->rr_bpm_x10 + 5U) / 10U),
             (unsigned int)((snap->hr_bpm_x10 + 5U) / 10U),
             (unsigned int)snap->distance_cm,
             (unsigned int)snap->radar_presence,
             (unsigned long)s_snap_ok_count,
             (unsigned long)s_snap_fail_count);
    lv_label_set_text(lbl_diag, buf);
}

#if (APP_DISPLAY_LVGL_TOUCH_ENABLE && APP_DISPLAY_LVGL_TOUCH_DIAG_ENABLE)
static void set_touch_diag(void)
{
    lv_port_indev_status_t status;
    char buf[80];

    lv_port_indev_get_status(&status);

    snprintf(buf, sizeof(buf),
             "TOUCH init=%lu r=%ld rd=%lu ok=%lu err=%lu",
             status.initialized ? 1UL : 0UL,
             (long)status.init_result,
             (unsigned long)status.read_count,
             (unsigned long)status.success_count,
             (unsigned long)status.error_count);
    lv_label_set_text(lbl_trend, buf);

    snprintf(buf, sizeof(buf),
             "ev=%lu tc=%lu press=%lu sw=%lu dir=%ld xy=%d,%d",
             (unsigned long)status.last_event,
             (unsigned long)status.last_touch_count,
             (unsigned long)status.press_count,
             (unsigned long)status.swipe_count,
             (long)status.last_swipe_dir,
             (int)status.last_x,
             (int)status.last_y);
    lv_label_set_text(lbl_diag, buf);
}
#endif

#if (APP_DISPLAY_LVGL_SNAPSHOT_PROBE_ENABLE)
static void run_snapshot_probe(uint32_t tick)
{
    volatile app_display_cm55_snapshot_t *snap = APP_DISPLAY_CM55_SNAPSHOT;
    char buf[80];
    uint32_t phase = tick % 8U;

    lv_label_set_text(lbl_subtitle, "LVGL snapshot probe");

    if (0U == phase) {
        snprintf(buf, sizeof(buf), "PROBE0 T=%lu no-read",
                 (unsigned long)tick);
        lv_label_set_text(lbl_trend, buf);
        lv_label_set_text(lbl_diag, "next: address only");
        return;
    }

    if (1U == phase) {
        snprintf(buf, sizeof(buf), "PROBE1 addr=0x%08lX",
                 (unsigned long)(uintptr_t)snap);
        lv_label_set_text(lbl_trend, buf);
        lv_label_set_text(lbl_diag, "next: read magic");
        return;
    }

    if (2U == phase) {
        uint32_t magic = snap->magic;
        snprintf(buf, sizeof(buf), "PROBE2 magic=0x%08lX",
                 (unsigned long)magic);
        lv_label_set_text(lbl_trend, buf);
        lv_label_set_text(lbl_diag, "next: read version");
        return;
    }

    if (3U == phase) {
        uint32_t version = snap->version;
        snprintf(buf, sizeof(buf), "PROBE3 version=%lu",
                 (unsigned long)version);
        lv_label_set_text(lbl_trend, buf);
        lv_label_set_text(lbl_diag, "next: read seq");
        return;
    }

    if (4U == phase) {
        uint32_t begin = snap->seq_begin;
        uint32_t end = snap->seq_end;
        snprintf(buf, sizeof(buf), "PROBE4 B=%lu E=%lu",
                 (unsigned long)begin,
                 (unsigned long)end);
        lv_label_set_text(lbl_trend, buf);
        lv_label_set_text(lbl_diag, "next: read heartbeat");
        return;
    }

    if (5U == phase) {
        uint32_t heartbeat = snap->heartbeat;
        snprintf(buf, sizeof(buf), "PROBE5 H=%lu",
                 (unsigned long)heartbeat);
        lv_label_set_text(lbl_trend, buf);
        lv_label_set_text(lbl_diag, "next: read vitals");
        return;
    }

    if (6U == phase) {
        uint32_t rr = snap->rr_bpm_x10;
        uint32_t hr = snap->hr_bpm_x10;
        uint32_t dist = snap->distance_cm;
        uint32_t presence = snap->radar_presence;
        snprintf(buf, sizeof(buf), "PROBE6 RR=%lu HR=%lu D=%lu P=%lu",
                 (unsigned long)rr,
                 (unsigned long)hr,
                 (unsigned long)dist,
                 (unsigned long)presence);
        lv_label_set_text(lbl_trend, buf);
        lv_label_set_text(lbl_diag, "next: repeat");
        return;
    }

    snprintf(buf, sizeof(buf), "PROBE7 T=%lu ok",
             (unsigned long)tick);
    lv_label_set_text(lbl_trend, buf);
    lv_label_set_text(lbl_diag, "probe loop alive");
}
#endif

#if (APP_DISPLAY_LVGL_MINIMAL_REALTIME_ENABLE)
static void run_minimal_realtime(uint32_t tick)
{
    volatile app_display_cm55_snapshot_t *snap = APP_DISPLAY_CM55_SNAPSHOT;
    char buf[80];

    uint32_t magic = snap->magic;
    uint32_t version = snap->version;
    uint32_t begin = snap->seq_begin;
    uint32_t end = snap->seq_end;
    uint32_t timestamp_ms = snap->timestamp_ms;
    uint32_t rr = snap->rr_bpm_x10;
    uint32_t hr = snap->hr_bpm_x10;
    uint32_t dist = snap->distance_cm;
    uint32_t presence = snap->radar_presence;
    uint32_t cough_prob = snap->mic_cough_prob_x1000;
    uint32_t cough_1min = snap->cough_count_1min;
    uint32_t cough_5min = snap->cough_count_5min;
    uint32_t cough_event_total = snap->cough_event_count_total;
    uint32_t last_cough_event_id = snap->last_cough_event_id;
    uint32_t alert_code = snap->alert_code;
    uint32_t flags = snap->flags;
    uint32_t ble_connected = snap->ble_connected;
    uint32_t wall_epoch_s = snap->wall_epoch_s;
    uint32_t wall_time_flags = snap->wall_time_flags;
    uint32_t now_ms = (timestamp_ms > 0U) ? timestamp_ms : (tick * 1000U);
    bool cough_event_active;
    uint32_t cough_30min;

    if ((APP_DISPLAY_CM55_SNAPSHOT_MAGIC == magic) &&
        (APP_DISPLAY_CM55_SNAPSHOT_VERSION == version) &&
        (begin == end))
    {
        s_snap_ok_count++;
    }
    else
    {
        s_snap_fail_count++;
    }

    set_wake_capsule();

    update_clock_label(wall_epoch_s, wall_time_flags);

    if (rr > 0U)
    {
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)((rr + 5U) / 10U));
        set_metric_value(lbl_rr_value, lbl_rr_unit, buf, 4);
        set_badge_pill(lbl_rr_badge, "平稳", C_GREEN_DIM, C_GREEN);
    }
    else
    {
        set_metric_value(lbl_rr_value, lbl_rr_unit, "--", 4);
        set_badge_pill(lbl_rr_badge, "暂无数据", C_CYAN_DIM, C_CYAN);
    }

    if (hr > 0U)
    {
        uint32_t hr_int = (hr + 5U) / 10U;
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)hr_int);
        set_metric_value(lbl_hr_value, lbl_hr_unit, buf, 4);
        if (hr_int > 100U)
            set_badge_pill(lbl_hr_badge, "偏高", C_ROSE_DIM, C_ROSE);
        else
            set_badge_pill(lbl_hr_badge, "正常", C_GREEN_DIM, C_GREEN);
    }
    else
    {
        set_metric_value(lbl_hr_value, lbl_hr_unit, "--", 4);
        set_badge_pill(lbl_hr_badge, "暂无数据", C_ROSE_DIM, C_ROSE);
    }

    cough_note_session_start(now_ms);
    metric_history_update(&s_rr_history, (uint16_t)rr, now_ms);
    metric_history_update(&s_hr_history, (uint16_t)hr, now_ms);

    (void)cough_1min;
    (void)cough_prob;
    (void)cough_5min;
    cough_event_active = update_cough_event_latch(
        last_cough_event_id,
        cough_event_total,
        ((DISPLAY_CM55_ALERT_COUGH_BURST == alert_code) &&
         (0u != (flags & DISPLAY_CM55_FLAG_ALERT_LATCHED)) &&
         (0u != last_cough_event_id)),
        now_ms);
    cough_30min = cough_recent_count(now_ms);
    update_cough_labels(cough_30min,
                        cough_event_total,
                        cough_event_active);

    if (dist > 0U)
    {
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)dist);
        set_metric_value(lbl_radar_value, lbl_radar_unit, buf, 4);
    }
    else
    {
        set_metric_value(lbl_radar_value, lbl_radar_unit, "--", 4);
    }

    if (0U != presence)
    {
        set_badge_pill(lbl_radar_presence, "已检测到人体", C_GREEN_DIM, C_GREEN);
    }
    else
    {
        set_badge_pill(lbl_radar_presence, "未检测到人体", C_PURPLE_DIM, C_PURPLE);
    }

#if (APP_DISPLAY_LVGL_FOOTER_DIAG_ENABLE)
    uint32_t heartbeat = snap->heartbeat;

    snprintf(buf, sizeof(buf), "MINRT T=%lu M=%08lX V=%lu B=%lu E=%lu H=%lu",
             (unsigned long)tick,
             (unsigned long)magic,
             (unsigned long)version,
             (unsigned long)begin,
             (unsigned long)end,
             (unsigned long)heartbeat);
    lv_label_set_text(lbl_trend, buf);

    snprintf(buf, sizeof(buf),
             "RR=%lu HR=%lu D=%lu P=%lu C=%lu/%lu Q=%lu OK=%lu F=%lu",
             (unsigned long)((rr + 5U) / 10U),
             (unsigned long)((hr + 5U) / 10U),
             (unsigned long)dist,
             (unsigned long)presence,
             (unsigned long)cough_1min,
             (unsigned long)cough_5min,
             (unsigned long)cough_prob,
             (unsigned long)s_snap_ok_count,
             (unsigned long)s_snap_fail_count);
    lv_label_set_text(lbl_diag, buf);
#endif

    lv_led_set_color(dot_mic, (flags & 0x01U) ? C_GREEN : C_TEXT_DIM);
    lv_led_set_color(dot_radar, (flags & 0x02U) ? C_GREEN : C_TEXT_DIM);
    lv_led_set_color(dot_ble, (0U != ble_connected) ? C_GREEN : C_TEXT_DIM);

    update_metric_detail(&s_rr_detail, &s_rr_history, (uint16_t)rr);
    update_metric_detail(&s_hr_detail, &s_hr_history, (uint16_t)hr);
    update_night_page(cough_event_total, cough_30min);
#if (APP_DISPLAY_LVGL_TOUCH_ENABLE && APP_DISPLAY_LVGL_TOUCH_DIAG_ENABLE)
    set_touch_diag();
#endif
}
#endif

/* ------------------------------------------------------------------ */
/* Init                                                               */
/* ------------------------------------------------------------------ */
void ui_health_dashboard_init(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, C_BG_DEEP, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    /* ---- Outer container ---- */
    lv_obj_t *outer = lv_obj_create(scr);
    lv_obj_set_size(outer, OUTER_W, OUTER_H);
    lv_obj_set_pos(outer, OUTER_X, OUTER_Y);
    lv_obj_set_style_bg_color(outer, C_BG_DEEP, 0);
    lv_obj_set_style_bg_opa(outer, LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(outer, OUTER_R, 0);
    lv_obj_set_style_border_color(outer, C_OUTER_BORDER, 0);
    lv_obj_set_style_border_width(outer, 1, 0);
    lv_obj_set_style_pad_all(outer, 0, 0);
    lv_obj_clear_flag(outer, LV_OBJ_FLAG_SCROLLABLE);
    page_home = outer;
    lv_obj_add_event_cb(page_home, page_gesture_cb, LV_EVENT_GESTURE, NULL);

    /* ---- Wake capsule ---- */
    lbl_subtitle = lv_label_create(outer);
    lv_obj_set_style_text_font(lbl_subtitle, FONT_28, 0);
    lv_obj_set_pos(lbl_subtitle, TITLE_X - OUTER_X, TITLE_Y - OUTER_Y);
    set_wake_capsule();

    lbl_clock = lv_label_create(outer);
    lv_label_set_text(lbl_clock, "--:--");
    lv_obj_set_style_text_color(lbl_clock, C_TEXT_DIM, 0);
    lv_obj_set_style_text_font(lbl_clock, FONT_UNIT, 0);
    lv_obj_align(lbl_clock, LV_ALIGN_TOP_MID, 0, 22);

    /* ---- Status capsule ---- */
    capsule_status = lv_obj_create(outer);
    lv_obj_set_size(capsule_status, CAPSULE_W, CAPSULE_H);
    lv_obj_set_pos(capsule_status, CAPSULE_X - OUTER_X, CAPSULE_Y - OUTER_Y);
    lv_obj_set_style_bg_color(capsule_status, C_CAPSULE_BG, 0);
    lv_obj_set_style_bg_opa(capsule_status, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(capsule_status, 23, 0);
    lv_obj_set_style_border_color(capsule_status, C_GREEN, 0);
    lv_obj_set_style_border_width(capsule_status, 1, 0);
    lv_obj_set_style_pad_all(capsule_status, 4, 0);
    lv_obj_clear_flag(capsule_status, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *dot_st = make_dot(capsule_status, C_GREEN);
    lv_obj_align(dot_st, LV_ALIGN_LEFT_MID, 8, 0);

    lbl_status = lv_label_create(capsule_status);
    lv_label_set_text(lbl_status, "实时监测中");
    lv_obj_set_style_text_color(lbl_status, C_GREEN, 0);
    lv_obj_set_style_text_font(lbl_status, FONT_20, 0);
    lv_obj_align_to(lbl_status, dot_st, LV_ALIGN_OUT_RIGHT_MID, 6, 0);

    /* ---- Cards ---- */
    lv_obj_t *card_rr = make_card(outer, CARD_X0 - OUTER_X, CARD_TOP_Y - OUTER_Y,
                                  CARD_W, CARD_H_L, C_CARD_BORDER);
    lv_obj_t *card_hr = make_card(outer, CARD_X1 - OUTER_X, CARD_TOP_Y - OUTER_Y,
                                  CARD_W, CARD_H_L, C_CARD_BORDER);
    lv_obj_t *card_cg = make_card(outer, CARD_X0 - OUTER_X, CARD_BOT_Y - OUTER_Y,
                                  CARD_W, CARD_H_L, C_CARD_BORDER);
    lv_obj_t *card_rd = make_card(outer, CARD_X1 - OUTER_X, CARD_BOT_Y - OUTER_Y,
                                  CARD_W, CARD_H_R, C_CARD_BORDER);

    /* -- Icon badges -- */
    lv_obj_t *badge_rr = make_icon_badge(card_rr, C_CYAN_DIM, C_CYAN, "R");
    lv_obj_align(badge_rr, LV_ALIGN_TOP_LEFT, 0, 4);

    lv_obj_t *badge_hr = make_icon_badge(card_hr, C_ROSE_DIM, C_ROSE, "H");
    lv_obj_align(badge_hr, LV_ALIGN_TOP_LEFT, 0, 4);

    lv_obj_t *badge_cg = make_icon_badge(card_cg, C_AMBER_DIM, C_AMBER, "C");
    lv_obj_align(badge_cg, LV_ALIGN_TOP_LEFT, 0, 4);

    lv_obj_t *badge_rd = make_icon_badge(card_rd, C_PURPLE_DIM, C_PURPLE, "D");
    lv_obj_align(badge_rd, LV_ALIGN_TOP_LEFT, 0, 4);

    /* --- 呼吸率 card --- */
    lv_obj_t *lbl_rr_title = lv_label_create(card_rr);
    lv_label_set_text(lbl_rr_title, "呼吸率");
    lv_obj_set_style_text_color(lbl_rr_title, C_TEXT, 0);
    lv_obj_set_style_text_font(lbl_rr_title, FONT_28, 0);
    lv_obj_align_to(lbl_rr_title, badge_rr, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    lbl_rr_value = lv_label_create(card_rr);
    lv_label_set_text(lbl_rr_value, "--");
    lv_obj_set_style_text_color(lbl_rr_value, C_CYAN, 0);
    lv_obj_set_style_text_font(lbl_rr_value, FONT_48, 0);
    lv_obj_align(lbl_rr_value, LV_ALIGN_TOP_LEFT, 84, 62);

    lbl_rr_unit = lv_label_create(card_rr);
    lv_label_set_text(lbl_rr_unit, "bpm");
    lv_obj_set_style_text_color(lbl_rr_unit, C_TEXT, 0);
    lv_obj_set_style_text_font(lbl_rr_unit, FONT_UNIT, 0);
    lv_obj_align_to(lbl_rr_unit, lbl_rr_value, LV_ALIGN_OUT_RIGHT_MID, 8, 4);

    lbl_rr_badge = make_badge_pill(card_rr, "暂无数据", C_CYAN_DIM, C_CYAN);
    lv_obj_align(lbl_rr_badge, LV_ALIGN_TOP_RIGHT, -8, 14);

    /* --- 心率 card --- */
    lv_obj_t *lbl_hr_title = lv_label_create(card_hr);
    lv_label_set_text(lbl_hr_title, "心率");
    lv_obj_set_style_text_color(lbl_hr_title, C_TEXT, 0);
    lv_obj_set_style_text_font(lbl_hr_title, FONT_28, 0);
    lv_obj_align_to(lbl_hr_title, badge_hr, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    lbl_hr_value = lv_label_create(card_hr);
    lv_label_set_text(lbl_hr_value, "--");
    lv_obj_set_style_text_color(lbl_hr_value, C_ROSE, 0);
    lv_obj_set_style_text_font(lbl_hr_value, FONT_48, 0);
    lv_obj_align(lbl_hr_value, LV_ALIGN_TOP_LEFT, 84, 62);

    lbl_hr_unit = lv_label_create(card_hr);
    lv_label_set_text(lbl_hr_unit, "bpm");
    lv_obj_set_style_text_color(lbl_hr_unit, C_TEXT, 0);
    lv_obj_set_style_text_font(lbl_hr_unit, FONT_UNIT, 0);
    lv_obj_align_to(lbl_hr_unit, lbl_hr_value, LV_ALIGN_OUT_RIGHT_MID, 8, 4);

    lbl_hr_badge = make_badge_pill(card_hr, "暂无数据", C_ROSE_DIM, C_ROSE);
    lv_obj_align(lbl_hr_badge, LV_ALIGN_TOP_RIGHT, -8, 14);

    /* --- 咳嗽事件 card --- */
    lv_obj_t *lbl_cg_title = lv_label_create(card_cg);
    lv_label_set_text(lbl_cg_title, "咳嗽事件");
    lv_obj_set_style_text_color(lbl_cg_title, C_TEXT, 0);
    lv_obj_set_style_text_font(lbl_cg_title, FONT_28, 0);
    lv_obj_align_to(lbl_cg_title, badge_cg, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    lbl_cough_all = lv_label_create(card_cg);
    lv_label_set_text(lbl_cough_all, "整夜: -- 次");
    lv_obj_set_style_text_color(lbl_cough_all, C_TEXT, 0);
    lv_obj_set_style_text_font(lbl_cough_all, FONT_28, 0);
    lv_obj_align(lbl_cough_all, LV_ALIGN_TOP_LEFT, 84, 62);

    lbl_cough_half = lv_label_create(card_cg);
    lv_label_set_text(lbl_cough_half, "最近30min: -- 次");
    lv_obj_set_style_text_color(lbl_cough_half, C_TEXT, 0);
    lv_obj_set_style_text_font(lbl_cough_half, FONT_28, 0);
    lv_obj_align(lbl_cough_half, LV_ALIGN_TOP_LEFT, 84, 98);

    lbl_cough_badge = make_badge_pill(card_cg, "无明显咳嗽", C_GREEN_DIM, C_GREEN);
    lv_obj_align(lbl_cough_badge, LV_ALIGN_TOP_RIGHT, -8, 14);

    /* --- 雷达 card --- */
    lv_obj_t *lbl_rd_title = lv_label_create(card_rd);
    lv_label_set_text(lbl_rd_title, "雷达");
    lv_obj_set_style_text_color(lbl_rd_title, C_TEXT, 0);
    lv_obj_set_style_text_font(lbl_rd_title, FONT_28, 0);
    lv_obj_align_to(lbl_rd_title, badge_rd, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

    lbl_radar_value = lv_label_create(card_rd);
    lv_label_set_text(lbl_radar_value, "--");
    lv_obj_set_style_text_color(lbl_radar_value, C_PURPLE, 0);
    lv_obj_set_style_text_font(lbl_radar_value, FONT_48, 0);
    lv_obj_align(lbl_radar_value, LV_ALIGN_TOP_LEFT, 84, 62);

    lbl_radar_unit = lv_label_create(card_rd);
    lv_label_set_text(lbl_radar_unit, "cm");
    lv_obj_set_style_text_color(lbl_radar_unit, C_TEXT, 0);
    lv_obj_set_style_text_font(lbl_radar_unit, FONT_UNIT, 0);
    lv_obj_align_to(lbl_radar_unit, lbl_radar_value, LV_ALIGN_OUT_RIGHT_MID, 8, 4);

    lbl_radar_presence = make_badge_pill(card_rd, "未检测到人体", C_PURPLE_DIM, C_PURPLE);
    lv_obj_align(lbl_radar_presence, LV_ALIGN_TOP_RIGHT, -8, 14);

    /* ---- Footer ---- */
    lv_obj_t *footer = lv_obj_create(outer);
    lv_obj_set_size(footer, FOOTER_W, FOOTER_H);
    lv_obj_set_pos(footer, FOOTER_X - OUTER_X, FOOTER_Y - OUTER_Y);
    lv_obj_set_style_bg_color(footer, C_FOOTER_BG, 0);
    lv_obj_set_style_bg_opa(footer, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(footer, 8, 0);
    lv_obj_set_style_border_color(footer, C_CARD_BORDER, 0);
    lv_obj_set_style_border_width(footer, 1, 0);
    lv_obj_set_style_pad_all(footer, 0, 0);
    lv_obj_clear_flag(footer, LV_OBJ_FLAG_SCROLLABLE);
#if !(APP_DISPLAY_LVGL_FOOTER_DIAG_ENABLE || \
      (APP_DISPLAY_LVGL_TOUCH_ENABLE && APP_DISPLAY_LVGL_TOUCH_DIAG_ENABLE))
    lv_obj_add_flag(footer, LV_OBJ_FLAG_HIDDEN);
#endif

    /* Trend text */
    lbl_trend = lv_label_create(footer);
    lv_label_set_text(lbl_trend, "LVGLDBG waiting");
    lv_obj_set_style_text_color(lbl_trend, C_TEXT_DIM, 0);
    lv_obj_set_style_text_font(lbl_trend, FONT_16, 0);
    lv_label_set_long_mode(lbl_trend, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(lbl_trend, FOOTER_W - 230);
    lv_obj_set_pos(lbl_trend, 12, 3);

    lbl_diag = lv_label_create(footer);
    lv_label_set_text(lbl_diag, "RR=-- HR=-- D=-- P=-- OK=0 F=0");
    lv_obj_set_style_text_color(lbl_diag, C_TEXT_DIM, 0);
    lv_obj_set_style_text_font(lbl_diag, FONT_16, 0);
    lv_label_set_long_mode(lbl_diag, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(lbl_diag, FOOTER_W - 230);
    lv_obj_set_pos(lbl_diag, 12, 22);

    /* Status dots — right side of footer */
    lv_obj_t *lbl_mic = lv_label_create(footer);
    lv_label_set_text(lbl_mic, "MIC");
    lv_obj_set_style_text_color(lbl_mic, C_TEXT_DIM, 0);
    lv_obj_set_style_text_font(lbl_mic, FONT_16, 0);
    lv_obj_set_pos(lbl_mic, FOOTER_W - 200, 14);
    dot_mic = make_dot(footer, C_TEXT_DIM);
    lv_obj_align_to(dot_mic, lbl_mic, LV_ALIGN_OUT_RIGHT_MID, 6, 0);

    lv_obj_t *lbl_radar_dot = lv_label_create(footer);
    lv_label_set_text(lbl_radar_dot, "Radar");
    lv_obj_set_style_text_color(lbl_radar_dot, C_TEXT_DIM, 0);
    lv_obj_set_style_text_font(lbl_radar_dot, FONT_16, 0);
    lv_obj_align_to(lbl_radar_dot, dot_mic, LV_ALIGN_OUT_RIGHT_MID, 14, 0);
    dot_radar = make_dot(footer, C_TEXT_DIM);
    lv_obj_align_to(dot_radar, lbl_radar_dot, LV_ALIGN_OUT_RIGHT_MID, 6, 0);

    lv_obj_t *lbl_ble = lv_label_create(footer);
    lv_label_set_text(lbl_ble, "BLE");
    lv_obj_set_style_text_color(lbl_ble, C_TEXT_DIM, 0);
    lv_obj_set_style_text_font(lbl_ble, FONT_16, 0);
    lv_obj_align_to(lbl_ble, dot_radar, LV_ALIGN_OUT_RIGHT_MID, 14, 0);
    dot_ble = make_dot(footer, C_TEXT_DIM);
    lv_obj_align_to(dot_ble, lbl_ble, LV_ALIGN_OUT_RIGHT_MID, 6, 0);

    add_gesture_bubble_to_children(page_home);
    create_metric_detail_page(scr, &s_rr_detail,
                              "呼吸 近30min", "2/4", C_CYAN);
    page_rr_detail = s_rr_detail.root;
    create_metric_detail_page(scr, &s_hr_detail,
                              "心率 近30min", "3/4", C_ROSE);
    page_hr_detail = s_hr_detail.root;
    create_night_page(scr);
#if (APP_DISPLAY_LVGL_TOUCH_ENABLE)
    lv_port_indev_set_swipe_cb(touch_swipe_cb);
#endif
    show_page(UI_PAGE_HOME);
}

/* ------------------------------------------------------------------ */
/* Update                                                             */
/* ------------------------------------------------------------------ */
void ui_health_dashboard_update(void)
{
    static uint32_t s_update_count;

    s_update_count++;

#if (APP_DISPLAY_LVGL_SKIP_SNAPSHOT_READ_ENABLE)
    {
        char diag_buf[64];

        snprintf(diag_buf, sizeof(diag_buf), "LVGLUPD T=%lu SNAP=OFF",
                 (unsigned long)s_update_count);
        lv_label_set_text(lbl_subtitle, "LVGL update only");
        lv_label_set_text(lbl_trend, diag_buf);
        lv_label_set_text(lbl_diag, "NO SNAP READ");
        LVGL_DASH_LOG("[LVGL_UPD] tick=%lu snap_read=off\r\n",
                      (unsigned long)s_update_count);
        return;
    }
#endif

#if (APP_DISPLAY_LVGL_SNAPSHOT_PROBE_ENABLE)
    run_snapshot_probe(s_update_count);
    return;
#endif

#if (APP_DISPLAY_LVGL_MINIMAL_REALTIME_ENABLE)
    run_minimal_realtime(s_update_count);
    return;
#endif

    app_display_cm55_snapshot_t snap;
    if (!read_snapshot(&snap)) {
        set_wake_capsule();
        update_clock_label(0u, 0u);
        set_snapshot_diag(&snap);
        return;
    }

    if (s_update_count <= 3U || (s_update_count % 30U) == 0U) {
        LVGL_DASH_LOG("[LVGL_UPD] ok=%lu fail=%lu rr=%u hr=%u dist=%u presence=%u\r\n",
                      (unsigned long)s_snap_ok_count, (unsigned long)s_snap_fail_count,
                      snap.rr_bpm_x10, snap.hr_bpm_x10, snap.distance_cm,
                      snap.radar_presence);
    }

    char buf[48];
    uint32_t cough_30min;
    bool cough_event_active;

    /* ---- Wake capsule: RAM-only demo count from radar distance/presence ---- */
    uint32_t wake_now_ms = (snap.timestamp_ms > 0U) ?
                           snap.timestamp_ms :
                           (s_update_count * 1000U);
    cough_note_session_start(wake_now_ms);
    metric_history_update(&s_rr_history, snap.rr_bpm_x10, wake_now_ms);
    metric_history_update(&s_hr_history, snap.hr_bpm_x10, wake_now_ms);
    update_wake_state(&snap, wake_now_ms);
    set_wake_capsule();

    update_clock_label(snap.wall_epoch_s, snap.wall_time_flags);

    /* ---- 呼吸率 (integer display) ---- */
    if (snap.rr_bpm_x10 > 0U) {
        uint16_t rr_int = (snap.rr_bpm_x10 + 5U) / 10U;  /* round */
        snprintf(buf, sizeof(buf), "%u", rr_int);
        set_metric_value(lbl_rr_value, lbl_rr_unit, buf, 4);
        set_badge_pill(lbl_rr_badge, "平稳", C_GREEN_DIM, C_GREEN);
    } else {
        set_metric_value(lbl_rr_value, lbl_rr_unit, "--", 4);
        set_badge_pill(lbl_rr_badge, "暂无数据", C_CYAN_DIM, C_CYAN);
    }

    /* ---- 心率 (integer display) ---- */
    if (snap.hr_bpm_x10 > 0U) {
        uint16_t hr_int = (snap.hr_bpm_x10 + 5U) / 10U;
        snprintf(buf, sizeof(buf), "%u", hr_int);
        set_metric_value(lbl_hr_value, lbl_hr_unit, buf, 4);
        if (hr_int > 100U) {
            set_badge_pill(lbl_hr_badge, "偏高", C_ROSE_DIM, C_ROSE);
        } else {
            set_badge_pill(lbl_hr_badge, "正常", C_GREEN_DIM, C_GREEN);
        }
    } else {
        set_metric_value(lbl_hr_value, lbl_hr_unit, "--", 4);
        set_badge_pill(lbl_hr_badge, "暂无数据", C_ROSE_DIM, C_ROSE);
    }

    /* ---- 咳嗽 ---- */
    cough_event_active = update_cough_event_latch(
        snap.last_cough_event_id,
        snap.cough_event_count_total,
        ((DISPLAY_CM55_ALERT_COUGH_BURST == snap.alert_code) &&
         (0u != (snap.flags & DISPLAY_CM55_FLAG_ALERT_LATCHED)) &&
         (0u != snap.last_cough_event_id)),
        wake_now_ms);
    cough_30min = cough_recent_count(wake_now_ms);
    update_cough_labels(cough_30min,
                        snap.cough_event_count_total,
                        cough_event_active);

    /* ---- 雷达 (integer display) ---- */
    if (snap.distance_cm > 0U) {
        snprintf(buf, sizeof(buf), "%u", snap.distance_cm);
        set_metric_value(lbl_radar_value, lbl_radar_unit, buf, 4);
    } else {
        set_metric_value(lbl_radar_value, lbl_radar_unit, "--", 4);
    }
    if (snap.radar_presence) {
        set_badge_pill(lbl_radar_presence, "已检测到人体", C_GREEN_DIM, C_GREEN);
    } else {
        set_badge_pill(lbl_radar_presence, "未检测到人体", C_PURPLE_DIM, C_PURPLE);
    }

    /* ---- Footer ---- */
    if (snap.health_state == 1U)
        lv_label_set_text(lbl_trend, "睡眠呼吸趋势平稳");
    else if (snap.health_state == 2U)
        lv_label_set_text(lbl_trend, "呼吸趋势异常，请关注");
    else if (snap.health_state == 3U)
        lv_label_set_text(lbl_trend, "警告：请检查");
    else
        lv_label_set_text(lbl_trend, "等待数据...");

    set_snapshot_diag(&snap);

    lv_led_set_color(dot_mic, (snap.flags & 0x01U) ? C_GREEN : C_TEXT_DIM);
    lv_led_set_color(dot_radar, (snap.flags & 0x02U) ? C_GREEN : C_TEXT_DIM);
    lv_led_set_color(dot_ble, snap.ble_connected ? C_GREEN : C_TEXT_DIM);

    update_metric_detail(&s_rr_detail, &s_rr_history, snap.rr_bpm_x10);
    update_metric_detail(&s_hr_detail, &s_hr_history, snap.hr_bpm_x10);
    update_night_page(snap.cough_event_count_total, cough_30min);
#if (APP_DISPLAY_LVGL_TOUCH_ENABLE && APP_DISPLAY_LVGL_TOUCH_DIAG_ENABLE)
    set_touch_diag();
#endif
}

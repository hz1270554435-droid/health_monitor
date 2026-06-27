/**
 * @file ui_health_dashboard.c
 * Health monitor dashboard UI — LVGL v2 polished product screen.
 *
 * Layout (800 × 480 visible, 832 frame, 32px HOFFSET):
 *   ┌─────────────────────────────────────────────────────────┐
 *   │  (outer rounded container 14..818, 14..466)              │
 *   │  夜间健康监测                      [● 实时监测中]         │
 *   │  今晚状态整体平稳                                        │
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
#include "fonts/lv_font_simsun_16_e84.h"
#include "fonts/lv_font_simsun_20_e84.h"
#include "fonts/lv_font_simsun_28_e84.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifndef APP_DISPLAY_LVGL_TIMEZONE_OFFSET_S
#define APP_DISPLAY_LVGL_TIMEZONE_OFFSET_S (8 * 60 * 60)
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
#define SUBTITLE_Y         (TITLE_Y + 42U)

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

/* Display-local state only for log dedup. Counts remain producer-owned. */
static bool s_logged_total_valid;
static uint32_t s_last_logged_total;

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

static void set_status_hint(const char *text, lv_color_t bg, lv_color_t fg)
{
    if (NULL == lbl_subtitle)
    {
        return;
    }

    lv_label_set_text(lbl_subtitle, text);
    lv_obj_set_style_text_color(lbl_subtitle, fg, 0);
    lv_obj_set_style_bg_color(lbl_subtitle, bg, 0);
    lv_obj_set_style_bg_opa(lbl_subtitle, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(lbl_subtitle, fg, 0);
    lv_obj_set_style_border_width(lbl_subtitle, 1, 0);
    lv_obj_set_style_radius(lbl_subtitle, 16, 0);
    lv_obj_set_style_pad_hor(lbl_subtitle, 12, 0);
    lv_obj_set_style_pad_ver(lbl_subtitle, 3, 0);
    lv_obj_set_width(lbl_subtitle, LV_SIZE_CONTENT);
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

static void update_cough_labels(uint16_t cough_5min,
                                uint32_t cough_event_count_total)
{
    char buf[48];
    s_last_logged_total = cough_event_count_total;
    s_logged_total_valid = true;

    snprintf(buf, sizeof(buf), "整夜: %lu 次",
             (unsigned long)cough_event_count_total);
    lv_label_set_text(lbl_cough_all, buf);
    snprintf(buf, sizeof(buf), "最近5min: %u 次",
             (unsigned int)cough_5min);
    lv_label_set_text(lbl_cough_half, buf);

    if (cough_5min > 0U)
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

    (void)tick;

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
    uint32_t health_state = snap->health_state;
    uint32_t flags = snap->flags;
    uint32_t ble_connected = snap->ble_connected;
    uint32_t wall_epoch_s = snap->wall_epoch_s;
    uint32_t wall_time_flags = snap->wall_time_flags;

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

    if (1U == health_state)
        set_status_hint("今晚状态整体平稳", C_GREEN_DIM, C_GREEN);
    else if (2U == health_state)
        set_status_hint("请注意呼吸异常", C_AMBER_DIM, C_AMBER);
    else if (3U == health_state)
        set_status_hint("警告：请检查", C_ROSE_DIM, C_ROSE);
    else
        set_status_hint("实时数据同步中", C_CYAN_DIM, C_CYAN);

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

    (void)cough_1min;
    (void)cough_prob;
    (void)timestamp_ms;
    (void)tick;
    update_cough_labels((uint16_t)cough_5min, cough_event_total);

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

    /* ---- Title ---- */
    lv_obj_t *lbl_title = lv_label_create(outer);
    lv_label_set_text(lbl_title, "夜间健康监测");
    lv_obj_set_style_text_color(lbl_title, C_TEXT, 0);
    lv_obj_set_style_text_font(lbl_title, FONT_28, 0);
    lv_obj_set_pos(lbl_title, TITLE_X - OUTER_X, TITLE_Y - OUTER_Y);

    lbl_subtitle = lv_label_create(outer);
    lv_obj_set_style_text_font(lbl_subtitle, FONT_20, 0);
    lv_obj_set_pos(lbl_subtitle, TITLE_X - OUTER_X, SUBTITLE_Y - OUTER_Y + 4);
    set_status_hint("今晚状态整体平稳", C_GREEN_DIM, C_GREEN);

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
#if !(APP_DISPLAY_LVGL_FOOTER_DIAG_ENABLE)
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
        lv_label_set_text(lbl_subtitle, "等待数据...");
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

    /* ---- Subtitle (also shows debug info) ---- */
    {
        volatile app_display_cm55_snapshot_t *raw = APP_DISPLAY_CM55_SNAPSHOT;
        APP_DISPLAY_CM55_INVALIDATE_CACHE((uint32_t)raw, sizeof(*raw));
        if (raw->magic != APP_DISPLAY_CM55_SNAPSHOT_MAGIC) {
            snprintf(buf, sizeof(buf), "MAGIC=0x%08lX", (unsigned long)raw->magic);
            set_status_hint(buf, C_ROSE_DIM, C_ROSE);
        } else if (snap.health_state == 1U) {
            set_status_hint("今晚状态整体平稳", C_GREEN_DIM, C_GREEN);
        } else if (snap.health_state == 2U) {
            set_status_hint("请注意呼吸异常", C_AMBER_DIM, C_AMBER);
        } else if (snap.health_state == 3U) {
            set_status_hint("警告：请检查", C_ROSE_DIM, C_ROSE);
        } else {
            snprintf(buf, sizeof(buf), "INIT seq=%lu rr=%u",
                     (unsigned long)raw->seq_begin, snap.rr_bpm_x10);
            set_status_hint(buf, C_CYAN_DIM, C_CYAN);
        }
    }

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
    update_cough_labels(snap.cough_count_5min,
                        snap.cough_event_count_total);

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
}

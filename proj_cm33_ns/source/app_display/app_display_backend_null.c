#include "app_display.h"

#if (APP_DISPLAY_ENABLE)

#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

static uint32_t last_snapshot_log_ms;
static uint32_t last_page_log_ms;
static e84_display_page_t last_logged_page = E84_DISPLAY_PAGE_COUNT;

static uint32_t app_display_backend_now_ms(void);
static void app_display_backend_print_float_3(float value);
#if (!APP_DISPLAY_FINAL_MOCK_ENABLE && !APP_DISPLAY_PRODUCT_SCREEN_ENABLE)
static void app_display_backend_print_home_page(
    const e84_display_view_model_t *view,
    uint32_t now_ms);
static void app_display_backend_print_alert_page(
    const e84_display_view_model_t *view,
    uint32_t now_ms);
static void app_display_backend_print_debug_page(
    const e84_display_view_model_t *view,
    uint32_t now_ms);
#endif
#if (APP_DISPLAY_FINAL_MOCK_ENABLE || APP_DISPLAY_PRODUCT_SCREEN_ENABLE)
static void app_display_backend_print_final_mock_screen(
    const e84_display_view_model_t *view,
    uint32_t now_ms);
#if (APP_DISPLAY_FINAL_UART_LOG_ENABLE)
static const char *app_display_backend_mock_scenario(
    const e84_display_snapshot_t *snapshot);
#endif
#endif

cy_rslt_t app_display_backend_null_init(void)
{
    return CY_RSLT_SUCCESS;
}

void app_display_backend_null_render_snapshot(
    const e84_display_snapshot_t *snapshot,
    bool force)
{
    uint32_t now_ms = app_display_backend_now_ms();

    if (NULL == snapshot)
    {
        return;
    }

    if (!APP_DISPLAY_NULL_SNAPSHOT_LOG_ENABLE)
    {
        return;
    }

    if ((!force) &&
        (0u != last_snapshot_log_ms) &&
        ((now_ms - last_snapshot_log_ms) <
         APP_DISPLAY_NULL_LOG_MIN_PERIOD_MS))
    {
        return;
    }

    last_snapshot_log_ms = now_ms;

    printf("[DISPLAY_SNAPSHOT] t_ms=%lu source_ts_ms=%lu health=%s "
           "alert=%s radar_presence=%u breath_rate_bpm=",
           (unsigned long)now_ms,
           (unsigned long)snapshot->timestamp_ms,
           e84_display_health_state_name(snapshot->health_state),
           e84_display_alert_name(snapshot->active_alert),
           snapshot->radar_presence ? 1u : 0u);
    app_display_backend_print_float_3(snapshot->breath_rate_bpm);
    printf(" heart_rate_bpm=");
    app_display_backend_print_float_3(snapshot->heart_rate_bpm);
    printf(" radar_quality=%u distance_cm=%u mic_cough_prob=",
           (unsigned int)snapshot->radar_quality,
           (unsigned int)snapshot->distance_cm);
    app_display_backend_print_float_3(snapshot->mic_cough_prob);
    printf(" cough_count_1min=%u cough_count_5min=%u audio_quality=%u "
           "fusion_confidence=%u ble_connected=%u flags=0x%08lx\r\n",
           (unsigned int)snapshot->cough_count_1min,
           (unsigned int)snapshot->cough_count_5min,
           (unsigned int)snapshot->audio_quality,
           (unsigned int)snapshot->fusion_confidence,
           snapshot->ble_connected ? 1u : 0u,
           (unsigned long)snapshot->flags);
    fflush(stdout);
}

void app_display_backend_null_render_alert(e84_display_alert_t alert,
                                           const char *action,
                                           uint8_t severity,
                                           uint8_t confidence,
                                           uint32_t flags,
                                           uint32_t now_ms)
{
    if (NULL == action)
    {
        action = "unknown";
    }

    printf("[DISPLAY_ALERT] t_ms=%lu action=%s alert=%s severity=%u "
           "confidence=%u flags=0x%08lx\r\n",
           (unsigned long)now_ms,
           action,
           e84_display_alert_name(alert),
           (unsigned int)severity,
           (unsigned int)confidence,
           (unsigned long)flags);
    fflush(stdout);
}

void app_display_backend_null_render_view(
    const e84_display_view_model_t *view,
    bool force)
{
    uint32_t now_ms = app_display_backend_now_ms();
    bool page_changed;

    if (NULL == view)
    {
        return;
    }

    page_changed = (last_logged_page != view->current_page);
    if ((!force) &&
        (!page_changed) &&
        (0u != last_page_log_ms) &&
        ((now_ms - last_page_log_ms) <
         APP_DISPLAY_NULL_LOG_MIN_PERIOD_MS))
    {
        return;
    }

    last_page_log_ms = now_ms;
    last_logged_page = view->current_page;

#if (APP_DISPLAY_FINAL_MOCK_ENABLE || APP_DISPLAY_PRODUCT_SCREEN_ENABLE)
    app_display_backend_print_final_mock_screen(view, now_ms);
#else
    switch (view->current_page)
    {
        case E84_DISPLAY_PAGE_ALERT:
            app_display_backend_print_alert_page(view, now_ms);
            break;

        case E84_DISPLAY_PAGE_DEBUG:
            app_display_backend_print_debug_page(view, now_ms);
            break;

        case E84_DISPLAY_PAGE_BOOT:
        case E84_DISPLAY_PAGE_HOME:
        default:
            app_display_backend_print_home_page(view, now_ms);
            break;
    }
#endif

    fflush(stdout);
}

static uint32_t app_display_backend_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

#if (!APP_DISPLAY_FINAL_MOCK_ENABLE && !APP_DISPLAY_PRODUCT_SCREEN_ENABLE)
static void app_display_backend_print_home_page(
    const e84_display_view_model_t *view,
    uint32_t now_ms)
{
    const e84_display_snapshot_t *snapshot = &view->snapshot;

    if (!APP_DISPLAY_NULL_PAGE_LOG_ENABLE)
    {
        return;
    }

    printf("[DISPLAY_PAGE] t_ms=%lu page=%s previous=%s reason=%s "
           "state=%s radar_presence=%u breath_rate_bpm=",
           (unsigned long)now_ms,
           e84_display_page_name(view->current_page),
           e84_display_page_name(view->previous_page),
           (NULL != view->refresh_reason) ?
           view->refresh_reason : "unknown",
           e84_display_health_state_name(view->health_state),
           snapshot->radar_presence ? 1u : 0u);
    app_display_backend_print_float_3(snapshot->breath_rate_bpm);
    printf(" heart_rate_bpm=");
    app_display_backend_print_float_3(snapshot->heart_rate_bpm);
    printf(" mic_cough_prob=");
    app_display_backend_print_float_3(snapshot->mic_cough_prob);
    printf(" cough_count_1min=%u audio_quality=%u radar_quality=%u "
           "fusion_confidence=%u ble_connected=%u dropped=%lu\r\n",
           (unsigned int)snapshot->cough_count_1min,
           (unsigned int)snapshot->audio_quality,
           (unsigned int)snapshot->radar_quality,
           (unsigned int)snapshot->fusion_confidence,
           snapshot->ble_connected ? 1u : 0u,
           (unsigned long)view->display_dropped_commands);
}

static void app_display_backend_print_alert_page(
    const e84_display_view_model_t *view,
    uint32_t now_ms)
{
    uint32_t elapsed_ms =
        now_ms - view->alert.raised_timestamp_ms;

    if (!APP_DISPLAY_NULL_PAGE_LOG_ENABLE)
    {
        return;
    }

    printf("[DISPLAY_PAGE] t_ms=%lu page=ALERT previous=%s reason=%s "
           "severity=%s code=%s title=\"%s\" message=\"%s\" "
           "raised_ms=%lu elapsed_ms=%lu timeout_ms=%lu dismissible=%u "
           "latched=%u state=%s\r\n",
           (unsigned long)now_ms,
           e84_display_page_name(view->previous_page),
           (NULL != view->refresh_reason) ?
           view->refresh_reason : "unknown",
           e84_display_severity_name(view->alert.severity),
           e84_display_alert_name(view->alert.code),
           (NULL != view->alert.title) ? view->alert.title : "",
           (NULL != view->alert.short_message) ?
           view->alert.short_message : "",
           (unsigned long)view->alert.raised_timestamp_ms,
           (unsigned long)elapsed_ms,
           (unsigned long)view->alert.timeout_ms,
           view->alert.dismissible ? 1u : 0u,
           view->alert.latched ? 1u : 0u,
           e84_display_health_state_name(view->health_state));
}

static void app_display_backend_print_debug_page(
    const e84_display_view_model_t *view,
    uint32_t now_ms)
{
    if (!APP_DISPLAY_NULL_PAGE_LOG_ENABLE)
    {
        return;
    }

    printf("[DISPLAY_PAGE] t_ms=%lu page=DEBUG previous=%s reason=%s "
           "last_snapshot_ts_ms=%lu active_page=%s active_alert=%s "
           "dropped=%lu smoke=%u last_refresh_ms=%lu dirty=%u\r\n",
           (unsigned long)now_ms,
           e84_display_page_name(view->previous_page),
           (NULL != view->refresh_reason) ?
           view->refresh_reason : "unknown",
           (unsigned long)view->snapshot.timestamp_ms,
           e84_display_page_name(view->current_page),
           e84_display_alert_name(view->alert.code),
           (unsigned long)view->display_dropped_commands,
           view->smoke_enabled ? 1u : 0u,
           (unsigned long)view->last_refresh_timestamp_ms,
           view->dirty ? 1u : 0u);
}
#endif /* !APP_DISPLAY_FINAL_MOCK_ENABLE && !APP_DISPLAY_PRODUCT_SCREEN_ENABLE */

static void app_display_backend_print_float_3(float value)
{
    const char *sign = "";
    uint32_t whole;
    uint32_t frac;

    if (0.0f > value)
    {
        sign = "-";
        value = -value;
    }

    whole = (uint32_t)value;
    frac = (uint32_t)(((value - (float)whole) * 1000.0f) + 0.5f);
    if (1000u <= frac)
    {
        whole++;
        frac -= 1000u;
    }

    printf("%s%lu.%03lu", sign, (unsigned long)whole, (unsigned long)frac);
}

#if (APP_DISPLAY_FINAL_MOCK_ENABLE || APP_DISPLAY_PRODUCT_SCREEN_ENABLE)
#if (APP_DISPLAY_FINAL_UART_LOG_ENABLE)
static const char *app_display_backend_radar_source_label(
    e84_display_radar_source_state_t state);
static const char *app_display_backend_radar_presence_text(
    const e84_display_snapshot_t *snapshot);
static const char *app_display_backend_radar_quality_label(
    const e84_display_snapshot_t *snapshot);
static const char *app_display_backend_main_status_text(
    const e84_display_snapshot_t *snapshot);
static const char *app_display_backend_radar_source_id(
    e84_display_radar_source_state_t state);
static const char *app_display_backend_radar_presence_id(
    const e84_display_snapshot_t *snapshot);
static const char *app_display_backend_radar_quality_id(
    const e84_display_snapshot_t *snapshot);
static void app_display_backend_print_final_audit(
    const e84_display_snapshot_t *snapshot,
    uint32_t now_ms,
    const char *path,
    const char *source,
    const char *scenario,
    bool rr_live,
    bool hr_live,
    bool distance_live);
#endif

static void app_display_backend_print_final_mock_screen(
    const e84_display_view_model_t *view,
    uint32_t now_ms)
{
#if (APP_DISPLAY_FINAL_UART_LOG_ENABLE)
    const e84_display_snapshot_t *snapshot = &view->snapshot;
    bool radar_normal =
        (E84_DISPLAY_RADAR_SOURCE_NORMAL == snapshot->radar_source_state);
    bool rr_valid =
        (0u != (snapshot->flags & E84_DISPLAY_FLAG_RR_VALID));
    bool hr_valid =
        (0u != (snapshot->flags & E84_DISPLAY_FLAG_HR_VALID));
#if (APP_DISPLAY_FINAL_MOCK_ENABLE)
    const char *path = "APP_DISPLAY_ENABLE+APP_DISPLAY_FINAL_MOCK_ENABLE";
    const char *source = "mock";
#else
    const char *path =
        "APP_DISPLAY_ENABLE+APP_DISPLAY_PRODUCT_SCREEN_ENABLE";
    const char *source = "summary_adapter";
#endif

    const char *scenario = app_display_backend_mock_scenario(snapshot);
    bool distance_live = radar_normal && (snapshot->distance_cm > 0u);

    printf("[DISPLAY_FINAL_SCREEN] t_ms=%lu path=%s source=%s scenario=%s "
           "radar_src=%s\r\n",
           (unsigned long)now_ms,
           path,
           source,
           scenario,
           e84_display_radar_source_state_name(
               snapshot->radar_source_state));
    printf("================================================\r\n");
    printf("  E84 夜间呼吸与咳嗽健康伴侣\r\n");
    printf("------------------------------------------------\r\n");
    printf("  状态: %s | 数据源: %s\r\n",
           app_display_backend_main_status_text(snapshot),
           app_display_backend_radar_source_label(
               snapshot->radar_source_state));
    printf("------------------------------------------------\r\n");
    printf("  雷达\r\n");
    printf("    来源: %s\r\n",
           app_display_backend_radar_source_label(
               snapshot->radar_source_state));
    printf("    人体: %s\r\n",
           app_display_backend_radar_presence_text(snapshot));
    printf("    呼吸率: ");
    if (radar_normal && rr_valid)
    {
        app_display_backend_print_float_3(snapshot->breath_rate_bpm);
        printf(" bpm\r\n");
    }
    else
    {
        printf("N/A\r\n");
    }
    printf("    心率: ");
    if (radar_normal && hr_valid)
    {
        app_display_backend_print_float_3(snapshot->heart_rate_bpm);
        printf(" bpm\r\n");
    }
    else
    {
        printf("N/A\r\n");
    }
    printf("    距离: ");
    if (distance_live)
    {
        app_display_backend_print_float_3(
            ((float)snapshot->distance_cm) / 100.0f);
        printf(" m\r\n");
    }
    else
    {
        printf("N/A\r\n");
    }
    printf("    质量: %s\r\n",
           app_display_backend_radar_quality_label(snapshot));
    printf("------------------------------------------------\r\n");
    printf("  咳嗽模型\r\n");
    printf("    状态: 模型未验证 / Not Verified\r\n");
    printf("    说明: 模型未完成板级验证\r\n");
    printf("------------------------------------------------\r\n");
    printf("  用于趋势观察与竞赛演示，不作为医学诊断\r\n");
    printf("================================================\r\n");
    app_display_backend_print_final_audit(snapshot,
                                          now_ms,
                                          path,
                                          source,
                                          scenario,
                                          radar_normal && rr_valid,
                                          radar_normal && hr_valid,
                                          distance_live);
#else
    (void)view;
    (void)now_ms;
#endif
}

#if (APP_DISPLAY_FINAL_UART_LOG_ENABLE)
static const char *app_display_backend_main_status_text(
    const e84_display_snapshot_t *snapshot)
{
    switch (snapshot->radar_source_state)
    {
        case E84_DISPLAY_RADAR_SOURCE_NORMAL:
            return "监测中";

        case E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE:
            return "数据暂不可用";

        case E84_DISPLAY_RADAR_SOURCE_STALE:
            return "雷达数据超时";

        case E84_DISPLAY_RADAR_SOURCE_LOW_QUALITY:
            return "信号较差";

        case E84_DISPLAY_RADAR_SOURCE_INVALID:
            return "数据无效";

        default:
            return "Not Verified";
    }
}

static const char *app_display_backend_mock_scenario(
    const e84_display_snapshot_t *snapshot)
{
    if (NULL == snapshot)
    {
        return "unknown";
    }

    switch (snapshot->radar_source_state)
    {
        case E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE:
            return "radar_unavailable";

        case E84_DISPLAY_RADAR_SOURCE_STALE:
            return "radar_stale";

        case E84_DISPLAY_RADAR_SOURCE_INVALID:
            return "radar_invalid";

        case E84_DISPLAY_RADAR_SOURCE_LOW_QUALITY:
            return "radar_low_quality";

        case E84_DISPLAY_RADAR_SOURCE_NORMAL:
        default:
            break;
    }

    switch (snapshot->health_state)
    {
        case E84_DISPLAY_HEALTH_NORMAL:
            return "real_radar_valid";

        case E84_DISPLAY_HEALTH_ATTENTION:
            return "attention";

        case E84_DISPLAY_HEALTH_WARNING:
            return "warning";

        default:
            return "other";
    }
}

static const char *app_display_backend_radar_source_label(
    e84_display_radar_source_state_t state)
{
    switch (state)
    {
        case E84_DISPLAY_RADAR_SOURCE_NORMAL:
            return "真实雷达联调";

        case E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE:
            return "雷达未连接";

        case E84_DISPLAY_RADAR_SOURCE_STALE:
            return "雷达数据超时";

        case E84_DISPLAY_RADAR_SOURCE_INVALID:
            return "数据无效";

        case E84_DISPLAY_RADAR_SOURCE_LOW_QUALITY:
            return "雷达信号较差";

        default:
            return "Not Verified";
    }
}

static const char *app_display_backend_radar_presence_text(
    const e84_display_snapshot_t *snapshot)
{
    switch (snapshot->radar_source_state)
    {
        case E84_DISPLAY_RADAR_SOURCE_NORMAL:
            return snapshot->radar_presence ? "有人" : "未检测到";

        case E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE:
            return "暂无数据";

        case E84_DISPLAY_RADAR_SOURCE_STALE:
            return "数据超时";

        case E84_DISPLAY_RADAR_SOURCE_INVALID:
            return "未验证";

        case E84_DISPLAY_RADAR_SOURCE_LOW_QUALITY:
            return "信号较差";

        default:
            return "Not Verified";
    }
}

static const char *app_display_backend_radar_quality_label(
    const e84_display_snapshot_t *snapshot)
{
    switch (snapshot->radar_source_state)
    {
        case E84_DISPLAY_RADAR_SOURCE_NORMAL:
            return (snapshot->radar_quality >= 35u) ? "良好" : "信号较差";

        case E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE:
            return "Not Verified";

        case E84_DISPLAY_RADAR_SOURCE_STALE:
            return "Not Verified";

        case E84_DISPLAY_RADAR_SOURCE_INVALID:
            return "Not Verified";

        case E84_DISPLAY_RADAR_SOURCE_LOW_QUALITY:
            return "信号较差";

        default:
            return "Not Verified";
    }
}

static void app_display_backend_print_final_audit(
    const e84_display_snapshot_t *snapshot,
    uint32_t now_ms,
    const char *path,
    const char *source,
    const char *scenario,
    bool rr_live,
    bool hr_live,
    bool distance_live)
{
    printf("[DISPLAY_FINAL_AUDIT] t_ms=%lu path=%s source=%s scenario=%s "
           "radar_src=%s source_id=%s presence_id=%s rr=%s hr=%s dist=%s "
           "rr_milli=%ld hr_milli=%ld distance_cm=%u quality_id=%s "
           "cough_not_verified=1 disclaimer=1\r\n",
           (unsigned long)now_ms,
           path,
           source,
           scenario,
           e84_display_radar_source_state_name(
               snapshot->radar_source_state),
           app_display_backend_radar_source_id(
               snapshot->radar_source_state),
           app_display_backend_radar_presence_id(snapshot),
           rr_live ? "live" : "NA",
           hr_live ? "live" : "NA",
           distance_live ? "live" : "NA",
           rr_live ? (long)(snapshot->breath_rate_bpm * 1000.0f) : -1L,
           hr_live ? (long)(snapshot->heart_rate_bpm * 1000.0f) : -1L,
           distance_live ? (unsigned int)snapshot->distance_cm : 0u,
           app_display_backend_radar_quality_id(snapshot));
}

static const char *app_display_backend_radar_source_id(
    e84_display_radar_source_state_t state)
{
    switch (state)
    {
        case E84_DISPLAY_RADAR_SOURCE_NORMAL:
            return "real_radar_joint";

        case E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE:
            return "radar_unavailable";

        case E84_DISPLAY_RADAR_SOURCE_STALE:
            return "radar_stale";

        case E84_DISPLAY_RADAR_SOURCE_INVALID:
            return "radar_invalid";

        case E84_DISPLAY_RADAR_SOURCE_LOW_QUALITY:
            return "radar_low_quality";

        default:
            return "not_verified";
    }
}

static const char *app_display_backend_radar_presence_id(
    const e84_display_snapshot_t *snapshot)
{
    switch (snapshot->radar_source_state)
    {
        case E84_DISPLAY_RADAR_SOURCE_NORMAL:
            return snapshot->radar_presence ? "person_present" :
                                              "not_detected";

        case E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE:
            return "no_data";

        case E84_DISPLAY_RADAR_SOURCE_STALE:
            return "timeout";

        case E84_DISPLAY_RADAR_SOURCE_INVALID:
            return "not_verified";

        case E84_DISPLAY_RADAR_SOURCE_LOW_QUALITY:
            return "low_quality";

        default:
            return "not_verified";
    }
}

static const char *app_display_backend_radar_quality_id(
    const e84_display_snapshot_t *snapshot)
{
    switch (snapshot->radar_source_state)
    {
        case E84_DISPLAY_RADAR_SOURCE_NORMAL:
            return (snapshot->radar_quality >= 35u) ? "good" :
                                                      "low_quality";

        case E84_DISPLAY_RADAR_SOURCE_LOW_QUALITY:
            return "low_quality";

        case E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE:
        case E84_DISPLAY_RADAR_SOURCE_STALE:
        case E84_DISPLAY_RADAR_SOURCE_INVALID:
        default:
            return "not_verified";
    }
}

#endif /* APP_DISPLAY_FINAL_UART_LOG_ENABLE */

#endif /* APP_DISPLAY_FINAL_MOCK_ENABLE || APP_DISPLAY_PRODUCT_SCREEN_ENABLE */

#endif /* APP_DISPLAY_ENABLE */

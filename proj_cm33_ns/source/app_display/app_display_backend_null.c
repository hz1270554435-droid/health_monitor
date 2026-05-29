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
static void app_display_backend_print_home_page(
    const e84_display_view_model_t *view,
    uint32_t now_ms);
static void app_display_backend_print_alert_page(
    const e84_display_view_model_t *view,
    uint32_t now_ms);
static void app_display_backend_print_debug_page(
    const e84_display_view_model_t *view,
    uint32_t now_ms);

cy_rslt_t app_display_backend_null_init(void)
{
    printf("[DISPLAY] backend=null init, min_log_ms=%lu, smoke=%lu\r\n",
           (unsigned long)APP_DISPLAY_NULL_LOG_MIN_PERIOD_MS,
           (unsigned long)APP_DISPLAY_SMOKE_ENABLE);
    fflush(stdout);
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
    printf(" radar_quality=%u mic_cough_prob=",
           (unsigned int)snapshot->radar_quality);
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

    fflush(stdout);
}

static uint32_t app_display_backend_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void app_display_backend_print_home_page(
    const e84_display_view_model_t *view,
    uint32_t now_ms)
{
    const e84_display_snapshot_t *snapshot = &view->snapshot;

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

#endif /* APP_DISPLAY_ENABLE */

#include "app_display.h"

#if (APP_DISPLAY_ENABLE)

#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

static uint32_t last_snapshot_log_ms;

static uint32_t app_display_backend_now_ms(void);
static void app_display_backend_print_float_3(float value);

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

static uint32_t app_display_backend_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
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

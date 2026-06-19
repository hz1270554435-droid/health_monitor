#include "app_display_summary_adapter.h"

#include "app_display.h"

#if (APP_DISPLAY_ENABLE && APP_DISPLAY_SUMMARY_ENABLE && \
     !APP_DISPLAY_SMOKE_ENABLE && !APP_DISPLAY_FINAL_MOCK_ENABLE)

#include <stdbool.h>
#include <string.h>

#ifndef APP_MONITOR_SUMMARY_ENABLE
#define APP_MONITOR_SUMMARY_ENABLE            (0u)
#endif

#ifndef APP_BLE_ENABLE
#define APP_BLE_ENABLE                        (0u)
#endif

#if (APP_MONITOR_SUMMARY_ENABLE)
#include "app_monitor_summary.h"
#include "../app_radar/app_radar_quality.h"
#else
#include "app_audio_deployment_config.h"
#include "../app_model_result_monitor/app_model_result_monitor.h"
#endif

#if (APP_BLE_ENABLE)
#include "app_ble_diag.h"
#endif

static uint32_t summary_last_publish_ms;
#if (APP_MONITOR_SUMMARY_ENABLE)
static uint32_t summary_last_event_id;
static e84_display_alert_t summary_last_display_alert;
#endif

#if (APP_MONITOR_SUMMARY_ENABLE)
static void app_display_summary_build_from_monitor(
    e84_display_snapshot_t *snapshot,
    const app_monitor_summary_snapshot_t *summary,
    uint32_t now_ms);
static e84_display_health_state_t app_display_summary_monitor_health(
    app_monitor_state_t state);
static e84_display_alert_t app_display_summary_monitor_alert(
    app_monitor_event_type_t event_type,
    app_monitor_alert_level_t alert_level);
static uint8_t app_display_summary_monitor_severity(
    app_monitor_alert_level_t alert_level);
static uint32_t app_display_summary_monitor_flags(
    const app_monitor_summary_event_t *event);
static void app_display_summary_publish_monitor_event(
    const app_monitor_summary_snapshot_t *summary);
#else
static void app_display_summary_build_snapshot(
    e84_display_snapshot_t *snapshot,
    const app_model_result_monitor_stats_t *model_stats,
    uint32_t now_ms);
static e84_display_health_state_t app_display_summary_select_health(
    const app_model_result_monitor_stats_t *model_stats);
static uint8_t app_display_summary_audio_quality(
    const app_model_result_monitor_stats_t *model_stats);
static float app_display_summary_clamp_probability(float value);
#endif
static bool app_display_summary_ble_connected(void);

cy_rslt_t app_display_summary_adapter_tick(uint32_t now_ms)
{
    e84_display_snapshot_t snapshot;

    if ((0u != summary_last_publish_ms) &&
        ((now_ms - summary_last_publish_ms) < APP_DISPLAY_SUMMARY_PERIOD_MS))
    {
        return CY_RSLT_SUCCESS;
    }

#if (APP_MONITOR_SUMMARY_ENABLE)
    {
        app_monitor_summary_snapshot_t summary;

        memset(&summary, 0, sizeof(summary));
        if (CY_RSLT_SUCCESS != app_monitor_summary_get_snapshot(&summary))
        {
            return CY_RSLT_TYPE_ERROR;
        }
        app_display_summary_build_from_monitor(&snapshot, &summary, now_ms);
        summary_last_publish_ms = now_ms;
        (void)app_display_publish_snapshot(&snapshot);
        app_display_summary_publish_monitor_event(&summary);
        return CY_RSLT_SUCCESS;
    }
#else
    {
        app_model_result_monitor_stats_t model_stats;

        memset(&model_stats, 0, sizeof(model_stats));
        app_model_result_monitor_get_stats(&model_stats);
        app_display_summary_build_snapshot(&snapshot, &model_stats, now_ms);
        summary_last_publish_ms = now_ms;
        return app_display_publish_snapshot(&snapshot);
    }
#endif
}

#if (APP_MONITOR_SUMMARY_ENABLE)
static void app_display_summary_build_from_monitor(
    e84_display_snapshot_t *snapshot,
    const app_monitor_summary_snapshot_t *summary,
    uint32_t now_ms)
{
    uint32_t flags = 0u;
    bool rr_valid;
    bool hr_valid;
    bool distance_valid;

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->timestamp_ms = now_ms;
    snapshot->health_state =
        app_display_summary_monitor_health(summary->monitor_state);
    snapshot->active_alert =
        app_display_summary_monitor_alert(summary->active_event_type,
                                          summary->alert_level);
    snapshot->radar_presence =
        (APP_MONITOR_PRESENCE_PRESENT == summary->radar.presence_state);
    snapshot->breath_rate_bpm =
        ((float)summary->radar.rr_bpm_x10) / 10.0f;
    snapshot->heart_rate_bpm =
        ((float)summary->radar.hr_bpm_x10) / 10.0f;
    snapshot->radar_quality = summary->radar.radar_quality;
    snapshot->distance_cm = summary->radar.distance_cm;
    snapshot->mic_cough_prob =
        ((float)summary->audio.cough_prob_x100) / 100.0f;
    snapshot->cough_count_1min = summary->cough_count_1min;
    snapshot->cough_count_5min = summary->cough_count_5min;
    snapshot->audio_quality = summary->audio.audio_quality;
    snapshot->fusion_confidence = summary->fusion_confidence;
    snapshot->ble_connected = app_display_summary_ble_connected() ||
                              summary->device.ble_connected;

    if (0u != (summary->source_valid_mask & APP_MONITOR_SOURCE_AUDIO))
    {
        flags |= E84_DISPLAY_FLAG_AUDIO_VALID;
    }
    if (0u != (summary->source_valid_mask & APP_MONITOR_SOURCE_RADAR))
    {
        flags |= E84_DISPLAY_FLAG_RADAR_VALID;
    }
    if (0u != (summary->source_valid_mask & APP_MONITOR_SOURCE_FUSION_SUMMARY))
    {
        flags |= E84_DISPLAY_FLAG_FUSION_VALID;
    }
    rr_valid =
        (APP_MONITOR_VITAL_NORMAL == summary->radar.breath_state) ||
        (APP_MONITOR_VITAL_LOW == summary->radar.breath_state) ||
        (APP_MONITOR_VITAL_HIGH == summary->radar.breath_state);
    hr_valid =
        (APP_MONITOR_VITAL_NORMAL == summary->radar.heart_state) ||
        (APP_MONITOR_VITAL_LOW == summary->radar.heart_state) ||
        (APP_MONITOR_VITAL_HIGH == summary->radar.heart_state);
    /* distance_valid is display-only: distance_cm=0 shows N/A in the backend
     * but does NOT force LOW_QUALITY state.  LD6002 may report distance=0
     * while still providing valid presence/RR/HR data. */
    distance_valid = (0u != summary->radar.distance_cm);

    if (rr_valid)
    {
        flags |= E84_DISPLAY_FLAG_RR_VALID;
    }
    if (hr_valid)
    {
        flags |= E84_DISPLAY_FLAG_HR_VALID;
    }
    if (snapshot->ble_connected)
    {
        flags |= E84_DISPLAY_FLAG_BLE_VALID;
    }
    if (summary->alert_level >= APP_MONITOR_ALERT_LEVEL_WARNING)
    {
        flags |= E84_DISPLAY_FLAG_ALERT_LATCHED;
    }

    snapshot->flags = flags;

    /* Map Monitor Summary radar flags to display radar source state.
     * Priority: unavailable > stale > low_quality > normal > invalid. */
    if (!summary->radar.ready)
    {
        snapshot->radar_source_state = E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE;
    }
    else if (summary->radar.stale)
    {
        snapshot->radar_source_state = E84_DISPLAY_RADAR_SOURCE_STALE;
    }
    /* LOW_QUALITY: quality below poor threshold, or required vitals missing.
     * distance_valid is NOT required — distance_cm=0 shows N/A in the backend
     * but the radar can still be NORMAL with valid presence/RR/HR. */
    else if (summary->radar.valid &&
             ((summary->radar.radar_quality < APP_RADAR_QUALITY_POOR_THRESHOLD) ||
              !rr_valid || !hr_valid))
    {
        snapshot->radar_source_state = E84_DISPLAY_RADAR_SOURCE_LOW_QUALITY;
    }
    else if (summary->radar.valid)
    {
        snapshot->radar_source_state = E84_DISPLAY_RADAR_SOURCE_NORMAL;
    }
    else
    {
        snapshot->radar_source_state = E84_DISPLAY_RADAR_SOURCE_INVALID;
    }

    /* Cough model is not board-verified in current phase. */
    flags |= E84_DISPLAY_FLAG_COUGH_MODEL_NOT_VERIFIED;
    snapshot->flags = flags;
    snapshot->cough_model_not_verified = true;
}

static e84_display_health_state_t app_display_summary_monitor_health(
    app_monitor_state_t state)
{
    switch (state)
    {
        case APP_MONITOR_STATE_IDLE:
        case APP_MONITOR_STATE_INIT:
            return E84_DISPLAY_HEALTH_INIT;

        case APP_MONITOR_STATE_MONITORING:
            return E84_DISPLAY_HEALTH_NORMAL;

        case APP_MONITOR_STATE_ATTENTION:
            return E84_DISPLAY_HEALTH_ATTENTION;

        case APP_MONITOR_STATE_WARNING:
            return E84_DISPLAY_HEALTH_WARNING;

        case APP_MONITOR_STATE_DEGRADED:
            return E84_DISPLAY_HEALTH_SENSOR_LOST;

        case APP_MONITOR_STATE_ERROR:
        default:
            return E84_DISPLAY_HEALTH_ERROR;
    }
}

static e84_display_alert_t app_display_summary_monitor_alert(
    app_monitor_event_type_t event_type,
    app_monitor_alert_level_t alert_level)
{
    if (APP_MONITOR_ALERT_LEVEL_ERROR == alert_level)
    {
        return E84_DISPLAY_ALERT_SYSTEM_ERROR;
    }
    if ((APP_MONITOR_EVENT_AUDIO_CANDIDATE == event_type) ||
        (APP_MONITOR_EVENT_COUGH_BURST == event_type))
    {
        return E84_DISPLAY_ALERT_COUGH_BURST;
    }
    if (APP_MONITOR_EVENT_VITALS_ATTENTION == event_type)
    {
        return E84_DISPLAY_ALERT_RESP_RATE_ABNORMAL;
    }
    if (APP_MONITOR_EVENT_SOURCE_CHANGED == event_type)
    {
        return E84_DISPLAY_ALERT_SENSOR_LOST;
    }

    return E84_DISPLAY_ALERT_NONE;
}

static uint8_t app_display_summary_monitor_severity(
    app_monitor_alert_level_t alert_level)
{
    switch (alert_level)
    {
        case APP_MONITOR_ALERT_LEVEL_ATTENTION:
            return (uint8_t)E84_DISPLAY_SEVERITY_ATTENTION;

        case APP_MONITOR_ALERT_LEVEL_WARNING:
            return (uint8_t)E84_DISPLAY_SEVERITY_WARNING;

        case APP_MONITOR_ALERT_LEVEL_ERROR:
            return (uint8_t)E84_DISPLAY_SEVERITY_ERROR;

        case APP_MONITOR_ALERT_LEVEL_INFO:
            return (uint8_t)E84_DISPLAY_SEVERITY_INFO;

        case APP_MONITOR_ALERT_LEVEL_NONE:
        default:
            return (uint8_t)E84_DISPLAY_SEVERITY_NONE;
    }
}

static void app_display_summary_publish_monitor_event(
    const app_monitor_summary_snapshot_t *summary)
{
    app_monitor_summary_event_t event;
    e84_display_alert_t display_alert;
    uint8_t severity;

    if (!app_monitor_summary_get_latest_event(&event))
    {
        return;
    }
    if (event.event_id == summary_last_event_id)
    {
        return;
    }
    summary_last_event_id = event.event_id;

    display_alert = app_display_summary_monitor_alert(event.event_type,
                                                      event.alert_level);
    severity = app_display_summary_monitor_severity(event.alert_level);
    if ((E84_DISPLAY_ALERT_NONE != display_alert) &&
        (severity >= (uint8_t)E84_DISPLAY_SEVERITY_WARNING))
    {
        summary_last_display_alert = display_alert;
        (void)app_display_raise_alert(display_alert,
                                      severity,
                                      event.confidence,
                                      app_display_summary_monitor_flags(&event));
    }
    else if ((APP_MONITOR_ALERT_LEVEL_NONE == event.alert_level) &&
             (E84_DISPLAY_ALERT_NONE != summary_last_display_alert))
    {
        (void)app_display_clear_alert(summary_last_display_alert);
        summary_last_display_alert = E84_DISPLAY_ALERT_NONE;
    }
}

static uint32_t app_display_summary_monitor_flags(
    const app_monitor_summary_event_t *event)
{
    uint32_t flags = 0u;

    if (0u != (event->source_valid_mask & APP_MONITOR_SOURCE_AUDIO))
    {
        flags |= E84_DISPLAY_FLAG_AUDIO_VALID;
    }
    if (0u != (event->source_valid_mask & APP_MONITOR_SOURCE_RADAR))
    {
        flags |= E84_DISPLAY_FLAG_RADAR_VALID;
    }
    if (0u != (event->source_valid_mask &
               APP_MONITOR_SOURCE_FUSION_SUMMARY))
    {
        flags |= E84_DISPLAY_FLAG_FUSION_VALID;
    }
    if (event->alert_level >= APP_MONITOR_ALERT_LEVEL_WARNING)
    {
        flags |= E84_DISPLAY_FLAG_ALERT_LATCHED;
    }

    return flags;
}
#else
static void app_display_summary_build_snapshot(
    e84_display_snapshot_t *snapshot,
    const app_model_result_monitor_stats_t *model_stats,
    uint32_t now_ms)
{
    uint32_t flags = 0u;
    bool ble_connected = app_display_summary_ble_connected();

    memset(snapshot, 0, sizeof(*snapshot));

    snapshot->timestamp_ms = now_ms;
    snapshot->health_state = app_display_summary_select_health(model_stats);
    snapshot->active_alert = E84_DISPLAY_ALERT_NONE;
    snapshot->radar_presence = false;
    snapshot->breath_rate_bpm = 0.0f;
    snapshot->heart_rate_bpm = 0.0f;
    snapshot->radar_quality = 0u;
    snapshot->mic_cough_prob = model_stats->has_live_result ?
        app_display_summary_clamp_probability(model_stats->last_cough_prob) :
        0.0f;
    snapshot->cough_count_1min = 0u;
    snapshot->cough_count_5min = 0u;
    snapshot->audio_quality = app_display_summary_audio_quality(model_stats);
    snapshot->fusion_confidence = 0u;
    snapshot->ble_connected = ble_connected;

    if (model_stats->has_live_result &&
        (APP_MODEL_INFERENCE_STATUS_OK == model_stats->last_status))
    {
        flags |= E84_DISPLAY_FLAG_AUDIO_VALID;
    }

    if (ble_connected)
    {
        flags |= E84_DISPLAY_FLAG_BLE_VALID;
    }

    snapshot->flags = flags;

    /* Fallback path: no radar source, no monitor summary. */
    snapshot->radar_source_state = E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE;

    /* Cough model is not board-verified in current phase. */
    flags |= E84_DISPLAY_FLAG_COUGH_MODEL_NOT_VERIFIED;
    snapshot->flags = flags;
    snapshot->cough_model_not_verified = true;
}

static e84_display_health_state_t app_display_summary_select_health(
    const app_model_result_monitor_stats_t *model_stats)
{
    if (NULL == model_stats)
    {
        return E84_DISPLAY_HEALTH_INIT;
    }

    if (model_stats->has_result &&
        (APP_MODEL_INFERENCE_STATUS_OK != model_stats->last_status))
    {
        return E84_DISPLAY_HEALTH_ERROR;
    }

    if (!model_stats->has_live_result)
    {
        return E84_DISPLAY_HEALTH_INIT;
    }

    if (model_stats->max_cough_prob_1s >= APP_AUDIO_ACTIVE_COUGH_THRESHOLD)
    {
        return E84_DISPLAY_HEALTH_ATTENTION;
    }

    return E84_DISPLAY_HEALTH_NORMAL;
}

static uint8_t app_display_summary_audio_quality(
    const app_model_result_monitor_stats_t *model_stats)
{
    if ((NULL == model_stats) || !model_stats->has_live_result)
    {
        return 0u;
    }

    return (APP_MODEL_INFERENCE_STATUS_OK == model_stats->last_status) ?
        100u : 0u;
}

static float app_display_summary_clamp_probability(float value)
{
    if (value < 0.0f)
    {
        return 0.0f;
    }

    if (value > 1.0f)
    {
        return 1.0f;
    }

    return value;
}
#endif

static bool app_display_summary_ble_connected(void)
{
#if (APP_BLE_ENABLE)
    return app_ble_is_connected();
#else
    return false;
#endif
}

#endif /* APP_DISPLAY_ENABLE && APP_DISPLAY_SUMMARY_ENABLE && !APP_DISPLAY_SMOKE_ENABLE && !APP_DISPLAY_FINAL_MOCK_ENABLE */

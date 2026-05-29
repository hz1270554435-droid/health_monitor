#include "app_display_summary_adapter.h"

#include "app_display.h"

#if (APP_DISPLAY_ENABLE && APP_DISPLAY_SUMMARY_ENABLE && \
     !APP_DISPLAY_SMOKE_ENABLE)

#include <stdbool.h>
#include <string.h>

#include "app_audio_deployment_config.h"
#include "../app_model_result_monitor/app_model_result_monitor.h"

#ifndef APP_BLE_ENABLE
#define APP_BLE_ENABLE                         (0u)
#endif

#if (APP_BLE_ENABLE)
#include "app_ble_diag.h"
#endif

static uint32_t summary_last_publish_ms;

static void app_display_summary_build_snapshot(
    e84_display_snapshot_t *snapshot,
    const app_model_result_monitor_stats_t *model_stats,
    uint32_t now_ms);
static e84_display_health_state_t app_display_summary_select_health(
    const app_model_result_monitor_stats_t *model_stats);
static uint8_t app_display_summary_audio_quality(
    const app_model_result_monitor_stats_t *model_stats);
static float app_display_summary_clamp_probability(float value);
static bool app_display_summary_ble_connected(void);

cy_rslt_t app_display_summary_adapter_tick(uint32_t now_ms)
{
    e84_display_snapshot_t snapshot;
    app_model_result_monitor_stats_t model_stats;

    if ((0u != summary_last_publish_ms) &&
        ((now_ms - summary_last_publish_ms) < APP_DISPLAY_SUMMARY_PERIOD_MS))
    {
        return CY_RSLT_SUCCESS;
    }

    memset(&model_stats, 0, sizeof(model_stats));
    app_model_result_monitor_get_stats(&model_stats);
    app_display_summary_build_snapshot(&snapshot, &model_stats, now_ms);

    summary_last_publish_ms = now_ms;
    return app_display_publish_snapshot(&snapshot);
}

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

static bool app_display_summary_ble_connected(void)
{
#if (APP_BLE_ENABLE)
    return app_ble_is_connected();
#else
    return false;
#endif
}

#endif /* APP_DISPLAY_ENABLE && APP_DISPLAY_SUMMARY_ENABLE && !APP_DISPLAY_SMOKE_ENABLE */

#include "app_monitor_summary.h"

#if (APP_MONITOR_SUMMARY_ENABLE)

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#define APP_MONITOR_AUDIO_MAX_QUALITY       (100u)
#define APP_MONITOR_AUDIO_MAX_PROB_X100     (100u)

typedef struct
{
    app_monitor_audio_input_t audio;
    app_monitor_device_input_t device;
    app_monitor_summary_snapshot_t snapshot;
    app_monitor_summary_event_t latest_event;
    app_monitor_event_type_t last_event_type;
    app_monitor_alert_level_t last_alert_level;
    app_monitor_state_t last_monitor_state;
    uint32_t last_event_ms;
    uint32_t last_diag_ms;
    uint32_t next_event_id;
    bool initialized;
    bool has_event;
} app_monitor_summary_owner_t;

static app_monitor_summary_owner_t monitor_summary_owner;

static void app_monitor_summary_copy_in_audio(
    const app_monitor_audio_input_t *audio);
static void app_monitor_summary_copy_in_device(
    const app_monitor_device_input_t *device);
static void app_monitor_summary_build_snapshot(
    app_monitor_summary_snapshot_t *snapshot,
    const app_monitor_audio_input_t *audio,
    const app_monitor_device_input_t *device,
    uint32_t now_ms,
    uint32_t sequence,
    uint32_t last_event_id);
static uint32_t app_monitor_summary_audio_reasons(
    const app_monitor_audio_input_t *audio);
static uint32_t app_monitor_summary_device_reasons(
    const app_monitor_device_input_t *device);
static uint32_t app_monitor_summary_source_ready_mask(
    const app_monitor_audio_input_t *audio,
    const app_monitor_device_input_t *device);
static uint32_t app_monitor_summary_source_valid_mask(
    const app_monitor_audio_input_t *audio,
    const app_monitor_device_input_t *device);
static uint32_t app_monitor_summary_source_stale_mask(
    const app_monitor_audio_input_t *audio);
static uint32_t app_monitor_summary_source_error_mask(
    const app_monitor_device_input_t *device);
static app_monitor_state_t app_monitor_summary_select_monitor_state(
    const app_monitor_audio_input_t *audio,
    const app_monitor_device_input_t *device);
static app_monitor_alert_level_t app_monitor_summary_select_alert_level(
    const app_monitor_audio_input_t *audio,
    const app_monitor_device_input_t *device);
static app_monitor_event_type_t app_monitor_summary_select_event_type(
    const app_monitor_summary_snapshot_t *old_snapshot,
    const app_monitor_summary_snapshot_t *new_snapshot);
static bool app_monitor_summary_event_allowed(
    app_monitor_event_type_t event_type,
    app_monitor_state_t monitor_state,
    app_monitor_alert_level_t alert_level,
    uint32_t now_ms);
static void app_monitor_summary_publish_event(
    const app_monitor_summary_snapshot_t *snapshot,
    app_monitor_event_type_t event_type,
    uint32_t now_ms);
static void app_monitor_summary_maybe_print_snapshot(
    const app_monitor_summary_snapshot_t *snapshot,
    uint32_t now_ms);
static void app_monitor_summary_print_event(
    const app_monitor_summary_event_t *event);
static uint8_t app_monitor_summary_clamp_u8(uint8_t value, uint8_t max_value);

cy_rslt_t app_monitor_summary_init(void)
{
    taskENTER_CRITICAL();
    memset(&monitor_summary_owner, 0, sizeof(monitor_summary_owner));
    monitor_summary_owner.next_event_id = 1u;
    monitor_summary_owner.initialized = true;
    taskEXIT_CRITICAL();

    return app_monitor_summary_tick(0u);
}

cy_rslt_t app_monitor_summary_update_audio(
    const app_monitor_audio_input_t *audio)
{
    if (NULL == audio)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    taskENTER_CRITICAL();
    app_monitor_summary_copy_in_audio(audio);
    taskEXIT_CRITICAL();

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_monitor_summary_update_device(
    const app_monitor_device_input_t *device)
{
    if (NULL == device)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    taskENTER_CRITICAL();
    app_monitor_summary_copy_in_device(device);
    taskEXIT_CRITICAL();

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_monitor_summary_update_radar(
    const app_monitor_radar_input_t *radar)
{
    (void)radar;

    taskENTER_CRITICAL();
    monitor_summary_owner.snapshot.radar.valid = false;
    monitor_summary_owner.snapshot.radar.ready = false;
    monitor_summary_owner.snapshot.radar.stale = false;
    monitor_summary_owner.snapshot.radar.radar_reason_flags =
        APP_MONITOR_REASON_RADAR_API_INACTIVE;
    taskEXIT_CRITICAL();

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_monitor_summary_tick(uint32_t now_ms)
{
    app_monitor_audio_input_t audio;
    app_monitor_device_input_t device;
    app_monitor_summary_snapshot_t old_snapshot;
    app_monitor_summary_snapshot_t new_snapshot;
    app_monitor_event_type_t event_type;
    uint32_t last_event_id;
    uint32_t sequence;
    bool initialized;

    taskENTER_CRITICAL();
    initialized = monitor_summary_owner.initialized;
    if (!initialized)
    {
        memset(&monitor_summary_owner, 0, sizeof(monitor_summary_owner));
        monitor_summary_owner.next_event_id = 1u;
        monitor_summary_owner.initialized = true;
    }
    audio = monitor_summary_owner.audio;
    device = monitor_summary_owner.device;
    old_snapshot = monitor_summary_owner.snapshot;
    last_event_id = monitor_summary_owner.latest_event.event_id;
    sequence = monitor_summary_owner.snapshot.sequence + 1u;
    taskEXIT_CRITICAL();

    app_monitor_summary_build_snapshot(&new_snapshot,
                                       &audio,
                                       &device,
                                       now_ms,
                                       sequence,
                                       last_event_id);
    event_type = app_monitor_summary_select_event_type(&old_snapshot,
                                                       &new_snapshot);

    taskENTER_CRITICAL();
    monitor_summary_owner.snapshot = new_snapshot;
    taskEXIT_CRITICAL();

    if ((APP_MONITOR_EVENT_NONE != event_type) &&
        app_monitor_summary_event_allowed(event_type,
                                          new_snapshot.monitor_state,
                                          new_snapshot.alert_level,
                                          now_ms))
    {
        app_monitor_summary_publish_event(&new_snapshot, event_type, now_ms);
    }

    app_monitor_summary_maybe_print_snapshot(&new_snapshot, now_ms);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_monitor_summary_get_snapshot(
    app_monitor_summary_snapshot_t *snapshot)
{
    if (NULL == snapshot)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    taskENTER_CRITICAL();
    *snapshot = monitor_summary_owner.snapshot;
    taskEXIT_CRITICAL();

    return CY_RSLT_SUCCESS;
}

bool app_monitor_summary_get_latest_event(
    app_monitor_summary_event_t *event)
{
    bool has_event;

    if (NULL == event)
    {
        return false;
    }

    taskENTER_CRITICAL();
    has_event = monitor_summary_owner.has_event;
    *event = monitor_summary_owner.latest_event;
    taskEXIT_CRITICAL();

    return has_event;
}

static void app_monitor_summary_copy_in_audio(
    const app_monitor_audio_input_t *audio)
{
    monitor_summary_owner.audio = *audio;
    monitor_summary_owner.audio.cough_prob_x100 =
        app_monitor_summary_clamp_u8(audio->cough_prob_x100,
                                     APP_MONITOR_AUDIO_MAX_PROB_X100);
    monitor_summary_owner.audio.event_threshold_x100 =
        app_monitor_summary_clamp_u8(audio->event_threshold_x100,
                                     APP_MONITOR_AUDIO_MAX_PROB_X100);
    monitor_summary_owner.audio.audio_quality =
        app_monitor_summary_clamp_u8(audio->audio_quality,
                                     APP_MONITOR_AUDIO_MAX_QUALITY);
}

static void app_monitor_summary_copy_in_device(
    const app_monitor_device_input_t *device)
{
    monitor_summary_owner.device = *device;
}

static void app_monitor_summary_build_snapshot(
    app_monitor_summary_snapshot_t *snapshot,
    const app_monitor_audio_input_t *audio,
    const app_monitor_device_input_t *device,
    uint32_t now_ms,
    uint32_t sequence,
    uint32_t last_event_id)
{
    uint32_t reason_flags;

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->timestamp_ms = now_ms;
    snapshot->sequence = sequence;
    snapshot->audio = *audio;
    snapshot->device = *device;
    snapshot->radar.valid = false;
    snapshot->radar.ready = false;
    snapshot->radar.stale = false;
    snapshot->radar.radar_reason_flags = APP_MONITOR_REASON_RADAR_UNAVAILABLE;

    snapshot->capability_mask = APP_MONITOR_SOURCE_AUDIO |
                                APP_MONITOR_SOURCE_DEVICE_SESSION |
                                APP_MONITOR_SOURCE_MODEL_SHARED |
                                APP_MONITOR_SOURCE_FUSION_SUMMARY;
    if (device->ble_connected)
    {
        snapshot->capability_mask |= APP_MONITOR_SOURCE_BLE;
    }

    snapshot->source_ready_mask =
        app_monitor_summary_source_ready_mask(audio, device);
    snapshot->source_valid_mask =
        app_monitor_summary_source_valid_mask(audio, device);
    snapshot->source_stale_mask =
        app_monitor_summary_source_stale_mask(audio);
    snapshot->source_error_mask =
        app_monitor_summary_source_error_mask(device);

    reason_flags = app_monitor_summary_audio_reasons(audio) |
                   app_monitor_summary_device_reasons(device) |
                   APP_MONITOR_REASON_FUSION_AUDIO_ONLY |
                   APP_MONITOR_REASON_FUSION_RADAR_ABSENT;
    snapshot->active_reason_flags = reason_flags;
    snapshot->monitor_state =
        app_monitor_summary_select_monitor_state(audio, device);
    snapshot->fusion_state = APP_MONITOR_FUSION_STATE_AUDIO_ONLY;
    snapshot->alert_level =
        app_monitor_summary_select_alert_level(audio, device);
    snapshot->fusion_confidence = audio->valid ? audio->audio_quality : 0u;
    snapshot->last_event_id = last_event_id;
}

static uint32_t app_monitor_summary_audio_reasons(
    const app_monitor_audio_input_t *audio)
{
    uint32_t flags = audio->reason_flags & 0x000000FFUL;

    if (audio->valid)
    {
        flags |= APP_MONITOR_REASON_AUDIO_RESULT_READY;
    }
    else
    {
        flags |= APP_MONITOR_REASON_AUDIO_UNAVAILABLE;
    }

    if (audio->cough_confirmed)
    {
        flags |= APP_MONITOR_REASON_AUDIO_COUGH_CANDIDATE;
    }
    if (audio->cough_density_high)
    {
        flags |= APP_MONITOR_REASON_AUDIO_DENSITY_HIGH;
    }
    if (audio->mic_quality_poor)
    {
        flags |= APP_MONITOR_REASON_AUDIO_QUALITY_POOR;
    }
    if (audio->stale)
    {
        flags |= APP_MONITOR_REASON_AUDIO_STALE;
    }

    return flags;
}

static uint32_t app_monitor_summary_device_reasons(
    const app_monitor_device_input_t *device)
{
    uint32_t flags = device->device_reason_flags & 0xFF000000UL;

    if (!device->monitor_requested)
    {
        flags |= APP_MONITOR_REASON_DEVICE_MONITOR_OFF;
    }
    if (!device->model_ready)
    {
        flags |= APP_MONITOR_REASON_DEVICE_MODEL_NOT_READY;
    }
    if (!device->shared_memory_ready)
    {
        flags |= APP_MONITOR_REASON_DEVICE_SHARED_NOT_READY;
    }
    if (!device->ble_connected)
    {
        flags |= APP_MONITOR_REASON_DEVICE_BLE_DISCONNECTED;
    }
    if (!device->time_synced)
    {
        flags |= APP_MONITOR_REASON_DEVICE_TIME_UNSYNCED;
    }

    return flags;
}

static uint32_t app_monitor_summary_source_ready_mask(
    const app_monitor_audio_input_t *audio,
    const app_monitor_device_input_t *device)
{
    uint32_t mask = APP_MONITOR_SOURCE_FUSION_SUMMARY;

    if (audio->ready)
    {
        mask |= APP_MONITOR_SOURCE_AUDIO;
    }
    if (device->session_active)
    {
        mask |= APP_MONITOR_SOURCE_DEVICE_SESSION;
    }
    if (device->shared_memory_ready || device->model_ready)
    {
        mask |= APP_MONITOR_SOURCE_MODEL_SHARED;
    }
    if (device->ble_connected)
    {
        mask |= APP_MONITOR_SOURCE_BLE;
    }

    return mask;
}

static uint32_t app_monitor_summary_source_valid_mask(
    const app_monitor_audio_input_t *audio,
    const app_monitor_device_input_t *device)
{
    uint32_t mask = APP_MONITOR_SOURCE_FUSION_SUMMARY;

    if (audio->valid && !audio->stale)
    {
        mask |= APP_MONITOR_SOURCE_AUDIO;
    }
    if (device->session_active)
    {
        mask |= APP_MONITOR_SOURCE_DEVICE_SESSION;
    }
    if (device->shared_memory_ready && device->model_ready)
    {
        mask |= APP_MONITOR_SOURCE_MODEL_SHARED;
    }
    if (device->ble_connected)
    {
        mask |= APP_MONITOR_SOURCE_BLE;
    }

    return mask;
}

static uint32_t app_monitor_summary_source_stale_mask(
    const app_monitor_audio_input_t *audio)
{
    return audio->stale ? APP_MONITOR_SOURCE_AUDIO : 0u;
}

static uint32_t app_monitor_summary_source_error_mask(
    const app_monitor_device_input_t *device)
{
    uint32_t mask = 0u;

    if (!device->shared_memory_ready || !device->model_ready)
    {
        mask |= APP_MONITOR_SOURCE_MODEL_SHARED;
    }

    return mask;
}

static app_monitor_state_t app_monitor_summary_select_monitor_state(
    const app_monitor_audio_input_t *audio,
    const app_monitor_device_input_t *device)
{
    if (!device->monitor_requested || !device->session_active)
    {
        return APP_MONITOR_STATE_IDLE;
    }
    if ((!device->shared_memory_ready || !device->model_ready) &&
        (audio->valid || audio->ready))
    {
        return APP_MONITOR_STATE_DEGRADED;
    }
    if (!device->shared_memory_ready || !device->model_ready)
    {
        return APP_MONITOR_STATE_ERROR;
    }
    if (!audio->valid || audio->stale || audio->mic_quality_poor)
    {
        return APP_MONITOR_STATE_DEGRADED;
    }
    if (audio->cough_density_high)
    {
        return APP_MONITOR_STATE_WARNING;
    }
    if (audio->cough_confirmed)
    {
        return APP_MONITOR_STATE_ATTENTION;
    }

    return APP_MONITOR_STATE_MONITORING;
}

static app_monitor_alert_level_t app_monitor_summary_select_alert_level(
    const app_monitor_audio_input_t *audio,
    const app_monitor_device_input_t *device)
{
    if ((!device->shared_memory_ready || !device->model_ready) &&
        (audio->valid || audio->ready))
    {
        return APP_MONITOR_ALERT_LEVEL_INFO;
    }
    if (!device->shared_memory_ready || !device->model_ready)
    {
        return APP_MONITOR_ALERT_LEVEL_ERROR;
    }
    if (!audio->valid || audio->stale || audio->mic_quality_poor)
    {
        return APP_MONITOR_ALERT_LEVEL_INFO;
    }
    if (audio->cough_density_high)
    {
        return APP_MONITOR_ALERT_LEVEL_WARNING;
    }
    if (audio->cough_confirmed)
    {
        return APP_MONITOR_ALERT_LEVEL_ATTENTION;
    }

    return APP_MONITOR_ALERT_LEVEL_NONE;
}

static app_monitor_event_type_t app_monitor_summary_select_event_type(
    const app_monitor_summary_snapshot_t *old_snapshot,
    const app_monitor_summary_snapshot_t *new_snapshot)
{
    if (old_snapshot->sequence == 0u)
    {
        return APP_MONITOR_EVENT_STATE_CHANGED;
    }
    if (old_snapshot->alert_level != new_snapshot->alert_level)
    {
        return APP_MONITOR_EVENT_ALERT_CHANGED;
    }
    if (old_snapshot->monitor_state != new_snapshot->monitor_state)
    {
        return APP_MONITOR_EVENT_STATE_CHANGED;
    }
    if ((old_snapshot->source_valid_mask != new_snapshot->source_valid_mask) ||
        (old_snapshot->source_stale_mask != new_snapshot->source_stale_mask) ||
        (old_snapshot->source_error_mask != new_snapshot->source_error_mask))
    {
        return APP_MONITOR_EVENT_SOURCE_CHANGED;
    }
    if (!old_snapshot->audio.cough_confirmed &&
        new_snapshot->audio.cough_confirmed)
    {
        return APP_MONITOR_EVENT_AUDIO_CANDIDATE;
    }

    return APP_MONITOR_EVENT_NONE;
}

static bool app_monitor_summary_event_allowed(
    app_monitor_event_type_t event_type,
    app_monitor_state_t monitor_state,
    app_monitor_alert_level_t alert_level,
    uint32_t now_ms)
{
    bool allowed = true;

    taskENTER_CRITICAL();
    if ((event_type == monitor_summary_owner.last_event_type) &&
        (monitor_state == monitor_summary_owner.last_monitor_state) &&
        (alert_level == monitor_summary_owner.last_alert_level) &&
        (0u != monitor_summary_owner.last_event_ms) &&
        ((now_ms - monitor_summary_owner.last_event_ms) <
         APP_MONITOR_SUMMARY_EVENT_COOLDOWN_MS))
    {
        allowed = false;
    }
    taskEXIT_CRITICAL();

    return allowed;
}

static void app_monitor_summary_publish_event(
    const app_monitor_summary_snapshot_t *snapshot,
    app_monitor_event_type_t event_type,
    uint32_t now_ms)
{
    app_monitor_summary_event_t event;

    memset(&event, 0, sizeof(event));
    event.timestamp_ms = now_ms;
    event.event_type = event_type;
    event.monitor_state = snapshot->monitor_state;
    event.fusion_state = snapshot->fusion_state;
    event.alert_level = snapshot->alert_level;
    event.reason_flags = snapshot->active_reason_flags;
    event.source_valid_mask = snapshot->source_valid_mask;
    event.source_stale_mask = snapshot->source_stale_mask;
    event.source_error_mask = snapshot->source_error_mask;
    event.confidence = snapshot->fusion_confidence;

    taskENTER_CRITICAL();
    event.event_id = monitor_summary_owner.next_event_id++;
    monitor_summary_owner.latest_event = event;
    monitor_summary_owner.snapshot.last_event_id = event.event_id;
    monitor_summary_owner.last_event_type = event_type;
    monitor_summary_owner.last_monitor_state = snapshot->monitor_state;
    monitor_summary_owner.last_alert_level = snapshot->alert_level;
    monitor_summary_owner.last_event_ms = now_ms;
    monitor_summary_owner.has_event = true;
    taskEXIT_CRITICAL();

    app_monitor_summary_print_event(&event);
}

static void app_monitor_summary_maybe_print_snapshot(
    const app_monitor_summary_snapshot_t *snapshot,
    uint32_t now_ms)
{
#if (APP_MONITOR_SUMMARY_DIAG_ENABLE)
    bool should_print = false;

    taskENTER_CRITICAL();
    if ((0u == monitor_summary_owner.last_diag_ms) ||
        ((now_ms - monitor_summary_owner.last_diag_ms) >=
         APP_MONITOR_SUMMARY_DIAG_PERIOD_MS))
    {
        monitor_summary_owner.last_diag_ms = now_ms;
        should_print = true;
    }
    taskEXIT_CRITICAL();

    if (should_print)
    {
        printf("[SUMMARY_SNAPSHOT] t_ms=%lu, seq=%lu, cough_prob_x100=%u, "
               "model_status=%u, input_seq=%lu, result_seq=%lu, "
               "audio_age_ms=%lu, monitor=%u, fusion=%u, alert=%u, "
               "ready=0x%08lx, valid=0x%08lx, stale=0x%08lx, error=0x%08lx, "
               "reason=0x%08lx, radar_valid=%u\r\n",
               (unsigned long)snapshot->timestamp_ms,
               (unsigned long)snapshot->sequence,
               (unsigned int)snapshot->audio.cough_prob_x100,
               (unsigned int)snapshot->audio.model_status,
               (unsigned long)snapshot->audio.input_sequence,
               (unsigned long)snapshot->audio.result_sequence,
               (unsigned long)snapshot->audio.age_ms,
               (unsigned int)snapshot->monitor_state,
               (unsigned int)snapshot->fusion_state,
               (unsigned int)snapshot->alert_level,
               (unsigned long)snapshot->source_ready_mask,
               (unsigned long)snapshot->source_valid_mask,
               (unsigned long)snapshot->source_stale_mask,
               (unsigned long)snapshot->source_error_mask,
               (unsigned long)snapshot->active_reason_flags,
               (unsigned int)(snapshot->radar.valid ? 1u : 0u));
    }
#else
    (void)snapshot;
    (void)now_ms;
#endif
}

static void app_monitor_summary_print_event(
    const app_monitor_summary_event_t *event)
{
#if (APP_MONITOR_SUMMARY_DIAG_ENABLE)
    printf("[SUMMARY_EVENT] t_ms=%lu, event_id=%lu, type=%u, seq=%lu, "
           "cough_prob_x100=%u, model_status=%u, input_seq=%lu, result_seq=%lu, "
           "audio_age_ms=%lu, monitor=%u, fusion=%u, alert=%u, "
           "valid=0x%08lx, stale=0x%08lx, error=0x%08lx, reason=0x%08lx, "
           "radar_valid=%u, confidence=%u\r\n",
           (unsigned long)event->timestamp_ms,
           (unsigned long)event->event_id,
           (unsigned int)event->event_type,
           (unsigned long)monitor_summary_owner.snapshot.sequence,
           (unsigned int)monitor_summary_owner.snapshot.audio.cough_prob_x100,
           (unsigned int)monitor_summary_owner.snapshot.audio.model_status,
           (unsigned long)monitor_summary_owner.snapshot.audio.input_sequence,
           (unsigned long)monitor_summary_owner.snapshot.audio.result_sequence,
           (unsigned long)monitor_summary_owner.snapshot.audio.age_ms,
           (unsigned int)event->monitor_state,
           (unsigned int)event->fusion_state,
           (unsigned int)event->alert_level,
           (unsigned long)event->source_valid_mask,
           (unsigned long)event->source_stale_mask,
           (unsigned long)event->source_error_mask,
           (unsigned long)event->reason_flags,
           (unsigned int)(monitor_summary_owner.snapshot.radar.valid ? 1u : 0u),
           (unsigned int)event->confidence);
#else
    (void)event;
#endif
}

static uint8_t app_monitor_summary_clamp_u8(uint8_t value, uint8_t max_value)
{
    return (value > max_value) ? max_value : value;
}

#endif /* APP_MONITOR_SUMMARY_ENABLE */

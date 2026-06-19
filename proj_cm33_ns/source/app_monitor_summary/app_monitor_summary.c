#include "app_monitor_summary.h"

#if (APP_MONITOR_SUMMARY_ENABLE)

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#define APP_MONITOR_MAX_PERCENT            (100u)
#define APP_MONITOR_MINUTES_1_MS           (60000u)
#define APP_MONITOR_MINUTES_5_MS           (300000u)
#define APP_MONITOR_INVALID_BPM_X10        (0u)

typedef struct
{
    app_monitor_audio_input_t audio;
    app_monitor_radar_input_t radar;
    app_monitor_device_input_t device;
    app_monitor_summary_snapshot_t snapshot;
    app_monitor_summary_event_t latest_event;
    app_monitor_summary_event_t event_ring[
        APP_MONITOR_SUMMARY_EVENT_RING_SIZE];
    app_monitor_session_summary_t session;
    app_monitor_session_summary_t night;
    app_monitor_event_type_t last_event_type;
    app_monitor_alert_level_t last_alert_level;
    app_monitor_state_t last_monitor_state;
    TaskHandle_t mock_task_handle;
    uint32_t event_ring_write_index;
    uint32_t event_ring_count;
    uint32_t last_event_ms;
    uint32_t last_diag_ms;
    uint32_t last_session_diag_ms;
    uint32_t next_event_id;
    uint32_t session_cough_prob_sum;
    uint32_t night_cough_prob_sum;
    uint8_t smoothed_cough_prob_x100;
    bool cough_latched;
    bool have_smoothed_cough;
    bool initialized;
    bool has_event;
    bool session_was_active;
} app_monitor_summary_owner_t;

static app_monitor_summary_owner_t monitor_summary_owner;

static void app_monitor_summary_ensure_initialized_locked(void);
static void app_monitor_summary_reset_period_locked(
    app_monitor_session_summary_t *summary,
    uint32_t *cough_prob_sum,
    uint32_t now_ms,
    bool active);
static void app_monitor_summary_copy_in_audio(
    const app_monitor_audio_input_t *audio);
static void app_monitor_summary_copy_in_device(
    const app_monitor_device_input_t *device);
static void app_monitor_summary_copy_in_radar(
    const app_monitor_radar_input_t *radar);
static void app_monitor_summary_build_snapshot(
    app_monitor_summary_snapshot_t *snapshot,
    const app_monitor_audio_input_t *audio_in,
    const app_monitor_radar_input_t *radar_in,
    const app_monitor_device_input_t *device,
    uint32_t cough_count_1min,
    uint32_t cough_count_5min,
    uint32_t now_ms,
    uint32_t sequence,
    uint32_t last_event_id,
    uint8_t previous_smoothed_prob_x100,
    bool have_previous_smoothed,
    bool previous_cough_latched,
    uint8_t *next_smoothed_prob_x100,
    bool *next_have_smoothed,
    bool *next_cough_latched);
static void app_monitor_summary_update_periods_locked(
    const app_monitor_summary_snapshot_t *snapshot,
    uint32_t now_ms);
static void app_monitor_summary_note_event_in_period_locked(
    app_monitor_session_summary_t *summary,
    const app_monitor_summary_event_t *event);
static uint32_t app_monitor_summary_count_recent_cough_locked(
    uint32_t now_ms,
    uint32_t window_ms);
static uint32_t app_monitor_summary_audio_reasons(
    const app_monitor_audio_input_t *audio);
static uint32_t app_monitor_summary_radar_reasons(
    const app_monitor_radar_input_t *radar);
static uint32_t app_monitor_summary_device_reasons(
    const app_monitor_device_input_t *device);
static uint32_t app_monitor_summary_source_ready_mask(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar,
    const app_monitor_device_input_t *device);
static uint32_t app_monitor_summary_source_valid_mask(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar,
    const app_monitor_device_input_t *device);
static uint32_t app_monitor_summary_source_stale_mask(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar);
static uint32_t app_monitor_summary_source_error_mask(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar,
    const app_monitor_device_input_t *device);
static app_monitor_state_t app_monitor_summary_select_monitor_state(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar,
    const app_monitor_device_input_t *device);
static app_monitor_fusion_state_t app_monitor_summary_select_fusion_state(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar,
    const app_monitor_device_input_t *device);
static app_monitor_alert_level_t app_monitor_summary_select_alert_level(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar,
    const app_monitor_device_input_t *device);
static app_monitor_event_type_t app_monitor_summary_select_active_event(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar);
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
static void app_monitor_summary_maybe_print_session(uint32_t now_ms);
static void app_monitor_summary_print_event(
    const app_monitor_summary_event_t *event);
static void app_monitor_summary_print_bpm_x10(uint16_t value);
static uint8_t app_monitor_summary_clamp_u8(uint8_t value, uint8_t max_value);
static uint16_t app_monitor_summary_clamp_bpm_x10(uint16_t value,
                                                  uint16_t max_value);
static uint8_t app_monitor_summary_smooth_prob(uint8_t previous,
                                               bool have_previous,
                                               uint8_t current);
static uint8_t app_monitor_summary_select_radar_quality(
    const app_monitor_radar_input_t *radar);
static uint8_t app_monitor_summary_select_vital_state(uint16_t value_x10,
                                                      uint16_t low_x10,
                                                      uint16_t high_x10);
static uint8_t app_monitor_summary_select_motion_state(
    const app_monitor_radar_input_t *radar);
static uint8_t app_monitor_summary_select_confidence(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar);
static uint8_t app_monitor_summary_event_source_flags(
    const app_monitor_summary_snapshot_t *snapshot,
    app_monitor_event_type_t event_type);
static bool app_monitor_summary_event_is_cough(
    app_monitor_event_type_t event_type);
static bool app_monitor_summary_event_is_warning(
    app_monitor_alert_level_t alert_level);
static uint32_t app_monitor_summary_now_ms(void);
#if (APP_MONITOR_SUMMARY_MOCK_ENABLE)
static void app_monitor_summary_mock_task(void *pvParameters);
static void app_monitor_summary_mock_make_inputs(uint32_t step,
                                                 uint32_t now_ms,
                                                 app_monitor_audio_input_t *audio,
                                                 app_monitor_radar_input_t *radar,
                                                 app_monitor_device_input_t *device);
#endif

cy_rslt_t app_monitor_summary_init(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    taskENTER_CRITICAL();
    memset(&monitor_summary_owner, 0, sizeof(monitor_summary_owner));
    monitor_summary_owner.next_event_id = 1u;
    monitor_summary_owner.initialized = true;
    app_monitor_summary_reset_period_locked(&monitor_summary_owner.night,
                                            &monitor_summary_owner.night_cough_prob_sum,
                                            0u,
                                            true);
    taskEXIT_CRITICAL();

#if (APP_MONITOR_SUMMARY_MOCK_ENABLE)
    if (NULL == monitor_summary_owner.mock_task_handle)
    {
        BaseType_t ret = xTaskCreate(app_monitor_summary_mock_task,
                                     "monitor_mock",
                                     APP_MONITOR_SUMMARY_TASK_STACK_SIZE,
                                     NULL,
                                     APP_MONITOR_SUMMARY_TASK_PRIORITY,
                                     &monitor_summary_owner.mock_task_handle);
        if (pdPASS != ret)
        {
            result = CY_RSLT_TYPE_ERROR;
        }
    }
#endif

    if (CY_RSLT_SUCCESS == result)
    {
        result = app_monitor_summary_tick(0u);
    }

    return result;
}

cy_rslt_t app_monitor_summary_update_audio(
    const app_monitor_audio_input_t *audio)
{
    if (NULL == audio)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    taskENTER_CRITICAL();
    app_monitor_summary_ensure_initialized_locked();
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
    app_monitor_summary_ensure_initialized_locked();
    app_monitor_summary_copy_in_device(device);
    taskEXIT_CRITICAL();

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_monitor_summary_update_radar(
    const app_monitor_radar_input_t *radar)
{
    if (NULL == radar)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    taskENTER_CRITICAL();
    app_monitor_summary_ensure_initialized_locked();
    app_monitor_summary_copy_in_radar(radar);
    taskEXIT_CRITICAL();

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_monitor_summary_tick(uint32_t now_ms)
{
    app_monitor_audio_input_t audio;
    app_monitor_radar_input_t radar;
    app_monitor_device_input_t device;
    app_monitor_summary_snapshot_t old_snapshot;
    app_monitor_summary_snapshot_t new_snapshot;
    app_monitor_event_type_t event_type;
    uint32_t last_event_id;
    uint32_t sequence;
    uint32_t cough_count_1min;
    uint32_t cough_count_5min;
    uint8_t previous_smoothed;
    uint8_t next_smoothed;
    bool previous_have_smoothed;
    bool next_have_smoothed;
    bool previous_latched;
    bool next_latched;

    taskENTER_CRITICAL();
    app_monitor_summary_ensure_initialized_locked();
    audio = monitor_summary_owner.audio;
    radar = monitor_summary_owner.radar;
    device = monitor_summary_owner.device;
    old_snapshot = monitor_summary_owner.snapshot;
    last_event_id = monitor_summary_owner.latest_event.event_id;
    sequence = monitor_summary_owner.snapshot.sequence + 1u;
    cough_count_1min =
        app_monitor_summary_count_recent_cough_locked(now_ms,
                                                      APP_MONITOR_MINUTES_1_MS);
    cough_count_5min =
        app_monitor_summary_count_recent_cough_locked(now_ms,
                                                      APP_MONITOR_MINUTES_5_MS);
    previous_smoothed = monitor_summary_owner.smoothed_cough_prob_x100;
    previous_have_smoothed = monitor_summary_owner.have_smoothed_cough;
    previous_latched = monitor_summary_owner.cough_latched;
    taskEXIT_CRITICAL();

    app_monitor_summary_build_snapshot(&new_snapshot,
                                       &audio,
                                       &radar,
                                       &device,
                                       cough_count_1min,
                                       cough_count_5min,
                                       now_ms,
                                       sequence,
                                       last_event_id,
                                       previous_smoothed,
                                       previous_have_smoothed,
                                       previous_latched,
                                       &next_smoothed,
                                       &next_have_smoothed,
                                       &next_latched);
    event_type = app_monitor_summary_select_event_type(&old_snapshot,
                                                       &new_snapshot);

    taskENTER_CRITICAL();
    monitor_summary_owner.snapshot = new_snapshot;
    monitor_summary_owner.smoothed_cough_prob_x100 = next_smoothed;
    monitor_summary_owner.have_smoothed_cough = next_have_smoothed;
    monitor_summary_owner.cough_latched = next_latched;
    app_monitor_summary_update_periods_locked(&new_snapshot, now_ms);
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
    app_monitor_summary_maybe_print_session(now_ms);

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
    app_monitor_summary_ensure_initialized_locked();
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
    app_monitor_summary_ensure_initialized_locked();
    has_event = monitor_summary_owner.has_event;
    *event = monitor_summary_owner.latest_event;
    taskEXIT_CRITICAL();

    return has_event;
}

bool app_monitor_summary_get_event_by_age(uint32_t index_from_latest,
                                          app_monitor_summary_event_t *event)
{
    bool ok = false;

    if (NULL == event)
    {
        return false;
    }

    taskENTER_CRITICAL();
    app_monitor_summary_ensure_initialized_locked();
    if (index_from_latest < monitor_summary_owner.event_ring_count)
    {
        uint32_t latest_index =
            (0u == monitor_summary_owner.event_ring_write_index) ?
            (APP_MONITOR_SUMMARY_EVENT_RING_SIZE - 1u) :
            (monitor_summary_owner.event_ring_write_index - 1u);
        uint32_t ring_index =
            (latest_index + APP_MONITOR_SUMMARY_EVENT_RING_SIZE -
             (index_from_latest % APP_MONITOR_SUMMARY_EVENT_RING_SIZE)) %
            APP_MONITOR_SUMMARY_EVENT_RING_SIZE;

        *event = monitor_summary_owner.event_ring[ring_index];
        ok = true;
    }
    else
    {
        *event = (app_monitor_summary_event_t){0};
    }
    taskEXIT_CRITICAL();

    return ok;
}

uint32_t app_monitor_summary_get_event_count(void)
{
    uint32_t count;

    taskENTER_CRITICAL();
    app_monitor_summary_ensure_initialized_locked();
    count = monitor_summary_owner.event_ring_count;
    taskEXIT_CRITICAL();

    return count;
}

cy_rslt_t app_monitor_summary_get_session_summary(
    app_monitor_session_summary_t *summary)
{
    if (NULL == summary)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    taskENTER_CRITICAL();
    app_monitor_summary_ensure_initialized_locked();
    *summary = monitor_summary_owner.session;
    taskEXIT_CRITICAL();

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_monitor_summary_get_night_summary(
    app_monitor_session_summary_t *summary)
{
    if (NULL == summary)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    taskENTER_CRITICAL();
    app_monitor_summary_ensure_initialized_locked();
    *summary = monitor_summary_owner.night;
    taskEXIT_CRITICAL();

    return CY_RSLT_SUCCESS;
}

static void app_monitor_summary_ensure_initialized_locked(void)
{
    if (!monitor_summary_owner.initialized)
    {
        memset(&monitor_summary_owner, 0, sizeof(monitor_summary_owner));
        monitor_summary_owner.next_event_id = 1u;
        monitor_summary_owner.initialized = true;
        app_monitor_summary_reset_period_locked(
            &monitor_summary_owner.night,
            &monitor_summary_owner.night_cough_prob_sum,
            0u,
            true);
    }
}

static void app_monitor_summary_reset_period_locked(
    app_monitor_session_summary_t *summary,
    uint32_t *cough_prob_sum,
    uint32_t now_ms,
    bool active)
{
    memset(summary, 0, sizeof(*summary));
    summary->active = active;
    summary->started_ms = now_ms;
    summary->updated_ms = now_ms;
    summary->rr_min_bpm_x10 = APP_MONITOR_INVALID_BPM_X10;
    summary->hr_min_bpm_x10 = APP_MONITOR_INVALID_BPM_X10;
    if (NULL != cough_prob_sum)
    {
        *cough_prob_sum = 0u;
    }
}

static void app_monitor_summary_copy_in_audio(
    const app_monitor_audio_input_t *audio)
{
    monitor_summary_owner.audio = *audio;
    monitor_summary_owner.audio.cough_prob_x100 =
        app_monitor_summary_clamp_u8(audio->cough_prob_x100,
                                     APP_MONITOR_MAX_PERCENT);
    monitor_summary_owner.audio.event_threshold_x100 =
        app_monitor_summary_clamp_u8(audio->event_threshold_x100,
                                     APP_MONITOR_MAX_PERCENT);
    monitor_summary_owner.audio.audio_quality =
        app_monitor_summary_clamp_u8(audio->audio_quality,
                                     APP_MONITOR_MAX_PERCENT);
}

static void app_monitor_summary_copy_in_device(
    const app_monitor_device_input_t *device)
{
    monitor_summary_owner.device = *device;
}

static void app_monitor_summary_copy_in_radar(
    const app_monitor_radar_input_t *radar)
{
    monitor_summary_owner.radar = *radar;
    monitor_summary_owner.radar.radar_quality =
        app_monitor_summary_clamp_u8(radar->radar_quality,
                                     APP_MONITOR_MAX_PERCENT);
    monitor_summary_owner.radar.motion_x100 =
        app_monitor_summary_clamp_u8(radar->motion_x100,
                                     APP_MONITOR_MAX_PERCENT);
    monitor_summary_owner.radar.rr_bpm_x10 =
        app_monitor_summary_clamp_bpm_x10(radar->rr_bpm_x10, 800u);
    monitor_summary_owner.radar.hr_bpm_x10 =
        app_monitor_summary_clamp_bpm_x10(radar->hr_bpm_x10, 2200u);
}

static void app_monitor_summary_build_snapshot(
    app_monitor_summary_snapshot_t *snapshot,
    const app_monitor_audio_input_t *audio_in,
    const app_monitor_radar_input_t *radar_in,
    const app_monitor_device_input_t *device,
    uint32_t cough_count_1min,
    uint32_t cough_count_5min,
    uint32_t now_ms,
    uint32_t sequence,
    uint32_t last_event_id,
    uint8_t previous_smoothed_prob_x100,
    bool have_previous_smoothed,
    bool previous_cough_latched,
    uint8_t *next_smoothed_prob_x100,
    bool *next_have_smoothed,
    bool *next_cough_latched)
{
    app_monitor_audio_input_t audio = *audio_in;
    app_monitor_radar_input_t radar = *radar_in;
    uint32_t reason_flags;
    uint8_t threshold = (0u != audio.event_threshold_x100) ?
        audio.event_threshold_x100 : APP_MONITOR_SUMMARY_COUGH_WARNING_X100;
    uint8_t raw_prob = app_monitor_summary_clamp_u8(audio.cough_prob_x100,
                                                    APP_MONITOR_MAX_PERCENT);
    uint8_t smoothed_prob = 0u;
    bool cough_latched = previous_cough_latched;

    memset(snapshot, 0, sizeof(*snapshot));

    if (audio.age_ms > APP_MONITOR_SUMMARY_AUDIO_STALE_MS)
    {
        audio.stale = true;
    }

    if (audio.valid && !audio.stale)
    {
        smoothed_prob = app_monitor_summary_smooth_prob(
            previous_smoothed_prob_x100,
            have_previous_smoothed,
            raw_prob);
        *next_have_smoothed = true;
    }
    else
    {
        smoothed_prob = 0u;
        *next_have_smoothed = false;
        cough_latched = false;
    }

    if (audio.valid && !audio.stale &&
        ((raw_prob >= threshold) || (smoothed_prob >= threshold) ||
         audio.cough_confirmed))
    {
        cough_latched = true;
    }
    else if ((raw_prob <= APP_MONITOR_SUMMARY_COUGH_RELEASE_X100) &&
             (smoothed_prob <= APP_MONITOR_SUMMARY_COUGH_RELEASE_X100))
    {
        cough_latched = false;
    }

    audio.cough_prob_x100 = smoothed_prob;
    audio.cough_confirmed = cough_latched;
    audio.cough_density_high =
        audio.cough_density_high ||
        (cough_count_1min >= APP_MONITOR_SUMMARY_COUGH_BURST_1MIN) ||
        (cough_count_5min >= APP_MONITOR_SUMMARY_COUGH_BURST_5MIN);
    if ((0u == audio.audio_quality) && audio.valid && !audio.stale)
    {
        audio.audio_quality = APP_MONITOR_MAX_PERCENT;
    }

    if (radar.age_ms > APP_MONITOR_SUMMARY_RADAR_STALE_MS)
    {
        radar.stale = true;
    }
    if (radar.valid)
    {
        radar.ready = true;
    }
    if (radar.valid && (APP_MONITOR_PRESENCE_UNKNOWN == radar.presence_state))
    {
        radar.presence_state = APP_MONITOR_PRESENCE_PRESENT;
    }
    radar.motion_state = app_monitor_summary_select_motion_state(&radar);
    radar.breath_state = app_monitor_summary_select_vital_state(
        radar.rr_bpm_x10,
        APP_MONITOR_SUMMARY_RR_LOW_BPM_X10,
        APP_MONITOR_SUMMARY_RR_HIGH_BPM_X10);
    radar.heart_state = app_monitor_summary_select_vital_state(
        radar.hr_bpm_x10,
        APP_MONITOR_SUMMARY_HR_LOW_BPM_X10,
        APP_MONITOR_SUMMARY_HR_HIGH_BPM_X10);
    radar.radar_quality = app_monitor_summary_select_radar_quality(&radar);

    snapshot->timestamp_ms = now_ms;
    snapshot->sequence = sequence;
    snapshot->audio = audio;
    snapshot->radar = radar;
    snapshot->device = *device;
    snapshot->capability_mask = APP_MONITOR_SOURCE_AUDIO |
                                APP_MONITOR_SOURCE_DEVICE_SESSION |
                                APP_MONITOR_SOURCE_MODEL_SHARED |
                                APP_MONITOR_SOURCE_FUSION_SUMMARY;
    if (radar.ready || radar.valid)
    {
        snapshot->capability_mask |= APP_MONITOR_SOURCE_RADAR;
    }
    if (device->ble_connected)
    {
        snapshot->capability_mask |= APP_MONITOR_SOURCE_BLE;
    }

    snapshot->source_ready_mask =
        app_monitor_summary_source_ready_mask(&audio, &radar, device);
    snapshot->source_valid_mask =
        app_monitor_summary_source_valid_mask(&audio, &radar, device);
    snapshot->source_stale_mask =
        app_monitor_summary_source_stale_mask(&audio, &radar);
    snapshot->source_error_mask =
        app_monitor_summary_source_error_mask(&audio, &radar, device);

    reason_flags = app_monitor_summary_audio_reasons(&audio) |
                   app_monitor_summary_radar_reasons(&radar) |
                   app_monitor_summary_device_reasons(device);
    if (!radar.valid)
    {
        reason_flags |= APP_MONITOR_REASON_FUSION_AUDIO_ONLY |
                        APP_MONITOR_REASON_FUSION_RADAR_ABSENT;
    }
    else if ((APP_MONITOR_PRESENCE_PRESENT == radar.presence_state) &&
             !radar.stale)
    {
        reason_flags |= APP_MONITOR_REASON_FUSION_RADAR_SUPPORT;
    }
    if ((APP_MONITOR_VITAL_LOW == radar.breath_state) ||
        (APP_MONITOR_VITAL_HIGH == radar.breath_state) ||
        (APP_MONITOR_VITAL_LOW == radar.heart_state) ||
        (APP_MONITOR_VITAL_HIGH == radar.heart_state))
    {
        reason_flags |= APP_MONITOR_REASON_FUSION_VITALS_ATTENTION;
    }

    snapshot->monitor_state =
        app_monitor_summary_select_monitor_state(&audio, &radar, device);
    snapshot->fusion_state =
        app_monitor_summary_select_fusion_state(&audio, &radar, device);
    snapshot->alert_level =
        app_monitor_summary_select_alert_level(&audio, &radar, device);
    snapshot->fusion_confidence =
        app_monitor_summary_select_confidence(&audio, &radar);
    snapshot->active_event_type =
        app_monitor_summary_select_active_event(&audio, &radar);
    if (APP_MONITOR_EVENT_NONE != snapshot->active_event_type)
    {
        reason_flags |= APP_MONITOR_REASON_FUSION_EVENT_ACTIVE;
    }
    snapshot->active_reason_flags = reason_flags;
    snapshot->cough_count_1min = (uint16_t)cough_count_1min;
    snapshot->cough_count_5min = (uint16_t)cough_count_5min;
    snapshot->event_count_total = monitor_summary_owner.night.event_count_total;
    snapshot->cough_event_count_total =
        monitor_summary_owner.night.cough_event_count;
    snapshot->last_event_id = last_event_id;
#if (APP_MONITOR_SUMMARY_MOCK_ENABLE)
    snapshot->flags |= APP_MONITOR_SNAPSHOT_FLAG_MOCK_DATA;
#else
    if (0u != (snapshot->source_valid_mask &
               (APP_MONITOR_SOURCE_AUDIO | APP_MONITOR_SOURCE_RADAR)))
    {
        snapshot->flags |= APP_MONITOR_SNAPSHOT_FLAG_PARTIAL_REAL_DATA;
    }
#endif

    *next_smoothed_prob_x100 = smoothed_prob;
    *next_cough_latched = cough_latched;
}

static void app_monitor_summary_update_periods_locked(
    const app_monitor_summary_snapshot_t *snapshot,
    uint32_t now_ms)
{
    bool session_active =
        snapshot->device.monitor_requested && snapshot->device.session_active;
    app_monitor_session_summary_t *periods[2] = {
        &monitor_summary_owner.night,
        &monitor_summary_owner.session
    };
    uint32_t *sums[2] = {
        &monitor_summary_owner.night_cough_prob_sum,
        &monitor_summary_owner.session_cough_prob_sum
    };

    if (session_active && !monitor_summary_owner.session_was_active)
    {
        app_monitor_summary_reset_period_locked(
            &monitor_summary_owner.session,
            &monitor_summary_owner.session_cough_prob_sum,
            now_ms,
            true);
    }
    monitor_summary_owner.session_was_active = session_active;
    monitor_summary_owner.session.active = session_active;

    for (uint32_t i = 0u; i < 2u; i++)
    {
        app_monitor_session_summary_t *summary = periods[i];
        uint32_t *sum = sums[i];
        bool update_this = (0u == i) || session_active;

        if (!update_this)
        {
            continue;
        }

        summary->active = true;
        summary->updated_ms = now_ms;
        summary->duration_ms = now_ms - summary->started_ms;
        summary->realtime_sample_count++;
        summary->reason_flags |= snapshot->active_reason_flags;
        summary->cough_count_1min = snapshot->cough_count_1min;
        summary->cough_count_5min = snapshot->cough_count_5min;
        if (summary->max_alert_level < (uint8_t)snapshot->alert_level)
        {
            summary->max_alert_level = (uint8_t)snapshot->alert_level;
        }
        if (summary->max_fusion_confidence < snapshot->fusion_confidence)
        {
            summary->max_fusion_confidence = snapshot->fusion_confidence;
        }
        if (summary->max_cough_prob_x100 < snapshot->audio.cough_prob_x100)
        {
            summary->max_cough_prob_x100 = snapshot->audio.cough_prob_x100;
        }
        *sum += snapshot->audio.cough_prob_x100;
        summary->avg_cough_prob_x100 =
            (uint8_t)(*sum / summary->realtime_sample_count);

        if (snapshot->radar.valid && !snapshot->radar.stale)
        {
            summary->rr_latest_bpm_x10 = snapshot->radar.rr_bpm_x10;
            summary->hr_latest_bpm_x10 = snapshot->radar.hr_bpm_x10;
            if (APP_MONITOR_INVALID_BPM_X10 != snapshot->radar.rr_bpm_x10)
            {
                if ((APP_MONITOR_INVALID_BPM_X10 == summary->rr_min_bpm_x10) ||
                    (snapshot->radar.rr_bpm_x10 < summary->rr_min_bpm_x10))
                {
                    summary->rr_min_bpm_x10 = snapshot->radar.rr_bpm_x10;
                }
                if (snapshot->radar.rr_bpm_x10 > summary->rr_max_bpm_x10)
                {
                    summary->rr_max_bpm_x10 = snapshot->radar.rr_bpm_x10;
                }
            }
            if (APP_MONITOR_INVALID_BPM_X10 != snapshot->radar.hr_bpm_x10)
            {
                if ((APP_MONITOR_INVALID_BPM_X10 == summary->hr_min_bpm_x10) ||
                    (snapshot->radar.hr_bpm_x10 < summary->hr_min_bpm_x10))
                {
                    summary->hr_min_bpm_x10 = snapshot->radar.hr_bpm_x10;
                }
                if (snapshot->radar.hr_bpm_x10 > summary->hr_max_bpm_x10)
                {
                    summary->hr_max_bpm_x10 = snapshot->radar.hr_bpm_x10;
                }
            }
        }
    }
}

static void app_monitor_summary_note_event_in_period_locked(
    app_monitor_session_summary_t *summary,
    const app_monitor_summary_event_t *event)
{
    summary->event_count_total++;
    if (app_monitor_summary_event_is_cough(event->event_type))
    {
        summary->cough_event_count++;
    }
    if (app_monitor_summary_event_is_warning(event->alert_level))
    {
        summary->warning_event_count++;
    }
    if (APP_MONITOR_EVENT_VITALS_ATTENTION == event->event_type)
    {
        summary->vitals_attention_count++;
    }
    if ((APP_MONITOR_EVENT_SOURCE_CHANGED == event->event_type) ||
        (APP_MONITOR_EVENT_SYSTEM_STATUS == event->event_type))
    {
        summary->source_event_count++;
    }
}

static uint32_t app_monitor_summary_count_recent_cough_locked(
    uint32_t now_ms,
    uint32_t window_ms)
{
    uint32_t count = 0u;

    for (uint32_t i = 0u; i < monitor_summary_owner.event_ring_count; i++)
    {
        const app_monitor_summary_event_t *event =
            &monitor_summary_owner.event_ring[i];
        if (app_monitor_summary_event_is_cough(event->event_type) &&
            (now_ms >= event->timestamp_ms) &&
            ((now_ms - event->timestamp_ms) <= window_ms))
        {
            count++;
        }
    }

    return count;
}

static uint32_t app_monitor_summary_audio_reasons(
    const app_monitor_audio_input_t *audio)
{
    uint32_t flags = audio->reason_flags & 0x000000FFUL;

    if (audio->valid && !audio->stale)
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

static uint32_t app_monitor_summary_radar_reasons(
    const app_monitor_radar_input_t *radar)
{
    uint32_t flags = radar->radar_reason_flags & 0x0000FF00UL;

    if (!radar->valid)
    {
        flags |= APP_MONITOR_REASON_RADAR_UNAVAILABLE;
        return flags;
    }
    if (radar->stale)
    {
        flags |= APP_MONITOR_REASON_RADAR_STALE;
    }
    if (APP_MONITOR_PRESENCE_PRESENT == radar->presence_state)
    {
        flags |= APP_MONITOR_REASON_RADAR_PRESENT;
    }
    else if (APP_MONITOR_PRESENCE_ABSENT == radar->presence_state)
    {
        flags |= APP_MONITOR_REASON_RADAR_NO_TARGET;
    }
    if (APP_MONITOR_MOTION_HIGH == radar->motion_state)
    {
        flags |= APP_MONITOR_REASON_RADAR_MOTION_HIGH;
    }
    if ((APP_MONITOR_VITAL_LOW == radar->breath_state) ||
        (APP_MONITOR_VITAL_HIGH == radar->breath_state) ||
        (APP_MONITOR_VITAL_LOW == radar->heart_state) ||
        (APP_MONITOR_VITAL_HIGH == radar->heart_state))
    {
        flags |= APP_MONITOR_REASON_RADAR_VITALS_ATTENTION;
    }
    if (radar->radar_quality < APP_MONITOR_SUMMARY_RADAR_POOR_QUALITY)
    {
        flags |= APP_MONITOR_REASON_RADAR_QUALITY_POOR;
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
    const app_monitor_radar_input_t *radar,
    const app_monitor_device_input_t *device)
{
    uint32_t mask = APP_MONITOR_SOURCE_FUSION_SUMMARY;

    if (audio->ready)
    {
        mask |= APP_MONITOR_SOURCE_AUDIO;
    }
    if (radar->ready)
    {
        mask |= APP_MONITOR_SOURCE_RADAR;
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
    const app_monitor_radar_input_t *radar,
    const app_monitor_device_input_t *device)
{
    uint32_t mask = APP_MONITOR_SOURCE_FUSION_SUMMARY;

    if (audio->valid && !audio->stale)
    {
        mask |= APP_MONITOR_SOURCE_AUDIO;
    }
    if (radar->valid && !radar->stale)
    {
        mask |= APP_MONITOR_SOURCE_RADAR;
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
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar)
{
    uint32_t mask = 0u;

    if (audio->stale)
    {
        mask |= APP_MONITOR_SOURCE_AUDIO;
    }
    if (radar->stale)
    {
        mask |= APP_MONITOR_SOURCE_RADAR;
    }

    return mask;
}

static uint32_t app_monitor_summary_source_error_mask(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar,
    const app_monitor_device_input_t *device)
{
    uint32_t mask = 0u;

    if (!device->shared_memory_ready || !device->model_ready)
    {
        mask |= APP_MONITOR_SOURCE_MODEL_SHARED;
    }
    if (audio->mic_quality_poor)
    {
        mask |= APP_MONITOR_SOURCE_AUDIO;
    }
    if (radar->valid &&
        (radar->radar_quality < APP_MONITOR_SUMMARY_RADAR_POOR_QUALITY))
    {
        mask |= APP_MONITOR_SOURCE_RADAR;
    }

    return mask;
}

static app_monitor_state_t app_monitor_summary_select_monitor_state(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar,
    const app_monitor_device_input_t *device)
{
    bool radar_vitals_attention =
        radar->valid && !radar->stale &&
        ((APP_MONITOR_VITAL_LOW == radar->breath_state) ||
         (APP_MONITOR_VITAL_HIGH == radar->breath_state) ||
         (APP_MONITOR_VITAL_LOW == radar->heart_state) ||
         (APP_MONITOR_VITAL_HIGH == radar->heart_state));

    if (!device->monitor_requested || !device->session_active)
    {
        return APP_MONITOR_STATE_IDLE;
    }
    if (!device->shared_memory_ready || !device->model_ready)
    {
        return (audio->ready || audio->valid) ?
            APP_MONITOR_STATE_DEGRADED : APP_MONITOR_STATE_ERROR;
    }
    if (!audio->valid || audio->stale || audio->mic_quality_poor)
    {
        return APP_MONITOR_STATE_DEGRADED;
    }
    if (audio->cough_density_high)
    {
        return APP_MONITOR_STATE_WARNING;
    }
    if (audio->cough_prob_x100 >= APP_MONITOR_SUMMARY_COUGH_WARNING_X100)
    {
        return (radar->valid &&
                (APP_MONITOR_PRESENCE_ABSENT == radar->presence_state)) ?
            APP_MONITOR_STATE_ATTENTION : APP_MONITOR_STATE_WARNING;
    }
    if (audio->cough_confirmed || radar_vitals_attention)
    {
        return APP_MONITOR_STATE_ATTENTION;
    }

    return APP_MONITOR_STATE_MONITORING;
}

static app_monitor_fusion_state_t app_monitor_summary_select_fusion_state(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar,
    const app_monitor_device_input_t *device)
{
    app_monitor_state_t state =
        app_monitor_summary_select_monitor_state(audio, radar, device);

    if (!device->monitor_requested || !device->session_active)
    {
        return APP_MONITOR_FUSION_STATE_UNKNOWN;
    }
    if ((APP_MONITOR_STATE_ERROR == state) ||
        (APP_MONITOR_STATE_DEGRADED == state))
    {
        return APP_MONITOR_FUSION_STATE_DEGRADED;
    }
    if (!radar->valid || radar->stale)
    {
        return APP_MONITOR_FUSION_STATE_AUDIO_ONLY;
    }
    if (APP_MONITOR_STATE_WARNING == state)
    {
        return APP_MONITOR_FUSION_STATE_WARNING;
    }
    if (APP_MONITOR_STATE_ATTENTION == state)
    {
        return APP_MONITOR_FUSION_STATE_ATTENTION;
    }

    return APP_MONITOR_FUSION_STATE_NORMAL;
}

static app_monitor_alert_level_t app_monitor_summary_select_alert_level(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar,
    const app_monitor_device_input_t *device)
{
    app_monitor_state_t state =
        app_monitor_summary_select_monitor_state(audio, radar, device);

    if (APP_MONITOR_STATE_ERROR == state)
    {
        return APP_MONITOR_ALERT_LEVEL_ERROR;
    }
    if (APP_MONITOR_STATE_DEGRADED == state)
    {
        return APP_MONITOR_ALERT_LEVEL_INFO;
    }
    if (APP_MONITOR_STATE_WARNING == state)
    {
        return APP_MONITOR_ALERT_LEVEL_WARNING;
    }
    if (APP_MONITOR_STATE_ATTENTION == state)
    {
        return APP_MONITOR_ALERT_LEVEL_ATTENTION;
    }

    return APP_MONITOR_ALERT_LEVEL_NONE;
}

static app_monitor_event_type_t app_monitor_summary_select_active_event(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar)
{
    bool vitals_attention =
        radar->valid && !radar->stale &&
        ((APP_MONITOR_VITAL_LOW == radar->breath_state) ||
         (APP_MONITOR_VITAL_HIGH == radar->breath_state) ||
         (APP_MONITOR_VITAL_LOW == radar->heart_state) ||
         (APP_MONITOR_VITAL_HIGH == radar->heart_state));

    if (audio->valid && !audio->stale && audio->cough_density_high)
    {
        return APP_MONITOR_EVENT_COUGH_BURST;
    }
    if (audio->valid && !audio->stale && audio->cough_confirmed)
    {
        return APP_MONITOR_EVENT_AUDIO_CANDIDATE;
    }
    if (vitals_attention)
    {
        return APP_MONITOR_EVENT_VITALS_ATTENTION;
    }

    return APP_MONITOR_EVENT_NONE;
}

static app_monitor_event_type_t app_monitor_summary_select_event_type(
    const app_monitor_summary_snapshot_t *old_snapshot,
    const app_monitor_summary_snapshot_t *new_snapshot)
{
    if (old_snapshot->sequence == 0u)
    {
        return APP_MONITOR_EVENT_SYSTEM_STATUS;
    }
    if ((APP_MONITOR_EVENT_NONE != new_snapshot->active_event_type) &&
        ((old_snapshot->active_event_type !=
          new_snapshot->active_event_type) ||
         (!old_snapshot->audio.cough_confirmed &&
          new_snapshot->audio.cough_confirmed)))
    {
        return new_snapshot->active_event_type;
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
    event.source_flags =
        app_monitor_summary_event_source_flags(snapshot, event_type);
    event.confidence = snapshot->fusion_confidence;
    event.cough_prob_x100 = snapshot->audio.cough_prob_x100;
    event.cough_count_1min = snapshot->cough_count_1min;
    event.cough_count_5min = snapshot->cough_count_5min;

    taskENTER_CRITICAL();
    event.event_id = monitor_summary_owner.next_event_id++;
    if (0u != monitor_summary_owner.last_event_ms)
    {
        event.duration_ms = now_ms - monitor_summary_owner.last_event_ms;
    }
    monitor_summary_owner.latest_event = event;
    monitor_summary_owner.snapshot.last_event_id = event.event_id;
    monitor_summary_owner.snapshot.event_count_total++;
    if (app_monitor_summary_event_is_cough(event.event_type))
    {
        monitor_summary_owner.snapshot.cough_event_count_total++;
    }
    monitor_summary_owner.event_ring[
        monitor_summary_owner.event_ring_write_index] = event;
    monitor_summary_owner.event_ring_write_index++;
    if (APP_MONITOR_SUMMARY_EVENT_RING_SIZE <=
        monitor_summary_owner.event_ring_write_index)
    {
        monitor_summary_owner.event_ring_write_index = 0u;
    }
    if (monitor_summary_owner.event_ring_count <
        APP_MONITOR_SUMMARY_EVENT_RING_SIZE)
    {
        monitor_summary_owner.event_ring_count++;
    }
    app_monitor_summary_note_event_in_period_locked(
        &monitor_summary_owner.night,
        &event);
    if (monitor_summary_owner.session.active)
    {
        app_monitor_summary_note_event_in_period_locked(
            &monitor_summary_owner.session,
            &event);
    }
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
        uint8_t mock_data =
            (0u != (snapshot->flags & APP_MONITOR_SNAPSHOT_FLAG_MOCK_DATA)) ?
            1u : 0u;
        uint8_t real_mic =
            (!mock_data &&
             (0u != (snapshot->source_valid_mask & APP_MONITOR_SOURCE_AUDIO))) ?
            1u : 0u;
        uint8_t real_radar =
            (!mock_data &&
             (0u != (snapshot->source_valid_mask & APP_MONITOR_SOURCE_RADAR))) ?
            1u : 0u;

        printf("[SUMMARY_REALTIME] t_ms=%lu seq=%lu mock=%u "
               "real_mic=%u real_radar=%u flags=0x%08lx "
               "cough_prob_x100=%u raw_age_ms=%lu cough_1m=%u "
               "cough_5m=%u rr=",
               (unsigned long)snapshot->timestamp_ms,
               (unsigned long)snapshot->sequence,
               (unsigned int)mock_data,
               (unsigned int)real_mic,
               (unsigned int)real_radar,
               (unsigned long)snapshot->flags,
               (unsigned int)snapshot->audio.cough_prob_x100,
               (unsigned long)snapshot->audio.age_ms,
               (unsigned int)snapshot->cough_count_1min,
               (unsigned int)snapshot->cough_count_5min);
        app_monitor_summary_print_bpm_x10(snapshot->radar.rr_bpm_x10);
        printf(" hr=");
        app_monitor_summary_print_bpm_x10(snapshot->radar.hr_bpm_x10);
        printf(" presence=%u motion=%u radar_valid=%u radar_quality=%u "
               "monitor=%u fusion=%u alert=%u active_event=%u "
               "confidence=%u ready=0x%08lx valid=0x%08lx stale=0x%08lx "
               "error=0x%08lx reason=0x%08lx ring_count=%lu\r\n",
               (unsigned int)snapshot->radar.presence_state,
               (unsigned int)snapshot->radar.motion_x100,
               (unsigned int)(snapshot->radar.valid ? 1u : 0u),
               (unsigned int)snapshot->radar.radar_quality,
               (unsigned int)snapshot->monitor_state,
               (unsigned int)snapshot->fusion_state,
               (unsigned int)snapshot->alert_level,
               (unsigned int)snapshot->active_event_type,
               (unsigned int)snapshot->fusion_confidence,
               (unsigned long)snapshot->source_ready_mask,
               (unsigned long)snapshot->source_valid_mask,
               (unsigned long)snapshot->source_stale_mask,
               (unsigned long)snapshot->source_error_mask,
               (unsigned long)snapshot->active_reason_flags,
               (unsigned long)monitor_summary_owner.event_ring_count);
    }
#else
    (void)snapshot;
    (void)now_ms;
#endif
}

static void app_monitor_summary_maybe_print_session(uint32_t now_ms)
{
#if (APP_MONITOR_SUMMARY_DIAG_ENABLE)
    app_monitor_session_summary_t session;
    app_monitor_session_summary_t night;
    app_monitor_summary_snapshot_t snapshot;
    bool should_print = false;

    taskENTER_CRITICAL();
    if ((0u == monitor_summary_owner.last_session_diag_ms) ||
        ((now_ms - monitor_summary_owner.last_session_diag_ms) >=
         APP_MONITOR_SUMMARY_SESSION_DIAG_PERIOD_MS))
    {
        monitor_summary_owner.last_session_diag_ms = now_ms;
        session = monitor_summary_owner.session;
        night = monitor_summary_owner.night;
        snapshot = monitor_summary_owner.snapshot;
        should_print = true;
    }
    taskEXIT_CRITICAL();

    if (should_print)
    {
        uint8_t mock_data =
            (0u != (snapshot.flags & APP_MONITOR_SNAPSHOT_FLAG_MOCK_DATA)) ?
            1u : 0u;
        uint8_t real_mic =
            (!mock_data &&
             (0u != (snapshot.source_valid_mask & APP_MONITOR_SOURCE_AUDIO))) ?
            1u : 0u;
        uint8_t real_radar =
            (!mock_data &&
             (0u != (snapshot.source_valid_mask & APP_MONITOR_SOURCE_RADAR))) ?
            1u : 0u;

        printf("[SUMMARY_SESSION] t_ms=%lu mock=%u real_mic=%u "
               "real_radar=%u active=%u duration_ms=%lu samples=%lu "
               "events=%lu cough_events=%lu warnings=%lu vitals=%lu "
               "cough_1m=%u cough_5m=%u max_prob=%u avg_prob=%u "
               "max_alert=%u max_conf=%u rr_latest=",
               (unsigned long)now_ms,
               (unsigned int)mock_data,
               (unsigned int)real_mic,
               (unsigned int)real_radar,
               session.active ? 1u : 0u,
               (unsigned long)session.duration_ms,
               (unsigned long)session.realtime_sample_count,
               (unsigned long)session.event_count_total,
               (unsigned long)session.cough_event_count,
               (unsigned long)session.warning_event_count,
               (unsigned long)session.vitals_attention_count,
               (unsigned int)session.cough_count_1min,
               (unsigned int)session.cough_count_5min,
               (unsigned int)session.max_cough_prob_x100,
               (unsigned int)session.avg_cough_prob_x100,
               (unsigned int)session.max_alert_level,
               (unsigned int)session.max_fusion_confidence);
        app_monitor_summary_print_bpm_x10(session.rr_latest_bpm_x10);
        printf(" hr_latest=");
        app_monitor_summary_print_bpm_x10(session.hr_latest_bpm_x10);
        printf(" reason=0x%08lx\r\n",
               (unsigned long)session.reason_flags);

        printf("[SUMMARY_NIGHT] t_ms=%lu mock=%u real_mic=%u "
               "real_radar=%u active=%u duration_ms=%lu samples=%lu "
               "events=%lu cough_events=%lu warnings=%lu vitals=%lu "
               "max_prob=%u avg_prob=%u max_alert=%u max_conf=%u "
               "reason=0x%08lx storage=ram_only\r\n",
               (unsigned long)now_ms,
               (unsigned int)mock_data,
               (unsigned int)real_mic,
               (unsigned int)real_radar,
               night.active ? 1u : 0u,
               (unsigned long)night.duration_ms,
               (unsigned long)night.realtime_sample_count,
               (unsigned long)night.event_count_total,
               (unsigned long)night.cough_event_count,
               (unsigned long)night.warning_event_count,
               (unsigned long)night.vitals_attention_count,
               (unsigned int)night.max_cough_prob_x100,
               (unsigned int)night.avg_cough_prob_x100,
               (unsigned int)night.max_alert_level,
               (unsigned int)night.max_fusion_confidence,
               (unsigned long)night.reason_flags);
    }
#else
    (void)now_ms;
#endif
}

static void app_monitor_summary_print_event(
    const app_monitor_summary_event_t *event)
{
#if (APP_MONITOR_SUMMARY_DIAG_ENABLE)
    uint8_t mock_data = (APP_MONITOR_SUMMARY_MOCK_ENABLE != 0u) ? 1u : 0u;
    uint8_t real_mic =
        (!mock_data && (0u != (event->source_flags & APP_MONITOR_SOURCE_AUDIO))) ?
        1u : 0u;
    uint8_t real_radar =
        (!mock_data && (0u != (event->source_flags & APP_MONITOR_SOURCE_RADAR))) ?
        1u : 0u;

    printf("[SUMMARY_EVENT] t_ms=%lu event_id=%lu mock=%u "
           "real_mic=%u real_radar=%u type=%u monitor=%u fusion=%u "
           "alert=%u confidence=%u cough_prob_x100=%u cough_1m=%u "
           "cough_5m=%u source_flags=0x%02x valid=0x%08lx "
           "stale=0x%08lx error=0x%08lx reason=0x%08lx duration_ms=%lu\r\n",
           (unsigned long)event->timestamp_ms,
           (unsigned long)event->event_id,
           (unsigned int)mock_data,
           (unsigned int)real_mic,
           (unsigned int)real_radar,
           (unsigned int)event->event_type,
           (unsigned int)event->monitor_state,
           (unsigned int)event->fusion_state,
           (unsigned int)event->alert_level,
           (unsigned int)event->confidence,
           (unsigned int)event->cough_prob_x100,
           (unsigned int)event->cough_count_1min,
           (unsigned int)event->cough_count_5min,
           (unsigned int)event->source_flags,
           (unsigned long)event->source_valid_mask,
           (unsigned long)event->source_stale_mask,
           (unsigned long)event->source_error_mask,
           (unsigned long)event->reason_flags,
           (unsigned long)event->duration_ms);
#else
    (void)event;
#endif
}

static void app_monitor_summary_print_bpm_x10(uint16_t value)
{
    printf("%lu.%lu",
           (unsigned long)(value / 10u),
           (unsigned long)(value % 10u));
}

static uint8_t app_monitor_summary_clamp_u8(uint8_t value, uint8_t max_value)
{
    return (value > max_value) ? max_value : value;
}

static uint16_t app_monitor_summary_clamp_bpm_x10(uint16_t value,
                                                  uint16_t max_value)
{
    return (value > max_value) ? max_value : value;
}

static uint8_t app_monitor_summary_smooth_prob(uint8_t previous,
                                               bool have_previous,
                                               uint8_t current)
{
    if (!have_previous)
    {
        return current;
    }

    return (uint8_t)(((uint32_t)previous * 3u + current + 2u) / 4u);
}

static uint8_t app_monitor_summary_select_radar_quality(
    const app_monitor_radar_input_t *radar)
{
    uint8_t quality = radar->radar_quality;

    if (!radar->valid)
    {
        return 0u;
    }
    if (0u == quality)
    {
        quality = (APP_MONITOR_PRESENCE_PRESENT == radar->presence_state) ?
            85u : 55u;
    }
    if (radar->stale && (quality > 30u))
    {
        quality = (uint8_t)(quality - 30u);
    }
    if ((APP_MONITOR_VITAL_INVALID == radar->breath_state) ||
        (APP_MONITOR_VITAL_INVALID == radar->heart_state))
    {
        quality = (quality > 20u) ? (uint8_t)(quality - 20u) : 0u;
    }

    return app_monitor_summary_clamp_u8(quality, APP_MONITOR_MAX_PERCENT);
}

static uint8_t app_monitor_summary_select_vital_state(uint16_t value_x10,
                                                      uint16_t low_x10,
                                                      uint16_t high_x10)
{
    if (APP_MONITOR_INVALID_BPM_X10 == value_x10)
    {
        return APP_MONITOR_VITAL_UNKNOWN;
    }
    if (value_x10 < low_x10)
    {
        return APP_MONITOR_VITAL_LOW;
    }
    if (value_x10 > high_x10)
    {
        return APP_MONITOR_VITAL_HIGH;
    }

    return APP_MONITOR_VITAL_NORMAL;
}

static uint8_t app_monitor_summary_select_motion_state(
    const app_monitor_radar_input_t *radar)
{
    if (!radar->valid)
    {
        return APP_MONITOR_MOTION_UNKNOWN;
    }
    if ((APP_MONITOR_MOTION_HIGH == radar->motion_state) ||
        (radar->motion_x100 >= APP_MONITOR_SUMMARY_MOTION_HIGH_X100))
    {
        return APP_MONITOR_MOTION_HIGH;
    }

    return APP_MONITOR_MOTION_LOW;
}

static uint8_t app_monitor_summary_select_confidence(
    const app_monitor_audio_input_t *audio,
    const app_monitor_radar_input_t *radar)
{
    uint32_t confidence = audio->valid ? audio->audio_quality : 0u;

    if (radar->valid && !radar->stale)
    {
        confidence = ((confidence * 2u) + radar->radar_quality) / 3u;
        if ((APP_MONITOR_PRESENCE_PRESENT == radar->presence_state) &&
            (confidence <= 95u))
        {
            confidence += 5u;
        }
        else if ((APP_MONITOR_PRESENCE_ABSENT == radar->presence_state) &&
                 (confidence > 25u))
        {
            confidence -= 25u;
        }
    }
    else
    {
        confidence = (confidence * 75u) / 100u;
    }

    return (uint8_t)((confidence > 100u) ? 100u : confidence);
}

static uint8_t app_monitor_summary_event_source_flags(
    const app_monitor_summary_snapshot_t *snapshot,
    app_monitor_event_type_t event_type)
{
    uint8_t flags = 0u;

    if ((APP_MONITOR_EVENT_AUDIO_CANDIDATE == event_type) ||
        (APP_MONITOR_EVENT_COUGH_BURST == event_type))
    {
        flags |= (uint8_t)APP_MONITOR_SOURCE_AUDIO;
    }
    if (snapshot->radar.valid)
    {
        flags |= (uint8_t)APP_MONITOR_SOURCE_RADAR;
    }
    if (APP_MONITOR_EVENT_SOURCE_CHANGED != event_type)
    {
        flags |= (uint8_t)APP_MONITOR_SOURCE_FUSION_SUMMARY;
    }

    return flags;
}

static bool app_monitor_summary_event_is_cough(
    app_monitor_event_type_t event_type)
{
    return (APP_MONITOR_EVENT_AUDIO_CANDIDATE == event_type) ||
           (APP_MONITOR_EVENT_COUGH_BURST == event_type);
}

static bool app_monitor_summary_event_is_warning(
    app_monitor_alert_level_t alert_level)
{
    return (APP_MONITOR_ALERT_LEVEL_WARNING == alert_level) ||
           (APP_MONITOR_ALERT_LEVEL_ERROR == alert_level);
}

static uint32_t app_monitor_summary_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

#if (APP_MONITOR_SUMMARY_MOCK_ENABLE)
static void app_monitor_summary_mock_task(void *pvParameters)
{
    uint32_t step = 0u;

    (void)pvParameters;

    for (;;)
    {
        app_monitor_audio_input_t audio;
        app_monitor_radar_input_t radar;
        app_monitor_device_input_t device;
        uint32_t now_ms = app_monitor_summary_now_ms();

        app_monitor_summary_mock_make_inputs(step,
                                             now_ms,
                                             &audio,
                                             &radar,
                                             &device);
        (void)app_monitor_summary_update_audio(&audio);
        (void)app_monitor_summary_update_radar(&radar);
        (void)app_monitor_summary_update_device(&device);
        (void)app_monitor_summary_tick(now_ms);

        step++;
        vTaskDelay(pdMS_TO_TICKS(APP_MONITOR_SUMMARY_MOCK_PERIOD_MS));
    }
}

static void app_monitor_summary_mock_make_inputs(uint32_t step,
                                                 uint32_t now_ms,
                                                 app_monitor_audio_input_t *audio,
                                                 app_monitor_radar_input_t *radar,
                                                 app_monitor_device_input_t *device)
{
    uint32_t phase = step % 28u;
    uint8_t cough_prob = 8u;
    uint16_t rr_x10 = 164u;
    uint16_t hr_x10 = 720u;
    uint8_t motion = 12u;
    uint8_t radar_quality = 88u;
    uint8_t presence = APP_MONITOR_PRESENCE_PRESENT;

    if ((6u == phase) || (8u == phase) || (10u == phase))
    {
        cough_prob = (uint8_t)(82u + phase);
        motion = 48u;
    }
    else if ((7u == phase) || (9u == phase) || (11u == phase))
    {
        cough_prob = 12u;
        motion = 25u;
    }
    else if ((12u <= phase) && (phase <= 15u))
    {
        cough_prob = 34u;
        rr_x10 = 336u;
        hr_x10 = 1460u;
        motion = 38u;
    }
    else if ((16u <= phase) && (phase <= 18u))
    {
        cough_prob = 18u;
        rr_x10 = 0u;
        hr_x10 = 0u;
        motion = 4u;
        radar_quality = 52u;
        presence = APP_MONITOR_PRESENCE_ABSENT;
    }
    else if ((19u <= phase) && (phase <= 20u))
    {
        cough_prob = 28u;
        motion = 82u;
    }
    else if (21u == phase)
    {
        cough_prob = 78u;
        motion = 70u;
    }

    memset(audio, 0, sizeof(*audio));
    audio->valid = true;
    audio->ready = true;
    audio->stale = false;
    audio->cough_prob_x100 = cough_prob;
    audio->event_threshold_x100 = APP_MONITOR_SUMMARY_COUGH_WARNING_X100;
    audio->audio_quality = 92u;
    audio->model_status = 0u;
    audio->input_sequence = step;
    audio->result_sequence = step;
    audio->age_ms = 0u;

    memset(radar, 0, sizeof(*radar));
    radar->valid = true;
    radar->ready = true;
    radar->stale = false;
    radar->presence_state = presence;
    radar->motion_x100 = motion;
    radar->radar_quality = radar_quality;
    radar->rr_bpm_x10 = rr_x10;
    radar->hr_bpm_x10 = hr_x10;
    radar->distance_cm =
        (APP_MONITOR_PRESENCE_PRESENT == presence) ? 120u : 0u;
    radar->age_ms = 0u;

    memset(device, 0, sizeof(*device));
    device->monitor_requested = true;
    device->ble_connected = false;
    device->time_synced = false;
    device->model_ready = true;
    device->shared_memory_ready = true;
    device->session_active = true;
    device->uptime_ms = now_ms;
}
#endif /* APP_MONITOR_SUMMARY_MOCK_ENABLE */

#endif /* APP_MONITOR_SUMMARY_ENABLE */

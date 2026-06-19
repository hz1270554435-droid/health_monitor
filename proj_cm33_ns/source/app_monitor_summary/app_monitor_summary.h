/*******************************************************************************
* File Name : app_monitor_summary.h
*
* Description : Optional firmware-side monitor summary owner API.
*******************************************************************************/

#ifndef __APP_MONITOR_SUMMARY_H__
#define __APP_MONITOR_SUMMARY_H__

#include <stdbool.h>
#include <stdint.h>

#include "cy_pdl.h"

#include "app_monitor_summary_types.h"

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef APP_MONITOR_SUMMARY_ENABLE
#define APP_MONITOR_SUMMARY_ENABLE              (0u)
#endif

#ifndef APP_MONITOR_SUMMARY_DIAG_ENABLE
#define APP_MONITOR_SUMMARY_DIAG_ENABLE         (0u)
#endif

#ifndef APP_MONITOR_SUMMARY_MOCK_ENABLE
#define APP_MONITOR_SUMMARY_MOCK_ENABLE         (0u)
#endif

#ifndef APP_MONITOR_SUMMARY_EVENT_COOLDOWN_MS
#define APP_MONITOR_SUMMARY_EVENT_COOLDOWN_MS   (1500u)
#endif

#ifndef APP_MONITOR_SUMMARY_EVENT_RING_SIZE
#define APP_MONITOR_SUMMARY_EVENT_RING_SIZE     (32u)
#endif

#ifndef APP_MONITOR_SUMMARY_DIAG_PERIOD_MS
#define APP_MONITOR_SUMMARY_DIAG_PERIOD_MS      (1000u)
#endif

#ifndef APP_MONITOR_SUMMARY_SESSION_DIAG_PERIOD_MS
#define APP_MONITOR_SUMMARY_SESSION_DIAG_PERIOD_MS (5000u)
#endif

#ifndef APP_MONITOR_SUMMARY_MOCK_PERIOD_MS
#define APP_MONITOR_SUMMARY_MOCK_PERIOD_MS      (500u)
#endif

#ifndef APP_MONITOR_SUMMARY_TASK_STACK_SIZE
#define APP_MONITOR_SUMMARY_TASK_STACK_SIZE     (1024u)
#endif

#ifndef APP_MONITOR_SUMMARY_TASK_PRIORITY
#define APP_MONITOR_SUMMARY_TASK_PRIORITY       (1u)
#endif

#ifndef APP_MONITOR_SUMMARY_AUDIO_STALE_MS
#define APP_MONITOR_SUMMARY_AUDIO_STALE_MS      (2000u)
#endif

#ifndef APP_MONITOR_SUMMARY_RADAR_STALE_MS
#define APP_MONITOR_SUMMARY_RADAR_STALE_MS      (3000u)
#endif

#ifndef APP_MONITOR_SUMMARY_COUGH_ATTENTION_X100
#define APP_MONITOR_SUMMARY_COUGH_ATTENTION_X100 (50u)
#endif

#ifndef APP_MONITOR_SUMMARY_COUGH_WARNING_X100
#define APP_MONITOR_SUMMARY_COUGH_WARNING_X100  (75u)
#endif

#ifndef APP_MONITOR_SUMMARY_COUGH_RELEASE_X100
#define APP_MONITOR_SUMMARY_COUGH_RELEASE_X100  (35u)
#endif

#ifndef APP_MONITOR_SUMMARY_COUGH_BURST_1MIN
#define APP_MONITOR_SUMMARY_COUGH_BURST_1MIN    (3u)
#endif

#ifndef APP_MONITOR_SUMMARY_COUGH_BURST_5MIN
#define APP_MONITOR_SUMMARY_COUGH_BURST_5MIN    (5u)
#endif

#ifndef APP_MONITOR_SUMMARY_MOTION_HIGH_X100
#define APP_MONITOR_SUMMARY_MOTION_HIGH_X100    (65u)
#endif

#ifndef APP_MONITOR_SUMMARY_RADAR_POOR_QUALITY
#define APP_MONITOR_SUMMARY_RADAR_POOR_QUALITY  (35u)
#endif

#ifndef APP_MONITOR_SUMMARY_RR_LOW_BPM_X10
#define APP_MONITOR_SUMMARY_RR_LOW_BPM_X10      (80u)
#endif

#ifndef APP_MONITOR_SUMMARY_RR_HIGH_BPM_X10
#define APP_MONITOR_SUMMARY_RR_HIGH_BPM_X10     (300u)
#endif

#ifndef APP_MONITOR_SUMMARY_HR_LOW_BPM_X10
#define APP_MONITOR_SUMMARY_HR_LOW_BPM_X10      (400u)
#endif

#ifndef APP_MONITOR_SUMMARY_HR_HIGH_BPM_X10
#define APP_MONITOR_SUMMARY_HR_HIGH_BPM_X10     (1400u)
#endif

#if ((APP_MONITOR_SUMMARY_ENABLE != 0u) && \
     (APP_MONITOR_SUMMARY_ENABLE != 1u))
#error "Unsupported APP_MONITOR_SUMMARY_ENABLE"
#endif

#if ((APP_MONITOR_SUMMARY_DIAG_ENABLE != 0u) && \
     (APP_MONITOR_SUMMARY_DIAG_ENABLE != 1u))
#error "Unsupported APP_MONITOR_SUMMARY_DIAG_ENABLE"
#endif

#if ((APP_MONITOR_SUMMARY_MOCK_ENABLE != 0u) && \
     (APP_MONITOR_SUMMARY_MOCK_ENABLE != 1u))
#error "Unsupported APP_MONITOR_SUMMARY_MOCK_ENABLE"
#endif

#if (APP_MONITOR_SUMMARY_EVENT_RING_SIZE < 4u)
#error "APP_MONITOR_SUMMARY_EVENT_RING_SIZE must be at least 4"
#endif

#if (APP_MONITOR_SUMMARY_ENABLE)

typedef struct
{
    bool active;
    uint32_t started_ms;
    uint32_t updated_ms;
    uint32_t duration_ms;
    uint32_t realtime_sample_count;
    uint32_t event_count_total;
    uint32_t cough_event_count;
    uint32_t warning_event_count;
    uint32_t vitals_attention_count;
    uint32_t source_event_count;
    uint32_t reason_flags;
    uint8_t max_alert_level;
    uint8_t max_fusion_confidence;
    uint8_t max_cough_prob_x100;
    uint8_t avg_cough_prob_x100;
    uint16_t cough_count_1min;
    uint16_t cough_count_5min;
    uint16_t rr_latest_bpm_x10;
    uint16_t hr_latest_bpm_x10;
    uint16_t rr_min_bpm_x10;
    uint16_t rr_max_bpm_x10;
    uint16_t hr_min_bpm_x10;
    uint16_t hr_max_bpm_x10;
} app_monitor_session_summary_t;

cy_rslt_t app_monitor_summary_init(void);
cy_rslt_t app_monitor_summary_update_audio(
    const app_monitor_audio_input_t *audio);
cy_rslt_t app_monitor_summary_update_device(
    const app_monitor_device_input_t *device);
cy_rslt_t app_monitor_summary_update_radar(
    const app_monitor_radar_input_t *radar);
cy_rslt_t app_monitor_summary_tick(uint32_t now_ms);
cy_rslt_t app_monitor_summary_get_snapshot(
    app_monitor_summary_snapshot_t *snapshot);
bool app_monitor_summary_get_latest_event(
    app_monitor_summary_event_t *event);
bool app_monitor_summary_get_event_by_age(uint32_t index_from_latest,
                                          app_monitor_summary_event_t *event);
uint32_t app_monitor_summary_get_event_count(void);
cy_rslt_t app_monitor_summary_get_session_summary(
    app_monitor_session_summary_t *summary);
cy_rslt_t app_monitor_summary_get_night_summary(
    app_monitor_session_summary_t *summary);

#else

typedef struct
{
    bool active;
    uint32_t started_ms;
    uint32_t updated_ms;
    uint32_t duration_ms;
    uint32_t realtime_sample_count;
    uint32_t event_count_total;
    uint32_t cough_event_count;
    uint32_t warning_event_count;
    uint32_t vitals_attention_count;
    uint32_t source_event_count;
    uint32_t reason_flags;
    uint8_t max_alert_level;
    uint8_t max_fusion_confidence;
    uint8_t max_cough_prob_x100;
    uint8_t avg_cough_prob_x100;
    uint16_t cough_count_1min;
    uint16_t cough_count_5min;
    uint16_t rr_latest_bpm_x10;
    uint16_t hr_latest_bpm_x10;
    uint16_t rr_min_bpm_x10;
    uint16_t rr_max_bpm_x10;
    uint16_t hr_min_bpm_x10;
    uint16_t hr_max_bpm_x10;
} app_monitor_session_summary_t;

static inline cy_rslt_t app_monitor_summary_init(void)
{
    return CY_RSLT_SUCCESS;
}

static inline cy_rslt_t app_monitor_summary_update_audio(
    const app_monitor_audio_input_t *audio)
{
    (void)audio;
    return CY_RSLT_SUCCESS;
}

static inline cy_rslt_t app_monitor_summary_update_device(
    const app_monitor_device_input_t *device)
{
    (void)device;
    return CY_RSLT_SUCCESS;
}

static inline cy_rslt_t app_monitor_summary_update_radar(
    const app_monitor_radar_input_t *radar)
{
    (void)radar;
    return CY_RSLT_SUCCESS;
}

static inline cy_rslt_t app_monitor_summary_tick(uint32_t now_ms)
{
    (void)now_ms;
    return CY_RSLT_SUCCESS;
}

static inline cy_rslt_t app_monitor_summary_get_snapshot(
    app_monitor_summary_snapshot_t *snapshot)
{
    if (NULL != snapshot)
    {
        *snapshot = (app_monitor_summary_snapshot_t){0};
    }
    return CY_RSLT_SUCCESS;
}

static inline bool app_monitor_summary_get_latest_event(
    app_monitor_summary_event_t *event)
{
    if (NULL != event)
    {
        *event = (app_monitor_summary_event_t){0};
    }
    return false;
}

static inline bool app_monitor_summary_get_event_by_age(
    uint32_t index_from_latest,
    app_monitor_summary_event_t *event)
{
    (void)index_from_latest;
    if (NULL != event)
    {
        *event = (app_monitor_summary_event_t){0};
    }
    return false;
}

static inline uint32_t app_monitor_summary_get_event_count(void)
{
    return 0u;
}

static inline cy_rslt_t app_monitor_summary_get_session_summary(
    app_monitor_session_summary_t *summary)
{
    if (NULL != summary)
    {
        *summary = (app_monitor_session_summary_t){0};
    }
    return CY_RSLT_SUCCESS;
}

static inline cy_rslt_t app_monitor_summary_get_night_summary(
    app_monitor_session_summary_t *summary)
{
    if (NULL != summary)
    {
        *summary = (app_monitor_session_summary_t){0};
    }
    return CY_RSLT_SUCCESS;
}

#endif /* APP_MONITOR_SUMMARY_ENABLE */

#if defined(__cplusplus)
}
#endif

#endif /* __APP_MONITOR_SUMMARY_H__ */

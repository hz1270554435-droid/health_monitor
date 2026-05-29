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

#ifndef APP_MONITOR_SUMMARY_EVENT_COOLDOWN_MS
#define APP_MONITOR_SUMMARY_EVENT_COOLDOWN_MS   (1500u)
#endif

#ifndef APP_MONITOR_SUMMARY_DIAG_PERIOD_MS
#define APP_MONITOR_SUMMARY_DIAG_PERIOD_MS      (1000u)
#endif

#if ((APP_MONITOR_SUMMARY_ENABLE != 0u) && \
     (APP_MONITOR_SUMMARY_ENABLE != 1u))
#error "Unsupported APP_MONITOR_SUMMARY_ENABLE"
#endif

#if ((APP_MONITOR_SUMMARY_DIAG_ENABLE != 0u) && \
     (APP_MONITOR_SUMMARY_DIAG_ENABLE != 1u))
#error "Unsupported APP_MONITOR_SUMMARY_DIAG_ENABLE"
#endif

#if (APP_MONITOR_SUMMARY_ENABLE)

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

#else

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

#endif /* APP_MONITOR_SUMMARY_ENABLE */

#if defined(__cplusplus)
}
#endif

#endif /* __APP_MONITOR_SUMMARY_H__ */

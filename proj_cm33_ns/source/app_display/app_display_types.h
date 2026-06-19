/*******************************************************************************
* File Name : app_display_types.h
*
* Description : Shared types for the optional E84 display/UI presentation layer.
*
* These types are local to CM33_NS presentation code. They are not a shared
* memory ABI and are not part of the BLE wire protocol.
*******************************************************************************/

#ifndef __APP_DISPLAY_TYPES_H__
#define __APP_DISPLAY_TYPES_H__

#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

typedef enum
{
    E84_DISPLAY_HEALTH_INIT = 0,
    E84_DISPLAY_HEALTH_NORMAL,
    E84_DISPLAY_HEALTH_ATTENTION,
    E84_DISPLAY_HEALTH_WARNING,
    E84_DISPLAY_HEALTH_SENSOR_LOST,
    E84_DISPLAY_HEALTH_ERROR,
    E84_DISPLAY_HEALTH_COUNT
} e84_display_health_state_t;

typedef enum
{
    E84_DISPLAY_ALERT_NONE = 0,
    E84_DISPLAY_ALERT_COUGH_BURST,
    E84_DISPLAY_ALERT_RESP_RATE_ABNORMAL,
    E84_DISPLAY_ALERT_HEART_RATE_ABNORMAL,
    E84_DISPLAY_ALERT_BREATHING_GAP,
    E84_DISPLAY_ALERT_SENSOR_LOST,
    E84_DISPLAY_ALERT_SYSTEM_ERROR,
    E84_DISPLAY_ALERT_COUNT
} e84_display_alert_t;

typedef enum
{
    E84_DISPLAY_PAGE_BOOT = 0,
    E84_DISPLAY_PAGE_HOME,
    E84_DISPLAY_PAGE_ALERT,
    E84_DISPLAY_PAGE_DEBUG,
    E84_DISPLAY_PAGE_COUNT
} e84_display_page_t;

typedef enum
{
    E84_DISPLAY_SEVERITY_NONE = 0,
    E84_DISPLAY_SEVERITY_INFO,
    E84_DISPLAY_SEVERITY_ATTENTION,
    E84_DISPLAY_SEVERITY_WARNING,
    E84_DISPLAY_SEVERITY_ERROR,
    E84_DISPLAY_SEVERITY_COUNT
} e84_display_severity_t;

/* Radar source state — maps Monitor Summary radar flags to display semantics.
 * These values drive boundary text selection in the display renderer.
 * NORMAL: real radar connected, data valid, quality acceptable.
 * UNAVAILABLE: radar source not ready or not present.
 * STALE: radar data exists but exceeds freshness TTL.
 * INVALID: radar data present but malformed or quality zero.
 * LOW_QUALITY: radar data present but quality below poor threshold. */
typedef enum
{
    E84_DISPLAY_RADAR_SOURCE_NORMAL = 0,
    E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE,
    E84_DISPLAY_RADAR_SOURCE_STALE,
    E84_DISPLAY_RADAR_SOURCE_INVALID,
    E84_DISPLAY_RADAR_SOURCE_LOW_QUALITY,
    E84_DISPLAY_RADAR_SOURCE_COUNT
} e84_display_radar_source_state_t;

#define E84_DISPLAY_FLAG_AUDIO_VALID              (1u << 0)
#define E84_DISPLAY_FLAG_RADAR_VALID              (1u << 1)
#define E84_DISPLAY_FLAG_FUSION_VALID             (1u << 2)
#define E84_DISPLAY_FLAG_BLE_VALID                (1u << 3)
#define E84_DISPLAY_FLAG_RR_VALID                 (1u << 4)
#define E84_DISPLAY_FLAG_HR_VALID                 (1u << 5)
#define E84_DISPLAY_FLAG_ALERT_LATCHED            (1u << 6)
#define E84_DISPLAY_FLAG_COUGH_MODEL_NOT_VERIFIED (1u << 7)

typedef struct
{
    uint32_t timestamp_ms;
    e84_display_health_state_t health_state;
    e84_display_alert_t active_alert;
    bool radar_presence;
    float breath_rate_bpm;
    float heart_rate_bpm;
    uint8_t radar_quality;
    uint16_t distance_cm;
    float mic_cough_prob;
    uint16_t cough_count_1min;
    uint16_t cough_count_5min;
    uint8_t audio_quality;
    uint8_t fusion_confidence;
    bool ble_connected;
    uint32_t flags;
    e84_display_radar_source_state_t radar_source_state;
    bool cough_model_not_verified;
} e84_display_snapshot_t;

typedef struct
{
    e84_display_alert_t code;
    e84_display_severity_t severity;
    const char *title;
    const char *short_message;
    uint32_t raised_timestamp_ms;
    uint32_t timeout_ms;
    bool dismissible;
    bool latched;
} e84_display_alert_presentation_t;

typedef struct
{
    e84_display_page_t current_page;
    e84_display_page_t previous_page;
    e84_display_health_state_t health_state;
    e84_display_alert_presentation_t alert;
    e84_display_snapshot_t snapshot;
    uint32_t display_dropped_commands;
    uint32_t last_refresh_timestamp_ms;
    const char *refresh_reason;
    bool smoke_enabled;
    bool dirty;
} e84_display_view_model_t;

const char *e84_display_health_state_name(
    e84_display_health_state_t state);
const char *e84_display_alert_name(e84_display_alert_t alert);
const char *e84_display_page_name(e84_display_page_t page);
const char *e84_display_severity_name(e84_display_severity_t severity);
const char *e84_display_radar_source_state_name(
    e84_display_radar_source_state_t state);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_DISPLAY_TYPES_H__ */

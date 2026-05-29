/*******************************************************************************
* File Name : app_monitor_summary_types.h
*
* Description : Local firmware types for the optional monitor summary owner.
*
* These types define the stable read-model surface for app_monitor_summary.
* The snapshot is intended to carry rich observations, lifecycle masks,
* metrics, reason flags, and only a minimal shared semantic layer
* (monitor_state / fusion_state / alert_level / confidence).
* Presentation consumers should extend or map this read model rather than
* bypassing it and rebuilding common rules independently.
*******************************************************************************/

#ifndef __APP_MONITOR_SUMMARY_TYPES_H__
#define __APP_MONITOR_SUMMARY_TYPES_H__

#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

typedef enum
{
    APP_MONITOR_STATE_INIT = 0,
    APP_MONITOR_STATE_IDLE,
    APP_MONITOR_STATE_MONITORING,
    APP_MONITOR_STATE_ATTENTION,
    APP_MONITOR_STATE_WARNING,
    APP_MONITOR_STATE_DEGRADED,
    APP_MONITOR_STATE_ERROR
} app_monitor_state_t;

typedef enum
{
    APP_MONITOR_FUSION_STATE_UNKNOWN = 0,
    APP_MONITOR_FUSION_STATE_AUDIO_ONLY,
    APP_MONITOR_FUSION_STATE_NORMAL,
    APP_MONITOR_FUSION_STATE_ATTENTION,
    APP_MONITOR_FUSION_STATE_WARNING,
    APP_MONITOR_FUSION_STATE_DEGRADED
} app_monitor_fusion_state_t;

typedef enum
{
    APP_MONITOR_ALERT_LEVEL_NONE = 0,
    APP_MONITOR_ALERT_LEVEL_INFO,
    APP_MONITOR_ALERT_LEVEL_ATTENTION,
    APP_MONITOR_ALERT_LEVEL_WARNING,
    APP_MONITOR_ALERT_LEVEL_ERROR
} app_monitor_alert_level_t;

typedef enum
{
    APP_MONITOR_EVENT_NONE = 0,
    APP_MONITOR_EVENT_STATE_CHANGED,
    APP_MONITOR_EVENT_ALERT_CHANGED,
    APP_MONITOR_EVENT_SOURCE_CHANGED,
    APP_MONITOR_EVENT_AUDIO_CANDIDATE,
    APP_MONITOR_EVENT_SYSTEM_STATUS
} app_monitor_event_type_t;

#define APP_MONITOR_SOURCE_AUDIO             (1UL << 0)
#define APP_MONITOR_SOURCE_RADAR             (1UL << 1)
#define APP_MONITOR_SOURCE_DEVICE_SESSION    (1UL << 2)
#define APP_MONITOR_SOURCE_MODEL_SHARED      (1UL << 3)
#define APP_MONITOR_SOURCE_BLE               (1UL << 4)
#define APP_MONITOR_SOURCE_FUSION_SUMMARY    (1UL << 5)

#define APP_MONITOR_REASON_AUDIO_RESULT_READY     (1UL << 0)
#define APP_MONITOR_REASON_AUDIO_COUGH_CANDIDATE  (1UL << 1)
#define APP_MONITOR_REASON_AUDIO_DENSITY_HIGH     (1UL << 2)
#define APP_MONITOR_REASON_AUDIO_QUALITY_POOR     (1UL << 3)
#define APP_MONITOR_REASON_AUDIO_STALE            (1UL << 4)
#define APP_MONITOR_REASON_AUDIO_UNAVAILABLE      (1UL << 5)

#define APP_MONITOR_REASON_RADAR_UNAVAILABLE      (1UL << 8)
#define APP_MONITOR_REASON_RADAR_API_INACTIVE     (1UL << 9)

#define APP_MONITOR_REASON_FUSION_AUDIO_ONLY      (1UL << 16)
#define APP_MONITOR_REASON_FUSION_RADAR_ABSENT    (1UL << 17)
#define APP_MONITOR_REASON_FUSION_DEGRADED        (1UL << 18)

#define APP_MONITOR_REASON_DEVICE_MONITOR_OFF     (1UL << 24)
#define APP_MONITOR_REASON_DEVICE_MODEL_NOT_READY (1UL << 25)
#define APP_MONITOR_REASON_DEVICE_SHARED_NOT_READY (1UL << 26)
#define APP_MONITOR_REASON_DEVICE_BLE_DISCONNECTED (1UL << 27)
#define APP_MONITOR_REASON_DEVICE_TIME_UNSYNCED   (1UL << 28)

typedef struct
{
    bool valid;
    bool ready;
    bool stale;
    bool mic_quality_poor;
    bool cough_confirmed;
    bool cough_density_high;
    uint8_t cough_prob_x100;
    uint8_t event_threshold_x100;
    uint8_t audio_quality;
    uint8_t model_status;
    uint32_t input_sequence;
    uint32_t result_sequence;
    uint32_t age_ms;
    uint32_t reason_flags;
} app_monitor_audio_input_t;

typedef struct
{
    bool valid;
    bool ready;
    bool stale;
    uint8_t presence_state;
    uint8_t motion_state;
    uint8_t breath_state;
    uint8_t radar_quality;
    uint16_t rr_bpm_x10;
    uint16_t hr_bpm_x10;
    uint16_t distance_cm;
    uint32_t age_ms;
    uint32_t radar_reason_flags;
} app_monitor_radar_input_t;

typedef struct
{
    bool monitor_requested;
    bool ble_connected;
    bool time_synced;
    bool model_ready;
    bool shared_memory_ready;
    bool session_active;
    uint32_t uptime_ms;
    uint32_t device_reason_flags;
} app_monitor_device_input_t;

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t sequence;
    app_monitor_state_t monitor_state;
    app_monitor_fusion_state_t fusion_state;
    app_monitor_alert_level_t alert_level;
    uint32_t active_reason_flags;
    uint32_t capability_mask;
    uint32_t source_ready_mask;
    uint32_t source_valid_mask;
    uint32_t source_stale_mask;
    uint32_t source_error_mask;
    app_monitor_audio_input_t audio;
    app_monitor_radar_input_t radar;
    app_monitor_device_input_t device;
    uint8_t fusion_confidence;
    uint16_t cough_count_1min;
    uint16_t cough_count_5min;
    uint32_t last_event_id;
    uint32_t flags;
} app_monitor_summary_snapshot_t;

typedef struct
{
    uint32_t event_id;
    uint32_t timestamp_ms;
    app_monitor_event_type_t event_type;
    app_monitor_state_t monitor_state;
    app_monitor_fusion_state_t fusion_state;
    app_monitor_alert_level_t alert_level;
    uint32_t reason_flags;
    uint32_t source_valid_mask;
    uint32_t source_stale_mask;
    uint32_t source_error_mask;
    uint8_t confidence;
    uint32_t duration_ms;
} app_monitor_summary_event_t;

#if defined(__cplusplus)
}
#endif

#endif /* __APP_MONITOR_SUMMARY_TYPES_H__ */

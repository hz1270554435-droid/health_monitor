#ifndef __APP_BLE_TYPES_H__
#define __APP_BLE_TYPES_H__

#include <stdbool.h>
#include <stdint.h>

#include "app_ble_config.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define APP_BLE_INVALID_U8                         (0xFFu)

#define APP_BLE_REALTIME_FRAME_LEN                 (19u)
#define APP_BLE_EVENT_FRAME_LEN                    (19u)
#define APP_BLE_CMD_MAX_PAYLOAD_LEN                (16u)
#define APP_BLE_CMD_RESPONSE_MAX_PAYLOAD_LEN       (16u)
#define APP_BLE_COMMAND_FRAME_MIN_LEN              (12u)
#define APP_BLE_COMMAND_FRAME_MAX_LEN              (APP_BLE_COMMAND_FRAME_MIN_LEN + \
                                                    APP_BLE_CMD_MAX_PAYLOAD_LEN)
#define APP_BLE_CMD_RESPONSE_FRAME_MIN_LEN         (13u)
#define APP_BLE_CMD_RESPONSE_FRAME_MAX_LEN         (APP_BLE_CMD_RESPONSE_FRAME_MIN_LEN + \
                                                    APP_BLE_CMD_RESPONSE_MAX_PAYLOAD_LEN)

typedef enum
{
    APP_BLE_OK = 0,
    APP_BLE_ERR_NOT_INIT = 1,
    APP_BLE_ERR_NOT_CONNECTED = 2,
    APP_BLE_ERR_NOT_SUBSCRIBED = 3,
    APP_BLE_ERR_QUEUE_FULL = 4,
    APP_BLE_ERR_INVALID_ARG = 5,
    APP_BLE_ERR_PACK_FAILED = 6,
    APP_BLE_ERR_CRC_FAILED = 7,
    APP_BLE_ERR_CMD_UNSUPPORTED = 8,
    APP_BLE_ERR_CMD_DENIED = 9,
    APP_BLE_ERR_STACK_FAILED = 10,
    APP_BLE_ERR_CMD_NOT_READY = 11,
    APP_BLE_ERR_BUSY = 12
} app_ble_error_t;

typedef enum
{
    APP_BLE_MSG_REALTIME = 0x10,
    APP_BLE_MSG_EVENT = 0x20,
    APP_BLE_MSG_COMMAND = 0x30,
    APP_BLE_MSG_COMMAND_RESPONSE = 0x31,
    APP_BLE_MSG_SUMMARY = 0x40,
    APP_BLE_MSG_DIAG = 0x50
} app_ble_msg_type_t;

typedef enum
{
    APP_BLE_CMD_PING = 0x01,
    APP_BLE_CMD_TIME_SYNC = 0x02,
    APP_BLE_CMD_START_MONITOR = 0x03,
    APP_BLE_CMD_STOP_MONITOR = 0x04,
    APP_BLE_CMD_SET_ALERT_THRESHOLD = 0x05,
    APP_BLE_CMD_GET_DEVICE_INFO = 0x06,
    APP_BLE_CMD_GET_NIGHT_SUMMARY = 0x07,
    APP_BLE_CMD_CLEAR_NIGHT_SUMMARY = 0x08,
    APP_BLE_CMD_SET_LOG_LEVEL = 0x09
} app_ble_cmd_id_t;

typedef enum
{
    APP_BLE_FUSION_NORMAL = 0,
    APP_BLE_FUSION_ATTENTION = 1,
    APP_BLE_FUSION_WARNING = 2,
    APP_BLE_FUSION_HIGH_RISK = 3,
    APP_BLE_FUSION_NO_TARGET = 4,
    APP_BLE_FUSION_SENSOR_FAULT = 5
} app_ble_fusion_state_t;

typedef enum
{
    APP_BLE_ALERT_NONE = 0,
    APP_BLE_ALERT_INFO = 1,
    APP_BLE_ALERT_WARNING = 2,
    APP_BLE_ALERT_HIGH_RISK = 3
} app_ble_alert_level_t;

typedef enum
{
    APP_BLE_EVENT_INFO = 0,
    APP_BLE_EVENT_COUGH = 1,
    APP_BLE_EVENT_SNORE = 2,
    APP_BLE_EVENT_MOTION = 3,
    APP_BLE_EVENT_NO_TARGET = 4,
    APP_BLE_EVENT_WARNING = 5,
    APP_BLE_EVENT_HIGH_RISK = 6,
    APP_BLE_EVENT_SENSOR_FAULT = 7
} app_ble_event_type_t;

#define APP_BLE_QUALITY_AUDIO_VALID        (1u << 0)
#define APP_BLE_QUALITY_RADAR_VALID        (1u << 1)
#define APP_BLE_QUALITY_FUSION_VALID       (1u << 2)
#define APP_BLE_QUALITY_HR_VALID           (1u << 3)
#define APP_BLE_QUALITY_RR_VALID           (1u << 4)
#define APP_BLE_QUALITY_MOTION_HIGH        (1u << 5)
#define APP_BLE_QUALITY_MODEL_READY        (1u << 6)
#define APP_BLE_QUALITY_TIME_SYNCED        (1u << 7)

#define APP_BLE_EVENT_SOURCE_AUDIO         (1u << 0)
#define APP_BLE_EVENT_SOURCE_RADAR         (1u << 1)
#define APP_BLE_EVENT_SOURCE_FUSION        (1u << 2)

typedef struct
{
    char device_name[32];
    uint16_t adv_interval_ms;
    uint16_t realtime_period_ms;
    uint8_t enable_pairing;
    uint8_t enable_diagnostics;
} app_ble_config_t;

typedef struct
{
    uint32_t ts_s;
    uint8_t rr_bpm;
    uint8_t hr_bpm;
    uint8_t presence;
    uint8_t motion;
    uint8_t cough_prob;
    uint8_t snore_prob;
    uint8_t fusion_state;
    uint8_t alert_level;
    uint8_t quality_flags;
} app_ble_realtime_sample_t;

typedef struct
{
    uint32_t event_id;
    uint32_t ts_s;
    uint8_t event_type;
    uint8_t severity;
    uint8_t confidence;
    uint8_t duration_s;
    uint8_t source_flags;
} app_ble_event_t;

typedef struct
{
    uint8_t seq;
    uint8_t command_id;
    uint32_t ts_s;
    uint8_t payload_len;
    uint8_t payload[APP_BLE_CMD_MAX_PAYLOAD_LEN];
} app_ble_command_t;

typedef struct
{
    uint8_t seq;
    uint8_t command_id;
    uint32_t ts_s;
    app_ble_error_t status;
    uint8_t payload_len;
    uint8_t payload[APP_BLE_CMD_RESPONSE_MAX_PAYLOAD_LEN];
} app_ble_cmd_response_t;

typedef struct
{
    uint32_t uptime_s;
    uint32_t notify_ok;
    uint32_t notify_fail;
    uint32_t realtime_drop;
    uint32_t event_drop;
    uint32_t realtime_no_sub;
    uint32_t realtime_busy_drop;
    uint32_t realtime_oversize_drop;
    uint32_t event_no_sub;
    uint32_t event_busy_drop;
    uint32_t event_oversize_drop;
    uint32_t transport_err;
    uint32_t cmd_rx;
    uint32_t cmd_ok;
    uint32_t cmd_unsupported;
    uint32_t cmd_not_ready;
    uint32_t cmd_denied;
    uint32_t cmd_crc_err;
    uint32_t cmd_rsp_notify_ok;
    uint32_t cmd_rsp_notify_fail;
    uint32_t cmd_rsp_no_sub;
    uint32_t cmd_rsp_busy_drop;
    uint32_t cmd_rsp_oversize_drop;
    uint32_t cmd_count;
    uint32_t cmd_drop;
    uint32_t disconnect_count;
    uint16_t mtu;
    uint8_t connected;
    uint8_t last_error;
} app_ble_diag_t;

#if defined(__cplusplus)
}
#endif

#endif /* __APP_BLE_TYPES_H__ */

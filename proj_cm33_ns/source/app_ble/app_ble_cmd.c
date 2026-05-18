#include "app_ble_cmd.h"

#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_ble_config.h"
#include "app_ble_diag.h"

#define APP_BLE_ALERT_THRESHOLD_MAX             (100u)
#define APP_BLE_ALERT_THRESHOLD_DEFAULT_COUGH   (75u)
#define APP_BLE_ALERT_THRESHOLD_DEFAULT_WARNING (2u)
#define APP_BLE_ALERT_THRESHOLD_DEFAULT_HIGH    (4u)

#define APP_BLE_DEVICE_INFO_PAYLOAD_LEN_V2      (7u)
#define APP_BLE_NOTIFY_PAYLOAD_MAX_DEFAULT      (20u)

#define APP_BLE_CAPABILITY_FLAG_ENABLE          (1u << 0)
#define APP_BLE_CAPABILITY_FLAG_FAKE_DATA       (1u << 1)
#define APP_BLE_CAPABILITY_FLAG_STACK           (1u << 2)
#define APP_BLE_CAPABILITY_FLAG_DIAG            (1u << 3)

#define APP_BLE_RUNTIME_FLAG_MONITOR_REQUESTED  (1u << 0)
#define APP_BLE_RUNTIME_FLAG_TIME_SYNCED        (1u << 1)

#if ((APP_BLE_CMD_RESPONSE_FRAME_MIN_LEN + APP_BLE_DEVICE_INFO_PAYLOAD_LEN_V2) > \
     APP_BLE_NOTIFY_PAYLOAD_MAX_DEFAULT)
#error "GET_DEVICE_INFO response exceeds default BLE notify payload"
#endif

static bool ble_time_synced;
static uint32_t ble_time_offset_s;
static bool monitor_requested;
static uint32_t last_start_ts_s;
static uint32_t last_stop_ts_s;
static app_ble_alert_threshold_config_t alert_threshold_config = {
    .cough_prob_threshold = APP_BLE_ALERT_THRESHOLD_DEFAULT_COUGH,
    .warning_event_threshold = APP_BLE_ALERT_THRESHOLD_DEFAULT_WARNING,
    .high_risk_event_threshold = APP_BLE_ALERT_THRESHOLD_DEFAULT_HIGH
};

static uint32_t app_ble_cmd_now_s(void);
static uint32_t app_ble_cmd_read_u32_le(const uint8_t *data);
static bool app_ble_cmd_validate_time_sync(uint32_t epoch_s);
static bool app_ble_cmd_validate_alert_threshold(
    const app_ble_alert_threshold_config_t *config);
static uint8_t app_ble_cmd_get_capability_flags(void);
static uint8_t app_ble_cmd_get_runtime_flags(void);
static void app_ble_cmd_fill_device_info(app_ble_cmd_response_t *resp);

cy_rslt_t app_ble_cmd_handle(const app_ble_command_t *cmd,
                             app_ble_cmd_response_t *resp)
{
    uint32_t epoch_s;
    uint32_t now_s;

    if ((NULL == cmd) || (NULL == resp))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    memset(resp, 0, sizeof(*resp));
    resp->seq = cmd->seq;
    resp->command_id = cmd->command_id;
    resp->ts_s = app_ble_cmd_now_s();
    resp->status = APP_BLE_OK;

    app_ble_diag_note_cmd();

    switch ((app_ble_cmd_id_t)cmd->command_id)
    {
        case APP_BLE_CMD_PING:
            resp->status = APP_BLE_OK;
            break;

        case APP_BLE_CMD_TIME_SYNC:
            if (4u != cmd->payload_len)
            {
                resp->status = APP_BLE_ERR_INVALID_ARG;
                break;
            }

            epoch_s = app_ble_cmd_read_u32_le(cmd->payload);
            if (!app_ble_cmd_validate_time_sync(epoch_s))
            {
                resp->status = APP_BLE_ERR_INVALID_ARG;
                break;
            }

            now_s = app_ble_cmd_now_s();
            ble_time_offset_s = epoch_s - now_s;
            ble_time_synced = true;
            resp->status = APP_BLE_OK;
            break;

        case APP_BLE_CMD_GET_DEVICE_INFO:
            app_ble_cmd_fill_device_info(resp);
            break;

        case APP_BLE_CMD_START_MONITOR:
            monitor_requested = true;
            last_start_ts_s = resp->ts_s;
            resp->status = APP_BLE_OK;
            break;

        case APP_BLE_CMD_STOP_MONITOR:
            monitor_requested = false;
            last_stop_ts_s = resp->ts_s;
            resp->status = APP_BLE_OK;
            break;

        case APP_BLE_CMD_SET_ALERT_THRESHOLD:
        {
            app_ble_alert_threshold_config_t next_config;

            if (3u != cmd->payload_len)
            {
                resp->status = APP_BLE_ERR_INVALID_ARG;
                break;
            }

            next_config.cough_prob_threshold = cmd->payload[0];
            next_config.warning_event_threshold = cmd->payload[1];
            next_config.high_risk_event_threshold = cmd->payload[2];

            if (!app_ble_cmd_validate_alert_threshold(&next_config))
            {
                resp->status = APP_BLE_ERR_INVALID_ARG;
                break;
            }

            alert_threshold_config = next_config;
            resp->status = APP_BLE_OK;
            break;
        }

        case APP_BLE_CMD_GET_NIGHT_SUMMARY:
        case APP_BLE_CMD_SET_LOG_LEVEL:
            resp->status = APP_BLE_ERR_CMD_NOT_READY;
            break;

        case APP_BLE_CMD_CLEAR_NIGHT_SUMMARY:
            resp->status = APP_BLE_ERR_CMD_DENIED;
            break;

        default:
            resp->status = APP_BLE_ERR_CMD_UNSUPPORTED;
            break;
    }

    if (APP_BLE_OK != resp->status)
    {
        app_ble_diag_set_last_error(resp->status);
    }

    return CY_RSLT_SUCCESS;
}

bool app_ble_cmd_time_is_synced(void)
{
    return ble_time_synced;
}

uint32_t app_ble_cmd_get_time_offset_s(void)
{
    return ble_time_offset_s;
}

bool app_ble_cmd_monitor_is_requested(void)
{
    return monitor_requested;
}

uint32_t app_ble_cmd_get_last_start_ts_s(void)
{
    return last_start_ts_s;
}

uint32_t app_ble_cmd_get_last_stop_ts_s(void)
{
    return last_stop_ts_s;
}

void app_ble_cmd_get_alert_threshold_config(
    app_ble_alert_threshold_config_t *config)
{
    if (NULL == config)
    {
        return;
    }

    *config = alert_threshold_config;
}

static uint32_t app_ble_cmd_now_s(void)
{
    return (uint32_t)((xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000u);
}

static uint32_t app_ble_cmd_read_u32_le(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8u) |
           ((uint32_t)data[2] << 16u) |
           ((uint32_t)data[3] << 24u);
}

static bool app_ble_cmd_validate_time_sync(uint32_t epoch_s)
{
    return (APP_BLE_TIME_SYNC_MIN_EPOCH_S <= epoch_s) &&
           (APP_BLE_TIME_SYNC_MAX_EPOCH_S >= epoch_s);
}

static bool app_ble_cmd_validate_alert_threshold(
    const app_ble_alert_threshold_config_t *config)
{
    if (NULL == config)
    {
        return false;
    }

    return (APP_BLE_ALERT_THRESHOLD_MAX >= config->cough_prob_threshold) &&
           (APP_BLE_ALERT_THRESHOLD_MAX >= config->warning_event_threshold) &&
           (APP_BLE_ALERT_THRESHOLD_MAX >= config->high_risk_event_threshold);
}

static uint8_t app_ble_cmd_get_capability_flags(void)
{
    uint8_t capability = 0u;

#if APP_BLE_ENABLE
    capability |= APP_BLE_CAPABILITY_FLAG_ENABLE;
#endif

#if APP_BLE_FAKE_DATA_ENABLE
    capability |= APP_BLE_CAPABILITY_FLAG_FAKE_DATA;
#endif

#if APP_BLE_STACK_ENABLE
    capability |= APP_BLE_CAPABILITY_FLAG_STACK;
#endif

#if APP_BLE_DIAG_ENABLE
    capability |= APP_BLE_CAPABILITY_FLAG_DIAG;
#endif

    return capability;
}

static uint8_t app_ble_cmd_get_runtime_flags(void)
{
    uint8_t runtime = 0u;

    if (monitor_requested)
    {
        runtime |= APP_BLE_RUNTIME_FLAG_MONITOR_REQUESTED;
    }

    if (ble_time_synced)
    {
        runtime |= APP_BLE_RUNTIME_FLAG_TIME_SYNCED;
    }

    return runtime;
}

static void app_ble_cmd_fill_device_info(app_ble_cmd_response_t *resp)
{
    if (NULL == resp)
    {
        return;
    }

    resp->payload[0] = APP_BLE_PROTOCOL_VERSION;
    resp->payload[1] = app_ble_cmd_get_capability_flags();
    resp->payload[2] = app_ble_cmd_get_runtime_flags();
    resp->payload[3] = alert_threshold_config.cough_prob_threshold;
    resp->payload[4] = alert_threshold_config.warning_event_threshold;
    resp->payload[5] = alert_threshold_config.high_risk_event_threshold;
    resp->payload[6] = 0u;
    resp->payload_len = APP_BLE_DEVICE_INFO_PAYLOAD_LEN_V2;
    resp->status = APP_BLE_OK;
}

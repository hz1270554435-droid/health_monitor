#include "app_ble_cmd.h"

#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_ble_config.h"
#include "app_ble_diag.h"

static bool ble_time_synced;
static uint32_t ble_time_offset_s;

static uint32_t app_ble_cmd_now_s(void);
static uint32_t app_ble_cmd_read_u32_le(const uint8_t *data);
static bool app_ble_cmd_validate_time_sync(uint32_t epoch_s);
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
        case APP_BLE_CMD_STOP_MONITOR:
        case APP_BLE_CMD_SET_ALERT_THRESHOLD:
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

static void app_ble_cmd_fill_device_info(app_ble_cmd_response_t *resp)
{
    if (NULL == resp)
    {
        return;
    }

    resp->payload[0] = APP_BLE_PROTOCOL_VERSION;
    resp->payload[1] = APP_BLE_ENABLE;
    resp->payload[2] = APP_BLE_FAKE_DATA_ENABLE;
    resp->payload[3] = APP_BLE_STACK_ENABLE;
    resp->payload[4] = APP_BLE_DIAG_ENABLE;
    resp->payload_len = 5u;
    resp->status = APP_BLE_OK;
}

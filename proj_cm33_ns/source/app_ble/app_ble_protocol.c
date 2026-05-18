#include "app_ble_protocol.h"

#include <stdbool.h>
#include <string.h>

static uint8_t realtime_seq;
static uint8_t event_seq;

static void app_ble_write_u32_le(uint8_t *out, uint32_t value);
static uint32_t app_ble_read_u32_le(const uint8_t *data);
static void app_ble_write_header(uint8_t *out,
                                 uint8_t msg_type,
                                 uint8_t seq,
                                 uint32_t ts_s,
                                 app_ble_error_t error);
static bool app_ble_value_percent_or_invalid(uint8_t value);
static bool app_ble_realtime_is_valid(
    const app_ble_realtime_sample_t *sample);
static bool app_ble_event_is_valid(const app_ble_event_t *event);
static bool app_ble_command_id_is_known(uint8_t command_id);

cy_rslt_t app_ble_pack_realtime(const app_ble_realtime_sample_t *sample,
                                uint8_t *out,
                                uint16_t out_size,
                                uint16_t *out_len)
{
    uint16_t pos = 0u;

    if ((NULL == sample) || (NULL == out) || (NULL == out_len) ||
        (APP_BLE_REALTIME_FRAME_LEN > out_size) ||
        !app_ble_realtime_is_valid(sample))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    app_ble_write_header(out,
                         APP_BLE_MSG_REALTIME,
                         realtime_seq++,
                         sample->ts_s,
                         APP_BLE_OK);
    pos = 9u;
    out[pos++] = sample->rr_bpm;
    out[pos++] = sample->hr_bpm;
    out[pos++] = sample->presence;
    out[pos++] = sample->motion;
    out[pos++] = sample->cough_prob;
    out[pos++] = sample->snore_prob;
    out[pos++] = sample->fusion_state;
    out[pos++] = sample->alert_level;
    out[pos++] = sample->quality_flags;
    out[pos] = app_ble_calc_crc8(out, pos);
    pos++;

    *out_len = pos;
    return (APP_BLE_REALTIME_FRAME_LEN == pos) ?
           CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
}

cy_rslt_t app_ble_pack_event(const app_ble_event_t *event,
                             uint8_t *out,
                             uint16_t out_size,
                             uint16_t *out_len)
{
    uint16_t pos = 0u;

    if ((NULL == event) || (NULL == out) || (NULL == out_len) ||
        (APP_BLE_EVENT_FRAME_LEN > out_size) ||
        !app_ble_event_is_valid(event))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    app_ble_write_header(out,
                         APP_BLE_MSG_EVENT,
                         event_seq++,
                         event->ts_s,
                         APP_BLE_OK);
    pos = 9u;
    app_ble_write_u32_le(&out[pos], event->event_id);
    pos += 4u;
    out[pos++] = event->event_type;
    out[pos++] = event->severity;
    out[pos++] = event->confidence;
    out[pos++] = event->duration_s;
    out[pos++] = event->source_flags;
    out[pos] = app_ble_calc_crc8(out, pos);
    pos++;

    *out_len = pos;
    return (APP_BLE_EVENT_FRAME_LEN == pos) ?
           CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
}

cy_rslt_t app_ble_unpack_command(const uint8_t *data,
                                 uint16_t len,
                                 app_ble_command_t *cmd)
{
    return (APP_BLE_OK == app_ble_unpack_command_status(data, len, cmd)) ?
           CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
}

app_ble_error_t app_ble_unpack_command_status(const uint8_t *data,
                                              uint16_t len,
                                              app_ble_command_t *cmd)
{
    uint8_t payload_len;

    if ((NULL == data) || (NULL == cmd) ||
        (APP_BLE_COMMAND_FRAME_MIN_LEN > len) ||
        (APP_BLE_COMMAND_FRAME_MAX_LEN < len))
    {
        return APP_BLE_ERR_INVALID_ARG;
    }

    if ((APP_BLE_PROTOCOL_MAGIC != data[0]) ||
        (APP_BLE_PROTOCOL_VERSION != data[1]) ||
        (APP_BLE_MSG_COMMAND != data[2]))
    {
        return APP_BLE_ERR_INVALID_ARG;
    }

    payload_len = data[10];
    if ((APP_BLE_CMD_MAX_PAYLOAD_LEN < payload_len) ||
        (len != (uint16_t)(APP_BLE_COMMAND_FRAME_MIN_LEN + payload_len)))
    {
        return APP_BLE_ERR_INVALID_ARG;
    }

    if (data[len - 1u] != app_ble_calc_crc8(data, (uint16_t)(len - 1u)))
    {
        return APP_BLE_ERR_CRC_FAILED;
    }

    if (!app_ble_command_id_is_known(data[9]))
    {
        return APP_BLE_ERR_CMD_UNSUPPORTED;
    }

    memset(cmd, 0, sizeof(*cmd));
    cmd->seq = data[3];
    cmd->ts_s = app_ble_read_u32_le(&data[4]);
    cmd->command_id = data[9];
    cmd->payload_len = payload_len;
    if (0u < payload_len)
    {
        memcpy(cmd->payload, &data[11], payload_len);
    }

    return APP_BLE_OK;
}

cy_rslt_t app_ble_pack_cmd_response(const app_ble_cmd_response_t *resp,
                                    uint8_t *out,
                                    uint16_t out_size,
                                    uint16_t *out_len)
{
    uint16_t pos;

    if ((NULL == resp) || (NULL == out) || (NULL == out_len) ||
        (APP_BLE_CMD_RESPONSE_MAX_PAYLOAD_LEN < resp->payload_len) ||
        (out_size <
         (uint16_t)(APP_BLE_CMD_RESPONSE_FRAME_MIN_LEN + resp->payload_len)))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    app_ble_write_header(out,
                         APP_BLE_MSG_COMMAND_RESPONSE,
                         resp->seq,
                         resp->ts_s,
                         resp->status);
    pos = 9u;
    out[pos++] = resp->command_id;
    out[pos++] = (uint8_t)resp->status;
    out[pos++] = resp->payload_len;
    if (0u < resp->payload_len)
    {
        memcpy(&out[pos], resp->payload, resp->payload_len);
        pos = (uint16_t)(pos + resp->payload_len);
    }
    out[pos] = app_ble_calc_crc8(out, pos);
    pos++;

    *out_len = pos;
    return CY_RSLT_SUCCESS;
}

uint8_t app_ble_calc_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0u;

    if (NULL == data)
    {
        return 0u;
    }

    for (uint16_t i = 0u; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; bit++)
        {
            if (0u != (crc & 0x80u))
            {
                crc = (uint8_t)((crc << 1u) ^ 0x07u);
            }
            else
            {
                crc = (uint8_t)(crc << 1u);
            }
        }
    }

    return crc;
}

void app_ble_protocol_reset_sequences(void)
{
    realtime_seq = 0u;
    event_seq = 0u;
}

const char *app_ble_error_name(app_ble_error_t error)
{
    switch (error)
    {
        case APP_BLE_OK:
            return "OK";
        case APP_BLE_ERR_NOT_INIT:
            return "NOT_INIT";
        case APP_BLE_ERR_NOT_CONNECTED:
            return "NOT_CONNECTED";
        case APP_BLE_ERR_NOT_SUBSCRIBED:
            return "NOT_SUBSCRIBED";
        case APP_BLE_ERR_QUEUE_FULL:
            return "QUEUE_FULL";
        case APP_BLE_ERR_INVALID_ARG:
            return "INVALID_ARG";
        case APP_BLE_ERR_PACK_FAILED:
            return "PACK_FAILED";
        case APP_BLE_ERR_CRC_FAILED:
            return "CRC_FAILED";
        case APP_BLE_ERR_CMD_UNSUPPORTED:
            return "CMD_UNSUPPORTED";
        case APP_BLE_ERR_CMD_DENIED:
            return "CMD_DENIED";
        case APP_BLE_ERR_STACK_FAILED:
            return "STACK_FAILED";
        case APP_BLE_ERR_CMD_NOT_READY:
            return "CMD_NOT_READY";
        case APP_BLE_ERR_BUSY:
            return "BUSY";
        default:
            return "UNKNOWN";
    }
}

const char *app_ble_cmd_name(uint8_t cmd_id)
{
    switch ((app_ble_cmd_id_t)cmd_id)
    {
        case APP_BLE_CMD_PING:
            return "PING";
        case APP_BLE_CMD_TIME_SYNC:
            return "TIME_SYNC";
        case APP_BLE_CMD_START_MONITOR:
            return "START_MONITOR";
        case APP_BLE_CMD_STOP_MONITOR:
            return "STOP_MONITOR";
        case APP_BLE_CMD_SET_ALERT_THRESHOLD:
            return "SET_ALERT_THRESHOLD";
        case APP_BLE_CMD_GET_DEVICE_INFO:
            return "GET_DEVICE_INFO";
        case APP_BLE_CMD_GET_NIGHT_SUMMARY:
            return "GET_NIGHT_SUMMARY";
        case APP_BLE_CMD_CLEAR_NIGHT_SUMMARY:
            return "CLEAR_NIGHT_SUMMARY";
        case APP_BLE_CMD_SET_LOG_LEVEL:
            return "SET_LOG_LEVEL";
        default:
            return "UNKNOWN_CMD";
    }
}

const char *app_ble_fusion_state_name(uint8_t state)
{
    switch ((app_ble_fusion_state_t)state)
    {
        case APP_BLE_FUSION_NORMAL:
            return "normal";
        case APP_BLE_FUSION_ATTENTION:
            return "attention";
        case APP_BLE_FUSION_WARNING:
            return "warning";
        case APP_BLE_FUSION_HIGH_RISK:
            return "high_risk";
        case APP_BLE_FUSION_NO_TARGET:
            return "no_target";
        case APP_BLE_FUSION_SENSOR_FAULT:
            return "sensor_fault";
        default:
            return "unknown";
    }
}

const char *app_ble_event_type_name(uint8_t event_type)
{
    switch ((app_ble_event_type_t)event_type)
    {
        case APP_BLE_EVENT_INFO:
            return "info";
        case APP_BLE_EVENT_COUGH:
            return "cough";
        case APP_BLE_EVENT_SNORE:
            return "snore";
        case APP_BLE_EVENT_MOTION:
            return "motion";
        case APP_BLE_EVENT_NO_TARGET:
            return "no_target";
        case APP_BLE_EVENT_WARNING:
            return "warning";
        case APP_BLE_EVENT_HIGH_RISK:
            return "high_risk";
        case APP_BLE_EVENT_SENSOR_FAULT:
            return "sensor_fault";
        default:
            return "unknown";
    }
}

static void app_ble_write_u32_le(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value & 0xFFu);
    out[1] = (uint8_t)((value >> 8u) & 0xFFu);
    out[2] = (uint8_t)((value >> 16u) & 0xFFu);
    out[3] = (uint8_t)((value >> 24u) & 0xFFu);
}

static uint32_t app_ble_read_u32_le(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8u) |
           ((uint32_t)data[2] << 16u) |
           ((uint32_t)data[3] << 24u);
}

static void app_ble_write_header(uint8_t *out,
                                 uint8_t msg_type,
                                 uint8_t seq,
                                 uint32_t ts_s,
                                 app_ble_error_t error)
{
    out[0] = APP_BLE_PROTOCOL_MAGIC;
    out[1] = APP_BLE_PROTOCOL_VERSION;
    out[2] = msg_type;
    out[3] = seq;
    app_ble_write_u32_le(&out[4], ts_s);
    out[8] = (uint8_t)error;
}

static bool app_ble_value_percent_or_invalid(uint8_t value)
{
    return (100u >= value) || (APP_BLE_INVALID_U8 == value);
}

static bool app_ble_realtime_is_valid(
    const app_ble_realtime_sample_t *sample)
{
    if (NULL == sample)
    {
        return false;
    }

    if (!((80u >= sample->rr_bpm) ||
          (APP_BLE_INVALID_U8 == sample->rr_bpm)))
    {
        return false;
    }

    if (!((220u >= sample->hr_bpm) ||
          (APP_BLE_INVALID_U8 == sample->hr_bpm)))
    {
        return false;
    }

    return app_ble_value_percent_or_invalid(sample->presence) &&
           app_ble_value_percent_or_invalid(sample->motion) &&
           app_ble_value_percent_or_invalid(sample->cough_prob) &&
           app_ble_value_percent_or_invalid(sample->snore_prob) &&
           (APP_BLE_FUSION_SENSOR_FAULT >= sample->fusion_state) &&
           (APP_BLE_ALERT_HIGH_RISK >= sample->alert_level);
}

static bool app_ble_event_is_valid(const app_ble_event_t *event)
{
    if (NULL == event)
    {
        return false;
    }

    return (APP_BLE_EVENT_SENSOR_FAULT >= event->event_type) &&
           (3u >= event->severity) &&
           (100u >= event->confidence);
}

static bool app_ble_command_id_is_known(uint8_t command_id)
{
    return ((APP_BLE_CMD_PING <= command_id) &&
            (APP_BLE_CMD_SET_LOG_LEVEL >= command_id));
}

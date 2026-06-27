#include "app_ble_stream.h"

#include "app_build_config.h"
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "app_ble_cmd.h"
#include "app_ble_config.h"
#if (APP_BLE_STACK_ENABLE)
#include "app_ble_core.h"
#include "app_ble_gatt.h"
#endif
#include "app_ble_diag.h"
#include "app_ble_protocol.h"

#if (APP_BLE_ENABLE)

#if (APP_BLE_SUMMARY_ENABLE && !APP_MONITOR_SUMMARY_ENABLE)
#error "APP_BLE_SUMMARY_ENABLE requires APP_MONITOR_SUMMARY_ENABLE"
#endif

#if (APP_BLE_SUMMARY_ENABLE)
#include "app_monitor_summary.h"
#endif

static QueueHandle_t ble_realtime_queue;
static QueueHandle_t ble_event_queue;
static TaskHandle_t ble_task_handle;

#if (APP_BLE_STACK_ENABLE)
typedef struct
{
    uint16_t len;
    uint8_t data[APP_BLE_COMMAND_FRAME_MAX_LEN];
} app_ble_raw_command_item_t;

static QueueHandle_t ble_cmd_raw_queue;
#endif

#if (APP_BLE_FAKE_DATA_ENABLE && !APP_BLE_SUMMARY_ENABLE)
static uint32_t fake_last_realtime_ms;
static uint32_t fake_last_event_ms;
static uint32_t fake_event_id;
#endif

#if (APP_BLE_FAKE_DATA_ENABLE && !APP_BLE_STACK_ENABLE)
static bool fake_commands_done;
#endif

#if (APP_BLE_SUMMARY_ENABLE)
static uint32_t summary_last_realtime_ms;
static uint32_t summary_last_event_id;
#endif
static uint32_t stat_last_print_ms;

static void app_ble_stream_task(void *pvParameters);
static uint32_t app_ble_stream_now_ms(void);
static uint32_t app_ble_stream_now_s(void);
static void app_ble_stream_process_raw_commands(void);
#if (APP_BLE_STACK_ENABLE)
static bool app_ble_stream_make_error_response_from_raw(
    const app_ble_raw_command_item_t *item,
    app_ble_error_t status,
    app_ble_cmd_response_t *resp);
static void app_ble_stream_send_cmd_response(
    const app_ble_cmd_response_t *resp);
#endif
#if (!APP_BLE_SUMMARY_ENABLE)
static void app_ble_stream_maybe_publish_fake(uint32_t now_ms);
#endif
#if (APP_BLE_SUMMARY_ENABLE)
static void app_ble_stream_maybe_publish_summary(uint32_t now_ms);
#endif
static void app_ble_stream_run_fake_commands_once(void);
#if (APP_BLE_FAKE_DATA_ENABLE && !APP_BLE_STACK_ENABLE)
static void app_ble_stream_run_fake_command(const app_ble_command_t *cmd);
static void app_ble_stream_make_time_sync_cmd(app_ble_command_t *cmd);
#endif
static void app_ble_stream_print_stat(uint32_t now_ms);
#if (APP_BLE_SUMMARY_ENABLE)
static void app_ble_stream_fill_realtime_from_summary(
    const app_monitor_summary_snapshot_t *summary,
    app_ble_realtime_sample_t *sample);
static void app_ble_stream_fill_event_from_summary(
    const app_monitor_summary_event_t *summary_event,
    app_ble_event_t *ble_event);
static uint8_t app_ble_stream_summary_fusion_state(
    const app_monitor_summary_snapshot_t *summary);
static uint8_t app_ble_stream_summary_alert_level(
    app_monitor_alert_level_t alert_level);
static uint8_t app_ble_stream_summary_event_type(
    const app_monitor_summary_event_t *event);
static uint8_t app_ble_stream_summary_quality_flags(
    const app_monitor_summary_snapshot_t *summary);
static uint8_t app_ble_stream_summary_presence(
    const app_monitor_summary_snapshot_t *summary);
static uint8_t app_ble_stream_summary_bpm_u8(uint16_t bpm_x10,
                                             bool valid);
static uint8_t app_ble_stream_summary_source_flags(uint32_t source_flags);
#endif

cy_rslt_t app_ble_stream_init(void)
{
    BaseType_t ret;

    if (NULL != ble_task_handle)
    {
        return CY_RSLT_SUCCESS;
    }

    app_ble_diag_reset();
    app_ble_protocol_reset_sequences();

#if (APP_BLE_STACK_ENABLE)
    if (CY_RSLT_SUCCESS != app_ble_core_init())
    {
        app_ble_diag_set_last_error(APP_BLE_ERR_STACK_FAILED);
        return CY_RSLT_TYPE_ERROR;
    }
#endif

    ble_realtime_queue = xQueueCreate(APP_BLE_REALTIME_QUEUE_DEPTH,
                                      sizeof(app_ble_realtime_sample_t));
    ble_event_queue = xQueueCreate(APP_BLE_EVENT_QUEUE_DEPTH,
                                   sizeof(app_ble_event_t));
#if (APP_BLE_STACK_ENABLE)
    ble_cmd_raw_queue = xQueueCreate(APP_BLE_CMD_RAW_QUEUE_DEPTH,
                                     sizeof(app_ble_raw_command_item_t));
#endif
    if ((NULL == ble_realtime_queue) || (NULL == ble_event_queue))
    {
        app_ble_diag_set_last_error(APP_BLE_ERR_QUEUE_FULL);
        return CY_RSLT_TYPE_ERROR;
    }
#if (APP_BLE_STACK_ENABLE)
    if (NULL == ble_cmd_raw_queue)
    {
        app_ble_diag_set_last_error(APP_BLE_ERR_QUEUE_FULL);
        return CY_RSLT_TYPE_ERROR;
    }
#endif

    ret = xTaskCreate(app_ble_stream_task,
                      "ble_stage1",
                      APP_BLE_TASK_STACK_SIZE,
                      NULL,
                      APP_BLE_TASK_PRIORITY,
                      &ble_task_handle);

    return (pdPASS == ret) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
}

cy_rslt_t app_ble_publish_realtime(
    const app_ble_realtime_sample_t *sample)
{
    app_ble_realtime_sample_t dropped;

    if ((NULL == sample) || (NULL == ble_realtime_queue))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    if (pdPASS == xQueueSend(ble_realtime_queue, sample, 0u))
    {
        return CY_RSLT_SUCCESS;
    }

    if (pdPASS == xQueueReceive(ble_realtime_queue, &dropped, 0u))
    {
        app_ble_diag_note_realtime_drop();
    }

    if (pdPASS == xQueueSend(ble_realtime_queue, sample, 0u))
    {
        return CY_RSLT_SUCCESS;
    }

    app_ble_diag_note_realtime_drop();
    return CY_RSLT_TYPE_ERROR;
}

cy_rslt_t app_ble_publish_event(const app_ble_event_t *event)
{
    if ((NULL == event) || (NULL == ble_event_queue))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    if (pdPASS != xQueueSend(ble_event_queue, event, 0u))
    {
        app_ble_diag_note_event_drop();
        return CY_RSLT_TYPE_ERROR;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_ble_stream_enqueue_raw_command(const uint8_t *data,
                                             uint16_t len)
{
#if (APP_BLE_STACK_ENABLE)
    app_ble_raw_command_item_t item;

    if ((NULL == data) || (0u == len) ||
        (APP_BLE_COMMAND_FRAME_MAX_LEN < len))
    {
        app_ble_diag_note_cmd_drop(APP_BLE_ERR_INVALID_ARG);
        return (cy_rslt_t)APP_BLE_ERR_INVALID_ARG;
    }

    if (NULL == ble_cmd_raw_queue)
    {
        app_ble_diag_note_cmd_drop(APP_BLE_ERR_NOT_INIT);
        return (cy_rslt_t)APP_BLE_ERR_NOT_INIT;
    }

    memset(&item, 0, sizeof(item));
    item.len = len;
    memcpy(item.data, data, len);

    if (pdPASS != xQueueSend(ble_cmd_raw_queue, &item, 0u))
    {
        app_ble_diag_note_cmd_drop(APP_BLE_ERR_QUEUE_FULL);
        return (cy_rslt_t)APP_BLE_ERR_QUEUE_FULL;
    }

    return CY_RSLT_SUCCESS;
#else
    (void)data;
    (void)len;
    return (cy_rslt_t)APP_BLE_ERR_CMD_NOT_READY;
#endif
}

void app_ble_stream_process(void)
{
    app_ble_realtime_sample_t sample;
    app_ble_event_t event;
    uint8_t frame[APP_BLE_CMD_RESPONSE_FRAME_MAX_LEN];
    uint16_t frame_len = 0u;

    app_ble_stream_process_raw_commands();

    while ((NULL != ble_event_queue) &&
           (pdPASS == xQueueReceive(ble_event_queue, &event, 0u)))
    {
#if (APP_BLE_STACK_ENABLE)
        if (CY_RSLT_SUCCESS == app_ble_pack_event(&event,
                                                  frame,
                                                  sizeof(frame),
                                                  &frame_len))
        {
            if (CY_RSLT_SUCCESS == app_ble_gatt_notify_event(frame,
                                                             frame_len))
            {
                app_ble_diag_note_notify_ok();
            }
        }
        else
        {
            app_ble_diag_note_notify_fail(APP_BLE_ERR_PACK_FAILED);
        }
#else
        if (CY_RSLT_SUCCESS == app_ble_pack_event(&event,
                                                  frame,
                                                  sizeof(frame),
                                                  &frame_len))
        {
            app_ble_diag_note_notify_ok();
#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
            printf("[BLE_FAKE] event seq=%u id=%lu type=%s conf=%u "
                   "len=%u crc=0x%02x\r\n",
                   (unsigned int)frame[3],
                   (unsigned long)event.event_id,
                   app_ble_event_type_name(event.event_type),
                   (unsigned int)event.confidence,
                   (unsigned int)frame_len,
                   (unsigned int)frame[frame_len - 1u]);
            printf("[BLE_EVENT] type=%s severity=%u conf=%u ts=%lu\r\n",
                   app_ble_event_type_name(event.event_type),
                   (unsigned int)event.severity,
                   (unsigned int)event.confidence,
                   (unsigned long)event.ts_s);
#endif
        }
        else
        {
            app_ble_diag_note_notify_fail(APP_BLE_ERR_PACK_FAILED);
        }
#endif
    }

    while ((NULL != ble_realtime_queue) &&
           (pdPASS == xQueueReceive(ble_realtime_queue, &sample, 0u)))
    {
        if (CY_RSLT_SUCCESS == app_ble_pack_realtime(&sample,
                                                     frame,
                                                     sizeof(frame),
                                                     &frame_len))
        {
#if (APP_BLE_STACK_ENABLE)
            if (CY_RSLT_SUCCESS == app_ble_gatt_notify_realtime(frame,
                                                                frame_len))
            {
                app_ble_diag_note_notify_ok();
            }
#else
            app_ble_diag_note_notify_ok();
#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
            printf("[BLE_FAKE] realtime seq=%u rr=%u hr=%u state=%s "
                   "len=%u crc=0x%02x\r\n",
                   (unsigned int)frame[3],
                   (unsigned int)sample.rr_bpm,
                   (unsigned int)sample.hr_bpm,
                   app_ble_fusion_state_name(sample.fusion_state),
                   (unsigned int)frame_len,
                   (unsigned int)frame[frame_len - 1u]);
#endif
#endif
        }
        else
        {
            app_ble_diag_note_notify_fail(APP_BLE_ERR_PACK_FAILED);
        }
    }
}

static void app_ble_stream_process_raw_commands(void)
{
#if (APP_BLE_STACK_ENABLE)
    app_ble_raw_command_item_t item;
    app_ble_command_t cmd;
    app_ble_cmd_response_t resp;
    app_ble_error_t decode_status;

    while ((NULL != ble_cmd_raw_queue) &&
           (pdPASS == xQueueReceive(ble_cmd_raw_queue, &item, 0u)))
    {
        memset(&cmd, 0, sizeof(cmd));
        memset(&resp, 0, sizeof(resp));
        app_ble_diag_note_cmd_rx();

        decode_status = app_ble_unpack_command_status(item.data,
                                                      item.len,
                                                      &cmd);
        if (APP_BLE_OK == decode_status)
        {
            if (CY_RSLT_SUCCESS != app_ble_cmd_handle(&cmd, &resp))
            {
                resp.seq = cmd.seq;
                resp.command_id = cmd.command_id;
                resp.ts_s = app_ble_stream_now_s();
                resp.status = APP_BLE_ERR_INVALID_ARG;
            }
            app_ble_diag_note_cmd_result(resp.status);
        }
        else
        {
            if (APP_BLE_ERR_CRC_FAILED == decode_status)
            {
                app_ble_diag_note_cmd_crc_err();
            }
            else
            {
                app_ble_diag_note_cmd_result(decode_status);
            }

            if (!app_ble_stream_make_error_response_from_raw(&item,
                                                             decode_status,
                                                             &resp))
            {
#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
                printf("[BLE_CMD] id=UNKNOWN_CMD result=%s drop len=%u\r\n",
                       app_ble_error_name(decode_status),
                       (unsigned int)item.len);
#endif
                continue;
            }
        }

#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
        if (((APP_BLE_CMD_START_MONITOR == resp.command_id) ||
             (APP_BLE_CMD_STOP_MONITOR == resp.command_id)) &&
            (APP_BLE_OK == resp.status))
        {
            printf("[BLE_CMD] id=%s result=%s monitor=%u\r\n",
                   app_ble_cmd_name(resp.command_id),
                   app_ble_error_name(resp.status),
                   app_ble_cmd_monitor_is_requested() ? 1u : 0u);
        }
        else if ((APP_BLE_CMD_SET_ALERT_THRESHOLD == resp.command_id) &&
                 (APP_BLE_OK == resp.status))
        {
            app_ble_alert_threshold_config_t threshold;

            app_ble_cmd_get_alert_threshold_config(&threshold);
            printf("[BLE_CMD] id=%s result=%s cough=%u warn=%u high=%u\r\n",
                   app_ble_cmd_name(resp.command_id),
                   app_ble_error_name(resp.status),
                   (unsigned int)threshold.cough_prob_threshold,
                   (unsigned int)threshold.warning_event_threshold,
                   (unsigned int)threshold.high_risk_event_threshold);
        }
        else
        {
            printf("[BLE_CMD] id=%s result=%s\r\n",
                   app_ble_cmd_name(resp.command_id),
                   app_ble_error_name(resp.status));
        }
#endif
        app_ble_stream_send_cmd_response(&resp);
    }
#endif
}

#if (APP_BLE_STACK_ENABLE)
static bool app_ble_stream_make_error_response_from_raw(
    const app_ble_raw_command_item_t *item,
    app_ble_error_t status,
    app_ble_cmd_response_t *resp)
{
    if ((NULL == item) || (NULL == resp) ||
        (APP_BLE_COMMAND_FRAME_MIN_LEN > item->len))
    {
        return false;
    }

    if ((APP_BLE_PROTOCOL_MAGIC != item->data[0]) ||
        (APP_BLE_PROTOCOL_VERSION != item->data[1]) ||
        (APP_BLE_MSG_COMMAND != item->data[2]))
    {
        return false;
    }

    memset(resp, 0, sizeof(*resp));
    resp->seq = item->data[3];
    resp->command_id = item->data[9];
    resp->ts_s = app_ble_stream_now_s();
    resp->status = status;

    return true;
}

static void app_ble_stream_send_cmd_response(
    const app_ble_cmd_response_t *resp)
{
    uint8_t frame[APP_BLE_CMD_RESPONSE_FRAME_MAX_LEN];
    uint16_t frame_len = 0u;
    cy_rslt_t result;

    if (NULL == resp)
    {
        app_ble_diag_note_cmd_rsp_notify_fail(APP_BLE_ERR_INVALID_ARG);
        return;
    }

    if (CY_RSLT_SUCCESS != app_ble_pack_cmd_response(resp,
                                                     frame,
                                                     sizeof(frame),
                                                     &frame_len))
    {
        app_ble_diag_note_cmd_rsp_notify_fail(APP_BLE_ERR_PACK_FAILED);
#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
        printf("[BLE_CMD_RSP] pack failed result=%s\r\n",
               app_ble_error_name(APP_BLE_ERR_PACK_FAILED));
#endif
        return;
    }

    result = app_ble_gatt_notify_cmd_response(frame, frame_len);
    if (CY_RSLT_SUCCESS == result)
    {
        app_ble_diag_note_cmd_rsp_notify_ok();
#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
        printf("[BLE_CMD_RSP] notify ok len=%u\r\n",
               (unsigned int)frame_len);
#endif
        return;
    }

#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
    printf("[BLE_CMD_RSP] notify skipped result=%s len=%u\r\n",
           app_ble_error_name((app_ble_error_t)result),
           (unsigned int)frame_len);
#endif
}
#endif

static void app_ble_stream_task(void *pvParameters)
{
    (void)pvParameters;

#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
    printf("[BLE_STAT] stage=1 enable=%u fake=%u stack=%u device=%s\r\n",
           (unsigned int)APP_BLE_ENABLE,
           (unsigned int)APP_BLE_FAKE_DATA_ENABLE,
           (unsigned int)APP_BLE_STACK_ENABLE,
           APP_BLE_DEVICE_NAME);
#endif

    for (;;)
    {
        uint32_t now_ms = app_ble_stream_now_ms();

        app_ble_stream_run_fake_commands_once();
#if (APP_BLE_SUMMARY_ENABLE)
        app_ble_stream_maybe_publish_summary(now_ms);
#else
        app_ble_stream_maybe_publish_fake(now_ms);
#endif
        app_ble_stream_process();
        app_ble_stream_print_stat(now_ms);

        vTaskDelay(pdMS_TO_TICKS(100u));
    }
}

static uint32_t app_ble_stream_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint32_t app_ble_stream_now_s(void)
{
    return (uint32_t)(app_ble_stream_now_ms() / 1000u);
}

#if (!APP_BLE_SUMMARY_ENABLE)
static void app_ble_stream_maybe_publish_fake(uint32_t now_ms)
{
#if (APP_BLE_FAKE_DATA_ENABLE)
    bool realtime_due = ((0u == fake_last_realtime_ms) ||
        ((now_ms - fake_last_realtime_ms) >= APP_BLE_REALTIME_PERIOD_MS));
    bool event_due = ((0u == fake_last_event_ms) ||
        ((now_ms - fake_last_event_ms) >= APP_BLE_EVENT_FAKE_PERIOD_MS));

#if (APP_BLE_STACK_ENABLE)
    if (event_due)
    {
        app_ble_event_t event = {
            .event_id = ++fake_event_id,
            .ts_s = app_ble_stream_now_s(),
            .event_type = APP_BLE_EVENT_COUGH,
            .severity = 2u,
            .confidence = 87u,
            .duration_s = 1u,
            .source_flags = (uint8_t)(APP_BLE_EVENT_SOURCE_AUDIO |
                                      APP_BLE_EVENT_SOURCE_FUSION)
        };

        (void)app_ble_publish_event(&event);
        fake_last_event_ms = now_ms;
        if (realtime_due)
        {
            app_ble_diag_note_realtime_busy_drop();
            realtime_due = false;
        }
    }
#endif

    if (realtime_due)
    {
        uint32_t tick_s = app_ble_stream_now_s();
        app_ble_realtime_sample_t sample = {
            .ts_s = tick_s,
            .rr_bpm = (uint8_t)(14u + (tick_s % 5u)),
            .hr_bpm = (uint8_t)(70u + (tick_s % 8u)),
            .presence = 100u,
            .motion = (uint8_t)((tick_s * 7u) % 40u),
            .cough_prob = (uint8_t)((0u == (tick_s % 15u)) ? 87u : 12u),
            .snore_prob = APP_BLE_INVALID_U8,
            .fusion_state = (uint8_t)((0u == (tick_s % 15u)) ?
                            APP_BLE_FUSION_WARNING :
                            APP_BLE_FUSION_NORMAL),
            .alert_level = (uint8_t)((0u == (tick_s % 15u)) ?
                           APP_BLE_ALERT_WARNING :
                           APP_BLE_ALERT_NONE),
            .quality_flags = (uint8_t)(APP_BLE_QUALITY_AUDIO_VALID |
                                       APP_BLE_QUALITY_RADAR_VALID |
                                       APP_BLE_QUALITY_FUSION_VALID |
                                       APP_BLE_QUALITY_HR_VALID |
                                       APP_BLE_QUALITY_RR_VALID |
                                       APP_BLE_QUALITY_MODEL_READY)
        };

        (void)app_ble_publish_realtime(&sample);
        fake_last_realtime_ms = now_ms;
    }

#if (!APP_BLE_STACK_ENABLE)
    if (event_due)
    {
        app_ble_event_t event = {
            .event_id = ++fake_event_id,
            .ts_s = app_ble_stream_now_s(),
            .event_type = APP_BLE_EVENT_COUGH,
            .severity = 2u,
            .confidence = 87u,
            .duration_s = 1u,
            .source_flags = (uint8_t)(APP_BLE_EVENT_SOURCE_AUDIO |
                                      APP_BLE_EVENT_SOURCE_FUSION)
        };

        (void)app_ble_publish_event(&event);
        fake_last_event_ms = now_ms;
    }
#endif
#else
    (void)now_ms;
#endif
}
#endif

#if (APP_BLE_SUMMARY_ENABLE)
static void app_ble_stream_maybe_publish_summary(uint32_t now_ms)
{
    bool realtime_due =
        ((0u == summary_last_realtime_ms) ||
         ((now_ms - summary_last_realtime_ms) >= APP_BLE_REALTIME_PERIOD_MS));
    app_monitor_summary_snapshot_t summary;
    app_monitor_summary_event_t summary_event;

    memset(&summary, 0, sizeof(summary));
    if (CY_RSLT_SUCCESS == app_monitor_summary_get_snapshot(&summary))
    {
        if (realtime_due)
        {
            app_ble_realtime_sample_t sample;

            app_ble_stream_fill_realtime_from_summary(&summary, &sample);
            (void)app_ble_publish_realtime(&sample);
            summary_last_realtime_ms = now_ms;
        }
    }

    if (app_monitor_summary_get_latest_event(&summary_event) &&
        (summary_event.event_id != summary_last_event_id))
    {
        app_ble_event_t event;

        app_ble_stream_fill_event_from_summary(&summary_event, &event);
        (void)app_ble_publish_event(&event);
        summary_last_event_id = summary_event.event_id;
    }
}
#endif

static void app_ble_stream_run_fake_commands_once(void)
{
#if (APP_BLE_FAKE_DATA_ENABLE && !APP_BLE_STACK_ENABLE)
    app_ble_command_t cmd;

    if (fake_commands_done)
    {
        return;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.command_id = APP_BLE_CMD_PING;
    app_ble_stream_run_fake_command(&cmd);

    app_ble_stream_make_time_sync_cmd(&cmd);
    app_ble_stream_run_fake_command(&cmd);

    memset(&cmd, 0, sizeof(cmd));
    cmd.seq = 2u;
    cmd.command_id = APP_BLE_CMD_START_MONITOR;
    app_ble_stream_run_fake_command(&cmd);

    memset(&cmd, 0, sizeof(cmd));
    cmd.seq = 3u;
    cmd.command_id = APP_BLE_CMD_CLEAR_NIGHT_SUMMARY;
    app_ble_stream_run_fake_command(&cmd);

    fake_commands_done = true;
#endif
}

#if (APP_BLE_SUMMARY_ENABLE)
static void app_ble_stream_fill_realtime_from_summary(
    const app_monitor_summary_snapshot_t *summary,
    app_ble_realtime_sample_t *sample)
{
    bool radar_valid =
        (0u != (summary->source_valid_mask & APP_MONITOR_SOURCE_RADAR));

    memset(sample, 0, sizeof(*sample));
    sample->ts_s = app_ble_stream_now_s();
    sample->rr_bpm =
        app_ble_stream_summary_bpm_u8(summary->radar.rr_bpm_x10,
                                      radar_valid);
    sample->hr_bpm =
        app_ble_stream_summary_bpm_u8(summary->radar.hr_bpm_x10,
                                      radar_valid);
    sample->presence = app_ble_stream_summary_presence(summary);
    sample->motion = radar_valid ?
        summary->radar.motion_x100 : APP_BLE_INVALID_U8;
    sample->cough_prob = summary->audio.valid ?
        summary->audio.cough_prob_x100 : APP_BLE_INVALID_U8;
    sample->snore_prob = APP_BLE_INVALID_U8;
    sample->fusion_state = app_ble_stream_summary_fusion_state(summary);
    sample->alert_level =
        app_ble_stream_summary_alert_level(summary->alert_level);
    sample->quality_flags = app_ble_stream_summary_quality_flags(summary);
}

static void app_ble_stream_fill_event_from_summary(
    const app_monitor_summary_event_t *summary_event,
    app_ble_event_t *ble_event)
{
    uint32_t duration_s =
        (0u == summary_event->duration_ms) ? 1u :
        ((summary_event->duration_ms + 999u) / 1000u);

    memset(ble_event, 0, sizeof(*ble_event));
    ble_event->event_id = summary_event->event_id;
    ble_event->ts_s = summary_event->timestamp_ms / 1000u;
    ble_event->event_type =
        app_ble_stream_summary_event_type(summary_event);
    ble_event->severity =
        app_ble_stream_summary_alert_level(summary_event->alert_level);
    ble_event->confidence = summary_event->confidence;
    ble_event->duration_s =
        (duration_s > 255u) ? 255u : (uint8_t)duration_s;
    ble_event->source_flags =
        app_ble_stream_summary_source_flags(summary_event->source_flags);
}

static uint8_t app_ble_stream_summary_fusion_state(
    const app_monitor_summary_snapshot_t *summary)
{
    if ((APP_MONITOR_STATE_ERROR == summary->monitor_state) ||
        (APP_MONITOR_STATE_DEGRADED == summary->monitor_state))
    {
        return APP_BLE_FUSION_SENSOR_FAULT;
    }
    if (summary->radar.valid &&
        (APP_MONITOR_PRESENCE_ABSENT == summary->radar.presence_state))
    {
        return APP_BLE_FUSION_NO_TARGET;
    }

    switch (summary->fusion_state)
    {
        case APP_MONITOR_FUSION_STATE_ATTENTION:
            return APP_BLE_FUSION_ATTENTION;

        case APP_MONITOR_FUSION_STATE_WARNING:
            return APP_BLE_FUSION_WARNING;

        case APP_MONITOR_FUSION_STATE_DEGRADED:
            return APP_BLE_FUSION_SENSOR_FAULT;

        case APP_MONITOR_FUSION_STATE_NORMAL:
        case APP_MONITOR_FUSION_STATE_AUDIO_ONLY:
        case APP_MONITOR_FUSION_STATE_UNKNOWN:
        default:
            return APP_BLE_FUSION_NORMAL;
    }
}

static uint8_t app_ble_stream_summary_alert_level(
    app_monitor_alert_level_t alert_level)
{
    switch (alert_level)
    {
        case APP_MONITOR_ALERT_LEVEL_INFO:
        case APP_MONITOR_ALERT_LEVEL_ATTENTION:
            return APP_BLE_ALERT_INFO;

        case APP_MONITOR_ALERT_LEVEL_WARNING:
            return APP_BLE_ALERT_WARNING;

        case APP_MONITOR_ALERT_LEVEL_ERROR:
            return APP_BLE_ALERT_HIGH_RISK;

        case APP_MONITOR_ALERT_LEVEL_NONE:
        default:
            return APP_BLE_ALERT_NONE;
    }
}

static uint8_t app_ble_stream_summary_event_type(
    const app_monitor_summary_event_t *event)
{
    switch (event->event_type)
    {
        case APP_MONITOR_EVENT_CONFIRMED_COUGH:
            return APP_BLE_EVENT_COUGH;

        case APP_MONITOR_EVENT_COUGH_BURST:
            return APP_BLE_EVENT_WARNING;

        case APP_MONITOR_EVENT_VITALS_ATTENTION:
            return APP_BLE_EVENT_WARNING;

        case APP_MONITOR_EVENT_SOURCE_CHANGED:
            return APP_BLE_EVENT_SENSOR_FAULT;

        case APP_MONITOR_EVENT_ALERT_CHANGED:
            return (APP_MONITOR_ALERT_LEVEL_WARNING <= event->alert_level) ?
                APP_BLE_EVENT_WARNING : APP_BLE_EVENT_INFO;

        case APP_MONITOR_EVENT_STATE_CHANGED:
        case APP_MONITOR_EVENT_SYSTEM_STATUS:
        default:
            return APP_BLE_EVENT_INFO;
    }
}

static uint8_t app_ble_stream_summary_quality_flags(
    const app_monitor_summary_snapshot_t *summary)
{
    uint8_t flags = 0u;

    if (0u != (summary->source_valid_mask & APP_MONITOR_SOURCE_AUDIO))
    {
        flags |= APP_BLE_QUALITY_AUDIO_VALID;
    }
    if (0u != (summary->source_valid_mask & APP_MONITOR_SOURCE_RADAR))
    {
        flags |= APP_BLE_QUALITY_RADAR_VALID;
    }
    if (0u != (summary->source_valid_mask &
               APP_MONITOR_SOURCE_FUSION_SUMMARY))
    {
        flags |= APP_BLE_QUALITY_FUSION_VALID;
    }
    if ((APP_MONITOR_VITAL_NORMAL == summary->radar.heart_state) ||
        (APP_MONITOR_VITAL_LOW == summary->radar.heart_state) ||
        (APP_MONITOR_VITAL_HIGH == summary->radar.heart_state))
    {
        flags |= APP_BLE_QUALITY_HR_VALID;
    }
    if ((APP_MONITOR_VITAL_NORMAL == summary->radar.breath_state) ||
        (APP_MONITOR_VITAL_LOW == summary->radar.breath_state) ||
        (APP_MONITOR_VITAL_HIGH == summary->radar.breath_state))
    {
        flags |= APP_BLE_QUALITY_RR_VALID;
    }
    if (APP_MONITOR_MOTION_HIGH == summary->radar.motion_state)
    {
        flags |= APP_BLE_QUALITY_MOTION_HIGH;
    }
    if (summary->device.model_ready)
    {
        flags |= APP_BLE_QUALITY_MODEL_READY;
    }
    if (summary->device.time_synced || app_ble_cmd_time_is_synced())
    {
        flags |= APP_BLE_QUALITY_TIME_SYNCED;
    }

    return flags;
}

static uint8_t app_ble_stream_summary_presence(
    const app_monitor_summary_snapshot_t *summary)
{
    if (!summary->radar.valid)
    {
        return APP_BLE_INVALID_U8;
    }
    if (APP_MONITOR_PRESENCE_PRESENT == summary->radar.presence_state)
    {
        return 100u;
    }
    if (APP_MONITOR_PRESENCE_ABSENT == summary->radar.presence_state)
    {
        return 0u;
    }

    return APP_BLE_INVALID_U8;
}

static uint8_t app_ble_stream_summary_bpm_u8(uint16_t bpm_x10,
                                             bool valid)
{
    uint32_t rounded;

    if (!valid || (0u == bpm_x10))
    {
        return APP_BLE_INVALID_U8;
    }

    rounded = ((uint32_t)bpm_x10 + 5u) / 10u;
    return (rounded > 255u) ? APP_BLE_INVALID_U8 : (uint8_t)rounded;
}

static uint8_t app_ble_stream_summary_source_flags(uint32_t source_flags)
{
    uint8_t flags = 0u;

    if (0u != (source_flags & APP_MONITOR_SOURCE_AUDIO))
    {
        flags |= APP_BLE_EVENT_SOURCE_AUDIO;
    }
    if (0u != (source_flags & APP_MONITOR_SOURCE_RADAR))
    {
        flags |= APP_BLE_EVENT_SOURCE_RADAR;
    }
    if (0u != (source_flags & APP_MONITOR_SOURCE_FUSION_SUMMARY))
    {
        flags |= APP_BLE_EVENT_SOURCE_FUSION;
    }

    return flags;
}
#endif

#if (APP_BLE_FAKE_DATA_ENABLE && !APP_BLE_STACK_ENABLE)
static void app_ble_stream_run_fake_command(const app_ble_command_t *cmd)
{
    app_ble_cmd_response_t resp;
    uint8_t frame[APP_BLE_CMD_RESPONSE_FRAME_MAX_LEN];
    uint16_t frame_len = 0u;

    if (NULL == cmd)
    {
        return;
    }

    if (CY_RSLT_SUCCESS != app_ble_cmd_handle(cmd, &resp))
    {
        app_ble_diag_note_notify_fail(APP_BLE_ERR_INVALID_ARG);
        return;
    }

    if (CY_RSLT_SUCCESS == app_ble_pack_cmd_response(&resp,
                                                     frame,
                                                     sizeof(frame),
                                                     &frame_len))
    {
        app_ble_diag_note_notify_ok();
    }
    else
    {
        app_ble_diag_note_notify_fail(APP_BLE_ERR_PACK_FAILED);
    }

#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
    if (((APP_BLE_CMD_START_MONITOR == cmd->command_id) ||
         (APP_BLE_CMD_STOP_MONITOR == cmd->command_id)) &&
        (APP_BLE_OK == resp.status))
    {
        printf("[BLE_CMD] id=%s result=%s monitor=%u resp_len=%u\r\n",
               app_ble_cmd_name(cmd->command_id),
               app_ble_error_name(resp.status),
               app_ble_cmd_monitor_is_requested() ? 1u : 0u,
               (unsigned int)frame_len);
    }
    else
    {
        printf("[BLE_CMD] id=%s result=%s resp_len=%u\r\n",
               app_ble_cmd_name(cmd->command_id),
               app_ble_error_name(resp.status),
               (unsigned int)frame_len);
    }
    if (APP_BLE_OK != resp.status)
    {
        printf("[BLE_ERR] code=%s detail=%s\r\n",
               app_ble_error_name(resp.status),
               app_ble_cmd_name(cmd->command_id));
    }
#endif
}

static void app_ble_stream_make_time_sync_cmd(app_ble_command_t *cmd)
{
    uint32_t epoch_s = 1735689600UL;

    if (NULL == cmd)
    {
        return;
    }

    memset(cmd, 0, sizeof(*cmd));
    cmd->seq = 1u;
    cmd->command_id = APP_BLE_CMD_TIME_SYNC;
    cmd->payload_len = 4u;
    cmd->payload[0] = (uint8_t)(epoch_s & 0xFFu);
    cmd->payload[1] = (uint8_t)((epoch_s >> 8u) & 0xFFu);
    cmd->payload[2] = (uint8_t)((epoch_s >> 16u) & 0xFFu);
    cmd->payload[3] = (uint8_t)((epoch_s >> 24u) & 0xFFu);
}
#endif

static void app_ble_stream_print_stat(uint32_t now_ms)
{
#if (APP_BLE_LOG_LEVEL >= APP_BLE_LOG_LEVEL_STAT)
    app_ble_diag_t diag;
    uint8_t monitor = app_ble_cmd_monitor_is_requested() ? 1u : 0u;
    app_ble_alert_threshold_config_t threshold;
#if (APP_BLE_STACK_ENABLE)
    uint8_t rt_tx_busy = app_ble_gatt_is_realtime_tx_busy() ? 1u : 0u;
    uint8_t evt_tx_busy = app_ble_gatt_is_event_tx_busy() ? 1u : 0u;
    uint8_t cmd_rsp_tx_busy =
        app_ble_gatt_is_cmd_response_tx_busy() ? 1u : 0u;
#else
    uint8_t rt_tx_busy = 0u;
    uint8_t evt_tx_busy = 0u;
    uint8_t cmd_rsp_tx_busy = 0u;
#endif

    if ((0u != stat_last_print_ms) &&
        ((now_ms - stat_last_print_ms) < APP_BLE_STAT_PRINT_PERIOD_MS))
    {
        return;
    }

    app_ble_cmd_get_alert_threshold_config(&threshold);

    if (CY_RSLT_SUCCESS == app_ble_get_diag(&diag))
    {
        printf("[BLE_STAT] conn=%u mtu=%u notify_ok=%lu notify_fail=%lu "
               "drop_rt=%lu no_sub_rt=%lu busy_rt=%lu oversize_rt=%lu "
               "drop_evt=%lu no_sub_evt=%lu busy_evt=%lu oversize_evt=%lu "
               "transport_err=%lu rt_tx_busy=%u evt_tx_busy=%u "
               "cmd_rsp_tx_busy=%u monitor=%u thr_cough=%u thr_warn=%u "
               "thr_high=%u cmd_rx=%lu cmd=%lu cmd_ok=%lu "
               "cmd_unsupported=%lu cmd_not_ready=%lu cmd_denied=%lu "
               "cmd_crc_err=%lu cmd_rsp_ok=%lu cmd_rsp_fail=%lu "
               "cmd_rsp_no_sub=%lu cmd_drop=%lu err=%u\r\n",
               (unsigned int)diag.connected,
               (unsigned int)diag.mtu,
               (unsigned long)diag.notify_ok,
               (unsigned long)diag.notify_fail,
               (unsigned long)diag.realtime_drop,
               (unsigned long)diag.realtime_no_sub,
               (unsigned long)diag.realtime_busy_drop,
               (unsigned long)diag.realtime_oversize_drop,
               (unsigned long)diag.event_drop,
               (unsigned long)diag.event_no_sub,
               (unsigned long)diag.event_busy_drop,
               (unsigned long)diag.event_oversize_drop,
               (unsigned long)diag.transport_err,
               (unsigned int)rt_tx_busy,
               (unsigned int)evt_tx_busy,
               (unsigned int)cmd_rsp_tx_busy,
               (unsigned int)monitor,
               (unsigned int)threshold.cough_prob_threshold,
               (unsigned int)threshold.warning_event_threshold,
               (unsigned int)threshold.high_risk_event_threshold,
               (unsigned long)diag.cmd_rx,
               (unsigned long)diag.cmd_count,
               (unsigned long)diag.cmd_ok,
               (unsigned long)diag.cmd_unsupported,
               (unsigned long)diag.cmd_not_ready,
               (unsigned long)diag.cmd_denied,
               (unsigned long)diag.cmd_crc_err,
               (unsigned long)diag.cmd_rsp_notify_ok,
               (unsigned long)diag.cmd_rsp_notify_fail,
               (unsigned long)diag.cmd_rsp_no_sub,
               (unsigned long)diag.cmd_drop,
               (unsigned int)diag.last_error);
    }

    stat_last_print_ms = now_ms;
#else
    (void)now_ms;
#endif
}

#endif /* APP_BLE_ENABLE */

#include "app_display.h"

#if (APP_DISPLAY_ENABLE)

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

typedef enum
{
    APP_DISPLAY_CMD_SNAPSHOT = 0,
    APP_DISPLAY_CMD_ALERT_RAISE,
    APP_DISPLAY_CMD_ALERT_CLEAR
} app_display_cmd_type_t;

typedef struct
{
    app_display_cmd_type_t type;
    e84_display_snapshot_t snapshot;
    e84_display_alert_t alert;
    uint8_t severity;
    uint8_t confidence;
    uint32_t flags;
} app_display_cmd_t;

cy_rslt_t app_display_backend_null_init(void);
void app_display_backend_null_render_snapshot(
    const e84_display_snapshot_t *snapshot,
    bool force);
void app_display_backend_null_render_alert(e84_display_alert_t alert,
                                           const char *action,
                                           uint8_t severity,
                                           uint8_t confidence,
                                           uint32_t flags,
                                           uint32_t now_ms);

static QueueHandle_t display_queue;
static TaskHandle_t display_task_handle;
static e84_display_snapshot_t current_snapshot;
static uint32_t display_dropped_commands;

static void app_display_task(void *pvParameters);
static cy_rslt_t app_display_enqueue(const app_display_cmd_t *cmd);
static bool app_display_snapshot_is_valid(
    const e84_display_snapshot_t *snapshot);
static uint32_t app_display_now_ms(void);
#if (APP_DISPLAY_SMOKE_ENABLE)
static void app_display_smoke_tick(uint32_t now_ms);
static void app_display_smoke_make_snapshot(
    e84_display_snapshot_t *snapshot,
    uint32_t now_ms,
    uint32_t step);
#endif

cy_rslt_t app_display_init(void)
{
    cy_rslt_t result;

    if (NULL != display_queue)
    {
        return CY_RSLT_SUCCESS;
    }

    memset(&current_snapshot, 0, sizeof(current_snapshot));
    current_snapshot.timestamp_ms = app_display_now_ms();
    current_snapshot.health_state = E84_DISPLAY_HEALTH_INIT;
    current_snapshot.active_alert = E84_DISPLAY_ALERT_NONE;

    display_queue = xQueueCreate(APP_DISPLAY_QUEUE_DEPTH,
                                 sizeof(app_display_cmd_t));
    if (NULL == display_queue)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    result = app_display_backend_null_init();
    if (CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_display_start(void)
{
    BaseType_t ret;

    if (NULL == display_queue)
    {
        cy_rslt_t result = app_display_init();
        if (CY_RSLT_SUCCESS != result)
        {
            return result;
        }
    }

    if (NULL != display_task_handle)
    {
        return CY_RSLT_SUCCESS;
    }

    ret = xTaskCreate(app_display_task,
                      "app_display",
                      APP_DISPLAY_TASK_STACK_SIZE,
                      NULL,
                      APP_DISPLAY_TASK_PRIORITY,
                      &display_task_handle);
    if (pdPASS != ret)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t app_display_publish_snapshot(
    const e84_display_snapshot_t *snapshot)
{
    app_display_cmd_t cmd;

    if (!app_display_snapshot_is_valid(snapshot))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = APP_DISPLAY_CMD_SNAPSHOT;
    cmd.snapshot = *snapshot;

    return app_display_enqueue(&cmd);
}

cy_rslt_t app_display_raise_alert(e84_display_alert_t alert,
                                  uint8_t severity,
                                  uint8_t confidence,
                                  uint32_t flags)
{
    app_display_cmd_t cmd;

    if ((alert <= E84_DISPLAY_ALERT_NONE) ||
        (alert >= E84_DISPLAY_ALERT_COUNT))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = APP_DISPLAY_CMD_ALERT_RAISE;
    cmd.alert = alert;
    cmd.severity = severity;
    cmd.confidence = confidence;
    cmd.flags = flags;

    return app_display_enqueue(&cmd);
}

cy_rslt_t app_display_clear_alert(e84_display_alert_t alert)
{
    app_display_cmd_t cmd;

    if (alert >= E84_DISPLAY_ALERT_COUNT)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = APP_DISPLAY_CMD_ALERT_CLEAR;
    cmd.alert = alert;

    return app_display_enqueue(&cmd);
}

const char *e84_display_health_state_name(
    e84_display_health_state_t state)
{
    switch (state)
    {
        case E84_DISPLAY_HEALTH_INIT:
            return "INIT";

        case E84_DISPLAY_HEALTH_NORMAL:
            return "NORMAL";

        case E84_DISPLAY_HEALTH_ATTENTION:
            return "ATTENTION";

        case E84_DISPLAY_HEALTH_WARNING:
            return "WARNING";

        case E84_DISPLAY_HEALTH_SENSOR_LOST:
            return "SENSOR_LOST";

        case E84_DISPLAY_HEALTH_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}

const char *e84_display_alert_name(e84_display_alert_t alert)
{
    switch (alert)
    {
        case E84_DISPLAY_ALERT_NONE:
            return "NONE";

        case E84_DISPLAY_ALERT_COUGH_BURST:
            return "COUGH_BURST";

        case E84_DISPLAY_ALERT_RESP_RATE_ABNORMAL:
            return "RESP_RATE_ABNORMAL";

        case E84_DISPLAY_ALERT_HEART_RATE_ABNORMAL:
            return "HEART_RATE_ABNORMAL";

        case E84_DISPLAY_ALERT_BREATHING_GAP:
            return "BREATHING_GAP";

        case E84_DISPLAY_ALERT_SENSOR_LOST:
            return "SENSOR_LOST";

        case E84_DISPLAY_ALERT_SYSTEM_ERROR:
            return "SYSTEM_ERROR";

        default:
            return "UNKNOWN";
    }
}

static void app_display_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        app_display_cmd_t cmd;

        if (pdPASS == xQueueReceive(display_queue,
                                    &cmd,
                                    pdMS_TO_TICKS(APP_DISPLAY_TASK_IDLE_MS)))
        {
            uint32_t now_ms = app_display_now_ms();

            switch (cmd.type)
            {
                case APP_DISPLAY_CMD_SNAPSHOT:
                    current_snapshot = cmd.snapshot;
                    app_display_backend_null_render_snapshot(
                        &current_snapshot,
                        true);
                    break;

                case APP_DISPLAY_CMD_ALERT_RAISE:
                    current_snapshot.timestamp_ms = now_ms;
                    current_snapshot.active_alert = cmd.alert;
                    current_snapshot.flags |=
                        (cmd.flags | E84_DISPLAY_FLAG_ALERT_LATCHED);
                    app_display_backend_null_render_alert(cmd.alert,
                                                          "raise",
                                                          cmd.severity,
                                                          cmd.confidence,
                                                          cmd.flags,
                                                          now_ms);
                    app_display_backend_null_render_snapshot(
                        &current_snapshot,
                        true);
                    break;

                case APP_DISPLAY_CMD_ALERT_CLEAR:
                    if ((E84_DISPLAY_ALERT_NONE == cmd.alert) ||
                        (current_snapshot.active_alert == cmd.alert))
                    {
                        current_snapshot.timestamp_ms = now_ms;
                        current_snapshot.active_alert =
                            E84_DISPLAY_ALERT_NONE;
                        current_snapshot.flags &=
                            ~E84_DISPLAY_FLAG_ALERT_LATCHED;
                    }
                    app_display_backend_null_render_alert(cmd.alert,
                                                          "clear",
                                                          0u,
                                                          0u,
                                                          0u,
                                                          now_ms);
                    app_display_backend_null_render_snapshot(
                        &current_snapshot,
                        true);
                    break;

                default:
                    break;
            }
        }
        else
        {
#if (APP_DISPLAY_SMOKE_ENABLE)
            app_display_smoke_tick(app_display_now_ms());
#else
            app_display_backend_null_render_snapshot(&current_snapshot, false);
#endif
        }
    }
}

static cy_rslt_t app_display_enqueue(const app_display_cmd_t *cmd)
{
    app_display_cmd_t dropped;

    if ((NULL == cmd) || (NULL == display_queue))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    if (pdPASS == xQueueSend(display_queue, cmd, 0u))
    {
        return CY_RSLT_SUCCESS;
    }

    if (pdPASS == xQueueReceive(display_queue, &dropped, 0u))
    {
        display_dropped_commands++;
    }

    if (pdPASS == xQueueSend(display_queue, cmd, 0u))
    {
        return CY_RSLT_SUCCESS;
    }

    display_dropped_commands++;
    return CY_RSLT_TYPE_ERROR;
}

static bool app_display_snapshot_is_valid(
    const e84_display_snapshot_t *snapshot)
{
    return ((NULL != snapshot) &&
            (snapshot->health_state < E84_DISPLAY_HEALTH_COUNT) &&
            (snapshot->active_alert < E84_DISPLAY_ALERT_COUNT));
}

static uint32_t app_display_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

#if (APP_DISPLAY_SMOKE_ENABLE)
static void app_display_smoke_tick(uint32_t now_ms)
{
    static uint32_t last_snapshot_ms;
    static uint32_t snapshot_step;
    static uint32_t last_alert_start_ms;
    static bool alert_active;

    if ((0u == last_snapshot_ms) ||
        ((now_ms - last_snapshot_ms) >=
         APP_DISPLAY_SMOKE_SNAPSHOT_PERIOD_MS))
    {
        e84_display_snapshot_t snapshot;

        app_display_smoke_make_snapshot(&snapshot, now_ms, snapshot_step);
        if (alert_active)
        {
            snapshot.active_alert = E84_DISPLAY_ALERT_COUGH_BURST;
            snapshot.flags |= E84_DISPLAY_FLAG_ALERT_LATCHED;
        }

        (void)app_display_publish_snapshot(&snapshot);
        last_snapshot_ms = now_ms;
        snapshot_step++;
    }

    if (!alert_active)
    {
        bool alert_due =
            ((0u == last_alert_start_ms) &&
             (now_ms >= APP_DISPLAY_SMOKE_ALERT_PERIOD_MS)) ||
            ((0u != last_alert_start_ms) &&
             ((now_ms - last_alert_start_ms) >=
              APP_DISPLAY_SMOKE_ALERT_PERIOD_MS));

        if (alert_due)
        {
            alert_active = true;
            last_alert_start_ms = now_ms;
            (void)app_display_raise_alert(E84_DISPLAY_ALERT_COUGH_BURST,
                                          2u,
                                          85u,
                                          E84_DISPLAY_FLAG_AUDIO_VALID |
                                          E84_DISPLAY_FLAG_FUSION_VALID);
        }
    }
    else if ((now_ms - last_alert_start_ms) >=
             APP_DISPLAY_SMOKE_ALERT_HOLD_MS)
    {
        alert_active = false;
        (void)app_display_clear_alert(E84_DISPLAY_ALERT_COUGH_BURST);
    }
}

static void app_display_smoke_make_snapshot(
    e84_display_snapshot_t *snapshot,
    uint32_t now_ms,
    uint32_t step)
{
    static const e84_display_health_state_t state_cycle[] = {
        E84_DISPLAY_HEALTH_INIT,
        E84_DISPLAY_HEALTH_NORMAL,
        E84_DISPLAY_HEALTH_ATTENTION,
        E84_DISPLAY_HEALTH_WARNING,
        E84_DISPLAY_HEALTH_NORMAL
    };
    e84_display_health_state_t state =
        state_cycle[step % (sizeof(state_cycle) / sizeof(state_cycle[0]))];

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->timestamp_ms = now_ms;
    snapshot->health_state = state;
    snapshot->active_alert = E84_DISPLAY_ALERT_NONE;
    snapshot->radar_presence = true;
    snapshot->radar_quality = 88u;
    snapshot->audio_quality = 92u;
    snapshot->fusion_confidence = 86u;
    snapshot->ble_connected = false;
    snapshot->flags = E84_DISPLAY_FLAG_AUDIO_VALID |
                      E84_DISPLAY_FLAG_RADAR_VALID |
                      E84_DISPLAY_FLAG_FUSION_VALID |
                      E84_DISPLAY_FLAG_RR_VALID |
                      E84_DISPLAY_FLAG_HR_VALID;

    switch (state)
    {
        case E84_DISPLAY_HEALTH_INIT:
            snapshot->breath_rate_bpm = 16.0f;
            snapshot->heart_rate_bpm = 72.0f;
            snapshot->mic_cough_prob = 0.05f;
            snapshot->fusion_confidence = 50u;
            break;

        case E84_DISPLAY_HEALTH_ATTENTION:
            snapshot->breath_rate_bpm = 18.0f;
            snapshot->heart_rate_bpm = 76.0f;
            snapshot->mic_cough_prob = 0.35f;
            snapshot->cough_count_1min = 1u;
            snapshot->cough_count_5min = 2u;
            snapshot->audio_quality = 84u;
            snapshot->fusion_confidence = 72u;
            break;

        case E84_DISPLAY_HEALTH_WARNING:
            snapshot->breath_rate_bpm = 22.0f;
            snapshot->heart_rate_bpm = 82.0f;
            snapshot->mic_cough_prob = 0.85f;
            snapshot->cough_count_1min = 3u;
            snapshot->cough_count_5min = 5u;
            snapshot->audio_quality = 78u;
            snapshot->fusion_confidence = 91u;
            break;

        case E84_DISPLAY_HEALTH_NORMAL:
        default:
            snapshot->breath_rate_bpm = 16.0f;
            snapshot->heart_rate_bpm = 72.0f;
            snapshot->mic_cough_prob = 0.05f;
            snapshot->cough_count_5min = (uint16_t)(step % 2u);
            break;
    }
}
#endif /* APP_DISPLAY_SMOKE_ENABLE */

#endif /* APP_DISPLAY_ENABLE */

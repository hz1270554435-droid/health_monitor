#include "app_display.h"
#include "app_build_config.h"

#if (APP_DISPLAY_ENABLE)

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#if (APP_DISPLAY_SUMMARY_ENABLE && !APP_DISPLAY_SMOKE_ENABLE && \
     !APP_DISPLAY_FINAL_MOCK_ENABLE)
#include "app_display_summary_adapter.h"
#endif

typedef enum
{
    APP_DISPLAY_CMD_SNAPSHOT = 0,
    APP_DISPLAY_CMD_ALERT_RAISE,
    APP_DISPLAY_CMD_ALERT_CLEAR,
    APP_DISPLAY_CMD_DEBUG_SET
} app_display_cmd_type_t;

typedef struct
{
    app_display_cmd_type_t type;
    e84_display_snapshot_t snapshot;
    e84_display_alert_t alert;
    uint8_t severity;
    uint8_t confidence;
    uint32_t flags;
    bool debug_enabled;
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
void app_display_backend_null_render_view(
    const e84_display_view_model_t *view,
    bool force);

static QueueHandle_t display_queue;
static TaskHandle_t display_task_handle;
static e84_display_snapshot_t current_snapshot;
static e84_display_view_model_t current_view;
static uint32_t display_dropped_commands;
#if (APP_DISPLAY_DEBUG_PAGE_ENABLE)
static bool debug_page_requested;
#endif

static void app_display_task(void *pvParameters);
static cy_rslt_t app_display_enqueue(const app_display_cmd_t *cmd);
static bool app_display_snapshot_is_valid(
    const e84_display_snapshot_t *snapshot);
static uint32_t app_display_now_ms(void);
static void app_display_view_init(uint32_t now_ms);
static bool app_display_reduce_snapshot(
    const e84_display_snapshot_t *snapshot,
    uint32_t now_ms);
static bool app_display_reduce_alert_raise(e84_display_alert_t alert,
                                           uint8_t severity,
                                           uint32_t flags,
                                           uint32_t now_ms);
static bool app_display_reduce_alert_clear(e84_display_alert_t alert,
                                           uint32_t now_ms,
                                           const char *reason);
static bool app_display_reduce_alert_timeout(uint32_t now_ms,
                                             e84_display_alert_t *alert);
static bool app_display_build_view_model(uint32_t now_ms,
                                         const char *reason);
static e84_display_page_t app_display_select_page(void);
static e84_display_severity_t app_display_normalize_severity(
    e84_display_alert_t alert,
    uint8_t severity);
static e84_display_severity_t app_display_default_alert_severity(
    e84_display_alert_t alert);
static const char *app_display_alert_title(e84_display_alert_t alert);
static const char *app_display_alert_message(e84_display_alert_t alert);
#if (APP_DISPLAY_DEBUG_PAGE_ENABLE && APP_DISPLAY_SMOKE_ENABLE)
static cy_rslt_t app_display_set_debug_page(bool enabled);
#endif
#if (APP_DISPLAY_SMOKE_ENABLE)
static void app_display_smoke_tick(uint32_t now_ms);
static void app_display_smoke_make_snapshot(
    e84_display_snapshot_t *snapshot,
    uint32_t now_ms,
    uint32_t step);
#endif
#if (APP_DISPLAY_FINAL_MOCK_ENABLE)
static void app_display_final_mock_tick(uint32_t now_ms);
static void app_display_final_mock_make_snapshot(
    e84_display_snapshot_t *snapshot,
    uint32_t now_ms,
    uint32_t step);
#endif
#if (APP_DISPLAY_LCD_ENABLE)
static cy_rslt_t app_display_backend_lcd_init(void);
#if (APP_DISPLAY_LCD_SMOKE_ONLY)
static void app_display_lcd_smoke_tick(uint32_t now_ms);
#endif
#if (APP_DISPLAY_LCD_BACKLIGHT_SMOKE_ONLY)
static void app_display_lcd_backlight_smoke_tick(uint32_t now_ms);
#endif
#endif

#if (APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE)
#include "app_display_cm55_bridge.h"
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
    app_display_view_init(current_snapshot.timestamp_ms);

    display_queue = xQueueCreate(APP_DISPLAY_QUEUE_DEPTH,
                                 sizeof(app_display_cmd_t));
    if (NULL == display_queue)
    {
        return CY_RSLT_TYPE_ERROR;
    }

#if (APP_DISPLAY_CM55_SNAPSHOT_BRIDGE_ENABLE)
    {
        cy_rslt_t bridge_result = app_display_cm55_bridge_init();
        if (CY_RSLT_SUCCESS != bridge_result)
        {
            printf("[DISPLAY_BRIDGE] init failed, result=0x%08lx\r\n",
                   (unsigned long)bridge_result);
            fflush(stdout);
        }
        else
        {
            printf("[DISPLAY_BRIDGE] init=ok\r\n");
            fflush(stdout);
        }
    }
#endif

#if (APP_DISPLAY_LCD_ENABLE)
    printf("[DISPLAY_BACKEND_SELECT] backend=lcd\r\n");
    fflush(stdout);
    result = app_display_backend_lcd_init();
    if (CY_RSLT_SUCCESS != result)
    {
        printf("[DISPLAY_LCD_INIT] result=fail reason=%s code=0x%08lx\r\n",
               app_display_backend_lcd_last_fail_reason(),
               (unsigned long)result);
        fflush(stdout);
    }
    else
    {
        printf("[DISPLAY_LCD_INIT] result=OK\r\n");
        fflush(stdout);
    }
#else
    printf("[DISPLAY_BACKEND_SELECT] backend=null\r\n");
    fflush(stdout);
    result = app_display_backend_null_init();
#endif
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

const char *e84_display_page_name(e84_display_page_t page)
{
    switch (page)
    {
        case E84_DISPLAY_PAGE_BOOT:
            return "BOOT";

        case E84_DISPLAY_PAGE_HOME:
            return "HOME";

        case E84_DISPLAY_PAGE_ALERT:
            return "ALERT";

        case E84_DISPLAY_PAGE_DEBUG:
            return "DEBUG";

        default:
            return "UNKNOWN";
    }
}

const char *e84_display_severity_name(e84_display_severity_t severity)
{
    switch (severity)
    {
        case E84_DISPLAY_SEVERITY_NONE:
            return "NONE";

        case E84_DISPLAY_SEVERITY_INFO:
            return "INFO";

        case E84_DISPLAY_SEVERITY_ATTENTION:
            return "ATTENTION";

        case E84_DISPLAY_SEVERITY_WARNING:
            return "WARNING";

        case E84_DISPLAY_SEVERITY_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}

const char *e84_display_radar_source_state_name(
    e84_display_radar_source_state_t state)
{
    switch (state)
    {
        case E84_DISPLAY_RADAR_SOURCE_NORMAL:
            return "NORMAL";

        case E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE:
            return "UNAVAILABLE";

        case E84_DISPLAY_RADAR_SOURCE_STALE:
            return "STALE";

        case E84_DISPLAY_RADAR_SOURCE_INVALID:
            return "INVALID";

        case E84_DISPLAY_RADAR_SOURCE_LOW_QUALITY:
            return "LOW_QUALITY";

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
                {
                    bool force_view =
                        app_display_reduce_snapshot(&cmd.snapshot, now_ms);
#if (APP_DISPLAY_FINAL_MOCK_ENABLE)
                    force_view = true;
#endif
#if (!APP_DISPLAY_FINAL_MOCK_ENABLE && !APP_DISPLAY_PRODUCT_SCREEN_ENABLE)
                    app_display_backend_null_render_snapshot(
                        &current_snapshot,
                        true);
#endif
                    app_display_backend_null_render_view(&current_view,
                                                         force_view);
                    break;
                }

                case APP_DISPLAY_CMD_ALERT_RAISE:
                    (void)app_display_reduce_alert_raise(cmd.alert,
                                                         cmd.severity,
                                                         cmd.flags,
                                                         now_ms);
                    app_display_backend_null_render_alert(cmd.alert,
                                                          "raise",
                                                          cmd.severity,
                                                          cmd.confidence,
                                                          cmd.flags,
                                                          now_ms);
#if (!APP_DISPLAY_FINAL_MOCK_ENABLE && !APP_DISPLAY_PRODUCT_SCREEN_ENABLE)
                    app_display_backend_null_render_snapshot(
                        &current_snapshot,
                        true);
#endif
                    app_display_backend_null_render_view(&current_view,
                                                         true);
                    break;

                case APP_DISPLAY_CMD_ALERT_CLEAR:
                    (void)app_display_reduce_alert_clear(cmd.alert,
                                                         now_ms,
                                                         "alert_clear");
                    app_display_backend_null_render_alert(cmd.alert,
                                                          "clear",
                                                          0u,
                                                          0u,
                                                          0u,
                                                          now_ms);
#if (!APP_DISPLAY_FINAL_MOCK_ENABLE && !APP_DISPLAY_PRODUCT_SCREEN_ENABLE)
                    app_display_backend_null_render_snapshot(
                        &current_snapshot,
                        true);
#endif
                    app_display_backend_null_render_view(&current_view,
                                                         true);
                    break;

                case APP_DISPLAY_CMD_DEBUG_SET:
#if (APP_DISPLAY_DEBUG_PAGE_ENABLE)
                    debug_page_requested = cmd.debug_enabled;
                    (void)app_display_build_view_model(now_ms,
                                                       cmd.debug_enabled ?
                                                       "debug_enter" :
                                                       "debug_exit");
                    app_display_backend_null_render_view(&current_view,
                                                         true);
#endif
                    break;

                default:
                    break;
            }
        }
        else
        {
            e84_display_alert_t timeout_alert = E84_DISPLAY_ALERT_NONE;
            uint32_t now_ms = app_display_now_ms();

            if (app_display_reduce_alert_timeout(now_ms, &timeout_alert))
            {
                app_display_backend_null_render_alert(timeout_alert,
                                                      "timeout",
                                                      0u,
                                                      0u,
                                                      0u,
                                                      now_ms);
#if (!APP_DISPLAY_FINAL_MOCK_ENABLE && !APP_DISPLAY_PRODUCT_SCREEN_ENABLE)
                app_display_backend_null_render_snapshot(&current_snapshot,
                                                         true);
#endif
                app_display_backend_null_render_view(&current_view, true);
            }
#if (APP_DISPLAY_LCD_BACKLIGHT_SMOKE_ONLY)
            app_display_lcd_backlight_smoke_tick(now_ms);
#elif (APP_DISPLAY_LCD_SMOKE_ONLY)
            app_display_lcd_smoke_tick(now_ms);
#elif (APP_DISPLAY_FINAL_MOCK_ENABLE)
            app_display_final_mock_tick(now_ms);
#elif (APP_DISPLAY_SMOKE_ENABLE)
            app_display_smoke_tick(now_ms);
#elif (APP_DISPLAY_SUMMARY_ENABLE)
            (void)app_display_summary_adapter_tick(now_ms);
#else
            app_display_backend_null_render_snapshot(&current_snapshot, false);
            app_display_backend_null_render_view(&current_view, false);
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

static void app_display_view_init(uint32_t now_ms)
{
    memset(&current_view, 0, sizeof(current_view));
    current_view.current_page = E84_DISPLAY_PAGE_BOOT;
    current_view.previous_page = E84_DISPLAY_PAGE_BOOT;
    current_view.health_state = E84_DISPLAY_HEALTH_INIT;
    current_view.alert.code = E84_DISPLAY_ALERT_NONE;
    current_view.alert.severity = E84_DISPLAY_SEVERITY_NONE;
    current_view.alert.title = app_display_alert_title(
        E84_DISPLAY_ALERT_NONE);
    current_view.alert.short_message = app_display_alert_message(
        E84_DISPLAY_ALERT_NONE);
    current_view.snapshot = current_snapshot;
    current_view.last_refresh_timestamp_ms = now_ms;
    current_view.refresh_reason = "init";
    current_view.smoke_enabled = (APP_DISPLAY_SMOKE_ENABLE != 0u);
    current_view.dirty = true;
}

static bool app_display_reduce_snapshot(
    const e84_display_snapshot_t *snapshot,
    uint32_t now_ms)
{
    if (NULL == snapshot)
    {
        return false;
    }

    current_snapshot = *snapshot;
    if (E84_DISPLAY_ALERT_NONE != current_view.alert.code)
    {
        current_snapshot.active_alert = current_view.alert.code;
        if (current_view.alert.latched)
        {
            current_snapshot.flags |= E84_DISPLAY_FLAG_ALERT_LATCHED;
        }
    }

    return app_display_build_view_model(now_ms, "snapshot");
}

static bool app_display_reduce_alert_raise(e84_display_alert_t alert,
                                           uint8_t severity,
                                           uint32_t flags,
                                           uint32_t now_ms)
{
    bool latched =
        (0u != (flags & E84_DISPLAY_FLAG_ALERT_LATCHED)) ||
        (E84_DISPLAY_ALERT_SYSTEM_ERROR == alert);

    current_view.alert.code = alert;
    current_view.alert.severity =
        app_display_normalize_severity(alert, severity);
    current_view.alert.title = app_display_alert_title(alert);
    current_view.alert.short_message = app_display_alert_message(alert);
    current_view.alert.raised_timestamp_ms = now_ms;
    current_view.alert.timeout_ms = APP_DISPLAY_ALERT_DEFAULT_TIMEOUT_MS;
    current_view.alert.dismissible = !latched;
    current_view.alert.latched = latched;

    current_snapshot.timestamp_ms = now_ms;
    current_snapshot.active_alert = alert;
    if (latched)
    {
        current_snapshot.flags |= E84_DISPLAY_FLAG_ALERT_LATCHED;
    }
    else
    {
        current_snapshot.flags &= ~E84_DISPLAY_FLAG_ALERT_LATCHED;
    }

    return app_display_build_view_model(now_ms, "alert_raise");
}

static bool app_display_reduce_alert_clear(e84_display_alert_t alert,
                                           uint32_t now_ms,
                                           const char *reason)
{
    if ((E84_DISPLAY_ALERT_NONE == alert) ||
        (current_view.alert.code == alert))
    {
        current_view.alert.code = E84_DISPLAY_ALERT_NONE;
        current_view.alert.severity = E84_DISPLAY_SEVERITY_NONE;
        current_view.alert.title = app_display_alert_title(
            E84_DISPLAY_ALERT_NONE);
        current_view.alert.short_message = app_display_alert_message(
            E84_DISPLAY_ALERT_NONE);
        current_view.alert.raised_timestamp_ms = 0u;
        current_view.alert.timeout_ms = 0u;
        current_view.alert.dismissible = false;
        current_view.alert.latched = false;

        current_snapshot.timestamp_ms = now_ms;
        current_snapshot.active_alert = E84_DISPLAY_ALERT_NONE;
        current_snapshot.flags &= ~E84_DISPLAY_FLAG_ALERT_LATCHED;
    }

    return app_display_build_view_model(now_ms, reason);
}

static bool app_display_reduce_alert_timeout(uint32_t now_ms,
                                             e84_display_alert_t *alert)
{
    uint32_t elapsed_ms;

    if (NULL != alert)
    {
        *alert = E84_DISPLAY_ALERT_NONE;
    }

    if ((E84_DISPLAY_ALERT_NONE == current_view.alert.code) ||
        current_view.alert.latched ||
        (0u == current_view.alert.timeout_ms))
    {
        return false;
    }

    elapsed_ms = now_ms - current_view.alert.raised_timestamp_ms;
    if (elapsed_ms < current_view.alert.timeout_ms)
    {
        return false;
    }

    if (NULL != alert)
    {
        *alert = current_view.alert.code;
    }

    (void)app_display_reduce_alert_clear(current_view.alert.code,
                                         now_ms,
                                         "alert_timeout");
    return true;
}

static bool app_display_build_view_model(uint32_t now_ms,
                                         const char *reason)
{
    e84_display_page_t old_page = current_view.current_page;

    current_view.previous_page = old_page;
    current_view.current_page = app_display_select_page();
    current_view.health_state = current_snapshot.health_state;
    current_view.snapshot = current_snapshot;
    current_view.display_dropped_commands = display_dropped_commands;
    current_view.last_refresh_timestamp_ms = now_ms;
    current_view.refresh_reason = (NULL != reason) ? reason : "unknown";
    current_view.smoke_enabled = (APP_DISPLAY_SMOKE_ENABLE != 0u);
    current_view.dirty = true;

    return (old_page != current_view.current_page);
}

static e84_display_page_t app_display_select_page(void)
{
    if ((E84_DISPLAY_ALERT_NONE != current_view.alert.code) &&
        (current_view.alert.severity >= E84_DISPLAY_SEVERITY_WARNING))
    {
        return E84_DISPLAY_PAGE_ALERT;
    }

#if (APP_DISPLAY_DEBUG_PAGE_ENABLE)
    if (debug_page_requested)
    {
        return E84_DISPLAY_PAGE_DEBUG;
    }
#endif

    if (E84_DISPLAY_HEALTH_INIT == current_snapshot.health_state)
    {
        return E84_DISPLAY_PAGE_BOOT;
    }

    return E84_DISPLAY_PAGE_HOME;
}

static e84_display_severity_t app_display_normalize_severity(
    e84_display_alert_t alert,
    uint8_t severity)
{
    e84_display_severity_t normalized;
    e84_display_severity_t alert_default =
        app_display_default_alert_severity(alert);

    if (severity >= (uint8_t)E84_DISPLAY_SEVERITY_COUNT)
    {
        normalized = alert_default;
    }
    else
    {
        normalized = (e84_display_severity_t)severity;
    }

    if (normalized < alert_default)
    {
        normalized = alert_default;
    }

    return normalized;
}

static e84_display_severity_t app_display_default_alert_severity(
    e84_display_alert_t alert)
{
    switch (alert)
    {
        case E84_DISPLAY_ALERT_SENSOR_LOST:
        case E84_DISPLAY_ALERT_SYSTEM_ERROR:
            return E84_DISPLAY_SEVERITY_ERROR;

        case E84_DISPLAY_ALERT_COUGH_BURST:
        case E84_DISPLAY_ALERT_RESP_RATE_ABNORMAL:
        case E84_DISPLAY_ALERT_HEART_RATE_ABNORMAL:
        case E84_DISPLAY_ALERT_BREATHING_GAP:
            return E84_DISPLAY_SEVERITY_WARNING;

        case E84_DISPLAY_ALERT_NONE:
        default:
            return E84_DISPLAY_SEVERITY_NONE;
    }
}

static const char *app_display_alert_title(e84_display_alert_t alert)
{
    switch (alert)
    {
        case E84_DISPLAY_ALERT_COUGH_BURST:
            return "Frequent cough detected";

        case E84_DISPLAY_ALERT_RESP_RATE_ABNORMAL:
            return "Respiration needs attention";

        case E84_DISPLAY_ALERT_HEART_RATE_ABNORMAL:
            return "Heart rate needs attention";

        case E84_DISPLAY_ALERT_BREATHING_GAP:
            return "Breathing gap detected";

        case E84_DISPLAY_ALERT_SENSOR_LOST:
            return "Sensor signal lost";

        case E84_DISPLAY_ALERT_SYSTEM_ERROR:
            return "System error";

        case E84_DISPLAY_ALERT_NONE:
        default:
            return "No active alert";
    }
}

static const char *app_display_alert_message(e84_display_alert_t alert)
{
    switch (alert)
    {
        case E84_DISPLAY_ALERT_COUGH_BURST:
            return "Cough burst pattern detected";

        case E84_DISPLAY_ALERT_RESP_RATE_ABNORMAL:
            return "Breathing rate outside normal range";

        case E84_DISPLAY_ALERT_HEART_RATE_ABNORMAL:
            return "Heart rate outside normal range";

        case E84_DISPLAY_ALERT_BREATHING_GAP:
            return "Breathing pause candidate";

        case E84_DISPLAY_ALERT_SENSOR_LOST:
            return "Check radar or audio sensor status";

        case E84_DISPLAY_ALERT_SYSTEM_ERROR:
            return "Display is showing a system alert";

        case E84_DISPLAY_ALERT_NONE:
        default:
            return "Daily status";
    }
}

#if (APP_DISPLAY_DEBUG_PAGE_ENABLE && APP_DISPLAY_SMOKE_ENABLE)
static cy_rslt_t app_display_set_debug_page(bool enabled)
{
    app_display_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = APP_DISPLAY_CMD_DEBUG_SET;
    cmd.debug_enabled = enabled;

    return app_display_enqueue(&cmd);
}
#endif

#if (APP_DISPLAY_SMOKE_ENABLE)
static void app_display_smoke_tick(uint32_t now_ms)
{
    static uint32_t last_snapshot_ms;
    static uint32_t snapshot_step;
    static uint32_t last_alert_start_ms;
    static bool alert_active;
#if (APP_DISPLAY_DEBUG_PAGE_ENABLE)
    static uint32_t last_debug_start_ms;
    static bool debug_active;
#endif

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
#if (APP_DISPLAY_DEBUG_PAGE_ENABLE)
            if (debug_active)
            {
                debug_active = false;
                (void)app_display_set_debug_page(false);
            }
#endif
            (void)app_display_raise_alert(E84_DISPLAY_ALERT_COUGH_BURST,
                                          (uint8_t)
                                          E84_DISPLAY_SEVERITY_WARNING,
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

#if (APP_DISPLAY_DEBUG_PAGE_ENABLE)
    if (!alert_active)
    {
        if (!debug_active)
        {
            bool debug_due =
                ((0u == last_debug_start_ms) && (now_ms >= 6000u)) ||
                ((0u != last_debug_start_ms) &&
                 ((now_ms - last_debug_start_ms) >= 30000u));

            if (debug_due)
            {
                debug_active = true;
                last_debug_start_ms = now_ms;
                (void)app_display_set_debug_page(true);
            }
        }
        else if ((now_ms - last_debug_start_ms) >= 3000u)
        {
            debug_active = false;
            (void)app_display_set_debug_page(false);
        }
    }
#endif
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
        E84_DISPLAY_HEALTH_SENSOR_LOST,
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

        case E84_DISPLAY_HEALTH_SENSOR_LOST:
            snapshot->radar_presence = false;
            snapshot->breath_rate_bpm = 0.0f;
            snapshot->heart_rate_bpm = 0.0f;
            snapshot->radar_quality = 10u;
            snapshot->mic_cough_prob = 0.05f;
            snapshot->audio_quality = 88u;
            snapshot->fusion_confidence = 25u;
            snapshot->flags &= ~(E84_DISPLAY_FLAG_RADAR_VALID |
                                 E84_DISPLAY_FLAG_RR_VALID |
                                 E84_DISPLAY_FLAG_HR_VALID);
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

#if (APP_DISPLAY_FINAL_MOCK_ENABLE)
static void app_display_final_mock_tick(uint32_t now_ms)
{
    static uint32_t last_snapshot_ms;
    static uint32_t snapshot_step;

    if ((0u == last_snapshot_ms) ||
        ((now_ms - last_snapshot_ms) >=
         APP_DISPLAY_FINAL_MOCK_PERIOD_MS))
    {
        e84_display_snapshot_t snapshot;

        app_display_final_mock_make_snapshot(&snapshot,
                                             now_ms,
                                             snapshot_step);
        (void)app_display_publish_snapshot(&snapshot);
        last_snapshot_ms = now_ms;
        snapshot_step++;
    }
}

static void app_display_final_mock_make_snapshot(
    e84_display_snapshot_t *snapshot,
    uint32_t now_ms,
    uint32_t step)
{
    /* 7 scenarios cycling through radar source states. */
    uint32_t scenario = step % 7u;

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->timestamp_ms = now_ms;
    snapshot->active_alert = E84_DISPLAY_ALERT_NONE;
    snapshot->ble_connected = false;

    /* Cough model is not board-verified in current phase. */
    snapshot->cough_model_not_verified = true;
    snapshot->flags = E84_DISPLAY_FLAG_AUDIO_VALID |
                      E84_DISPLAY_FLAG_COUGH_MODEL_NOT_VERIFIED;

    switch (scenario)
    {
        case 0u: /* real_radar_valid: NORMAL health, real radar */
            snapshot->health_state = E84_DISPLAY_HEALTH_NORMAL;
            snapshot->radar_presence = true;
            snapshot->breath_rate_bpm = 16.8f;
            snapshot->heart_rate_bpm = 71.0f;
            snapshot->radar_quality = 90u;
            snapshot->distance_cm = 120u;
            snapshot->mic_cough_prob = 0.08f;
            snapshot->audio_quality = 92u;
            snapshot->fusion_confidence = 86u;
            snapshot->cough_count_5min = (uint16_t)(step % 2u);
            snapshot->radar_source_state = E84_DISPLAY_RADAR_SOURCE_NORMAL;
            snapshot->flags |= E84_DISPLAY_FLAG_RADAR_VALID |
                               E84_DISPLAY_FLAG_FUSION_VALID |
                               E84_DISPLAY_FLAG_RR_VALID |
                               E84_DISPLAY_FLAG_HR_VALID;
            break;

        case 1u: /* attention: radar valid, cough elevated */
            snapshot->health_state = E84_DISPLAY_HEALTH_ATTENTION;
            snapshot->radar_presence = true;
            snapshot->breath_rate_bpm = 17.8f;
            snapshot->heart_rate_bpm = 72.0f;
            snapshot->radar_quality = 88u;
            snapshot->distance_cm = 115u;
            snapshot->mic_cough_prob = 0.64f;
            snapshot->cough_count_1min = 1u;
            snapshot->cough_count_5min = 2u;
            snapshot->audio_quality = 91u;
            snapshot->fusion_confidence = 82u;
            snapshot->radar_source_state = E84_DISPLAY_RADAR_SOURCE_NORMAL;
            snapshot->flags |= E84_DISPLAY_FLAG_RADAR_VALID |
                               E84_DISPLAY_FLAG_FUSION_VALID |
                               E84_DISPLAY_FLAG_RR_VALID |
                               E84_DISPLAY_FLAG_HR_VALID;
            break;

        case 2u: /* warning: radar valid, cough high */
            snapshot->health_state = E84_DISPLAY_HEALTH_WARNING;
            snapshot->radar_presence = true;
            snapshot->breath_rate_bpm = 22.4f;
            snapshot->heart_rate_bpm = 88.0f;
            snapshot->radar_quality = 76u;
            snapshot->distance_cm = 105u;
            snapshot->mic_cough_prob = 0.86f;
            snapshot->cough_count_1min = 3u;
            snapshot->cough_count_5min = 6u;
            snapshot->audio_quality = 80u;
            snapshot->fusion_confidence = 91u;
            snapshot->radar_source_state = E84_DISPLAY_RADAR_SOURCE_NORMAL;
            snapshot->flags |= E84_DISPLAY_FLAG_RADAR_VALID |
                               E84_DISPLAY_FLAG_FUSION_VALID |
                               E84_DISPLAY_FLAG_RR_VALID |
                               E84_DISPLAY_FLAG_HR_VALID;
            break;

        case 3u: /* radar_unavailable: no radar source */
            snapshot->health_state = E84_DISPLAY_HEALTH_SENSOR_LOST;
            snapshot->radar_presence = false;
            snapshot->breath_rate_bpm = 0.0f;
            snapshot->heart_rate_bpm = 0.0f;
            snapshot->radar_quality = 0u;
            snapshot->mic_cough_prob = 0.12f;
            snapshot->cough_count_1min = 0u;
            snapshot->cough_count_5min = 1u;
            snapshot->audio_quality = 89u;
            snapshot->fusion_confidence = 30u;
            snapshot->radar_source_state =
                E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE;
            break;

        case 4u: /* radar_stale: radar data too old */
            snapshot->health_state = E84_DISPLAY_HEALTH_ATTENTION;
            snapshot->radar_presence = false;
            snapshot->breath_rate_bpm = 0.0f;
            snapshot->heart_rate_bpm = 0.0f;
            snapshot->radar_quality = 45u;
            snapshot->mic_cough_prob = 0.35f;
            snapshot->cough_count_1min = 1u;
            snapshot->cough_count_5min = 2u;
            snapshot->audio_quality = 88u;
            snapshot->fusion_confidence = 40u;
            snapshot->radar_source_state = E84_DISPLAY_RADAR_SOURCE_STALE;
            snapshot->flags |= E84_DISPLAY_FLAG_RADAR_VALID;
            break;

        case 5u: /* radar_low_quality: radar signal poor */
            snapshot->health_state = E84_DISPLAY_HEALTH_NORMAL;
            snapshot->radar_presence = false;
            snapshot->breath_rate_bpm = 0.0f;
            snapshot->heart_rate_bpm = 0.0f;
            snapshot->radar_quality = 20u;
            snapshot->mic_cough_prob = 0.15f;
            snapshot->cough_count_1min = 0u;
            snapshot->cough_count_5min = 0u;
            snapshot->audio_quality = 90u;
            snapshot->fusion_confidence = 35u;
            snapshot->radar_source_state =
                E84_DISPLAY_RADAR_SOURCE_LOW_QUALITY;
            snapshot->flags |= E84_DISPLAY_FLAG_RADAR_VALID;
            break;

        case 6u: /* radar_invalid: radar data malformed or quality zero */
            snapshot->health_state = E84_DISPLAY_HEALTH_NORMAL;
            snapshot->radar_presence = false;
            snapshot->breath_rate_bpm = 0.0f;
            snapshot->heart_rate_bpm = 0.0f;
            snapshot->radar_quality = 0u;
            snapshot->mic_cough_prob = 0.10f;
            snapshot->cough_count_1min = 0u;
            snapshot->cough_count_5min = 0u;
            snapshot->audio_quality = 91u;
            snapshot->fusion_confidence = 0u;
            snapshot->radar_source_state =
                E84_DISPLAY_RADAR_SOURCE_INVALID;
            break;

        default:
            snapshot->health_state = E84_DISPLAY_HEALTH_INIT;
            snapshot->radar_source_state =
                E84_DISPLAY_RADAR_SOURCE_UNAVAILABLE;
            break;
    }
}
#endif /* APP_DISPLAY_FINAL_MOCK_ENABLE */

#if (APP_DISPLAY_LCD_ENABLE)
/* LCD backend wrapper - delegates to app_display_backend_lcd.c */
static cy_rslt_t app_display_backend_lcd_init(void)
{
    printf("[DISPLAY_LCD_INIT] begin\r\n");
    fflush(stdout);
    return app_display_backend_lcd_hw_init();
}

#if (APP_DISPLAY_LCD_SMOKE_ONLY)
static void app_display_lcd_smoke_tick(uint32_t now_ms)
{
    static uint32_t last_smoke_ms;
    static uint32_t smoke_tick_count;
    static bool smoke_init_ok;

    if ((0u == last_smoke_ms) ||
        ((now_ms - last_smoke_ms) >= 2000u))
    {
        last_smoke_ms = now_ms;
        smoke_tick_count++;

        if (!smoke_init_ok)
        {
            /* LCD already initialized in app_display_init */
            smoke_init_ok = true;
            printf("[DISPLAY_LCD_SMOKE] init=ok draw=pending tick=%lu\r\n",
                   (unsigned long)smoke_tick_count);
            fflush(stdout);
        }

        /* Draw a simple smoke screen */
        e84_display_snapshot_t smoke_snapshot;
        memset(&smoke_snapshot, 0, sizeof(smoke_snapshot));
        smoke_snapshot.timestamp_ms = now_ms;
        smoke_snapshot.health_state = E84_DISPLAY_HEALTH_NORMAL;
        smoke_snapshot.radar_source_state = E84_DISPLAY_RADAR_SOURCE_NORMAL;
        smoke_snapshot.radar_presence = true;
        smoke_snapshot.breath_rate_bpm = 16.8f;
        smoke_snapshot.heart_rate_bpm = 71.0f;
        smoke_snapshot.radar_quality = 90u;
        smoke_snapshot.distance_cm = 120u;
        smoke_snapshot.cough_model_not_verified = true;
        smoke_snapshot.flags = E84_DISPLAY_FLAG_AUDIO_VALID |
                               E84_DISPLAY_FLAG_RADAR_VALID |
                               E84_DISPLAY_FLAG_RR_VALID |
                               E84_DISPLAY_FLAG_HR_VALID |
                               E84_DISPLAY_FLAG_COUGH_MODEL_NOT_VERIFIED;

        cy_rslt_t draw_result =
            app_display_backend_lcd_render_snapshot(&smoke_snapshot, true);

        if (CY_RSLT_SUCCESS == draw_result)
        {
            printf("[DISPLAY_LCD_SMOKE] init=ok draw=ok tick=%lu\r\n",
                   (unsigned long)smoke_tick_count);
        }
        else
        {
            printf("[DISPLAY_LCD_SMOKE] init=ok draw=fail "
                   "reason=frame_transfer_failed tick=%lu\r\n",
                   (unsigned long)smoke_tick_count);
        }
        fflush(stdout);
    }
}
#endif /* APP_DISPLAY_LCD_SMOKE_ONLY */

#if (APP_DISPLAY_LCD_BACKLIGHT_SMOKE_ONLY)
static void app_display_lcd_backlight_smoke_tick(uint32_t now_ms)
{
    static uint32_t last_smoke_ms;
    static uint32_t smoke_tick_count;
    static bool backlight_tested;

    if ((0u == last_smoke_ms) ||
        ((now_ms - last_smoke_ms) >= 3000u))
    {
        last_smoke_ms = now_ms;
        smoke_tick_count++;

        if (!backlight_tested)
        {
            backlight_tested = true;
            printf("[DISPLAY_LCD_BACKLIGHT] begin\r\n");
            fflush(stdout);

            /* Step 1: Assert STBYB HIGH (P0.0) - display standby control */
            printf("[DISPLAY_LCD_BACKLIGHT] stbyb_port=0 stbyb_pin=0\r\n");
            fflush(stdout);
            Cy_GPIO_Pin_FastInit(GPIO_PRT0, 0u,
                                 CY_GPIO_DM_STRONG_IN_OFF, 1u,
                                 HSIOM_SEL_GPIO);
            Cy_GPIO_Write(GPIO_PRT0, 0u, 1u);
            Cy_SysLib_Delay(50u);
            printf("[DISPLAY_LCD_BACKLIGHT] stbyb=high result=ok\r\n");
            fflush(stdout);

            /* Step 2: Try backlight on P20.6 HIGH */
            printf("[DISPLAY_LCD_BACKLIGHT] bl_port=20 bl_pin=6\r\n");
            fflush(stdout);
            Cy_GPIO_Pin_FastInit(GPIO_PRT20, 6u,
                                 CY_GPIO_DM_STRONG_IN_OFF, 1u,
                                 HSIOM_SEL_GPIO);
            Cy_GPIO_Write(GPIO_PRT20, 6u, 1u);
            printf("[DISPLAY_LCD_BACKLIGHT] bl=high result=ok\r\n");
            fflush(stdout);

            /* Step 3: Also try LOW in case active-low */
            /* (leave HIGH for now, user can test LOW separately) */

            printf("[DISPLAY_LCD_BACKLIGHT] set=on result=ok\r\n");
            fflush(stdout);
        }

        printf("[DISPLAY_LCD_BACKLIGHT] tick=%lu status=on\r\n",
               (unsigned long)smoke_tick_count);
        fflush(stdout);
    }
}
#endif /* APP_DISPLAY_LCD_BACKLIGHT_SMOKE_ONLY */

#endif /* APP_DISPLAY_LCD_ENABLE */

#endif /* APP_DISPLAY_ENABLE */

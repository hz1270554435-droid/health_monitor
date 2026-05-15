#include "app_ble_diag.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

static app_ble_diag_t ble_diag;

void app_ble_diag_reset(void)
{
    memset(&ble_diag, 0, sizeof(ble_diag));
    ble_diag.mtu = 23u;
}

cy_rslt_t app_ble_get_diag(app_ble_diag_t *diag)
{
    if (NULL == diag)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    *diag = ble_diag;
    diag->uptime_s =
        (uint32_t)((xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000u);
    return CY_RSLT_SUCCESS;
}

bool app_ble_is_connected(void)
{
    return (0u != ble_diag.connected);
}

void app_ble_diag_note_notify_ok(void)
{
    ble_diag.notify_ok++;
}

void app_ble_diag_note_notify_fail(app_ble_error_t error)
{
    ble_diag.notify_fail++;
    app_ble_diag_set_last_error(error);
}

void app_ble_diag_note_realtime_drop(void)
{
    ble_diag.realtime_drop++;
    app_ble_diag_set_last_error(APP_BLE_ERR_QUEUE_FULL);
}

void app_ble_diag_note_event_drop(void)
{
    ble_diag.event_drop++;
    app_ble_diag_set_last_error(APP_BLE_ERR_QUEUE_FULL);
}

void app_ble_diag_note_cmd(void)
{
    ble_diag.cmd_count++;
}

void app_ble_diag_note_disconnect(void)
{
    ble_diag.disconnect_count++;
    ble_diag.connected = 0u;
}

void app_ble_diag_set_last_error(app_ble_error_t error)
{
    ble_diag.last_error = (uint8_t)error;
}

void app_ble_diag_set_mtu(uint16_t mtu)
{
    ble_diag.mtu = mtu;
}

void app_ble_diag_set_connected(bool connected)
{
    ble_diag.connected = connected ? 1u : 0u;
}

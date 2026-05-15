#ifndef __APP_BLE_DIAG_H__
#define __APP_BLE_DIAG_H__

#include <stdbool.h>

#include "cy_pdl.h"

#include "app_ble_types.h"

#if defined(__cplusplus)
extern "C" {
#endif

void app_ble_diag_reset(void);
cy_rslt_t app_ble_get_diag(app_ble_diag_t *diag);
bool app_ble_is_connected(void);

void app_ble_diag_note_notify_ok(void);
void app_ble_diag_note_notify_fail(app_ble_error_t error);
void app_ble_diag_note_realtime_drop(void);
void app_ble_diag_note_event_drop(void);
void app_ble_diag_note_cmd(void);
void app_ble_diag_note_disconnect(void);
void app_ble_diag_set_last_error(app_ble_error_t error);
void app_ble_diag_set_mtu(uint16_t mtu);
void app_ble_diag_set_connected(bool connected);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_BLE_DIAG_H__ */

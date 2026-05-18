#ifndef __APP_BLE_GATT_H__
#define __APP_BLE_GATT_H__

#include <stdbool.h>
#include <stdint.h>

#include "cy_pdl.h"

#include "app_ble_config.h"

#if (APP_BLE_ENABLE && APP_BLE_STACK_ENABLE)
#include "wiced_bt_gatt.h"
#endif

#if defined(__cplusplus)
extern "C" {
#endif

cy_rslt_t app_ble_gatt_init(void);
bool app_ble_gatt_is_ready(void);
bool app_ble_gatt_is_available(void);
bool app_ble_gatt_is_realtime_subscribed(void);
bool app_ble_gatt_is_realtime_tx_busy(void);
bool app_ble_gatt_is_event_subscribed(void);
bool app_ble_gatt_is_event_tx_busy(void);
bool app_ble_gatt_is_cmd_response_subscribed(void);
bool app_ble_gatt_is_cmd_response_tx_busy(void);
void app_ble_gatt_on_disconnected(void);

cy_rslt_t app_ble_gatt_notify_realtime(const uint8_t *data, uint16_t len);
cy_rslt_t app_ble_gatt_notify_event(const uint8_t *data, uint16_t len);
cy_rslt_t app_ble_gatt_notify_cmd_response(const uint8_t *data, uint16_t len);

#if (APP_BLE_ENABLE && APP_BLE_STACK_ENABLE)
wiced_bt_gatt_status_t app_ble_gatt_on_event(
    wiced_bt_gatt_evt_t event,
    wiced_bt_gatt_event_data_t *event_data);
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __APP_BLE_GATT_H__ */

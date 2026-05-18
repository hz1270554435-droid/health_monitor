#ifndef __APP_BLE_CORE_H__
#define __APP_BLE_CORE_H__

#include <stdbool.h>
#include <stdint.h>

#include "cy_pdl.h"

#if defined(__cplusplus)
extern "C" {
#endif

cy_rslt_t app_ble_core_init(void);
bool app_ble_core_is_enabled(void);
bool app_ble_core_is_stack_enabled(void);
bool app_ble_core_is_connected(void);
bool app_ble_core_is_advertising(void);
uint16_t app_ble_core_get_conn_id(void);
uint16_t app_ble_core_get_mtu(void);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_BLE_CORE_H__ */

#ifndef __APP_BLE_CMD_H__
#define __APP_BLE_CMD_H__

#include "cy_pdl.h"

#include "app_ble_types.h"

#if defined(__cplusplus)
extern "C" {
#endif

cy_rslt_t app_ble_cmd_handle(const app_ble_command_t *cmd,
                             app_ble_cmd_response_t *resp);

bool app_ble_cmd_time_is_synced(void);
uint32_t app_ble_cmd_get_time_offset_s(void);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_BLE_CMD_H__ */

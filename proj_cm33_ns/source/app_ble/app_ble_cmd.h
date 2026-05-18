#ifndef __APP_BLE_CMD_H__
#define __APP_BLE_CMD_H__

#include "cy_pdl.h"

#include "app_ble_types.h"

#if defined(__cplusplus)
extern "C" {
#endif

cy_rslt_t app_ble_cmd_handle(const app_ble_command_t *cmd,
                             app_ble_cmd_response_t *resp);

typedef struct
{
    uint8_t cough_prob_threshold;
    uint8_t warning_event_threshold;
    uint8_t high_risk_event_threshold;
} app_ble_alert_threshold_config_t;

bool app_ble_cmd_time_is_synced(void);
uint32_t app_ble_cmd_get_time_offset_s(void);
bool app_ble_cmd_monitor_is_requested(void);
uint32_t app_ble_cmd_get_last_start_ts_s(void);
uint32_t app_ble_cmd_get_last_stop_ts_s(void);
void app_ble_cmd_get_alert_threshold_config(
    app_ble_alert_threshold_config_t *config);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_BLE_CMD_H__ */

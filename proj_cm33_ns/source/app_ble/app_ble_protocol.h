#ifndef __APP_BLE_PROTOCOL_H__
#define __APP_BLE_PROTOCOL_H__

#include <stdint.h>

#include "cy_pdl.h"

#include "app_ble_types.h"

#if defined(__cplusplus)
extern "C" {
#endif

cy_rslt_t app_ble_pack_realtime(const app_ble_realtime_sample_t *sample,
                                uint8_t *out,
                                uint16_t out_size,
                                uint16_t *out_len);

cy_rslt_t app_ble_pack_event(const app_ble_event_t *event,
                             uint8_t *out,
                             uint16_t out_size,
                             uint16_t *out_len);

cy_rslt_t app_ble_unpack_command(const uint8_t *data,
                                 uint16_t len,
                                 app_ble_command_t *cmd);

app_ble_error_t app_ble_unpack_command_status(const uint8_t *data,
                                              uint16_t len,
                                              app_ble_command_t *cmd);

cy_rslt_t app_ble_pack_cmd_response(const app_ble_cmd_response_t *resp,
                                    uint8_t *out,
                                    uint16_t out_size,
                                    uint16_t *out_len);

uint8_t app_ble_calc_crc8(const uint8_t *data, uint16_t len);
void app_ble_protocol_reset_sequences(void);

const char *app_ble_error_name(app_ble_error_t error);
const char *app_ble_cmd_name(uint8_t cmd_id);
const char *app_ble_fusion_state_name(uint8_t state);
const char *app_ble_event_type_name(uint8_t event_type);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_BLE_PROTOCOL_H__ */

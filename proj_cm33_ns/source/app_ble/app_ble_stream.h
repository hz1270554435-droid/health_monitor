#ifndef __APP_BLE_STREAM_H__
#define __APP_BLE_STREAM_H__

#include "cy_pdl.h"

#include "app_ble_types.h"

#if defined(__cplusplus)
extern "C" {
#endif

cy_rslt_t app_ble_stream_init(void);
cy_rslt_t app_ble_publish_realtime(
    const app_ble_realtime_sample_t *sample);
cy_rslt_t app_ble_publish_event(const app_ble_event_t *event);
void app_ble_stream_process(void);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_BLE_STREAM_H__ */

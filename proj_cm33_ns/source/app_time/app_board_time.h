#ifndef __APP_BOARD_TIME_H__
#define __APP_BOARD_TIME_H__

#include <stdbool.h>
#include <stdint.h>

#include "cy_pdl.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define APP_BOARD_TIME_SOURCE_NONE     (0u)
#define APP_BOARD_TIME_SOURCE_BLE      (1u)
#define APP_BOARD_TIME_SOURCE_BLE_FAKE (2u)
#define APP_BOARD_TIME_SOURCE_BOOTSTRAP (3u)

#define APP_BOARD_TIME_FLAG_VALID      (1u << 0)
#define APP_BOARD_TIME_FLAG_NVM_LOADED (1u << 1)
#define APP_BOARD_TIME_FLAG_NVM_SAVED  (1u << 2)
#define APP_BOARD_TIME_FLAG_BLE_SYNCED (1u << 3)

cy_rslt_t app_board_time_init(void);
cy_rslt_t app_board_time_calibrate_epoch_s(uint32_t epoch_s, uint32_t source);
bool app_board_time_now_epoch_s(uint32_t *epoch_s);
bool app_board_time_is_synced(void);
uint32_t app_board_time_get_offset_s(void);
uint32_t app_board_time_get_flags(void);
uint32_t app_board_time_get_source(void);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_BOARD_TIME_H__ */

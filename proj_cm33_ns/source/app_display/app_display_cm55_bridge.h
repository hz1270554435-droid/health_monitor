/*******************************************************************************
* File Name : app_display_cm55_bridge.h
*
* Description : CM33-side producer for the display snapshot bridge.
*******************************************************************************/

#ifndef __APP_DISPLAY_CM55_BRIDGE_H__
#define __APP_DISPLAY_CM55_BRIDGE_H__

#include "cy_result.h"
#include "app_display_types.h"

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * Initialize the display snapshot bridge region.
 * Call once during CM33 boot, before any publish.
 */
cy_rslt_t app_display_cm55_bridge_init(void);

/**
 * Publish a display snapshot to the CM55 bridge region.
 * Copies fields from e84_display_snapshot_t into the shared memory struct
 * with seq_begin/seq_end bracketing and __DMB().
 */
cy_rslt_t app_display_cm55_bridge_publish(
    const e84_display_snapshot_t *snapshot);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_DISPLAY_CM55_BRIDGE_H__ */

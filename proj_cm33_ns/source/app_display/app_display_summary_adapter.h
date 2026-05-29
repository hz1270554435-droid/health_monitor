/*******************************************************************************
* File Name : app_display_summary_adapter.h
*
* Description : Optional read-only summary adapter for the Display/UI layer.
*******************************************************************************/

#ifndef __APP_DISPLAY_SUMMARY_ADAPTER_H__
#define __APP_DISPLAY_SUMMARY_ADAPTER_H__

#include <stdint.h>

#include "cy_pdl.h"

#if defined(__cplusplus)
extern "C" {
#endif

cy_rslt_t app_display_summary_adapter_tick(uint32_t now_ms);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_DISPLAY_SUMMARY_ADAPTER_H__ */

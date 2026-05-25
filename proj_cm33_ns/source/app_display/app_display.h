/*******************************************************************************
* File Name : app_display.h
*
* Description : Optional Display/UI presentation layer entry points.
*
* Stage 1 is intentionally conservative:
* - disabled by default through APP_DISPLAY_ENABLE=0
* - CM33_NS only
* - queue driven from task context
* - null backend only; no MIPI-DSI/LCD/GUI stack is initialized here
*******************************************************************************/

#ifndef __APP_DISPLAY_H__
#define __APP_DISPLAY_H__

#include "cy_pdl.h"

#include "app_display_types.h"

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef APP_DISPLAY_ENABLE
#define APP_DISPLAY_ENABLE                       (0u)
#endif

#ifndef APP_DISPLAY_TASK_STACK_SIZE
#define APP_DISPLAY_TASK_STACK_SIZE              (1024u)
#endif

#ifndef APP_DISPLAY_TASK_PRIORITY
#define APP_DISPLAY_TASK_PRIORITY                (1u)
#endif

#ifndef APP_DISPLAY_QUEUE_DEPTH
#define APP_DISPLAY_QUEUE_DEPTH                  (4u)
#endif

#ifndef APP_DISPLAY_TASK_IDLE_MS
#define APP_DISPLAY_TASK_IDLE_MS                 (250u)
#endif

#ifndef APP_DISPLAY_NULL_LOG_MIN_PERIOD_MS
#define APP_DISPLAY_NULL_LOG_MIN_PERIOD_MS       (2000u)
#endif

#ifndef APP_DISPLAY_SMOKE_ENABLE
#define APP_DISPLAY_SMOKE_ENABLE                 (0u)
#endif

#ifndef APP_DISPLAY_SMOKE_SNAPSHOT_PERIOD_MS
#define APP_DISPLAY_SMOKE_SNAPSHOT_PERIOD_MS     (1000u)
#endif

#ifndef APP_DISPLAY_SMOKE_ALERT_PERIOD_MS
#define APP_DISPLAY_SMOKE_ALERT_PERIOD_MS        (12000u)
#endif

#ifndef APP_DISPLAY_SMOKE_ALERT_HOLD_MS
#define APP_DISPLAY_SMOKE_ALERT_HOLD_MS          (4000u)
#endif

#if ((APP_DISPLAY_ENABLE != 0u) && (APP_DISPLAY_ENABLE != 1u))
#error "Unsupported APP_DISPLAY_ENABLE"
#endif

#if ((APP_DISPLAY_SMOKE_ENABLE != 0u) && \
     (APP_DISPLAY_SMOKE_ENABLE != 1u))
#error "Unsupported APP_DISPLAY_SMOKE_ENABLE"
#endif

#if (APP_DISPLAY_ENABLE)

cy_rslt_t app_display_init(void);
cy_rslt_t app_display_start(void);
cy_rslt_t app_display_publish_snapshot(
    const e84_display_snapshot_t *snapshot);
cy_rslt_t app_display_raise_alert(e84_display_alert_t alert,
                                  uint8_t severity,
                                  uint8_t confidence,
                                  uint32_t flags);
cy_rslt_t app_display_clear_alert(e84_display_alert_t alert);

#else

static inline cy_rslt_t app_display_init(void)
{
    return CY_RSLT_SUCCESS;
}

static inline cy_rslt_t app_display_start(void)
{
    return CY_RSLT_SUCCESS;
}

static inline cy_rslt_t app_display_publish_snapshot(
    const e84_display_snapshot_t *snapshot)
{
    (void)snapshot;
    return CY_RSLT_SUCCESS;
}

static inline cy_rslt_t app_display_raise_alert(e84_display_alert_t alert,
                                                uint8_t severity,
                                                uint8_t confidence,
                                                uint32_t flags)
{
    (void)alert;
    (void)severity;
    (void)confidence;
    (void)flags;
    return CY_RSLT_SUCCESS;
}

static inline cy_rslt_t app_display_clear_alert(e84_display_alert_t alert)
{
    (void)alert;
    return CY_RSLT_SUCCESS;
}

#endif /* APP_DISPLAY_ENABLE */

#if defined(__cplusplus)
}
#endif

#endif /* __APP_DISPLAY_H__ */

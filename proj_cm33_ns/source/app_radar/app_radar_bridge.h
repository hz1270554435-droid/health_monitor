#ifndef __APP_RADAR_BRIDGE_H__
#define __APP_RADAR_BRIDGE_H__

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "cy_pdl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* ---------------------------------------------------------------------------
 * P1B-02 radar observation bridge.
 *
 * This module consumes LD6002 parser blocks, decodes them into a radar
 * observation accumulator, computes radar_quality via P1B-01 producer,
 * maps results to app_monitor_radar_input_t, and calls
 * app_monitor_summary_update_radar().
 *
 * Profile policy: bridge and test decoder are MUTUALLY EXCLUSIVE consumers
 * of the parser queue. Exactly one may call app_uart_radar_receive_block()
 * at runtime.
 *
 * This module does NOT:
 * - implement real fusion
 * - produce BLE/App real-source payloads
 * - change shared-memory ABI
 * - change BLE wire format
 * - implement App Night Summary
 * --------------------------------------------------------------------------- */

/* Build-time profile selection.
 *
 * APP_RADAR_BRIDGE_ENABLE=1: bridge task consumes parser queue, test decoder
 *                            is disabled (APP_UART_RADAR_TEST_ENABLE=0).
 * APP_RADAR_BRIDGE_ENABLE=0 (default): test decoder consumes parser queue,
 *                            bridge task is not created.
 *
 * These MUST be mutually exclusive. Do NOT enable both simultaneously.
 */
#ifndef APP_RADAR_BRIDGE_ENABLE
#define APP_RADAR_BRIDGE_ENABLE                 (0u)
#endif

/* Bridge task configuration. */
#define APP_RADAR_BRIDGE_TASK_STACK_SIZE        (2048u)
#define APP_RADAR_BRIDGE_TASK_PRIORITY          (configMAX_PRIORITIES - 2u)
#define APP_RADAR_BRIDGE_RX_WAIT_MS             (100u)
#define APP_RADAR_BRIDGE_LOG_MS                 (5000u)

/* LD6002 presence raw values (verified 2026-06-22 from real UART log).
 * LD6002 sends: 0 = no human, 1 = human present. */
#define APP_RADAR_BRIDGE_PRESENCE_UNKNOWN_RAW   (2u)
#define APP_RADAR_BRIDGE_PRESENCE_ABSENT_RAW    (0u)
#define APP_RADAR_BRIDGE_PRESENCE_PRESENT_RAW   (1u)

/* LD6002 TYPE definitions (mirrors app_uart_radar.c). */
#define APP_RADAR_BRIDGE_TYPE_FIRMWARE_STATUS    (0xFFFFu)
#define APP_RADAR_BRIDGE_TYPE_HUMAN_STATUS       (0x0F09u)
#define APP_RADAR_BRIDGE_TYPE_HUMAN_POSITION     (0x0A04u)
#define APP_RADAR_BRIDGE_TYPE_PHASE              (0x0A13u)
#define APP_RADAR_BRIDGE_TYPE_BREATH_RATE        (0x0A14u)
#define APP_RADAR_BRIDGE_TYPE_HEART_RATE         (0x0A15u)
#define APP_RADAR_BRIDGE_TYPE_TARGET_RANGE       (0x0A16u)
#define APP_RADAR_BRIDGE_TYPE_TRACK_POSITION     (0x0A17u)

/* ---------------------------------------------------------------------------
 * API
 * --------------------------------------------------------------------------- */

/**
 * Initialize and create the bridge task.
 * Must be called AFTER app_uart_radar_task_init().
 * Only creates the task when APP_RADAR_BRIDGE_ENABLE=1.
 *
 * @return CY_RSLT_SUCCESS on success.
 */
cy_rslt_t app_radar_bridge_init(void);

/**
 * Get the bridge task handle. NULL if bridge is not enabled.
 */
TaskHandle_t app_radar_bridge_get_task_handle(void);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_RADAR_BRIDGE_H__ */

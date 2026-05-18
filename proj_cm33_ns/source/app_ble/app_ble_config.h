#ifndef __APP_BLE_CONFIG_H__
#define __APP_BLE_CONFIG_H__

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef APP_BLE_ENABLE
#define APP_BLE_ENABLE                    (0u)
#endif

#ifndef APP_BLE_FAKE_DATA_ENABLE
#define APP_BLE_FAKE_DATA_ENABLE          (0u)
#endif

#ifndef APP_BLE_STACK_ENABLE
#define APP_BLE_STACK_ENABLE              (0u)
#endif

#ifndef APP_BLE_DIAG_ENABLE
#define APP_BLE_DIAG_ENABLE               (1u)
#endif

#ifndef APP_BLE_LOG_LEVEL
#define APP_BLE_LOG_LEVEL                 (1u)
#endif

#define APP_BLE_LOG_LEVEL_QUIET           (0u)
#define APP_BLE_LOG_LEVEL_STAT            (1u)
#define APP_BLE_LOG_LEVEL_DEBUG           (2u)

#define APP_BLE_DEVICE_NAME               "E84-HealthMonitor"
#define APP_BLE_PROTOCOL_MAGIC            (0xE8u)
#define APP_BLE_PROTOCOL_VERSION          (0x01u)

#define APP_BLE_REALTIME_PERIOD_MS        (1000u)
#define APP_BLE_EVENT_FAKE_PERIOD_MS      (15000u)
#define APP_BLE_STAT_PRINT_PERIOD_MS      (5000u)

#define APP_BLE_TASK_STACK_SIZE           (1024u)
#define APP_BLE_TASK_PRIORITY             (2u)

#define APP_BLE_REALTIME_QUEUE_DEPTH      (4u)
#define APP_BLE_EVENT_QUEUE_DEPTH         (8u)
#define APP_BLE_CMD_RAW_QUEUE_DEPTH       (4u)

#define APP_BLE_TIME_SYNC_MIN_EPOCH_S     (1577836800UL)
#define APP_BLE_TIME_SYNC_MAX_EPOCH_S     (4102444800UL)

#if ((APP_BLE_ENABLE != 0u) && (APP_BLE_ENABLE != 1u))
#error "Unsupported APP_BLE_ENABLE"
#endif

#if ((APP_BLE_FAKE_DATA_ENABLE != 0u) && (APP_BLE_FAKE_DATA_ENABLE != 1u))
#error "Unsupported APP_BLE_FAKE_DATA_ENABLE"
#endif

#if ((APP_BLE_STACK_ENABLE != 0u) && (APP_BLE_STACK_ENABLE != 1u))
#error "Unsupported APP_BLE_STACK_ENABLE"
#endif

#if ((APP_BLE_DIAG_ENABLE != 0u) && (APP_BLE_DIAG_ENABLE != 1u))
#error "Unsupported APP_BLE_DIAG_ENABLE"
#endif

#if ((APP_BLE_STACK_ENABLE != 0u) && (APP_BLE_ENABLE == 0u))
#error "APP_BLE_STACK_ENABLE requires APP_BLE_ENABLE"
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __APP_BLE_CONFIG_H__ */

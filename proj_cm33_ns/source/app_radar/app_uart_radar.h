#ifndef __APP_UART_RADAR_H__
#define __APP_UART_RADAR_H__

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* LD6002 TF frame format:
 * SOF(1) + ID(2, BE) + LEN(2, BE) + TYPE(2, BE) + HEAD_CKSUM(1)
 * + DATA(LEN, LE payload fields) + DATA_CKSUM(1, only when LEN > 0).
 */
#define APP_UART_RADAR_SOF                         (0x01u)
#define APP_UART_RADAR_HEADER_SIZE                 (8u)
#define APP_UART_RADAR_DATA_CKSUM_SIZE             (1u)
#define APP_UART_RADAR_MAX_DATA_SIZE               (1024u)
#define APP_UART_RADAR_MAX_FRAME_SIZE              (APP_UART_RADAR_HEADER_SIZE + \
                                                    APP_UART_RADAR_MAX_DATA_SIZE + \
                                                    APP_UART_RADAR_DATA_CKSUM_SIZE)
#define APP_UART_RADAR_BLOCK_COUNT                 (6u)
#define APP_UART_RADAR_QUEUE_LENGTH                (APP_UART_RADAR_BLOCK_COUNT)
#define APP_UART_RADAR_INVALID_BLOCK_INDEX         (0xFFu)

#define APP_UART_RADAR_TASK_STACK_SIZE             (1024u)
#define APP_UART_RADAR_TASK_PRIORITY               (configMAX_PRIORITIES - 3u)
#define APP_UART_RADAR_RX_POLL_MS                  (1u)

/* Test task: consumes the radar queue and prints decoded summary through the
 * existing retarget/debug UART. Do not enable another real inference consumer
 * on the same radar queue at the same time.
 */
#ifndef APP_UART_RADAR_TEST_ENABLE
#define APP_UART_RADAR_TEST_ENABLE                 (1u)
#endif

#define APP_UART_RADAR_TEST_TASK_STACK_SIZE        (1024u)
#define APP_UART_RADAR_TEST_TASK_PRIORITY          (APP_UART_RADAR_TASK_PRIORITY - 1u)
#define APP_UART_RADAR_TEST_PRINT_MS               (1000u)
#define APP_UART_RADAR_TEST_RX_WAIT_MS             (100u)

#ifndef APP_UART_RADAR_TEST_PRINT_EACH_BLOCK
#define APP_UART_RADAR_TEST_PRINT_EACH_BLOCK       (1u)
#endif

#ifndef APP_UART_RADAR_TEST_HEX_DUMP_BYTES
#define APP_UART_RADAR_TEST_HEX_DUMP_BYTES         (64u)
#endif

#ifndef APP_UART_RADAR_HW
#define APP_UART_RADAR_HW                          Radar_uart_HW
#endif

#ifndef APP_UART_RADAR_CONFIG
#define APP_UART_RADAR_CONFIG                      Radar_uart_config
#endif

#ifndef APP_UART_RADAR_RX_PORT
#define APP_UART_RADAR_RX_PORT                     P17_0_PORT
#endif

#ifndef APP_UART_RADAR_RX_PIN
#define APP_UART_RADAR_RX_PIN                      P17_0_PIN
#endif

#ifndef APP_UART_RADAR_RX_HSIOM
#define APP_UART_RADAR_RX_HSIOM                    P17_0_SCB5_UART_RX
#endif

/* Radar is now routed on SCB5 with RX=P17_0 and TX=P17_1. Keep TX enabled so
 * later code can send LD6002 commands without another pin-setup change.
 */
#ifndef APP_UART_RADAR_USE_TX_PIN
#define APP_UART_RADAR_USE_TX_PIN                  (1u)
#endif

#ifndef APP_UART_RADAR_TX_PORT
#define APP_UART_RADAR_TX_PORT                     P17_1_PORT
#endif

#ifndef APP_UART_RADAR_TX_PIN
#define APP_UART_RADAR_TX_PIN                      P17_1_PIN
#endif

#ifndef APP_UART_RADAR_TX_HSIOM
#define APP_UART_RADAR_TX_HSIOM                    P17_1_SCB5_UART_TX
#endif

typedef struct
{
    /* Pointer to the complete TF frame. Valid until app_uart_radar_release_block(). */
    const uint8_t *frame;
    /* Pointer to DATA payload. NULL when data_len is 0. */
    const uint8_t *data;
    uint16_t frame_len;
    uint16_t data_len;
    uint16_t type;
    uint16_t frame_id;
    uint8_t block_index;
    uint32_t sequence;
    uint32_t dropped_count;
} app_uart_radar_block_t;

cy_rslt_t app_uart_radar_task_init(void);
cy_rslt_t app_uart_radar_test_task_init(void);
void app_uart_radar_task(void *pvParameters);
void app_uart_radar_test_task(void *pvParameters);
QueueHandle_t app_uart_radar_get_queue(void);
bool app_uart_radar_receive_block(app_uart_radar_block_t *block,
                                  TickType_t ticks_to_wait);
void app_uart_radar_release_block(uint8_t block_index);
uint32_t app_uart_radar_get_dropped_count(void);
uint32_t app_uart_radar_get_bad_frame_count(void);
uint32_t app_uart_radar_get_uart_error_count(void);
uint32_t app_uart_radar_get_received_count(void);
uint32_t app_uart_radar_get_resync_count(void);
uint16_t app_uart_radar_get_max_frame_len(void);
void radar_data_test(void);

cy_rslt_t APP_UART_RADAR_Init(void);
cy_rslt_t APP_UART_RADAR_GetData(uint32_t *receive_buffer);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_UART_RADAR_H__ */

#include "app_uart_radar.h"

#include <stdio.h>
#include <string.h>

/* LD6002 breath/heart project message types used by the test decoder. */
#define APP_UART_RADAR_TYPE_FIRMWARE_STATUS        (0xFFFFu)
#define APP_UART_RADAR_TYPE_HUMAN_STATUS           (0x0F09u)
#define APP_UART_RADAR_TYPE_HUMAN_POSITION         (0x0A04u)
#define APP_UART_RADAR_TYPE_PHASE                  (0x0A13u)
#define APP_UART_RADAR_TYPE_BREATH_RATE            (0x0A14u)
#define APP_UART_RADAR_TYPE_HEART_RATE             (0x0A15u)
#define APP_UART_RADAR_TYPE_TARGET_RANGE           (0x0A16u)
#define APP_UART_RADAR_TYPE_TRACK_POSITION         (0x0A17u)

typedef enum
{
    APP_UART_RADAR_ERROR_NONE = 0,
    APP_UART_RADAR_ERROR_BACKPRESSURE,
    APP_UART_RADAR_ERROR_LEN,
    APP_UART_RADAR_ERROR_HEAD_CKSUM,
    APP_UART_RADAR_ERROR_DATA_CKSUM,
    APP_UART_RADAR_ERROR_QUEUE_FULL,
    APP_UART_RADAR_ERROR_UART
} app_uart_radar_error_t;

/* 每个静态缓冲块的所有权状态。READY 状态必须由消费者 release 后才能复用。 */
typedef enum
{
    APP_UART_RADAR_BLOCK_FREE = 0,
    APP_UART_RADAR_BLOCK_FILLING,
    APP_UART_RADAR_BLOCK_READY
} app_uart_radar_block_state_t;

/* 串口字节流解析状态机，对应 LD6002 TF 帧的每个字段。 */
typedef enum
{
    APP_UART_RADAR_WAIT_SOF = 0,
    APP_UART_RADAR_READ_ID_MSB,
    APP_UART_RADAR_READ_ID_LSB,
    APP_UART_RADAR_READ_LEN_MSB,
    APP_UART_RADAR_READ_LEN_LSB,
    APP_UART_RADAR_READ_TYPE_MSB,
    APP_UART_RADAR_READ_TYPE_LSB,
    APP_UART_RADAR_READ_HEAD_CKSUM,
    APP_UART_RADAR_READ_DATA,
    APP_UART_RADAR_READ_DATA_CKSUM
} app_uart_radar_rx_state_t;

/* 当前正在组包的解析上下文。 */
typedef struct
{
    app_uart_radar_rx_state_t state;
    uint8_t block_index;
    uint16_t frame_pos;
    uint16_t data_pos;
    uint16_t data_len;
    uint16_t frame_id;
    uint16_t type;
    uint8_t head_xor;
    uint8_t data_xor;
} app_uart_radar_parser_t;

typedef struct
{
    uint32_t total_frames;
    uint32_t invalid_blocks;
    uint32_t sequence_gaps;
    uint32_t total_bytes;
    uint32_t expected_sequence;
    bool has_expected_sequence;
    uint16_t last_frame_id;
    uint16_t last_type;
    uint16_t last_data_len;
    uint16_t last_human_state;
    int32_t last_breath_rate_centi;
    int32_t last_heart_rate_centi;
    uint32_t last_range_flag;
    int32_t last_range_centi;
    int32_t last_x_centi;
    int32_t last_y_centi;
    int32_t last_z_centi;
    uint32_t last_target_count;
    uint8_t last_project_id;
    uint8_t last_major_version;
    uint8_t last_sub_version;
    uint8_t last_modified_version;
    bool has_human_state;
    bool has_breath_rate;
    bool has_heart_rate;
    bool has_range;
    bool has_track_position;
    bool has_target_count;
    bool has_firmware_status;
} app_uart_radar_test_stats_t;

static cy_stc_scb_uart_context_t radar_uart_context;
static QueueHandle_t radar_block_queue = NULL;
static TaskHandle_t radar_task_handle = NULL;
static TaskHandle_t radar_test_task_handle = NULL;

/* 静态帧池：避免在串口接收路径里动态申请内存。
 * 放到 shared SoC memory，后续 CM55 推理侧如需访问可以继续沿用这个地址。
 */
static uint8_t radar_frame_buffer[APP_UART_RADAR_BLOCK_COUNT]
                                 [APP_UART_RADAR_MAX_FRAME_SIZE]
    __attribute__((section(".cy_shared_socmem"))) = {0};

/* FREE block 采用简单栈管理；队列只传递 app_uart_radar_block_t 描述符。 */
static app_uart_radar_block_state_t block_state[APP_UART_RADAR_BLOCK_COUNT];
static uint8_t free_block_stack[APP_UART_RADAR_BLOCK_COUNT];
static uint8_t free_block_count;
static app_uart_radar_parser_t parser = {
    .state = APP_UART_RADAR_WAIT_SOF,
    .block_index = APP_UART_RADAR_INVALID_BLOCK_INDEX
};

static bool radar_uart_initialized;
static bool radar_backpressure_active;
static app_uart_radar_error_t radar_last_error;
static uint32_t radar_received_count;
static uint32_t radar_frame_sequence;
static uint32_t radar_dropped_count;
static uint32_t radar_bad_frame_count;
static uint32_t radar_uart_error_count;
static uint32_t radar_resync_count;
static uint16_t radar_max_frame_len;

static cy_rslt_t app_uart_radar_hw_init(void);
static void app_uart_radar_reset_stream_state(void);
static bool app_uart_radar_claim_rx_block(void);
static void app_uart_radar_enter_backpressure(void);
static void app_uart_radar_note_bad_frame(app_uart_radar_error_t error);
static void app_uart_radar_parser_reset(void);
static void app_uart_radar_start_frame(uint8_t byte);
static void app_uart_radar_store_header_byte(uint8_t byte);
static void app_uart_radar_publish_current_frame(void);
static void app_uart_radar_process_byte(uint8_t byte);
static void app_uart_radar_poll_uart(void);
static uint16_t app_uart_radar_read_le_u16(const uint8_t *data);
static uint32_t app_uart_radar_read_le_u32(const uint8_t *data);
static int32_t app_uart_radar_float_to_centi(const uint8_t *data);
static void app_uart_radar_print_centi(int32_t value);
static uint32_t app_uart_radar_checksum_u8(const uint8_t *data,
                                           uint16_t len);
static bool app_uart_radar_descriptor_is_valid(const app_uart_radar_block_t *block);
static void app_uart_radar_test_print_block(const app_uart_radar_block_t *block,
                                            bool valid);
static void app_uart_radar_test_update_stats(app_uart_radar_test_stats_t *stats,
                                             const app_uart_radar_block_t *block,
                                             bool valid);
static const char *app_uart_radar_error_to_string(app_uart_radar_error_t error);
static void app_uart_radar_test_print_stats(const app_uart_radar_test_stats_t *stats,
                                            uint32_t delta_frames,
                                            uint32_t delta_invalid,
                                            uint32_t delta_gaps,
                                            uint32_t elapsed_ms);

cy_rslt_t app_uart_radar_task_init(void)
{
    BaseType_t ret;

    if (NULL != radar_task_handle)
    {
        return CY_RSLT_SUCCESS;
    }

    /* 该队列由雷达接收任务写入，由测试任务或正式推理任务读取。
     * 不要让两个消费者同时读取同一个队列，否则同一帧会被其中一个任务“抢走”。
     */
    if (NULL == radar_block_queue)
    {
        radar_block_queue = xQueueCreate(APP_UART_RADAR_QUEUE_LENGTH,
                                         sizeof(app_uart_radar_block_t));
        if (NULL == radar_block_queue)
        {
            return CY_RSLT_TYPE_ERROR;
        }
    }

    ret = xTaskCreate(app_uart_radar_task,
                      "app_uart_radar",
                      APP_UART_RADAR_TASK_STACK_SIZE,
                      NULL,
                      APP_UART_RADAR_TASK_PRIORITY,
                      &radar_task_handle);

    return (pdPASS == ret) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
}

cy_rslt_t app_uart_radar_test_task_init(void)
{
    BaseType_t ret;

    if (NULL != radar_test_task_handle)
    {
        return CY_RSLT_SUCCESS;
    }

    ret = xTaskCreate(app_uart_radar_test_task,
                      "radar_test",
                      APP_UART_RADAR_TEST_TASK_STACK_SIZE,
                      NULL,
                      APP_UART_RADAR_TEST_TASK_PRIORITY,
                      &radar_test_task_handle);

    return (pdPASS == ret) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
}

QueueHandle_t app_uart_radar_get_queue(void)
{
    return radar_block_queue;
}

bool app_uart_radar_receive_block(app_uart_radar_block_t *block,
                                  TickType_t ticks_to_wait)
{
    if ((NULL == radar_block_queue) || (NULL == block))
    {
        return false;
    }

    return (pdPASS == xQueueReceive(radar_block_queue, block, ticks_to_wait));
}

void app_uart_radar_release_block(uint8_t block_index)
{
    if (APP_UART_RADAR_BLOCK_COUNT <= block_index)
    {
        return;
    }

    taskENTER_CRITICAL();
    if (APP_UART_RADAR_BLOCK_READY == block_state[block_index])
    {
        block_state[block_index] = APP_UART_RADAR_BLOCK_FREE;
        if (APP_UART_RADAR_BLOCK_COUNT > free_block_count)
        {
            free_block_stack[free_block_count] = block_index;
            free_block_count++;
        }
    }
    taskEXIT_CRITICAL();
}

uint32_t app_uart_radar_get_dropped_count(void)
{
    return radar_dropped_count;
}

uint32_t app_uart_radar_get_bad_frame_count(void)
{
    return radar_bad_frame_count;
}

uint32_t app_uart_radar_get_uart_error_count(void)
{
    return radar_uart_error_count;
}

uint32_t app_uart_radar_get_received_count(void)
{
    return radar_received_count;
}

uint32_t app_uart_radar_get_resync_count(void)
{
    return radar_resync_count;
}

uint16_t app_uart_radar_get_max_frame_len(void)
{
    return radar_max_frame_len;
}

void app_uart_radar_task(void *pvParameters)
{
    (void)pvParameters;

    /* Initialize only the SCB5 radar UART. Debug logs continue through
     * retarget/debug UART so test output never shares the radar port.
     */
    if (CY_RSLT_SUCCESS != app_uart_radar_hw_init())
    {
        configASSERT(0);
        vTaskDelete(NULL);
    }

    app_uart_radar_reset_stream_state();

    for (;;)
    {
        app_uart_radar_poll_uart();
        vTaskDelay(pdMS_TO_TICKS(APP_UART_RADAR_RX_POLL_MS));
    }
}

void app_uart_radar_test_task(void *pvParameters)
{
    (void)pvParameters;

    radar_data_test();
}

cy_rslt_t APP_UART_RADAR_Init(void)
{
    return app_uart_radar_task_init();
}

cy_rslt_t APP_UART_RADAR_GetData(uint32_t *receive_buffer)
{
    app_uart_radar_block_t block;

    if (NULL == receive_buffer)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    if (!app_uart_radar_receive_block(&block, portMAX_DELAY))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    *receive_buffer = ((uint32_t)block.type << 16) | block.data_len;
    app_uart_radar_release_block(block.block_index);

    return CY_RSLT_SUCCESS;
}

static cy_rslt_t app_uart_radar_hw_init(void)
{
    cy_rslt_t result;

    if (radar_uart_initialized)
    {
        return CY_RSLT_SUCCESS;
    }

    /* 串口占用保护：如果工程配置误把 LD6002 串口指向 debug UART，
     * 这里直接失败，避免雷达接收初始化破坏 printf/下载调试串口。
     */
#if defined(CYBSP_DEBUG_UART_HW)
    if (APP_UART_RADAR_HW == CYBSP_DEBUG_UART_HW)
    {
        return CY_RSLT_TYPE_ERROR;
    }
#endif

    const cy_stc_gpio_pin_config_t rx_pin_config = {
        .outVal = 1,
        .driveMode = CY_GPIO_DM_HIGHZ,
        .hsiom = APP_UART_RADAR_RX_HSIOM,
        .intEdge = CY_GPIO_INTR_DISABLE,
        .intMask = 0UL,
        .vtrip = CY_GPIO_VTRIP_CMOS,
        .slewRate = CY_GPIO_SLEW_FAST,
        .driveSel = CY_GPIO_DRIVE_1_2,
        .vregEn = 0UL,
        .ibufMode = 0UL,
        .vtripSel = 0UL,
        .vrefSel = 0UL,
        .vohSel = 0UL,
        .pullUpRes = CY_GPIO_PULLUP_RES_DISABLE,
        .nonSec = 1,
    };

#if (APP_UART_RADAR_USE_TX_PIN)
    const cy_stc_gpio_pin_config_t tx_pin_config = {
        .outVal = 1,
        .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
        .hsiom = APP_UART_RADAR_TX_HSIOM,
        .intEdge = CY_GPIO_INTR_DISABLE,
        .intMask = 0UL,
        .vtrip = CY_GPIO_VTRIP_CMOS,
        .slewRate = CY_GPIO_SLEW_FAST,
        .driveSel = CY_GPIO_DRIVE_1_2,
        .vregEn = 0UL,
        .ibufMode = 0UL,
        .vtripSel = 0UL,
        .vrefSel = 0UL,
        .vohSel = 0UL,
        .pullUpRes = CY_GPIO_PULLUP_RES_DISABLE,
        .nonSec = 1,
    };
#endif

    /* 只强制配置 LD6002 RX 引脚。TX 默认不占用，避免和板级其它功能复用冲突；
     * 以后如果要给雷达下发命令，可将 APP_UART_RADAR_USE_TX_PIN 置 1。
     */
    (void)Cy_GPIO_Pin_Init(APP_UART_RADAR_RX_PORT,
                           APP_UART_RADAR_RX_PIN,
                           &rx_pin_config);
#if (APP_UART_RADAR_USE_TX_PIN)
    (void)Cy_GPIO_Pin_Init(APP_UART_RADAR_TX_PORT,
                           APP_UART_RADAR_TX_PIN,
                           &tx_pin_config);
#endif

    result = (cy_rslt_t)Cy_SCB_UART_Init(APP_UART_RADAR_HW,
                                         &APP_UART_RADAR_CONFIG,
                                         &radar_uart_context);
    if (CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    Cy_SCB_UART_ClearRxFifo(APP_UART_RADAR_HW);
    Cy_SCB_UART_Enable(APP_UART_RADAR_HW);
    radar_uart_initialized = true;

    return CY_RSLT_SUCCESS;
}

static void app_uart_radar_reset_stream_state(void)
{
    taskENTER_CRITICAL();

    memset((void *)block_state, 0, sizeof(block_state));
    free_block_count = 0;
    for (uint8_t i = APP_UART_RADAR_BLOCK_COUNT; i > 0u; i--)
    {
        free_block_stack[free_block_count] = (uint8_t)(i - 1u);
        free_block_count++;
    }

    if (NULL != radar_block_queue)
    {
        (void)xQueueReset(radar_block_queue);
    }

    radar_frame_sequence = 0;
    radar_received_count = 0;
    radar_dropped_count = 0;
    radar_bad_frame_count = 0;
    radar_uart_error_count = 0;
    radar_resync_count = 0;
    radar_max_frame_len = 0;
    radar_backpressure_active = false;
    radar_last_error = APP_UART_RADAR_ERROR_NONE;
    parser.block_index = APP_UART_RADAR_INVALID_BLOCK_INDEX;

    taskEXIT_CRITICAL();

    (void)app_uart_radar_claim_rx_block();
}

static bool app_uart_radar_claim_rx_block(void)
{
    uint8_t block_index = APP_UART_RADAR_INVALID_BLOCK_INDEX;
    bool claimed = false;

    taskENTER_CRITICAL();
    if (0u < free_block_count)
    {
        free_block_count--;
        block_index = free_block_stack[free_block_count];
        block_state[block_index] = APP_UART_RADAR_BLOCK_FILLING;
        claimed = true;
    }
    taskEXIT_CRITICAL();

    if (claimed)
    {
        radar_backpressure_active = false;
        parser.block_index = block_index;
        app_uart_radar_parser_reset();
    }

    return claimed;
}

static void app_uart_radar_enter_backpressure(void)
{
    if (!radar_backpressure_active)
    {
        radar_dropped_count++;
        radar_last_error = APP_UART_RADAR_ERROR_BACKPRESSURE;
        radar_backpressure_active = true;
    }
}

static void app_uart_radar_note_bad_frame(app_uart_radar_error_t error)
{
    radar_bad_frame_count++;
    radar_resync_count++;
    radar_last_error = error;
    app_uart_radar_parser_reset();
}

static void app_uart_radar_parser_reset(void)
{
    parser.state = APP_UART_RADAR_WAIT_SOF;
    parser.frame_pos = 0;
    parser.data_pos = 0;
    parser.data_len = 0;
    parser.frame_id = 0;
    parser.type = 0;
    parser.head_xor = 0;
    parser.data_xor = 0;
}

static void app_uart_radar_start_frame(uint8_t byte)
{
    uint8_t *frame = radar_frame_buffer[parser.block_index];

    app_uart_radar_parser_reset();
    frame[parser.frame_pos++] = byte;
    parser.head_xor = byte;
    parser.state = APP_UART_RADAR_READ_ID_MSB;
}

static void app_uart_radar_store_header_byte(uint8_t byte)
{
    uint8_t *frame = radar_frame_buffer[parser.block_index];

    frame[parser.frame_pos++] = byte;
    parser.head_xor ^= byte;
}

static void app_uart_radar_publish_current_frame(void)
{
    uint8_t block_index = parser.block_index;
    app_uart_radar_block_t block = {
        .frame = &radar_frame_buffer[block_index][0],
        .data = (0u < parser.data_len) ?
                &radar_frame_buffer[block_index][APP_UART_RADAR_HEADER_SIZE] :
                NULL,
        .frame_len = parser.frame_pos,
        .data_len = parser.data_len,
        .type = parser.type,
        .frame_id = parser.frame_id,
        .block_index = block_index,
        .sequence = radar_frame_sequence++,
        .dropped_count = radar_dropped_count,
    };

    taskENTER_CRITICAL();
    block_state[block_index] = APP_UART_RADAR_BLOCK_READY;
    taskEXIT_CRITICAL();

    if ((NULL == radar_block_queue) ||
        (pdPASS != xQueueSend(radar_block_queue, &block, 0)))
    {
        radar_dropped_count++;
        radar_last_error = APP_UART_RADAR_ERROR_QUEUE_FULL;
        app_uart_radar_release_block(block_index);
    }
    else
    {
        radar_received_count++;
        if (radar_max_frame_len < block.frame_len)
        {
            radar_max_frame_len = block.frame_len;
        }
    }

    parser.block_index = APP_UART_RADAR_INVALID_BLOCK_INDEX;
    if (!app_uart_radar_claim_rx_block())
    {
        app_uart_radar_enter_backpressure();
        app_uart_radar_parser_reset();
    }
}

static void app_uart_radar_process_byte(uint8_t byte)
{
    uint8_t *frame;
    uint8_t expected_cksum;

    if ((APP_UART_RADAR_INVALID_BLOCK_INDEX == parser.block_index) &&
        !app_uart_radar_claim_rx_block())
    {
        app_uart_radar_enter_backpressure();
        return;
    }

    frame = radar_frame_buffer[parser.block_index];

    switch (parser.state)
    {
        case APP_UART_RADAR_WAIT_SOF:
            if (APP_UART_RADAR_SOF == byte)
            {
                app_uart_radar_start_frame(byte);
            }
            break;

        case APP_UART_RADAR_READ_ID_MSB:
            app_uart_radar_store_header_byte(byte);
            parser.frame_id = ((uint16_t)byte << 8);
            parser.state = APP_UART_RADAR_READ_ID_LSB;
            break;

        case APP_UART_RADAR_READ_ID_LSB:
            app_uart_radar_store_header_byte(byte);
            parser.frame_id |= byte;
            parser.state = APP_UART_RADAR_READ_LEN_MSB;
            break;

        case APP_UART_RADAR_READ_LEN_MSB:
            app_uart_radar_store_header_byte(byte);
            parser.data_len = ((uint16_t)byte << 8);
            parser.state = APP_UART_RADAR_READ_LEN_LSB;
            break;

        case APP_UART_RADAR_READ_LEN_LSB:
            app_uart_radar_store_header_byte(byte);
            parser.data_len |= byte;
            if (APP_UART_RADAR_MAX_DATA_SIZE < parser.data_len)
            {
                app_uart_radar_note_bad_frame(APP_UART_RADAR_ERROR_LEN);
            }
            else
            {
                parser.state = APP_UART_RADAR_READ_TYPE_MSB;
            }
            break;

        case APP_UART_RADAR_READ_TYPE_MSB:
            app_uart_radar_store_header_byte(byte);
            parser.type = ((uint16_t)byte << 8);
            parser.state = APP_UART_RADAR_READ_TYPE_LSB;
            break;

        case APP_UART_RADAR_READ_TYPE_LSB:
            app_uart_radar_store_header_byte(byte);
            parser.type |= byte;
            parser.state = APP_UART_RADAR_READ_HEAD_CKSUM;
            break;

        case APP_UART_RADAR_READ_HEAD_CKSUM:
            expected_cksum = (uint8_t)(~parser.head_xor);
            if (expected_cksum != byte)
            {
                app_uart_radar_note_bad_frame(APP_UART_RADAR_ERROR_HEAD_CKSUM);
                if (APP_UART_RADAR_SOF == byte)
                {
                    app_uart_radar_start_frame(byte);
                }
                break;
            }

            frame[parser.frame_pos++] = byte;
            if (0u == parser.data_len)
            {
                app_uart_radar_publish_current_frame();
            }
            else
            {
                parser.data_pos = 0;
                parser.data_xor = 0;
                parser.state = APP_UART_RADAR_READ_DATA;
            }
            break;

        case APP_UART_RADAR_READ_DATA:
            frame[parser.frame_pos++] = byte;
            parser.data_xor ^= byte;
            parser.data_pos++;
            if (parser.data_len <= parser.data_pos)
            {
                parser.state = APP_UART_RADAR_READ_DATA_CKSUM;
            }
            break;

        case APP_UART_RADAR_READ_DATA_CKSUM:
            expected_cksum = (uint8_t)(~parser.data_xor);
            if (expected_cksum == byte)
            {
                frame[parser.frame_pos++] = byte;
                app_uart_radar_publish_current_frame();
            }
            else
            {
                app_uart_radar_note_bad_frame(APP_UART_RADAR_ERROR_DATA_CKSUM);
                if (APP_UART_RADAR_SOF == byte)
                {
                    app_uart_radar_start_frame(byte);
                }
            }
            break;

        default:
            app_uart_radar_parser_reset();
            break;
    }
}

static void app_uart_radar_poll_uart(void)
{
    uint32_t rx_status = Cy_SCB_UART_GetRxFifoStatus(APP_UART_RADAR_HW);
    const uint32_t rx_error_mask = CY_SCB_UART_RX_OVERFLOW |
                                   CY_SCB_UART_RX_UNDERFLOW |
                                   CY_SCB_UART_RX_ERR_FRAME |
                                   CY_SCB_UART_RX_ERR_PARITY;

    if (0u != (rx_status & rx_error_mask))
    {
        radar_uart_error_count++;
        app_uart_radar_note_bad_frame(APP_UART_RADAR_ERROR_UART);
        if (0u != (rx_status & (CY_SCB_UART_RX_OVERFLOW |
                                CY_SCB_UART_RX_ERR_FRAME |
                                CY_SCB_UART_RX_ERR_PARITY)))
        {
            Cy_SCB_UART_ClearRxFifo(APP_UART_RADAR_HW);
        }
        Cy_SCB_UART_ClearRxFifoStatus(APP_UART_RADAR_HW,
                                      rx_status & rx_error_mask);
    }

    while (0u < Cy_SCB_UART_GetNumInRxFifo(APP_UART_RADAR_HW))
    {
        uint32_t value = Cy_SCB_UART_Get(APP_UART_RADAR_HW);
        if (CY_SCB_UART_RX_NO_DATA != value)
        {
            app_uart_radar_process_byte((uint8_t)value);
        }
    }
}

static uint16_t app_uart_radar_read_le_u16(const uint8_t *data)
{
    return ((uint16_t)data[1] << 8) | data[0];
}

static uint32_t app_uart_radar_read_le_u32(const uint8_t *data)
{
    return ((uint32_t)data[3] << 24) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[1] << 8) |
           data[0];
}

static int32_t app_uart_radar_float_to_centi(const uint8_t *data)
{
    uint32_t raw = app_uart_radar_read_le_u32(data);
    float value;

    memcpy(&value, &raw, sizeof(value));

    if (0.0f <= value)
    {
        return (int32_t)((value * 100.0f) + 0.5f);
    }
    return (int32_t)((value * 100.0f) - 0.5f);
}

static void app_uart_radar_print_centi(int32_t value)
{
    int32_t integer = value / 100;
    int32_t fraction = value % 100;

    if (0 > fraction)
    {
        fraction = -fraction;
    }

    printf("%ld.%02lu", (long)integer, (unsigned long)fraction);
}

static uint32_t app_uart_radar_checksum_u8(const uint8_t *data,
                                           uint16_t len)
{
    uint32_t checksum = 0;

    if (NULL == data)
    {
        return 0;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        checksum += data[i];
    }

    return checksum;
}

static bool app_uart_radar_descriptor_is_valid(const app_uart_radar_block_t *block)
{
    uint16_t expected_len;

    if ((NULL == block) ||
        (APP_UART_RADAR_BLOCK_COUNT <= block->block_index) ||
        (&radar_frame_buffer[block->block_index][0] != block->frame) ||
        (APP_UART_RADAR_MAX_DATA_SIZE < block->data_len))
    {
        return false;
    }

    expected_len = APP_UART_RADAR_HEADER_SIZE + block->data_len;
    if (0u < block->data_len)
    {
        expected_len += APP_UART_RADAR_DATA_CKSUM_SIZE;
        if (&block->frame[APP_UART_RADAR_HEADER_SIZE] != block->data)
        {
            return false;
        }
    }
    else if (NULL != block->data)
    {
        return false;
    }

    return (expected_len == block->frame_len);
}

static void app_uart_radar_test_print_block(const app_uart_radar_block_t *block,
                                            bool valid)
{
    uint16_t dump_len = 0;
    uint32_t checksum = 0;

    if (NULL == block)
    {
        printf("[RADAR_BLOCK] invalid=null\r\n");
        return;
    }

    if (valid)
    {
        dump_len = block->frame_len;
        if (APP_UART_RADAR_TEST_HEX_DUMP_BYTES < dump_len)
        {
            dump_len = APP_UART_RADAR_TEST_HEX_DUMP_BYTES;
        }
        checksum = app_uart_radar_checksum_u8(block->frame, block->frame_len);
    }

    printf("[RADAR_BLOCK] seq=%lu, index=%u, valid=%u, id=0x%04x, "
           "type=0x%04x, frame_len=%u, data_len=%u, driver_drop=%lu, "
           "msg_drop=%lu, checksum=0x%08lx",
           (unsigned long)block->sequence,
           (unsigned int)block->block_index,
           valid ? 1u : 0u,
           (unsigned int)block->frame_id,
           (unsigned int)block->type,
           (unsigned int)block->frame_len,
           (unsigned int)block->data_len,
           (unsigned long)app_uart_radar_get_dropped_count(),
           (unsigned long)block->dropped_count,
           (unsigned long)checksum);

#if (APP_UART_RADAR_TEST_HEX_DUMP_BYTES > 0u)
    if (valid && (0u < dump_len))
    {
        printf(", frame=");
        for (uint16_t i = 0; i < dump_len; i++)
        {
            printf("%02x", (unsigned int)block->frame[i]);
            if ((uint16_t)(i + 1u) < dump_len)
            {
                printf(" ");
            }
        }
        if (dump_len < block->frame_len)
        {
            printf(" ...");
        }
    }
#endif

    printf("\r\n");
}

static void app_uart_radar_test_update_stats(app_uart_radar_test_stats_t *stats,
                                             const app_uart_radar_block_t *block,
                                             bool valid)
{
    const uint8_t *data;

    if (!valid)
    {
        stats->invalid_blocks++;
        return;
    }

    if (stats->has_expected_sequence &&
        (block->sequence != stats->expected_sequence))
    {
        stats->sequence_gaps++;
    }
    stats->expected_sequence = block->sequence + 1u;
    stats->has_expected_sequence = true;

    stats->total_frames++;
    stats->total_bytes += block->frame_len;
    stats->last_frame_id = block->frame_id;
    stats->last_type = block->type;
    stats->last_data_len = block->data_len;

    data = block->data;
    switch (block->type)
    {
        case APP_UART_RADAR_TYPE_FIRMWARE_STATUS:
            if (4u <= block->data_len)
            {
                stats->last_project_id = data[0];
                stats->last_major_version = data[1];
                stats->last_sub_version = data[2];
                stats->last_modified_version = data[3];
                stats->has_firmware_status = true;
            }
            break;

        case APP_UART_RADAR_TYPE_HUMAN_STATUS:
            if (2u <= block->data_len)
            {
                stats->last_human_state = app_uart_radar_read_le_u16(data);
                stats->has_human_state = true;
            }
            break;

        case APP_UART_RADAR_TYPE_HUMAN_POSITION:
            if (4u <= block->data_len)
            {
                stats->last_target_count = app_uart_radar_read_le_u32(data);
                stats->has_target_count = true;
            }
            break;

        case APP_UART_RADAR_TYPE_BREATH_RATE:
            if (4u <= block->data_len)
            {
                stats->last_breath_rate_centi =
                    app_uart_radar_float_to_centi(data);
                stats->has_breath_rate = true;
            }
            break;

        case APP_UART_RADAR_TYPE_HEART_RATE:
            if (4u <= block->data_len)
            {
                stats->last_heart_rate_centi =
                    app_uart_radar_float_to_centi(data);
                stats->has_heart_rate = true;
            }
            break;

        case APP_UART_RADAR_TYPE_TARGET_RANGE:
            if (8u <= block->data_len)
            {
                stats->last_range_flag = app_uart_radar_read_le_u32(data);
                stats->last_range_centi =
                    app_uart_radar_float_to_centi(&data[4]);
                stats->has_range = true;
            }
            break;

        case APP_UART_RADAR_TYPE_TRACK_POSITION:
            if (12u <= block->data_len)
            {
                stats->last_x_centi = app_uart_radar_float_to_centi(data);
                stats->last_y_centi = app_uart_radar_float_to_centi(&data[4]);
                stats->last_z_centi = app_uart_radar_float_to_centi(&data[8]);
                stats->has_track_position = true;
            }
            break;

        case APP_UART_RADAR_TYPE_PHASE:
        default:
            break;
    }
}

static const char *app_uart_radar_error_to_string(app_uart_radar_error_t error)
{
    switch (error)
    {
        case APP_UART_RADAR_ERROR_NONE:
            return "none";
        case APP_UART_RADAR_ERROR_BACKPRESSURE:
            return "backpressure";
        case APP_UART_RADAR_ERROR_LEN:
            return "len";
        case APP_UART_RADAR_ERROR_HEAD_CKSUM:
            return "head_cksum";
        case APP_UART_RADAR_ERROR_DATA_CKSUM:
            return "data_cksum";
        case APP_UART_RADAR_ERROR_QUEUE_FULL:
            return "queue_full";
        case APP_UART_RADAR_ERROR_UART:
            return "uart";
        default:
            return "unknown";
    }
}

static void app_uart_radar_test_print_stats(const app_uart_radar_test_stats_t *stats,
                                            uint32_t delta_frames,
                                            uint32_t delta_invalid,
                                            uint32_t delta_gaps,
                                            uint32_t elapsed_ms)
{
    /* 测试输出走 debug UART。若同时启用 PDM_TEST，两边都会打印，正式推理时
     * 建议只保留一个消费者/测试打印任务，避免日志互相穿插。
     */
    printf("[RADAR_TEST] frames=%lu, elapsed_ms=%lu, invalid=%lu, seq_gap=%lu, "
           "received=%lu, driver_drop=%lu, bad_frame=%lu, resync=%lu, "
           "uart_err=%lu, max_len=%u, bytes=%lu, last_err=%s, "
           "last_id=0x%04x, last_type=0x%04x, len=%u",
           (unsigned long)delta_frames,
           (unsigned long)elapsed_ms,
           (unsigned long)delta_invalid,
           (unsigned long)delta_gaps,
           (unsigned long)app_uart_radar_get_received_count(),
           (unsigned long)app_uart_radar_get_dropped_count(),
           (unsigned long)app_uart_radar_get_bad_frame_count(),
           (unsigned long)app_uart_radar_get_resync_count(),
           (unsigned long)app_uart_radar_get_uart_error_count(),
           (unsigned int)app_uart_radar_get_max_frame_len(),
           (unsigned long)stats->total_bytes,
           app_uart_radar_error_to_string(radar_last_error),
           (unsigned int)stats->last_frame_id,
           (unsigned int)stats->last_type,
           (unsigned int)stats->last_data_len);

    if (stats->has_human_state)
    {
        printf(", human=%u", (unsigned int)stats->last_human_state);
    }
    if (stats->has_target_count)
    {
        printf(", targets=%lu", (unsigned long)stats->last_target_count);
    }
    if (stats->has_breath_rate)
    {
        printf(", breath=");
        app_uart_radar_print_centi(stats->last_breath_rate_centi);
    }
    if (stats->has_heart_rate)
    {
        printf(", heart=");
        app_uart_radar_print_centi(stats->last_heart_rate_centi);
    }
    if (stats->has_range)
    {
        printf(", range_flag=%lu, range=",
               (unsigned long)stats->last_range_flag);
        app_uart_radar_print_centi(stats->last_range_centi);
    }
    if (stats->has_track_position)
    {
        printf(", pos=(");
        app_uart_radar_print_centi(stats->last_x_centi);
        printf(",");
        app_uart_radar_print_centi(stats->last_y_centi);
        printf(",");
        app_uart_radar_print_centi(stats->last_z_centi);
        printf(")");
    }
    if (stats->has_firmware_status)
    {
        printf(", fw_project=%u, fw=%u.%u.%u",
               (unsigned int)stats->last_project_id,
               (unsigned int)stats->last_major_version,
               (unsigned int)stats->last_sub_version,
               (unsigned int)stats->last_modified_version);
    }

    printf("\r\n");
}

void radar_data_test(void)
{
    app_uart_radar_block_t block;
    app_uart_radar_test_stats_t stats = {0};
    uint32_t last_print_frames = 0;
    uint32_t last_print_invalid = 0;
    uint32_t last_print_gaps = 0;
    TickType_t last_print_tick = xTaskGetTickCount();

    for (;;)
    {
        bool got_block = app_uart_radar_receive_block(
            &block,
            pdMS_TO_TICKS(APP_UART_RADAR_TEST_RX_WAIT_MS));

        if (got_block)
        {
            bool valid = app_uart_radar_descriptor_is_valid(&block);

#if (APP_UART_RADAR_TEST_PRINT_EACH_BLOCK)
            app_uart_radar_test_print_block(&block, valid);
#endif

            app_uart_radar_test_update_stats(&stats, &block, valid);
            app_uart_radar_release_block(block.block_index);
        }

        TickType_t now = xTaskGetTickCount();
        if (pdMS_TO_TICKS(APP_UART_RADAR_TEST_PRINT_MS) <=
            (now - last_print_tick))
        {
            uint32_t delta_frames = stats.total_frames - last_print_frames;
            uint32_t delta_invalid = stats.invalid_blocks - last_print_invalid;
            uint32_t delta_gaps = stats.sequence_gaps - last_print_gaps;
            uint32_t elapsed_ms = (uint32_t)((now - last_print_tick) *
                                             portTICK_PERIOD_MS);

            app_uart_radar_test_print_stats(&stats,
                                            delta_frames,
                                            delta_invalid,
                                            delta_gaps,
                                            elapsed_ms);

            last_print_frames = stats.total_frames;
            last_print_invalid = stats.invalid_blocks;
            last_print_gaps = stats.sequence_gaps;
            last_print_tick = now;
        }
    }
}

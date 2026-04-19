#include "app_csv_export.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_pdm_pcm.h"
#include "app_uart_radar.h"

#define APP_CSV_EXPORT_RADAR_TYPE_TEXT              (0x0100u)
#define APP_CSV_EXPORT_RADAR_TYPE_HUMAN_STATUS      (0x0F09u)
#define APP_CSV_EXPORT_RADAR_TYPE_HUMAN_POSITION    (0x0A04u)
#define APP_CSV_EXPORT_RADAR_TYPE_PHASE             (0x0A13u)
#define APP_CSV_EXPORT_RADAR_TYPE_BREATH_RATE       (0x0A14u)
#define APP_CSV_EXPORT_RADAR_TYPE_HEART_RATE        (0x0A15u)
#define APP_CSV_EXPORT_RADAR_TYPE_TARGET_RANGE      (0x0A16u)
#define APP_CSV_EXPORT_RADAR_TYPE_TRACK_POSITION    (0x0A17u)

static TaskHandle_t csv_export_task_handle = NULL;

#if (!APP_CSV_EXPORT_MIC_RAW_ENABLE)
static uint16_t app_csv_export_abs_i16(int16_t sample);
#endif
static bool app_csv_export_mic_block_is_valid(const app_pdm_pcm_block_t *block);
static bool app_csv_export_radar_block_is_valid(const app_uart_radar_block_t *block);
static void app_csv_export_print_header(void);
static void app_csv_export_print_mic_block(const app_pdm_pcm_block_t *block);
static void app_csv_export_print_radar_block(const app_uart_radar_block_t *block);
static void app_csv_export_print_hex(const uint8_t *data, uint16_t len);
static void app_csv_export_print_radar_decoded(const app_uart_radar_block_t *block);
static void app_csv_export_print_ascii_field(const uint8_t *data, uint16_t len);
static uint16_t app_csv_export_read_le_u16(const uint8_t *data);
static uint32_t app_csv_export_read_le_u32(const uint8_t *data);
static int32_t app_csv_export_read_le_i32(const uint8_t *data);
static int32_t app_csv_export_float_to_milli(const uint8_t *data);
static void app_csv_export_print_milli(int32_t value);

cy_rslt_t app_csv_export_task_init(void)
{
    BaseType_t ret;

    if (NULL != csv_export_task_handle)
    {
        return CY_RSLT_SUCCESS;
    }

    ret = xTaskCreate(app_csv_export_task,
                      "csv_export",
                      APP_CSV_EXPORT_TASK_STACK_SIZE,
                      NULL,
                      APP_CSV_EXPORT_TASK_PRIORITY,
                      &csv_export_task_handle);

    return (pdPASS == ret) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
}

void app_csv_export_task(void *pvParameters)
{
    (void)pvParameters;

    app_csv_export_print_header();

    for (;;)
    {
        bool did_work = false;
        app_pdm_pcm_block_t mic_block;
        app_uart_radar_block_t radar_block;

        if (app_pdm_pcm_receive_block(&mic_block, 0))
        {
            app_csv_export_print_mic_block(&mic_block);
            app_pdm_pcm_release_block(mic_block.block_index);
            did_work = true;
        }

        if (app_uart_radar_receive_block(&radar_block,
                                         pdMS_TO_TICKS(APP_CSV_EXPORT_RADAR_RX_WAIT_MS)))
        {
            app_csv_export_print_radar_block(&radar_block);
            app_uart_radar_release_block(radar_block.block_index);
            did_work = true;
        }

        if (!did_work)
        {
            vTaskDelay(pdMS_TO_TICKS(APP_CSV_EXPORT_IDLE_DELAY_MS));
        }
    }
}

#if (!APP_CSV_EXPORT_MIC_RAW_ENABLE)
static uint16_t app_csv_export_abs_i16(int16_t sample)
{
    return (sample < 0) ? (uint16_t)(-(int32_t)sample) : (uint16_t)sample;
}
#endif

static bool app_csv_export_mic_block_is_valid(const app_pdm_pcm_block_t *block)
{
    return ((NULL != block) &&
            (NULL != block->data) &&
            (APP_PDM_PCM_BLOCK_SAMPLES == block->sample_count) &&
            (APP_PDM_PCM_SAMPLES_PER_CH_PER_BLOCK == block->samples_per_channel) &&
            (APP_PDM_PCM_BLOCK_COUNT > block->block_index) &&
            (&recorded_data[block->block_index][0] == block->data));
}

static bool app_csv_export_radar_block_is_valid(const app_uart_radar_block_t *block)
{
    uint16_t expected_len;

    if ((NULL == block) ||
        (NULL == block->frame) ||
        (APP_UART_RADAR_BLOCK_COUNT <= block->block_index) ||
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

static void app_csv_export_print_header(void)
{
    printf("device,tick_ms,sequence,valid,block_index,sample_index,left,right,"
           "sample_count,min,max,peak_abs,mean_abs,frame_id,radar_type,"
           "frame_len,data_len,driver_drop,msg_drop,frame_hex,radar_decoded\r\n");
}

static void app_csv_export_print_mic_block(const app_pdm_pcm_block_t *block)
{
    bool valid = app_csv_export_mic_block_is_valid(block);
    uint32_t tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    if (!valid)
    {
        printf("mic,%lu,%lu,0,%u,,,,,,,,,,,,,%lu,,,\r\n",
               (unsigned long)tick_ms,
               (NULL != block) ? (unsigned long)block->sequence : 0ul,
               (NULL != block) ? (unsigned int)block->block_index : 0u,
               (unsigned long)app_pdm_pcm_get_dropped_count());
        return;
    }

#if (APP_CSV_EXPORT_MIC_RAW_ENABLE)
    for (uint16_t i = 0; i < block->samples_per_channel; i++)
    {
        uint16_t base = (uint16_t)(i * NUM_CHANNELS);
        int16_t left = block->data[base];
        int16_t right = block->data[base + 1u];

        printf("mic,%lu,%lu,1,%u,%u,%d,%d,,,,,,,,,,%lu,%lu,,\r\n",
               (unsigned long)tick_ms,
               (unsigned long)block->sequence,
               (unsigned int)block->block_index,
               (unsigned int)i,
               (int)left,
               (int)right,
               (unsigned long)app_pdm_pcm_get_dropped_count(),
               (unsigned long)block->dropped_count);
    }
#else
    int16_t min_sample = INT16_MAX;
    int16_t max_sample = INT16_MIN;
    uint16_t peak_abs = 0;
    uint64_t abs_sum = 0;

    for (uint16_t i = 0; i < block->sample_count; i++)
    {
        int16_t sample = block->data[i];
        uint16_t abs_sample = app_csv_export_abs_i16(sample);

        if (sample < min_sample)
        {
            min_sample = sample;
        }
        if (sample > max_sample)
        {
            max_sample = sample;
        }
        if (abs_sample > peak_abs)
        {
            peak_abs = abs_sample;
        }

        abs_sum += abs_sample;
    }

    printf("mic,%lu,%lu,1,%u,,,,%u,%d,%d,%u,%lu,,,,,%lu,%lu,,\r\n",
           (unsigned long)tick_ms,
           (unsigned long)block->sequence,
           (unsigned int)block->block_index,
           (unsigned int)block->sample_count,
           (int)min_sample,
           (int)max_sample,
           (unsigned int)peak_abs,
           (unsigned long)(abs_sum / block->sample_count),
           (unsigned long)app_pdm_pcm_get_dropped_count(),
           (unsigned long)block->dropped_count);
#endif
}

static void app_csv_export_print_radar_block(const app_uart_radar_block_t *block)
{
    bool valid = app_csv_export_radar_block_is_valid(block);
    uint32_t tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    printf("radar,%lu,%lu,%u,%u,,,,,,,,,%u,0x%04x,%u,%u,%lu,%lu,",
           (unsigned long)tick_ms,
           (NULL != block) ? (unsigned long)block->sequence : 0ul,
           valid ? 1u : 0u,
           (NULL != block) ? (unsigned int)block->block_index : 0u,
           (NULL != block) ? (unsigned int)block->frame_id : 0u,
           (NULL != block) ? (unsigned int)block->type : 0u,
           (NULL != block) ? (unsigned int)block->frame_len : 0u,
           (NULL != block) ? (unsigned int)block->data_len : 0u,
           (unsigned long)app_uart_radar_get_dropped_count(),
           (NULL != block) ? (unsigned long)block->dropped_count : 0ul);

#if (APP_CSV_EXPORT_RADAR_FRAME_HEX_ENABLE)
    if (valid)
    {
        app_csv_export_print_hex(block->frame, block->frame_len);
    }
#endif

    printf(",");
    if (valid)
    {
        app_csv_export_print_radar_decoded(block);
    }

    printf("\r\n");
}

static void app_csv_export_print_hex(const uint8_t *data, uint16_t len)
{
    static const char hex[] = "0123456789ABCDEF";

    if (NULL == data)
    {
        return;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t value = data[i];
        putchar((int)hex[(value >> 4) & 0x0Fu]);
        putchar((int)hex[value & 0x0Fu]);
    }
}

static void app_csv_export_print_radar_decoded(const app_uart_radar_block_t *block)
{
    const uint8_t *data;
    uint32_t target_count;
    uint32_t range_flag;

    if ((NULL == block) || (NULL == block->data))
    {
        return;
    }

    data = block->data;
    switch (block->type)
    {
        case APP_CSV_EXPORT_RADAR_TYPE_TEXT:
            printf("text=");
            app_csv_export_print_ascii_field(data, block->data_len);
            break;

        case APP_CSV_EXPORT_RADAR_TYPE_HUMAN_STATUS:
            if (2u <= block->data_len)
            {
                printf("human_present=%u",
                       (0u != app_csv_export_read_le_u16(data)) ? 1u : 0u);
            }
            break;

        case APP_CSV_EXPORT_RADAR_TYPE_HUMAN_POSITION:
            if (4u <= block->data_len)
            {
                target_count = app_csv_export_read_le_u32(data);
                printf("target_count=%lu", (unsigned long)target_count);

                if ((0u < target_count) && (24u <= block->data_len))
                {
                    printf(";x_m=");
                    app_csv_export_print_milli(app_csv_export_float_to_milli(&data[4]));
                    printf(";y_m=");
                    app_csv_export_print_milli(app_csv_export_float_to_milli(&data[8]));
                    printf(";z_m=");
                    app_csv_export_print_milli(app_csv_export_float_to_milli(&data[12]));
                    printf(";dop_idx=%ld",
                           (long)app_csv_export_read_le_i32(&data[16]));
                    printf(";cluster_id=%ld",
                           (long)app_csv_export_read_le_i32(&data[20]));
                }
            }
            break;

        case APP_CSV_EXPORT_RADAR_TYPE_PHASE:
            if (12u <= block->data_len)
            {
                printf("total_phase=");
                app_csv_export_print_milli(app_csv_export_float_to_milli(data));
                printf(";breath_phase=");
                app_csv_export_print_milli(app_csv_export_float_to_milli(&data[4]));
                printf(";heart_phase=");
                app_csv_export_print_milli(app_csv_export_float_to_milli(&data[8]));
            }
            break;

        case APP_CSV_EXPORT_RADAR_TYPE_BREATH_RATE:
            if (4u <= block->data_len)
            {
                printf("breath_rate_per_min=");
                app_csv_export_print_milli(app_csv_export_float_to_milli(data));
            }
            break;

        case APP_CSV_EXPORT_RADAR_TYPE_HEART_RATE:
            if (4u <= block->data_len)
            {
                printf("heart_rate_bpm=");
                app_csv_export_print_milli(app_csv_export_float_to_milli(data));
            }
            break;

        case APP_CSV_EXPORT_RADAR_TYPE_TARGET_RANGE:
            if (8u <= block->data_len)
            {
                range_flag = app_csv_export_read_le_u32(data);
                printf("range_flag=%lu", (unsigned long)range_flag);
                if (0u != range_flag)
                {
                    printf(";range_cm=");
                    app_csv_export_print_milli(app_csv_export_float_to_milli(&data[4]));
                }
            }
            break;

        case APP_CSV_EXPORT_RADAR_TYPE_TRACK_POSITION:
            if (12u <= block->data_len)
            {
                printf("x_m=");
                app_csv_export_print_milli(app_csv_export_float_to_milli(data));
                printf(";y_m=");
                app_csv_export_print_milli(app_csv_export_float_to_milli(&data[4]));
                printf(";z_m=");
                app_csv_export_print_milli(app_csv_export_float_to_milli(&data[8]));
            }
            break;

        default:
            break;
    }
}

static void app_csv_export_print_ascii_field(const uint8_t *data, uint16_t len)
{
    if (NULL == data)
    {
        return;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t ch = data[i];

        if ((',' == ch) || ('\r' == ch) || ('\n' == ch))
        {
            putchar(';');
        }
        else if ((' ' <= ch) && (ch <= '~'))
        {
            putchar((int)ch);
        }
        else
        {
            putchar('.');
        }
    }
}

static uint16_t app_csv_export_read_le_u16(const uint8_t *data)
{
    return ((uint16_t)data[1] << 8) | data[0];
}

static uint32_t app_csv_export_read_le_u32(const uint8_t *data)
{
    return ((uint32_t)data[3] << 24) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[1] << 8) |
           data[0];
}

static int32_t app_csv_export_read_le_i32(const uint8_t *data)
{
    return (int32_t)app_csv_export_read_le_u32(data);
}

static int32_t app_csv_export_float_to_milli(const uint8_t *data)
{
    uint32_t raw = app_csv_export_read_le_u32(data);
    float value;

    memcpy(&value, &raw, sizeof(value));

    if (0.0f <= value)
    {
        return (int32_t)((value * 1000.0f) + 0.5f);
    }
    return (int32_t)((value * 1000.0f) - 0.5f);
}

static void app_csv_export_print_milli(int32_t value)
{
    uint32_t magnitude;
    uint32_t integer;
    uint32_t fraction;

    if (0 > value)
    {
        putchar('-');
        magnitude = (uint32_t)(-value);
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    integer = magnitude / 1000u;
    fraction = magnitude % 1000u;
    printf("%lu.%03lu", (unsigned long)integer, (unsigned long)fraction);
}

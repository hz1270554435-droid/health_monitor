#include "app_get_data.h"
#include <stdint.h>
#include <stdio.h>

/*******************************************************************************
* 全局变量
*******************************************************************************/
static TaskHandle_t inference_task_handle = NULL;

static uint16_t mic_data_abs_i16(int16_t sample)
{
    return (sample < 0) ? (uint16_t)(-(int32_t)sample) : (uint16_t)sample;
}

/*******************************************************************************
* 函数名称: app_get_data_test_task_init
********************************************************************************
* 功能说明:
*  创建 PDM 数据读取自检任务。
*
*  注意：该任务会作为 PDM block 队列的消费者持续取数并释放 block。
*  后续接入正式推理任务时，不要让测试任务和推理任务同时消费同一个队列。
*
* 参数:
*  无
*
* 返回:
*  CY_RSLT_SUCCESS  - 任务创建成功
*  CY_RSLT_TYPE_ERROR - 任务创建失败
*
*******************************************************************************/
cy_rslt_t inference_task_init(void)
{
    BaseType_t ret;

    ret = xTaskCreate(inference_task,
                      "inference_task",
                      APP_INFERENCE_TASK_STACK_SIZE,
                      NULL,
                      APP_INFERENCE_TASK_PRIORITY,
                      &inference_task_handle);

    return (pdPASS == ret) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
}

/*******************************************************************************
* 函数名称: app_get_data_test_task
********************************************************************************
* 功能说明:
*  从 PDM PCM 队列接收 10 ms 音频块，检查数据描述符、序号连续性和
*  基本数据幅值。该任务用于证明 ISR -> 环形缓冲区 -> 队列 -> 消费者
*  这条链路可以稳定工作。
*
*******************************************************************************/
void inference_task(void *pvParameters)
{
    (void)pvParameters;

    mic_data_test();
}

void mic_data_test(void)
{

    app_pdm_pcm_block_t block;
    uint32_t total_blocks = 0;
    uint32_t invalid_blocks = 0;
    uint32_t sequence_gaps = 0;
    uint32_t last_print_blocks = 0;
    uint32_t last_print_invalid = 0;
    uint32_t last_print_gaps = 0;
    uint32_t expected_sequence = 0;
    bool has_expected_sequence = false;
    TickType_t last_print_tick = xTaskGetTickCount();
    int16_t interval_min_sample = INT16_MAX;
    int16_t interval_max_sample = INT16_MIN;
    uint16_t interval_peak_abs = 0;
    uint64_t interval_abs_sum = 0;
    uint32_t interval_sample_count = 0;
    uint32_t interval_checksum = 0;

    for (;;)
    {
        if (!app_pdm_pcm_receive_block(&block, portMAX_DELAY))
        {
            continue;
        }

        bool valid = true;

        if ((NULL == block.data) ||
            (APP_PDM_PCM_BLOCK_SAMPLES != block.sample_count) ||
            (APP_PDM_PCM_SAMPLES_PER_CH_PER_BLOCK != block.samples_per_channel) ||
            (APP_PDM_PCM_BLOCK_COUNT <= block.block_index) ||
            (&recorded_data[block.block_index][0] != block.data))
        {
            valid = false;
        }

        if (has_expected_sequence && (block.sequence != expected_sequence))
        {
            sequence_gaps++;
        }
        expected_sequence = block.sequence + 1u;
        has_expected_sequence = true;

        if (valid)
        {
            for (uint16_t i = 0; i < block.sample_count; i++)
            {
                int16_t sample = block.data[i];
                uint16_t abs_sample = mic_data_abs_i16(sample);

                if (sample < interval_min_sample)
                {
                    interval_min_sample = sample;
                }
                if (sample > interval_max_sample)
                {
                    interval_max_sample = sample;
                }

                if (abs_sample > interval_peak_abs)
                {
                    interval_peak_abs = abs_sample;
                }

                interval_abs_sum += abs_sample;
                interval_sample_count++;
                interval_checksum += (uint16_t)sample;
            }
        }
        else
        {
            invalid_blocks++;
        }

        total_blocks++;
        app_pdm_pcm_release_block(block.block_index);

        TickType_t now = xTaskGetTickCount();
        if (pdMS_TO_TICKS(1000u) <= (now - last_print_tick))
        {
            uint32_t delta_blocks = total_blocks - last_print_blocks;
            uint32_t delta_invalid = invalid_blocks - last_print_invalid;
            uint32_t delta_gaps = sequence_gaps - last_print_gaps;
            uint32_t elapsed_ms = (uint32_t)((now - last_print_tick) *
                                             portTICK_PERIOD_MS);
            uint32_t mean_abs = (0u < interval_sample_count) ?
                                (uint32_t)(interval_abs_sum / interval_sample_count) :
                                0u;
            int16_t print_min = (0u < interval_sample_count) ?
                                interval_min_sample : 0;
            int16_t print_max = (0u < interval_sample_count) ?
                                interval_max_sample : 0;

            printf("[PDM_TEST] blocks=%lu, elapsed_ms=%lu, invalid=%lu, "
                   "seq_gap=%lu, driver_drop=%lu, msg_drop=%lu, "
                   "min=%d, max=%d, peak_abs=%u, mean_abs=%lu, "
                   "checksum=0x%08lx\r\n",
                   (unsigned long)delta_blocks,
                   (unsigned long)elapsed_ms,
                   (unsigned long)delta_invalid,
                   (unsigned long)delta_gaps,
                   (unsigned long)app_pdm_pcm_get_dropped_count(),
                   (unsigned long)block.dropped_count,
                   (int)print_min,
                   (int)print_max,
                   (unsigned int)interval_peak_abs,
                   (unsigned long)mean_abs,
                   (unsigned long)interval_checksum);

            last_print_blocks = total_blocks;
            last_print_invalid = invalid_blocks;
            last_print_gaps = sequence_gaps;
            last_print_tick = now;
            interval_min_sample = INT16_MAX;
            interval_max_sample = INT16_MIN;
            interval_peak_abs = 0;
            interval_abs_sum = 0;
            interval_sample_count = 0;
            interval_checksum = 0;
        }
    }
}

#include "app_model_result_monitor.h"

#include <stdio.h>
#include <string.h>

/* CM33 结果观察任务。
 *
 * 这个模块刻意只读 CM55 的 result 区，不接触 input_state/audio_payload：
 * - input 区由 app_audio_preprocess 生产、CM55 消费；
 * - result 区由 CM55 写入、本模块观察；
 * - PDM 队列仍然只有 app_audio_preprocess 一个正式消费者。
 */

static TaskHandle_t model_result_monitor_task_handle = NULL;
static app_model_result_monitor_stats_t model_result_monitor_stats;

static const char *app_model_result_monitor_status_name(uint8_t status);
static uint32_t app_model_result_monitor_now_ms(void);

cy_rslt_t app_model_result_monitor_task_init(void)
{
    BaseType_t ret;

    if (NULL != model_result_monitor_task_handle)
    {
        return CY_RSLT_SUCCESS;
    }

    ret = xTaskCreate(app_model_result_monitor_task,
                      "model_result",
                      APP_MODEL_RESULT_MONITOR_TASK_STACK_SIZE,
                      NULL,
                      APP_MODEL_RESULT_MONITOR_TASK_PRIORITY,
                      &model_result_monitor_task_handle);

    return (pdPASS == ret) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
}

void app_model_result_monitor_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        volatile app_model_shared_region_t *shared = APP_MODEL_SHARED_REGION;

        APP_MODEL_SHARED_INVALIDATE_CACHE((void *)shared, sizeof(*shared));

        if ((APP_MODEL_SHARED_MAGIC != shared->magic) ||
            (APP_MODEL_SHARED_VERSION != shared->version))
        {
            model_result_monitor_stats.shared_not_ready++;
            vTaskDelay(pdMS_TO_TICKS(APP_MODEL_RESULT_MONITOR_POLL_MS));
            continue;
        }

        if (APP_MODEL_SHARED_RESULT_WRITING == shared->result_state)
        {
            model_result_monitor_stats.result_writing++;
            vTaskDelay(pdMS_TO_TICKS(APP_MODEL_RESULT_MONITOR_POLL_MS));
            continue;
        }

        if ((APP_MODEL_SHARED_RESULT_READY == shared->result_state) &&
            (!model_result_monitor_stats.has_result ||
             (model_result_monitor_stats.last_result_sequence !=
              shared->result_sequence)))
        {
            app_model_inference_result_t result;
            uint32_t result_sequence = shared->result_sequence;

            memcpy(&result, (const void *)&shared->result, sizeof(result));

            /* 复制后再确认一次状态和序号，避免刚好撞上 CM55 覆盖最新结果。 */
            __DMB();
            APP_MODEL_SHARED_INVALIDATE_CACHE((void *)shared, sizeof(*shared));
            if ((APP_MODEL_SHARED_RESULT_READY != shared->result_state) ||
                (result_sequence != shared->result_sequence))
            {
                vTaskDelay(pdMS_TO_TICKS(APP_MODEL_RESULT_MONITOR_POLL_MS));
                continue;
            }

            model_result_monitor_stats.has_result = true;
            model_result_monitor_stats.results_seen++;
            model_result_monitor_stats.last_result_sequence =
                result_sequence;
            model_result_monitor_stats.last_status = result.status;

            /* 只打印整数摘要，避免为了调试输出引入 printf float 链接开销。
             * 真实模型接入后，如需观察 scores[]，建议在专用调试工具中读取共享内存。
             */
            printf("[MODEL_RESULT] t_ms=%lu, input_seq=%lu, result_seq=%lu, "
                   "status=%s(%u), class_count=%u, infer_ms=%lu\r\n",
                   (unsigned long)app_model_result_monitor_now_ms(),
                   (unsigned long)result.input_sequence,
                   (unsigned long)result_sequence,
                   app_model_result_monitor_status_name(result.status),
                   (unsigned int)result.status,
                   (unsigned int)result.class_count,
                   (unsigned long)result.inference_time_ms);

#if (APP_MODEL_RESULT_MONITOR_CLEAR_AFTER_READ)
            shared->result_state = APP_MODEL_SHARED_RESULT_EMPTY;
            __DMB();
            APP_MODEL_SHARED_CLEAN_CACHE((void *)shared, sizeof(*shared));
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(APP_MODEL_RESULT_MONITOR_POLL_MS));
    }
}

void app_model_result_monitor_get_stats(
    app_model_result_monitor_stats_t *stats)
{
    if (NULL != stats)
    {
        *stats = model_result_monitor_stats;
    }
}

static const char *app_model_result_monitor_status_name(uint8_t status)
{
    switch ((app_model_inference_status_t)status)
    {
        case APP_MODEL_INFERENCE_STATUS_OK:
            return "OK";

        case APP_MODEL_INFERENCE_STATUS_INVALID_INPUT:
            return "INVALID_INPUT";

        case APP_MODEL_INFERENCE_STATUS_MODEL_NOT_READY:
            return "MODEL_NOT_READY";

        default:
            return "UNKNOWN";
    }
}

static uint32_t app_model_result_monitor_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

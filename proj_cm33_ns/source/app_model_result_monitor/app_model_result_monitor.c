#include "app_model_result_monitor.h"

#include <stdio.h>
#include <string.h>

#include "app_audio_preprocess.h"

/* CM33 结果观察任务。
 *
 * 这个模块刻意只读 CM55 的 result 区，不接触 input_state/audio_payload：
 * - input 区由 app_audio_preprocess 生产、CM55 消费；
 * - result 区由 CM55 写入、本模块观察；
 * - PDM 队列仍然只有 app_audio_preprocess 一个正式消费者。
 */

static TaskHandle_t model_result_monitor_task_handle = NULL;
static app_model_result_monitor_stats_t model_result_monitor_stats;

#ifndef APP_MODEL_DEMO_COUGH_THRESHOLD
#define APP_MODEL_DEMO_COUGH_THRESHOLD    (0.75f)
#endif

#ifndef APP_MODEL_RESULT_MONITOR_IDLE_DIAG_MS
#define APP_MODEL_RESULT_MONITOR_IDLE_DIAG_MS    (5000u)
#endif

static const char *app_model_result_monitor_status_name(uint8_t status);
static void app_model_result_monitor_print_float(float value);
static void app_model_result_monitor_print_idle_diag(
    const volatile app_model_shared_region_t *shared);
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
        static uint32_t last_idle_diag_ms;
        uint32_t now_ms = app_model_result_monitor_now_ms();

        APP_MODEL_SHARED_INVALIDATE_CACHE((void *)shared, sizeof(*shared));

        if ((APP_MODEL_SHARED_MAGIC != shared->magic) ||
            (APP_MODEL_SHARED_VERSION != shared->version))
        {
            model_result_monitor_stats.shared_not_ready++;
            if ((now_ms - last_idle_diag_ms) >=
                APP_MODEL_RESULT_MONITOR_IDLE_DIAG_MS)
            {
                printf("[MODEL_DIAG] t_ms=%lu, shared_not_ready, magic=0x%08lx, "
                       "version=%lu\r\n",
                       (unsigned long)now_ms,
                       (unsigned long)shared->magic,
                       (unsigned long)shared->version);
                fflush(stdout);
                last_idle_diag_ms = now_ms;
            }
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

            if ((APP_MODEL_INFERENCE_STATUS_OK == result.status) &&
                (APP_MODEL_RESULT_SEQUENCE_SMOKE_BASE <= result_sequence) &&
                (2u <= result.class_count))
            {
                /* 烟雾测试结果打印。
                 * scores[0]/[1] 是 AUDIO_compute() 的两个 float 输出；
                 * scores[2] 是板端 softmax 后的 cough_prob；
                 * scores[5] 是 PC expected_output.json 中的 cough_prob；
                 * scores[6] 是二者绝对误差。
                 *
                 * 这里手动按 6 位小数打印 float，避免启用 printf 的 %f 链接开销。
                 */
                printf("[MODEL_SMOKE] t_ms=%lu, sample=%lu, out0=",
                       (unsigned long)app_model_result_monitor_now_ms(),
                       (unsigned long)result.input_sequence);
                app_model_result_monitor_print_float(result.scores[0]);
                printf(", out1=");
                app_model_result_monitor_print_float(result.scores[1]);
                printf(", cough_prob=");
                app_model_result_monitor_print_float(result.scores[2]);
                printf(", expected_prob=");
                app_model_result_monitor_print_float(result.scores[5]);
                printf(", diff=");
                app_model_result_monitor_print_float(result.scores[6]);
                printf(", infer_ms=%lu\r\n",
                       (unsigned long)result.inference_time_ms);
            }
            else if ((APP_MODEL_INFERENCE_STATUS_OK == result.status) &&
                     (2u <= result.class_count))
            {
                const char *decision =
                    (APP_MODEL_DEMO_COUGH_THRESHOLD <= result.scores[2]) ?
                    "COUGH" : "NON_COUGH";

                /* MIC 第一版 demo 结果打印。
                 * scores[0]/[1] 是模型两个 logits；
                 * scores[2] 是 CM55 softmax 后的 cough_prob；
                 * scores[3] 是 CM33 前处理窗口能量；
                 * scores[4] 是 CM33 选择的 MIC 通道，0=左、1=右、255=混合。
                 *
                 * 这里仍手动打印 float，避免打开 printf %f 的额外链接开销。
                 */
                printf("[MODEL_DEMO] t_ms=%lu, input_seq=%lu, out0=",
                       (unsigned long)app_model_result_monitor_now_ms(),
                       (unsigned long)result.input_sequence);
                app_model_result_monitor_print_float(result.scores[0]);
                printf(", out1=");
                app_model_result_monitor_print_float(result.scores[1]);
                printf(", cough_prob=");
                app_model_result_monitor_print_float(result.scores[2]);
                printf(", decision=%s, energy=", decision);
                app_model_result_monitor_print_float(result.scores[3]);
                printf(", channel=%lu, infer_ms=%lu\r\n",
                       (unsigned long)result.scores[4],
                       (unsigned long)result.inference_time_ms);
            }
            else
            {
                printf("[MODEL_RESULT] t_ms=%lu, input_seq=%lu, result_seq=%lu, "
                       "status=%s(%u), class_count=%u, infer_ms=%lu\r\n",
                       (unsigned long)app_model_result_monitor_now_ms(),
                       (unsigned long)result.input_sequence,
                       (unsigned long)result_sequence,
                       app_model_result_monitor_status_name(result.status),
                       (unsigned int)result.status,
                       (unsigned int)result.class_count,
                       (unsigned long)result.inference_time_ms);
            }

#if (APP_MODEL_RESULT_MONITOR_CLEAR_AFTER_READ)
            shared->result_state = APP_MODEL_SHARED_RESULT_EMPTY;
            __DMB();
            APP_MODEL_SHARED_CLEAN_CACHE((void *)shared, sizeof(*shared));
#endif
            last_idle_diag_ms = now_ms;
        }
        else if ((now_ms - last_idle_diag_ms) >=
                 APP_MODEL_RESULT_MONITOR_IDLE_DIAG_MS)
        {
            app_model_result_monitor_print_idle_diag(shared);
            fflush(stdout);
            last_idle_diag_ms = now_ms;
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

        case APP_MODEL_INFERENCE_STATUS_MODEL_ERROR:
            return "MODEL_ERROR";

        default:
            return "UNKNOWN";
    }
}

static void app_model_result_monitor_print_float(float value)
{
    const char *sign = "";
    uint32_t whole;
    uint32_t frac;

    if (0.0f > value)
    {
        sign = "-";
        value = -value;
    }

    whole = (uint32_t)value;
    frac = (uint32_t)(((value - (float)whole) * 1000000.0f) + 0.5f);
    if (1000000u <= frac)
    {
        whole++;
        frac -= 1000000u;
    }

    printf("%s%lu.%06lu", sign, (unsigned long)whole, (unsigned long)frac);
}

static void app_model_result_monitor_print_idle_diag(
    const volatile app_model_shared_region_t *shared)
{
    app_audio_preprocess_stats_t stats;

    app_audio_preprocess_get_stats(&stats);

    /* 无结果时的低频诊断。
     * 这条日志用于区分“串口/烧录没有起来”和“模型链路暂时没有输出”：
     * - blocks=0：PDM 采集任务还没有给前处理送 block；
     * - ready/published 增长但 consumer 不增长：CM55 没消费输入，优先检查 CM55 烧录；
     * - consumer 增长但没有 MODEL_DEMO：CM55 可能在模型初始化/推理阶段异常；
     * - gated 增长：音频能量低于门限，可能环境太安静或 MIC 增益偏低。
     */
    printf("[MODEL_DIAG] t_ms=%lu, input_state=%lu, result_state=%lu, "
           "producer=%lu, consumer=%lu, result_seq=%lu, blocks=%lu, "
           "ready=%lu, published=%lu, gated=%lu, busy=%lu, energy=",
           (unsigned long)app_model_result_monitor_now_ms(),
           (unsigned long)shared->input_state,
           (unsigned long)shared->result_state,
           (unsigned long)shared->producer_sequence,
           (unsigned long)shared->consumer_sequence,
           (unsigned long)shared->result_sequence,
           (unsigned long)stats.blocks_received,
           (unsigned long)stats.windows_ready,
           (unsigned long)stats.windows_published,
           (unsigned long)stats.windows_energy_gated,
           (unsigned long)stats.shared_busy);
    app_model_result_monitor_print_float(stats.last_energy);
    printf(", dropped=%lu\r\n",
           (unsigned long)app_pdm_pcm_get_dropped_count());
}

static uint32_t app_model_result_monitor_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

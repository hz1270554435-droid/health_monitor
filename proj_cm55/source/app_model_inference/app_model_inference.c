#include "app_model_inference.h"

#include <string.h>

/* 本文件是 CM55 侧模型任务框架。
 *
 * 数据边界：
 * - CM33 只把“已前处理、已量化”的模型输入写入共享内存；
 * - CM55 从共享内存取走输入后复制到本地缓冲，再释放共享输入槽；
 * - 原始 PDM/PCM 永远不进入这块共享协议，避免采集数据和模型输入互相干扰。
 *
 * 后续模型接入点：
 * - 替换 app_model_inference_run_model() 中的占位实现；
 * - 如果模型输入 shape、量化方式或输出类别数改变，同步更新 shared/app_model_shared.h；
 * - 如果模型需要 tensor arena，请在 CM55 本模块内单独分配，不要占用 CM33 原始采集缓冲。
 */

static TaskHandle_t model_inference_task_handle = NULL;
static app_model_inference_stats_t model_inference_stats;
static uint8_t model_input_payload[APP_MODEL_AUDIO_FEATURE_MAX_BYTES];

static bool app_model_inference_try_take_input(
    app_model_audio_feature_desc_t *desc,
    uint8_t *payload,
    uint16_t payload_capacity);
static bool app_model_inference_desc_is_valid(
    const app_model_audio_feature_desc_t *desc);
static app_model_inference_status_t app_model_inference_run_model(
    const app_model_audio_feature_desc_t *desc,
    const uint8_t *payload,
    app_model_inference_result_t *result);
static void app_model_inference_publish_result(
    const app_model_audio_feature_desc_t *desc,
    const app_model_inference_result_t *result,
    uint32_t inference_time_ms);
static uint32_t app_model_inference_now_ms(void);

cy_rslt_t app_model_inference_task_init(void)
{
    BaseType_t ret;

    if (NULL != model_inference_task_handle)
    {
        return CY_RSLT_SUCCESS;
    }

    ret = xTaskCreate(app_model_inference_task,
                      "model_infer",
                      APP_MODEL_INFERENCE_TASK_STACK_SIZE,
                      NULL,
                      APP_MODEL_INFERENCE_TASK_PRIORITY,
                      &model_inference_task_handle);

    return (pdPASS == ret) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
}

void app_model_inference_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        app_model_audio_feature_desc_t desc;

        if (app_model_inference_try_take_input(&desc,
                                               model_input_payload,
                                               sizeof(model_input_payload)))
        {
            app_model_inference_result_t result;
            uint32_t start_ms = app_model_inference_now_ms();
            app_model_inference_status_t status;
            uint32_t elapsed_ms;

            memset(&result, 0, sizeof(result));
            status = app_model_inference_run_model(&desc,
                                                   model_input_payload,
                                                   &result);
            elapsed_ms = app_model_inference_now_ms() - start_ms;

            result.status = (uint8_t)status;
            app_model_inference_publish_result(&desc, &result, elapsed_ms);

            model_inference_stats.inference_runs++;
            model_inference_stats.last_input_sequence = desc.sequence;
            model_inference_stats.last_inference_time_ms = elapsed_ms;
            model_inference_stats.last_status = (uint8_t)status;
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(APP_MODEL_INFERENCE_POLL_DELAY_MS));
        }
    }
}

void app_model_inference_get_stats(app_model_inference_stats_t *stats)
{
    if (NULL != stats)
    {
        *stats = model_inference_stats;
    }
}

static bool app_model_inference_try_take_input(
    app_model_audio_feature_desc_t *desc,
    uint8_t *payload,
    uint16_t payload_capacity)
{
    volatile app_model_shared_region_t *shared = APP_MODEL_SHARED_REGION;
    uint16_t payload_bytes;

    APP_MODEL_SHARED_INVALIDATE_CACHE((void *)shared, sizeof(*shared));

    if ((APP_MODEL_SHARED_MAGIC != shared->magic) ||
        (APP_MODEL_SHARED_VERSION != shared->version))
    {
        model_inference_stats.shared_not_ready++;
        return false;
    }

    if (APP_MODEL_SHARED_INPUT_READY != shared->input_state)
    {
        return false;
    }

    shared->input_state = APP_MODEL_SHARED_INPUT_READING;
    __DMB();
    APP_MODEL_SHARED_CLEAN_CACHE((void *)shared, sizeof(*shared));

    *desc = shared->audio;
    payload_bytes = desc->payload_bytes;

    if (!app_model_inference_desc_is_valid(desc) ||
        (payload_capacity < payload_bytes))
    {
        app_model_inference_result_t result;

        memset(&result, 0, sizeof(result));
        result.status = (uint8_t)APP_MODEL_INFERENCE_STATUS_INVALID_INPUT;

        shared->consumer_sequence = desc->sequence;
        shared->input_state = APP_MODEL_SHARED_INPUT_EMPTY;
        __DMB();
        APP_MODEL_SHARED_CLEAN_CACHE((void *)shared, sizeof(*shared));

        app_model_inference_publish_result(desc, &result, 0u);
        model_inference_stats.invalid_inputs++;
        return false;
    }

    memcpy(payload, (const void *)&shared->audio_payload[0], payload_bytes);

    /* 已经复制到 CM55 本地缓冲后立即释放输入槽。
     * 这样 CM33 可以继续发布下一帧，真正模型推理耗时不会阻塞前处理写入。
     */
    shared->consumer_sequence = desc->sequence;
    shared->input_state = APP_MODEL_SHARED_INPUT_EMPTY;
    __DMB();
    APP_MODEL_SHARED_CLEAN_CACHE((void *)shared, sizeof(*shared));

    model_inference_stats.inputs_consumed++;
    return true;
}

static bool app_model_inference_desc_is_valid(
    const app_model_audio_feature_desc_t *desc)
{
    uint32_t expected_payload_bytes;

    if ((NULL == desc) || (0u == desc->valid))
    {
        return false;
    }

    if ((0u == desc->mel_bin_count) ||
        (APP_MODEL_AUDIO_FEATURE_MAX_MEL_BINS < desc->mel_bin_count) ||
        (0u == desc->time_bin_count) ||
        (APP_MODEL_AUDIO_FEATURE_MAX_TIME_BINS < desc->time_bin_count))
    {
        return false;
    }

    expected_payload_bytes = (uint32_t)desc->mel_bin_count *
                             (uint32_t)desc->time_bin_count;
    if ((0u == expected_payload_bytes) ||
        (APP_MODEL_AUDIO_FEATURE_MAX_BYTES < expected_payload_bytes) ||
        (desc->payload_bytes != expected_payload_bytes))
    {
        return false;
    }

    if ((APP_MODEL_AUDIO_QUANT_INT8 != desc->quant_type) &&
        (APP_MODEL_AUDIO_QUANT_UINT8 != desc->quant_type))
    {
        return false;
    }

    if (0.0f >= desc->quant_scale)
    {
        return false;
    }

    if ((0u == desc->sample_rate_hz) ||
        (0u == desc->window_ms) ||
        (0u == desc->frame_len_ms) ||
        (0u == desc->frame_hop_ms) ||
        (0u == desc->fft_size))
    {
        return false;
    }

    return true;
}

static app_model_inference_status_t app_model_inference_run_model(
    const app_model_audio_feature_desc_t *desc,
    const uint8_t *payload,
    app_model_inference_result_t *result)
{
    /* 占位推理入口。
     * 正式接入模型时，把 payload 按 desc->quant_type 解释为 int8_t 或 uint8_t，
     * shape 为 [mel_bin_count, time_bin_count]，再送入模型输入 tensor。
     *
     * 推荐后续在这里拆出几个明确步骤：
     * 1. 模型初始化：加载模型、分配/绑定 tensor arena、解析输入输出 tensor；
     * 2. 输入校验：确认 desc 中的 mel/time/quant 参数和训练模型完全一致；
     * 3. 输入填充：把 payload 复制到模型输入 tensor，必要时按模型要求转置；
     * 4. 推理执行：调用 NNLite/TFLM/其它运行时；
     * 5. 输出后处理：把类别数、分数和状态填入 app_model_inference_result_t。
     *
     * 这里不伪造分类结果，避免业务层误以为已经有可用模型输出。
     */
    (void)payload;

    result->input_sequence = desc->sequence;
    result->timestamp_ms = app_model_inference_now_ms();
    result->class_count = 0u;

    return APP_MODEL_INFERENCE_STATUS_MODEL_NOT_READY;
}

static void app_model_inference_publish_result(
    const app_model_audio_feature_desc_t *desc,
    const app_model_inference_result_t *result,
    uint32_t inference_time_ms)
{
    volatile app_model_shared_region_t *shared = APP_MODEL_SHARED_REGION;
    app_model_inference_result_t local_result = *result;

    local_result.input_sequence = desc->sequence;
    local_result.timestamp_ms = app_model_inference_now_ms();
    local_result.inference_time_ms = inference_time_ms;

    /* 结果区当前采用“最新结果覆盖旧结果”的语义。
     * 后续如果 CM33 需要可靠消费每个结果，可以再扩展为结果队列或双缓冲。
     */
    shared->result_state = APP_MODEL_SHARED_RESULT_WRITING;
    shared->result = local_result;
    shared->result_sequence = desc->sequence;
    __DMB();
    shared->result_state = APP_MODEL_SHARED_RESULT_READY;
    APP_MODEL_SHARED_CLEAN_CACHE((void *)shared, sizeof(*shared));
}

static uint32_t app_model_inference_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

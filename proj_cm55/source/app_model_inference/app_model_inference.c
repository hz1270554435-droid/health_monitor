#include "app_model_inference.h"

#include <math.h>
#include <string.h>

#include "app_audio_deployment_config.h"
#include "app_model_smoke.h"
#include APP_AUDIO_ACTIVE_MODEL_HEADER

#ifndef APP_MODEL_INPUT_DUMP_ENABLE
#define APP_MODEL_INPUT_DUMP_ENABLE             (0u)
#endif

#ifndef APP_MODEL_INPUT_DUMP_WINDOWS
#define APP_MODEL_INPUT_DUMP_WINDOWS            (3u)
#endif

#if (APP_MODEL_INPUT_DUMP_ENABLE) && (APP_MODEL_INFERENCE_MAX_SCORES < 14u)
#error "APP_MODEL_INPUT_DUMP_ENABLE requires at least 14 score slots"
#endif

/* 本文件是 CM55 侧模型任务框架。
 *
 * 数据边界：
 * - CM33 只把“已前处理、float32 40x101”的模型输入写入共享内存；
 * - CM55 从共享内存取走输入后复制到本地缓冲，再释放共享输入槽；
 * - 原始 PDM/PCM 永远不进入这块共享协议，避免采集数据和模型输入互相干扰。
 *
 * 当前模型接入点：
 * - active model compute API 输入必须是 PC/CM33 一致的 NCHW 展平 float[40 * 101]；
 * - 如果模型输入 shape、格式或输出类别数改变，同步更新 shared/app_model_shared.h；
 * - 模型内部状态由导出代码管理，本模块只持有一份本地输入缓冲。
 */

static TaskHandle_t model_inference_task_handle = NULL;
static app_model_inference_stats_t model_inference_stats;
static float model_input_payload[APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS];
static bool model_runtime_initialized;
#if (APP_MODEL_INPUT_DUMP_ENABLE)
static uint32_t model_input_dump_results_written;
#endif

static bool app_model_inference_try_take_input(
    app_model_audio_feature_desc_t *desc,
    float *payload,
    uint16_t payload_capacity_bytes);
static bool app_model_inference_desc_is_valid(
    const app_model_audio_feature_desc_t *desc);
static app_model_inference_status_t app_model_inference_run_model(
    const app_model_audio_feature_desc_t *desc,
    const float *payload,
    app_model_inference_result_t *result);
static void app_model_inference_publish_result(
    const app_model_audio_feature_desc_t *desc,
    const app_model_inference_result_t *result,
    uint32_t inference_time_ms);
#if (APP_MODEL_INPUT_DUMP_ENABLE)
static void app_model_inference_fill_input_stats(
    const app_model_audio_feature_desc_t *desc,
    const float *payload,
    app_model_inference_result_t *result);
#endif
static float app_model_inference_softmax_cough_prob(float output0,
                                                    float output1);
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

#if (APP_MODEL_SMOKE_TEST_ENABLE)
    /* 第一阶段 baseline：先用 PC 生成的 Log-Mel 测试向量直接跑 active model API。
     * 该测试不依赖实时 MIC 前处理，便于确认模型代码、ML runtime 和 CM55 链路已经部署成功。
     */
    (void)app_model_smoke_run_once();
#endif

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
    float *payload,
    uint16_t payload_capacity_bytes)
{
    volatile app_model_shared_region_t *shared = APP_MODEL_SHARED_REGION;
    uint16_t payload_bytes;

    /* 先失效本地 cache，确保读到的是 CM33 最近一次写入的共享区内容。 */
    APP_MODEL_SHARED_INVALIDATE_CACHE((void *)shared, sizeof(*shared));

    /* 共享区未初始化或协议版本不匹配时，直接跳过本轮轮询。 */
    if ((APP_MODEL_SHARED_MAGIC != shared->magic) ||
        (APP_MODEL_SHARED_VERSION != shared->version))
    {
        model_inference_stats.shared_not_ready++;
        return false;
    }

    /* 只有当 CM33 已明确把状态切到 READY 时，CM55 才能尝试取走输入。 */
    if (APP_MODEL_SHARED_INPUT_READY != shared->input_state)
    {
        return false;
    }

    /* 抢占输入槽所有权，告诉 CM33 当前帧正在被 CM55 读取。 */
    shared->input_state = APP_MODEL_SHARED_INPUT_READING;
    __DMB();
    APP_MODEL_SHARED_CLEAN_CACHE((void *)shared, sizeof(*shared));

    *desc = shared->audio;
    payload_bytes = desc->payload_bytes;

    /* 描述符或 payload 非法时，发布 INVALID_INPUT 结果并把输入槽释放回 EMPTY。 */
    if (!app_model_inference_desc_is_valid(desc) ||
        (payload_capacity_bytes < payload_bytes))
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

    /* 先复制到 CM55 本地缓冲，再做真正推理，避免长时间占用共享输入槽。 */
    memcpy((void *)payload, (const void *)&shared->audio_payload[0],
           payload_bytes);

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

#if (APP_MODEL_INPUT_DUMP_ENABLE)
static void app_model_inference_fill_input_stats(
    const app_model_audio_feature_desc_t *desc,
    const float *payload,
    app_model_inference_result_t *result)
{
    const uint32_t fnv_offset_basis = 2166136261u;
    const uint32_t fnv_prime = 16777619u;
    uint32_t element_count;
    uint32_t byte_count;
    const uint8_t *payload_bytes;
    float min_value;
    float max_value;
    double sum = 0.0;
    double square_sum = 0.0;
    float mean;
    float variance;
    float stddev;
    uint32_t hash = fnv_offset_basis;
    uint32_t floor_count = 0u;
    uint32_t near_zero_count = 0u;

    if ((NULL == desc) || (NULL == payload) || (NULL == result) ||
        (model_input_dump_results_written >= APP_MODEL_INPUT_DUMP_WINDOWS))
    {
        return;
    }

    element_count = (uint32_t)desc->mel_bin_count *
                    (uint32_t)desc->time_bin_count;
    if (0u == element_count)
    {
        return;
    }

    byte_count = element_count * (uint32_t)sizeof(payload[0]);
    payload_bytes = (const uint8_t *)payload;
    for (uint32_t i = 0u; i < byte_count; i++)
    {
        hash ^= (uint32_t)payload_bytes[i];
        hash *= fnv_prime;
    }

    min_value = payload[0];
    max_value = payload[0];
    for (uint32_t i = 0u; i < element_count; i++)
    {
        float value = payload[i];

        if (value < min_value)
        {
            min_value = value;
        }
        if (max_value < value)
        {
            max_value = value;
        }
        if (value <= -79.9f)
        {
            floor_count++;
        }
        if (fabsf(value) <= 1.0e-3f)
        {
            near_zero_count++;
        }
        sum += value;
        square_sum += ((double)value * (double)value);
    }

    mean = (float)(sum / (double)element_count);
    variance = (float)((square_sum / (double)element_count) -
                       ((double)mean * (double)mean));
    if (0.0f > variance)
    {
        variance = 0.0f;
    }
    stddev = sqrtf(variance);

    result->scores[5] = min_value;
    result->scores[6] = max_value;
    result->scores[7] = mean;
    result->scores[8] = stddev;
    result->scores[9] = (float)(hash & 0xffffu);
    result->scores[10] = (float)floor_count;
    result->scores[11] = (float)near_zero_count;
    result->scores[12] = payload[0];
    result->scores[13] = payload[element_count - 1u];
    model_input_dump_results_written++;
}
#endif

static bool app_model_inference_desc_is_valid(
    const app_model_audio_feature_desc_t *desc)
{
    uint32_t expected_payload_bytes;

    if ((NULL == desc) || (0u == desc->valid))
    {
        return false;
    }

    if ((APP_MODEL_AUDIO_MODEL_MEL_BINS != desc->mel_bin_count) ||
        (APP_MODEL_AUDIO_MODEL_TIME_BINS != desc->time_bin_count))
    {
        return false;
    }

    expected_payload_bytes = (uint32_t)desc->mel_bin_count *
                             (uint32_t)desc->time_bin_count *
                             (uint32_t)sizeof(float);
    if ((0u == expected_payload_bytes) ||
        (APP_MODEL_AUDIO_FEATURE_MAX_BYTES < expected_payload_bytes) ||
        (desc->payload_bytes != expected_payload_bytes))
    {
        return false;
    }

    if ((APP_MODEL_AUDIO_QUANT_FLOAT32 != desc->quant_type) ||
        (0 != desc->quant_zero_point) ||
        (0.0f >= desc->quant_scale))
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
    const float *payload,
    app_model_inference_result_t *result)
{
    /* 正式 MIC 推理入口。
     *
     * CM33 已经把 1 s MIC 窗口转换成和 PC 测试向量一致的 NCHW 展平 float32：
     * payload[mel * 101 + time]。这里不再做反量化或额外转置，直接送入
     * 已验证过的 active model compute API。
     */
    float output[APP_AUDIO_ACTIVE_MODEL_DATA_OUT_COUNT] = { 0.0f, 0.0f };

    /* 任何一个关键输入为空，都说明调用链路不完整，直接按非法输入处理。 */
    if ((NULL == desc) || (NULL == payload) || (NULL == result))
    {
        return APP_MODEL_INFERENCE_STATUS_INVALID_INPUT;
    }

    /* 模型运行时只做一次初始化；成功后通过静态标志避免每帧重复初始化。 */
    if (!model_runtime_initialized)
    {
        if (APP_AUDIO_ACTIVE_MODEL_RET_SUCCESS != APP_AUDIO_ACTIVE_MODEL_INIT())
        {
            return APP_MODEL_INFERENCE_STATUS_MODEL_ERROR;
        }
        model_runtime_initialized = true;
    }

    /* 每次推理前执行 soft reset，确保模型内部状态回到已知初始状态，
     * 避免上一次窗口残留状态影响当前这一帧结果。
     */
    if (APP_AUDIO_ACTIVE_MODEL_RET_SUCCESS != APP_AUDIO_ACTIVE_MODEL_SOFT_RESET())
    {
        return APP_MODEL_INFERENCE_STATUS_MODEL_ERROR;
    }

#if (APP_MODEL_INPUT_DUMP_ENABLE)
    /* 调试模式下，把输入统计写入 result->scores 的保留槽位，
     * 便于在不额外打印大块特征的情况下快速核对输入分布。
     */
    app_model_inference_fill_input_stats(desc, payload, result);
#endif

    /* 真正调用导入后的模型计算入口。 */
    APP_AUDIO_ACTIVE_MODEL_COMPUTE(payload, output);

    /* 把模型原始输出与辅助调试字段统一封装进共享结果结构。 */
    result->input_sequence = desc->sequence;
    result->timestamp_ms = app_model_inference_now_ms();
    result->class_count = APP_AUDIO_ACTIVE_MODEL_DATA_OUT_COUNT;
    result->scores[0] = output[0];
    result->scores[1] = output[1];
    result->scores[2] =
        app_model_inference_softmax_cough_prob(output[0], output[1]);
    result->scores[3] = desc->energy;
    result->scores[4] = (float)desc->selected_channel;

    return APP_MODEL_INFERENCE_STATUS_OK;
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

static float app_model_inference_softmax_cough_prob(float output0,
                                                    float output1)
{
    float max_logit = (output0 > output1) ? output0 : output1;
    float exp0 = expf(output0 - max_logit);
    float exp1 = expf(output1 - max_logit);
    float denom = exp0 + exp1;

    return (0.0f < denom) ? (exp1 / denom) : 0.0f;
}

static uint32_t app_model_inference_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

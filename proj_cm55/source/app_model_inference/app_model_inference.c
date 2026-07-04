#include "app_model_inference.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "app_audio_deployment_config.h"
#include "app_model_ipc_smoke.h"
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

#if (APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE) && (APP_MODEL_INFERENCE_MAX_SCORES < 14u)
#error "APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE requires at least 14 score slots"
#endif

/* 本文件是 CM55 侧模型任务框架。
 *
 * 数据边界：
 * - CM33 只把“已前处理、float32 40xT”的模型输入写入共享内存；
 * - CM55 从共享内存取走输入后复制到本地缓冲，再释放共享输入槽；
 * - 原始 PDM/PCM 永远不进入这块共享协议，避免采集数据和模型输入互相干扰。
 *
 * 当前模型接入点：
 * - active model compute API 输入必须是 PC/CM33 一致的 NCHW 展平 float[40 * T]；
 * - 如果模型输入 shape、格式或输出类别数改变，同步更新 shared/app_model_shared.h；
 * - 模型内部状态由导出代码管理，本模块只持有一份本地输入缓冲。
 */

static TaskHandle_t model_inference_task_handle = NULL;
static app_model_inference_stats_t model_inference_stats;
static float model_input_payload[APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS];
static bool model_runtime_initialized;

#if (APP_AUDIO_MODEL_SELECT == APP_AUDIO_MODEL_SELECT_3W_E2_PEAK_PREVIEW)
#define APP_MODEL_3W_WINDOW_COUNT                 (3u)
#define APP_MODEL_3W_INPUT_ELEMENTS               \
    (APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS * APP_MODEL_3W_WINDOW_COUNT)
#define APP_MODEL_3W_REPLAY_PHASE_NONE            (0u)
#define APP_MODEL_3W_REPLAY_PHASE_PREV2           (1u)
#define APP_MODEL_3W_REPLAY_PHASE_PREV1           (2u)
#define APP_MODEL_3W_REPLAY_PHASE_CURRENT         (3u)
#define APP_MODEL_3W_REPLAY_PHASE_MASK            (0xF0u)
#define APP_MODEL_3W_REPLAY_PHASE_SHIFT           (4u)
#define APP_MODEL_3W_POLICY_THRESHOLD             (0.95f)
#define APP_MODEL_3W_POLICY_LOCAL_CONTRAST_MIN    (0.10f)
#define APP_MODEL_3W_POLICY_REFRACTORY_TICKS      (10u)
#define APP_MODEL_3W_SCORE_BUFFER_VALID           (5u)
#define APP_MODEL_3W_SCORE_WARMUP                 (6u)
#define APP_MODEL_3W_SCORE_LOCAL_BACKGROUND       (7u)
#define APP_MODEL_3W_SCORE_LOCAL_CONTRAST         (8u)
#define APP_MODEL_3W_SCORE_THRESHOLD_PASS         (9u)
#define APP_MODEL_3W_SCORE_CONTRAST_PASS          (10u)
#define APP_MODEL_3W_SCORE_REFRACTORY_ACTIVE      (11u)
#define APP_MODEL_3W_SCORE_CONFIRMED_EVENT        (12u)
#define APP_MODEL_3W_SCORE_EVENT_ID               (13u)
#define APP_MODEL_3W_SCORE_TICK_ID                (14u)

static float model_input_payload_3w[APP_MODEL_3W_INPUT_ELEMENTS];
static float model_window_history[APP_MODEL_3W_WINDOW_COUNT]
                                 [APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS];
static float model_prob_history[APP_MODEL_3W_WINDOW_COUNT];
static uint32_t model_3w_tick_id;
static uint32_t model_3w_event_id;
static uint32_t model_3w_last_confirmed_tick;
static uint32_t model_3w_valid_window_count;

static void app_model_inference_selector6_shift_windows(const float *payload);
static void app_model_inference_selector6_store_replay_phase(uint8_t phase,
                                                             const float *payload);
static void app_model_inference_selector6_compose_input(float *payload_3w);
static bool app_model_inference_selector6_should_run_inference(
    const app_model_audio_feature_desc_t *desc,
    uint8_t replay_phase);
static void app_model_inference_selector6_fill_result(
    const app_model_audio_feature_desc_t *desc,
    app_model_inference_result_t *result,
    const float *output,
    float cough_prob);
static float app_model_inference_selector6_background(void);
static uint8_t app_model_inference_selector6_replay_phase(
    const app_model_audio_feature_desc_t *desc);
#endif
#if (APP_MODEL_INPUT_DUMP_ENABLE)
static uint32_t model_input_dump_results_written;
#endif
#if (APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE)
static uint32_t ipc_smoke_last_payload_hash;
static uint32_t ipc_smoke_last_expected_hash;
static uint32_t ipc_smoke_last_payload_mismatch;
static uint32_t ipc_smoke_last_sentinel_mismatch;
static uint32_t ipc_smoke_last_sequence_mismatch;
static uint32_t ipc_smoke_last_producer_sequence;
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
#if (APP_MODEL_SMOKE_TEST_ENABLE)
static bool app_model_inference_wait_for_shared_ready(uint32_t timeout_ms);
#endif
#if (APP_MODEL_INPUT_DUMP_ENABLE)
static void app_model_inference_fill_input_stats(
    const app_model_audio_feature_desc_t *desc,
    const float *payload,
    app_model_inference_result_t *result);
#endif
#if (APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE)
static void app_model_inference_check_ipc_payload(
    const app_model_audio_feature_desc_t *desc,
    const float *payload,
    uint32_t producer_sequence);
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
     * 这里先等待 shared region 的 magic/version 就绪；否则如果 smoke 比 CM33
     * 共享区初始化更早执行，会因为 result 发布前置检查失败而静默丢失全部
     * [MODEL_SMOKE] 结果。
     */
    (void)app_model_inference_wait_for_shared_ready(2000u);
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

    /* Acquire CM33 writes before trusting descriptor or payload. */
    __DMB();

    /* 抢占输入槽所有权，告诉 CM33 当前帧正在被 CM55 读取。 */
    shared->input_state = APP_MODEL_SHARED_INPUT_READING;
    __DMB();
    APP_MODEL_SHARED_CLEAN_CACHE((void *)shared, sizeof(*shared));

#if (APP_MODEL_IPC_SMOKE_CM55_DELAY_MS > 0u)
    vTaskDelay(pdMS_TO_TICKS(APP_MODEL_IPC_SMOKE_CM55_DELAY_MS));
#endif

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

#if (APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE)
    app_model_inference_check_ipc_payload(desc,
                                          payload,
                                          shared->producer_sequence);
#endif

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

#if (APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE)
static void app_model_inference_check_ipc_payload(
    const app_model_audio_feature_desc_t *desc,
    const float *payload,
    uint32_t producer_sequence)
{
    uint32_t element_count;
    uint32_t mid_index;
    uint32_t last_index;
    uint32_t observed_hash;
    uint32_t expected_hash;
    float expected_first;
    float expected_middle;
    float expected_last;

    if ((NULL == desc) || (NULL == payload))
    {
        return;
    }

    element_count = (uint32_t)desc->mel_bin_count *
                    (uint32_t)desc->time_bin_count;
    if (0u == element_count)
    {
        return;
    }

    mid_index = app_model_ipc_smoke_mid_index(element_count);
    last_index = app_model_ipc_smoke_last_index(element_count);
    observed_hash = app_model_ipc_smoke_hash_payload(payload, element_count);
    expected_hash = app_model_ipc_smoke_expected_hash(desc->sequence,
                                                      element_count);
    expected_first = app_model_ipc_smoke_payload_value(desc->sequence, 0u);
    expected_middle = app_model_ipc_smoke_payload_value(desc->sequence,
                                                        mid_index);
    expected_last = app_model_ipc_smoke_payload_value(desc->sequence,
                                                      last_index);

    ipc_smoke_last_payload_hash = observed_hash;
    ipc_smoke_last_expected_hash = expected_hash;
    ipc_smoke_last_payload_mismatch = (observed_hash != expected_hash) ? 1u : 0u;
    ipc_smoke_last_sentinel_mismatch =
        (!app_model_ipc_smoke_float_equal(payload[0], expected_first) ||
         !app_model_ipc_smoke_float_equal(payload[mid_index], expected_middle) ||
         !app_model_ipc_smoke_float_equal(payload[last_index], expected_last)) ? 1u : 0u;
    ipc_smoke_last_sequence_mismatch =
        (producer_sequence != desc->sequence) ? 1u : 0u;
    ipc_smoke_last_producer_sequence = producer_sequence;
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
     * payload[mel * T + time]。这里不再做反量化或额外转置，直接送入
     * 已验证过的 active model compute API。
     */
    float output[APP_AUDIO_ACTIVE_MODEL_DATA_OUT_COUNT] = { 0.0f, 0.0f };
#if (APP_AUDIO_MODEL_SELECT == APP_AUDIO_MODEL_SELECT_HZ2_0_B0_CLEANLINE)
    float aux_logits[APP_AUDIO_ACTIVE_MODEL_AUX_OUT_COUNT] = { 0.0f };
#endif

    /* 任何一个关键输入为空，都说明调用链路不完整，直接按非法输入处理。 */
    if ((NULL == desc) || (NULL == payload) || (NULL == result))
    {
        return APP_MODEL_INFERENCE_STATUS_INVALID_INPUT;
    }

#if (APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE)
    result->input_sequence = desc->sequence;
    result->timestamp_ms = app_model_inference_now_ms();
    result->class_count = 2u;
    result->scores[0] = 0.0f;
    result->scores[1] = 0.0f;
    result->scores[2] = 0.0f;
    result->scores[3] = desc->energy;
    result->scores[4] = (float)desc->selected_channel;
    result->scores[5] = (float)app_model_ipc_smoke_hash_low16(
        ipc_smoke_last_payload_hash);
    result->scores[6] = (float)app_model_ipc_smoke_hash_high16(
        ipc_smoke_last_payload_hash);
    result->scores[7] = (float)app_model_ipc_smoke_hash_low16(
        ipc_smoke_last_expected_hash);
    result->scores[8] = (float)app_model_ipc_smoke_hash_high16(
        ipc_smoke_last_expected_hash);
    result->scores[9] = (float)ipc_smoke_last_payload_mismatch;
    result->scores[10] = (float)ipc_smoke_last_sentinel_mismatch;
    result->scores[11] = (float)ipc_smoke_last_sequence_mismatch;
    result->scores[12] = (float)app_model_ipc_smoke_hash_low16(
        ipc_smoke_last_producer_sequence);
    result->scores[13] = (float)app_model_ipc_smoke_hash_high16(
        ipc_smoke_last_producer_sequence);
    return APP_MODEL_INFERENCE_STATUS_OK;
#endif

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

#if (APP_AUDIO_MODEL_SELECT == APP_AUDIO_MODEL_SELECT_3W_E2_PEAK_PREVIEW)
    {
        uint8_t replay_phase =
            app_model_inference_selector6_replay_phase(desc);
        bool should_run_inference;

        if (0u != replay_phase)
        {
            app_model_inference_selector6_store_replay_phase(replay_phase,
                                                             payload);
        }
        else
        {
            app_model_inference_selector6_shift_windows(payload);
        }

        should_run_inference =
            app_model_inference_selector6_should_run_inference(desc,
                                                               replay_phase);

        if (!should_run_inference)
        {
            result->input_sequence = desc->sequence;
            result->timestamp_ms = app_model_inference_now_ms();
            result->class_count = 0u;
            result->scores[0] = 0.0f;
            result->scores[1] = 0.0f;
            result->scores[2] = 0.0f;
            result->scores[3] = desc->energy;
            result->scores[4] = (float)desc->selected_channel;
            result->scores[APP_MODEL_3W_SCORE_BUFFER_VALID] =
                (model_3w_valid_window_count >= APP_MODEL_3W_WINDOW_COUNT) ? 1.0f : 0.0f;
            result->scores[APP_MODEL_3W_SCORE_WARMUP] =
                (model_3w_valid_window_count < APP_MODEL_3W_WINDOW_COUNT) ? 1.0f : 0.0f;
            result->scores[APP_MODEL_3W_SCORE_LOCAL_BACKGROUND] = 0.0f;
            result->scores[APP_MODEL_3W_SCORE_LOCAL_CONTRAST] = 0.0f;
            result->scores[APP_MODEL_3W_SCORE_THRESHOLD_PASS] = 0.0f;
            result->scores[APP_MODEL_3W_SCORE_CONTRAST_PASS] = 0.0f;
            result->scores[APP_MODEL_3W_SCORE_REFRACTORY_ACTIVE] = 0.0f;
            result->scores[APP_MODEL_3W_SCORE_CONFIRMED_EVENT] = 0.0f;
            result->scores[APP_MODEL_3W_SCORE_EVENT_ID] = (float)model_3w_event_id;
            result->scores[APP_MODEL_3W_SCORE_TICK_ID] = (float)model_3w_tick_id;
            return APP_MODEL_INFERENCE_STATUS_OK;
        }
    }

    result->input_sequence = desc->sequence;
    result->timestamp_ms = app_model_inference_now_ms();
    result->class_count = APP_AUDIO_ACTIVE_MODEL_DATA_OUT_COUNT;
    result->scores[0] = 0.0f;
    result->scores[1] = 0.0f;
    result->scores[2] = 0.0f;
    result->scores[3] = desc->energy;
    result->scores[4] = (float)desc->selected_channel;
    result->scores[APP_MODEL_3W_SCORE_BUFFER_VALID] =
        (model_3w_valid_window_count >= APP_MODEL_3W_WINDOW_COUNT) ? 1.0f : 0.0f;
    result->scores[APP_MODEL_3W_SCORE_WARMUP] =
        (model_3w_valid_window_count < APP_MODEL_3W_WINDOW_COUNT) ? 1.0f : 0.0f;
    result->scores[APP_MODEL_3W_SCORE_LOCAL_BACKGROUND] = 0.0f;
    result->scores[APP_MODEL_3W_SCORE_LOCAL_CONTRAST] = 0.0f;
    result->scores[APP_MODEL_3W_SCORE_THRESHOLD_PASS] = 0.0f;
    result->scores[APP_MODEL_3W_SCORE_CONTRAST_PASS] = 0.0f;
    result->scores[APP_MODEL_3W_SCORE_REFRACTORY_ACTIVE] = 0.0f;
    result->scores[APP_MODEL_3W_SCORE_CONFIRMED_EVENT] = 0.0f;
    result->scores[APP_MODEL_3W_SCORE_EVENT_ID] = (float)model_3w_event_id;
    result->scores[APP_MODEL_3W_SCORE_TICK_ID] = (float)model_3w_tick_id;

    if (model_3w_valid_window_count < APP_MODEL_3W_WINDOW_COUNT)
    {
        model_3w_tick_id++;
        return APP_MODEL_INFERENCE_STATUS_OK;
    }

    app_model_inference_selector6_compose_input(model_input_payload_3w);
    APP_AUDIO_ACTIVE_MODEL_COMPUTE(model_input_payload_3w, output);

    app_model_inference_selector6_fill_result(desc,
                                              result,
                                              output,
                                              app_model_inference_softmax_cough_prob(
                                                  output[0],
                                                  output[1]));
    model_3w_tick_id++;
    return APP_MODEL_INFERENCE_STATUS_OK;
#endif

#if (APP_MODEL_INPUT_DUMP_ENABLE)
    /* 调试模式下，把输入统计写入 result->scores 的保留槽位，
     * 便于在不额外打印大块特征的情况下快速核对输入分布。
     */
    app_model_inference_fill_input_stats(desc, payload, result);
#endif

#if (APP_AUDIO_MODEL_SELECT == APP_AUDIO_MODEL_SELECT_HZ2_0_B0_CLEANLINE)
    if (APP_AUDIO_ACTIVE_MODEL_RET_SUCCESS !=
        AUDIO_compute(payload, aux_logits, output))
    {
        return APP_MODEL_INFERENCE_STATUS_MODEL_ERROR;
    }
#else
    /* 真正调用导入后的模型计算入口。 */
    APP_AUDIO_ACTIVE_MODEL_COMPUTE(payload, output);
#endif

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

    /* Release complete result contents before publishing READY. */
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

#if (APP_MODEL_SMOKE_TEST_ENABLE)
static bool app_model_inference_wait_for_shared_ready(uint32_t timeout_ms)
{
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    for (;;)
    {
        volatile app_model_shared_region_t *shared = APP_MODEL_SHARED_REGION;

        APP_MODEL_SHARED_INVALIDATE_CACHE((void *)shared, sizeof(*shared));
        if ((APP_MODEL_SHARED_MAGIC == shared->magic) &&
            (APP_MODEL_SHARED_VERSION == shared->version))
        {
            return true;
        }

        if ((xTaskGetTickCount() - start_tick) >= timeout_ticks)
        {
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(10u));
    }
}
#endif

#if (APP_AUDIO_MODEL_SELECT == APP_AUDIO_MODEL_SELECT_3W_E2_PEAK_PREVIEW)
static void app_model_inference_selector6_shift_windows(const float *payload)
{
    if (NULL == payload)
    {
        return;
    }

    memcpy(&model_window_history[0][0],
           &model_window_history[1][0],
           sizeof(model_window_history[0]));
    memcpy(&model_window_history[1][0],
           &model_window_history[2][0],
           sizeof(model_window_history[1]));
    memcpy(&model_window_history[2][0],
           payload,
           sizeof(model_window_history[2]));

    if (model_3w_valid_window_count < APP_MODEL_3W_WINDOW_COUNT)
    {
        model_3w_valid_window_count++;
    }
}

static void app_model_inference_selector6_store_replay_phase(uint8_t phase,
                                                             const float *payload)
{
    if (NULL == payload)
    {
        return;
    }

    if (APP_MODEL_3W_REPLAY_PHASE_PREV2 == phase)
    {
        memcpy(&model_window_history[0][0],
               payload,
               sizeof(model_window_history[0]));
    }
    else if (APP_MODEL_3W_REPLAY_PHASE_PREV1 == phase)
    {
        memcpy(&model_window_history[1][0],
               payload,
               sizeof(model_window_history[1]));
    }
    else if (APP_MODEL_3W_REPLAY_PHASE_CURRENT == phase)
    {
        memcpy(&model_window_history[2][0],
               payload,
               sizeof(model_window_history[2]));
        if (model_3w_valid_window_count < APP_MODEL_3W_WINDOW_COUNT)
        {
            model_3w_valid_window_count++;
        }
    }
}

static void app_model_inference_selector6_compose_input(float *payload_3w)
{
    if (NULL == payload_3w)
    {
        return;
    }

    memcpy(&payload_3w[0],
           &model_window_history[0][0],
           sizeof(model_window_history[0]));
    memcpy(&payload_3w[APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS],
           &model_window_history[1][0],
           sizeof(model_window_history[1]));
    memcpy(&payload_3w[APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS * 2u],
           &model_window_history[2][0],
           sizeof(model_window_history[2]));
}

static bool app_model_inference_selector6_should_run_inference(
    const app_model_audio_feature_desc_t *desc,
    uint8_t replay_phase)
{
    if (NULL == desc)
    {
        return false;
    }

    if (0u == replay_phase)
    {
        return true;
    }

    if (APP_MODEL_3W_REPLAY_PHASE_CURRENT == replay_phase)
    {
        model_3w_tick_id =
            (uint32_t)((desc->sequence - 1u) / APP_MODEL_3W_WINDOW_COUNT);
        return true;
    }

    return false;
}

static float app_model_inference_selector6_background(void)
{
    float sorted[APP_MODEL_3W_WINDOW_COUNT];
    uint32_t count = 0u;

    for (uint32_t i = 0u; i < APP_MODEL_3W_WINDOW_COUNT; i++)
    {
        if (0.0f < model_prob_history[i])
        {
            sorted[count++] = model_prob_history[i];
        }
    }

    if (0u == count)
    {
        return 0.0f;
    }

    for (uint32_t i = 0u; i < count; i++)
    {
        for (uint32_t j = i + 1u; j < count; j++)
        {
            if (sorted[j] < sorted[i])
            {
                float tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }

    if (0u != (count & 1u))
    {
        return sorted[count / 2u];
    }

    return 0.5f * (sorted[(count / 2u) - 1u] + sorted[count / 2u]);
}

static void app_model_inference_selector6_fill_result(
    const app_model_audio_feature_desc_t *desc,
    app_model_inference_result_t *result,
    const float *output,
    float cough_prob)
{
    float local_background;
    float local_contrast;
    bool threshold_pass;
    bool contrast_pass;
    bool refractory_active;
    bool confirmed_event;

    local_background = app_model_inference_selector6_background();
    local_contrast = cough_prob - local_background;
    threshold_pass = (cough_prob >= APP_MODEL_3W_POLICY_THRESHOLD);
    contrast_pass = (local_contrast >= APP_MODEL_3W_POLICY_LOCAL_CONTRAST_MIN);
    refractory_active =
        (model_3w_last_confirmed_tick != 0xffffffffu) &&
        ((model_3w_tick_id - model_3w_last_confirmed_tick) <=
         APP_MODEL_3W_POLICY_REFRACTORY_TICKS);
    confirmed_event = threshold_pass && contrast_pass && (!refractory_active);

    result->input_sequence = desc->sequence;
    result->timestamp_ms = app_model_inference_now_ms();
    result->class_count = APP_AUDIO_ACTIVE_MODEL_DATA_OUT_COUNT;
    result->scores[0] = output[0];
    result->scores[1] = output[1];
    result->scores[2] = cough_prob;
    result->scores[3] = desc->energy;
    result->scores[4] = (float)desc->selected_channel;
    result->scores[APP_MODEL_3W_SCORE_BUFFER_VALID] = 1.0f;
    result->scores[APP_MODEL_3W_SCORE_WARMUP] = 0.0f;
    result->scores[APP_MODEL_3W_SCORE_LOCAL_BACKGROUND] = local_background;
    result->scores[APP_MODEL_3W_SCORE_LOCAL_CONTRAST] = local_contrast;
    result->scores[APP_MODEL_3W_SCORE_THRESHOLD_PASS] =
        threshold_pass ? 1.0f : 0.0f;
    result->scores[APP_MODEL_3W_SCORE_CONTRAST_PASS] =
        contrast_pass ? 1.0f : 0.0f;
    result->scores[APP_MODEL_3W_SCORE_REFRACTORY_ACTIVE] =
        refractory_active ? 1.0f : 0.0f;
    result->scores[APP_MODEL_3W_SCORE_CONFIRMED_EVENT] =
        confirmed_event ? 1.0f : 0.0f;

    if (confirmed_event)
    {
        model_3w_event_id++;
        model_3w_last_confirmed_tick = model_3w_tick_id;
    }

    result->scores[APP_MODEL_3W_SCORE_EVENT_ID] = (float)model_3w_event_id;
    result->scores[APP_MODEL_3W_SCORE_TICK_ID] = (float)model_3w_tick_id;

    memmove(&model_prob_history[0],
            &model_prob_history[1],
            sizeof(model_prob_history[0]) * (APP_MODEL_3W_WINDOW_COUNT - 1u));
    model_prob_history[APP_MODEL_3W_WINDOW_COUNT - 1u] = cough_prob;
}

static uint8_t app_model_inference_selector6_replay_phase(
    const app_model_audio_feature_desc_t *desc)
{
    if (NULL == desc)
    {
        return 0u;
    }

    return (uint8_t)((desc->selected_channel & APP_MODEL_3W_REPLAY_PHASE_MASK) >>
                     APP_MODEL_3W_REPLAY_PHASE_SHIFT);
}
#endif

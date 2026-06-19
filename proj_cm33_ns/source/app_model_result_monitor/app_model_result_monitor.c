#include "app_model_result_monitor.h"

#include <stdio.h>
#include <string.h>

#include "app_audio_preprocess.h"
#include "app_audio_deployment_config.h"
#include "app_model_ipc_smoke.h"
#include "app_monitor_summary.h"

#if (APP_MODEL_INFERENCE_MAX_SCORES < 5u)
#error "app_model_result_monitor requires at least 5 score slots"
#endif

#if (APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE) && (APP_MODEL_INFERENCE_MAX_SCORES < 14u)
#error "APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE requires at least 14 score slots"
#endif

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
#define APP_MODEL_DEMO_COUGH_THRESHOLD    APP_AUDIO_ACTIVE_COUGH_THRESHOLD
#endif

#ifndef APP_MODEL_RESULT_MONITOR_IDLE_DIAG_MS
#define APP_MODEL_RESULT_MONITOR_IDLE_DIAG_MS    (5000u)
#endif

#ifndef APP_MODEL_RUNTIME_PROFILE_ENABLE
#define APP_MODEL_RUNTIME_PROFILE_ENABLE         (0u)
#endif

#ifndef APP_MODEL_SMOKE_TEST_ENABLE
#define APP_MODEL_SMOKE_TEST_ENABLE              (0u)
#endif

#ifndef APP_MODEL_LOG_LEVEL
#define APP_MODEL_LOG_LEVEL                      (1u)
#endif

#ifndef APP_MODEL_LOG_RATE_LIMIT_MS
#define APP_MODEL_LOG_RATE_LIMIT_MS              (1000u)
#endif

#ifndef APP_MODEL_PRINT_FLOAT_ENABLE
#define APP_MODEL_PRINT_FLOAT_ENABLE             (1u)
#endif

#ifndef APP_MODEL_INPUT_DUMP_ENABLE
#define APP_MODEL_INPUT_DUMP_ENABLE              (0u)
#endif

#ifndef APP_MODEL_INPUT_DUMP_WINDOWS
#define APP_MODEL_INPUT_DUMP_WINDOWS             (3u)
#endif

#ifndef APP_MODEL_EVENT_THRESHOLD
#define APP_MODEL_EVENT_THRESHOLD                (0.95f)
#endif

#ifndef APP_MODEL_EVENT_MIN_HITS
#define APP_MODEL_EVENT_MIN_HITS                 (2u)
#endif

#ifndef APP_MODEL_EVENT_WINDOW_MS
#define APP_MODEL_EVENT_WINDOW_MS                (1200u)
#endif

#ifndef APP_MODEL_EVENT_COOLDOWN_MS
#define APP_MODEL_EVENT_COOLDOWN_MS              (1500u)
#endif

#ifndef APP_MODEL_EVENT_MIN_ENERGY
#define APP_MODEL_EVENT_MIN_ENERGY               (0.0f)
#endif

#ifndef APP_MODEL_SUPPRESS_LOG_RATE_LIMIT_MS
#define APP_MODEL_SUPPRESS_LOG_RATE_LIMIT_MS     APP_MODEL_LOG_RATE_LIMIT_MS
#endif

#ifndef APP_BLE_ENABLE
#define APP_BLE_ENABLE                           (0u)
#endif

#ifndef APP_BLE_FAKE_DATA_ENABLE
#define APP_BLE_FAKE_DATA_ENABLE                 (0u)
#endif

#ifndef APP_BLE_STACK_ENABLE
#define APP_BLE_STACK_ENABLE                     (0u)
#endif

#if (APP_BLE_ENABLE)
#include "app_ble_cmd.h"
#include "app_ble_diag.h"
#endif

#ifndef APP_MONITOR_SUMMARY_AUDIO_STALE_MS
#define APP_MONITOR_SUMMARY_AUDIO_STALE_MS        (2000u)
#endif

#define APP_MODEL_LOG_LEVEL_QUIET                (0u)
#define APP_MODEL_LOG_LEVEL_STAT                 (1u)
#define APP_MODEL_LOG_LEVEL_DEBUG                (2u)

#if (APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE || APP_AUDIO_EVENT_PCM_DUMP_ENABLE)
#define APP_MODEL_EVENT_DUMP_CONTEXT_ENABLE      (1u)
#else
#define APP_MODEL_EVENT_DUMP_CONTEXT_ENABLE      (0u)
#endif

typedef struct
{
    float last_cough_prob;
    float max_cough_prob_1s;
    uint32_t decision_count_cough_1s;
    uint32_t decision_count_non_cough_1s;
    uint32_t infer_ms_total_1s;
    uint32_t infer_ms_count_1s;
    uint32_t infer_ms_max_1s;
    uint32_t last_input_sequence;
    uint32_t last_result_sequence;
    uint32_t event_id;
    uint32_t last_stat_ms;
    uint32_t prev_dropped_total;
    uint32_t prev_mel_ms_total;
    uint32_t prev_mel_windows_profiled;
    uint32_t prev_condition_ms_total;
    uint32_t prev_condition_windows_profiled;
    uint32_t prev_spectrum_ms_total;
    uint32_t prev_spectrum_windows_profiled;
    uint32_t prev_melbank_ms_total;
    uint32_t prev_melbank_windows_profiled;
    uint32_t prev_suppress_count_total;
    uint32_t event_candidate_start_ms;
    uint32_t event_candidate_hits;
    uint32_t last_suppress_log_ms;
    uint32_t last_event_ms;
    float event_candidate_max_energy;
    float last_event_max_energy;
    float last_suppressed_energy;
    float max_suppressed_energy_1s;
} app_model_result_monitor_runtime_t;

#if (APP_MODEL_EVENT_DUMP_CONTEXT_ENABLE)
#define APP_MODEL_EVENT_CONTEXT_DEPTH            (5u)

typedef struct
{
    uint32_t t_ms;
    uint32_t input_sequence;
    uint32_t result_sequence;
    float cough_prob;
    float energy;
    bool threshold_pass;
    bool valid;
} app_model_event_context_entry_t;
#endif

static app_model_result_monitor_runtime_t model_result_runtime;
#if (APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE)
static uint32_t ipc_smoke_result_check_logs;
static uint32_t ipc_smoke_result_partial_rejected;
#endif
#if (APP_MODEL_EVENT_DUMP_CONTEXT_ENABLE)
static app_model_event_context_entry_t model_event_context[
    APP_MODEL_EVENT_CONTEXT_DEPTH];
static uint32_t model_event_context_write_index;
#endif

static const char *app_model_result_monitor_status_name(uint8_t status);
static void app_model_result_monitor_print_float(float value);
static void app_model_result_monitor_print_float_force(float value);
static void app_model_result_monitor_print_deployment_info(void);
static void app_model_result_monitor_update_status_counts(
    const app_model_inference_result_t *result);
static void app_model_result_monitor_note_live_result(
    const app_model_inference_result_t *result,
    uint32_t result_sequence,
    uint32_t now_ms);
static void app_model_result_monitor_bridge_summary(uint32_t now_ms);
static bool app_model_result_monitor_has_live_scores(
    const app_model_inference_result_t *result);
#if (APP_MODEL_EVENT_DUMP_CONTEXT_ENABLE)
static void app_model_result_monitor_note_event_context(
    const app_model_inference_result_t *result,
    uint32_t result_sequence,
    uint32_t now_ms);
static void app_model_result_monitor_print_event_context(uint32_t event_seq);
#endif
static void app_model_result_monitor_print_smoke_result(
    const app_model_inference_result_t *result);
#if (APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE)
static void app_model_result_monitor_check_ipc_smoke_result(
    const app_model_inference_result_t *result,
    uint32_t result_sequence,
    uint32_t now_ms);
#endif
#if (APP_MODEL_LOG_LEVEL >= APP_MODEL_LOG_LEVEL_DEBUG)
static void app_model_result_monitor_print_demo_result(
    const app_model_inference_result_t *result,
    uint32_t result_sequence);
#endif
static void app_model_result_monitor_print_event(
    const app_model_inference_result_t *result,
    uint32_t result_sequence,
    const char *decision);
static void app_model_result_monitor_print_suppress(
    const app_model_inference_result_t *result,
    uint32_t result_sequence,
    uint32_t now_ms,
    float max_energy);
static bool app_model_result_monitor_should_print_event(
    const app_model_inference_result_t *result,
    uint32_t result_sequence,
    uint32_t now_ms);
#if (!APP_MODEL_SMOKE_TEST_ENABLE)
#if (APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE)
static void app_model_result_monitor_maybe_print_ipc_smoke_stat(
    const volatile app_model_shared_region_t *shared,
    uint32_t now_ms);
#else
static void app_model_result_monitor_maybe_print_stat(
    const volatile app_model_shared_region_t *shared,
    uint32_t now_ms);
#endif
#endif
static void app_model_result_monitor_print_idle_diag(
    const volatile app_model_shared_region_t *shared);
static uint32_t app_model_result_monitor_log_start(void);
static void app_model_result_monitor_log_end(uint32_t start_ms);
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

#if (APP_MODEL_LOG_LEVEL >= APP_MODEL_LOG_LEVEL_STAT)
    app_model_result_monitor_print_deployment_info();
#endif

    for (;;)
    {
        volatile app_model_shared_region_t *shared;
        static uint32_t last_idle_diag_ms;
        static uint32_t last_boot_gate_diag_ms;
        static bool boot_gate_diag_printed;
        uint32_t now_ms = app_model_result_monitor_now_ms();

        /* Do not invalidate or inspect retained shared contents until the
         * CM33 boot owner has completed the magic-last reset sequence.
         */
        if (!app_model_shared_boot_is_ready())
        {
            if ((!boot_gate_diag_printed) ||
                ((now_ms - last_boot_gate_diag_ms) >=
                 APP_MODEL_RESULT_MONITOR_IDLE_DIAG_MS))
            {
#if (APP_MODEL_LOG_LEVEL >= APP_MODEL_LOG_LEVEL_STAT)
                uint32_t log_start_ms =
                    app_model_result_monitor_log_start();

                printf("[IPC_BOOT_GATE] core=CM33 task=model_result "
                       "t_ms=%lu ready=0\r\n",
                       (unsigned long)now_ms);
                fflush(stdout);
                app_model_result_monitor_log_end(log_start_ms);
#endif
                last_boot_gate_diag_ms = now_ms;
                boot_gate_diag_printed = true;
            }

            vTaskDelay(pdMS_TO_TICKS(APP_MODEL_RESULT_MONITOR_POLL_MS));
            continue;
        }

        shared = APP_MODEL_SHARED_REGION;
        APP_MODEL_SHARED_INVALIDATE_CACHE((void *)shared, sizeof(*shared));

        if ((APP_MODEL_SHARED_MAGIC != shared->magic) ||
            (APP_MODEL_SHARED_VERSION != shared->version))
        {
            model_result_monitor_stats.shared_not_ready++;
            if ((now_ms - last_idle_diag_ms) >=
                APP_MODEL_RESULT_MONITOR_IDLE_DIAG_MS)
            {
#if (APP_MODEL_LOG_LEVEL >= APP_MODEL_LOG_LEVEL_STAT)
                uint32_t log_start_ms =
                    app_model_result_monitor_log_start();

                printf("[MODEL_DIAG] t_ms=%lu, shared_not_ready, magic=0x%08lx, "
                       "version=%lu\r\n",
                       (unsigned long)now_ms,
                       (unsigned long)shared->magic,
                       (unsigned long)shared->version);
                fflush(stdout);
                app_model_result_monitor_log_end(log_start_ms);
#endif
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
            uint32_t result_sequence;

            /* Acquire CM55 result writes before copying result fields. */
            __DMB();
            result_sequence = shared->result_sequence;

            /* 先把结果槽复制到本地栈变量，避免后续打印过程反复直接访问共享区。 */
            memcpy(&result, (const void *)&shared->result, sizeof(result));

            /* 复制后再确认一次状态和序号，避免刚好撞上 CM55 覆盖最新结果。
             * 只有“状态仍是 READY 且 result_sequence 未变化”时，才认为这份副本有效。
             */
            __DMB();
            APP_MODEL_SHARED_INVALIDATE_CACHE((void *)shared, sizeof(*shared));
            if ((APP_MODEL_SHARED_RESULT_READY != shared->result_state) ||
                (result_sequence != shared->result_sequence))
            {
#if (APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE)
                ipc_smoke_result_partial_rejected++;
                if (ipc_smoke_result_check_logs < APP_MODEL_IPC_SMOKE_LOG_LIMIT)
                {
                    app_model_ipc_smoke_log_lock();
                    printf("[IPC_RESULT_REJECT] core=CM33 epoch=%lu t_ms=%lu "
                           "reason=unstable_copy, "
                           "seq_before=%lu, seq_after=%lu, state_after=%lu, "
                           "partial_result_accepted=0, partial_rejected_total=%lu\r\n",
                           (unsigned long)APP_MODEL_IPC_SMOKE_BOOT_EPOCH,
                           (unsigned long)app_model_result_monitor_now_ms(),
                           (unsigned long)result_sequence,
                           (unsigned long)shared->result_sequence,
                           (unsigned long)shared->result_state,
                           (unsigned long)ipc_smoke_result_partial_rejected);
                    fflush(stdout);
                    app_model_ipc_smoke_log_unlock();
                }
#endif
                vTaskDelay(pdMS_TO_TICKS(APP_MODEL_RESULT_MONITOR_POLL_MS));
                continue;
            }

            model_result_monitor_stats.has_result = true;
            model_result_monitor_stats.results_seen++;
            model_result_monitor_stats.last_result_sequence =
                result_sequence;
            model_result_monitor_stats.last_input_sequence =
                result.input_sequence;
            model_result_monitor_stats.last_status = result.status;
            app_model_result_monitor_update_status_counts(&result);

#if (APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE)
            app_model_result_monitor_check_ipc_smoke_result(&result,
                                                            result_sequence,
                                                            now_ms);
#endif

            if ((APP_MODEL_INFERENCE_STATUS_OK == result.status) &&
                (APP_MODEL_RESULT_SEQUENCE_SMOKE_BASE <= result_sequence) &&
                (2u <= result.class_count))
            {
                app_model_result_monitor_print_smoke_result(&result);
            }
            else if (app_model_result_monitor_has_live_scores(&result))
            {
                const char *decision =
                    (APP_MODEL_DEMO_COUGH_THRESHOLD <= result.scores[2]) ?
                    "COUGH" : "NON_COUGH";

                /* 对正式业务结果，先更新运行时统计，再按日志级别决定是否打印。 */
                app_model_result_monitor_note_live_result(&result,
                                                          result_sequence,
                                                          now_ms);
#if (APP_MODEL_EVENT_DUMP_CONTEXT_ENABLE)
                app_model_result_monitor_note_event_context(
                    &result,
                    result_sequence,
                    app_model_result_monitor_now_ms());
#endif

#if (APP_MODEL_LOG_LEVEL >= APP_MODEL_LOG_LEVEL_DEBUG)
                app_model_result_monitor_print_demo_result(&result,
                                                           result_sequence);
#endif
#if (APP_MODEL_LOG_LEVEL >= APP_MODEL_LOG_LEVEL_STAT)
                if (app_model_result_monitor_should_print_event(
                        &result,
                        result_sequence,
                        app_model_result_monitor_now_ms()))
                {
                    app_model_result_monitor_print_event(&result,
                                                         result_sequence,
                                                         decision);
#if (APP_MODEL_EVENT_DUMP_CONTEXT_ENABLE)
                    if (
#if (APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE)
                        app_audio_preprocess_event_feature_dump_can_emit()
#else
                        false
#endif
#if (APP_AUDIO_EVENT_PCM_DUMP_ENABLE)
                        || app_audio_preprocess_event_pcm_dump_can_emit()
#endif
                    )
                    {
                        app_model_result_monitor_print_event_context(
                            result.input_sequence);
#if (APP_AUDIO_EVENT_PCM_DUMP_ENABLE)
                        app_audio_preprocess_dump_event_pcm(
                            result.input_sequence,
                            result_sequence,
                            model_result_runtime.event_id,
                            result.scores[2],
                            result.scores[3]);
#endif
#if (APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE)
                        app_audio_preprocess_dump_event_feature(
                            result.input_sequence,
                            result_sequence,
                            model_result_runtime.event_id,
                            result.scores[2],
                            result.scores[3]);
#endif
                    }
#endif
                }
#endif
            }
            else
            {
#if (APP_MODEL_LOG_LEVEL >= APP_MODEL_LOG_LEVEL_STAT)
                uint32_t log_start_ms =
                    app_model_result_monitor_log_start();

                printf("[MODEL_RESULT] t_ms=%lu, input_seq=%lu, result_seq=%lu, "
                       "status=%s(%u), class_count=%u, infer_ms=%lu\r\n",
                       (unsigned long)app_model_result_monitor_now_ms(),
                       (unsigned long)result.input_sequence,
                       (unsigned long)result_sequence,
                       app_model_result_monitor_status_name(result.status),
                       (unsigned int)result.status,
                       (unsigned int)result.class_count,
                       (unsigned long)result.inference_time_ms);
                app_model_result_monitor_log_end(log_start_ms);
#endif
            }

#if (APP_MODEL_RESULT_MONITOR_CLEAR_AFTER_READ)
            __DMB();
            shared->result_state = APP_MODEL_SHARED_RESULT_EMPTY;
            __DMB();
            APP_MODEL_SHARED_CLEAN_CACHE((void *)shared, sizeof(*shared));
#endif
            last_idle_diag_ms = now_ms;
        }
        else if (((now_ms - last_idle_diag_ms) >=
                  APP_MODEL_RESULT_MONITOR_IDLE_DIAG_MS))
        {
#if (APP_MODEL_LOG_LEVEL >= APP_MODEL_LOG_LEVEL_STAT)
            app_model_result_monitor_print_idle_diag(shared);
            fflush(stdout);
#endif
            last_idle_diag_ms = now_ms;
        }

#if (!APP_MODEL_SMOKE_TEST_ENABLE)
        app_model_result_monitor_bridge_summary(app_model_result_monitor_now_ms());
#if (APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE)
        app_model_result_monitor_maybe_print_ipc_smoke_stat(
            shared,
            app_model_result_monitor_now_ms());
#else
        app_model_result_monitor_maybe_print_stat(
            shared,
            app_model_result_monitor_now_ms());
#endif
#endif

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
#if (APP_MODEL_PRINT_FLOAT_ENABLE)
    app_model_result_monitor_print_float_force(value);
#else
    (void)value;
    printf("na");
#endif
}

static void app_model_result_monitor_print_float_force(float value)
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

static void app_model_result_monitor_print_deployment_info(void)
{
    uint32_t log_start_ms = app_model_result_monitor_log_start();

    printf("[MODEL_INFO] model_name=%s, model_version=%s, input_shape=%s, "
           "class_order=%s, threshold=",
           APP_AUDIO_ACTIVE_MODEL_NAME,
           APP_AUDIO_ACTIVE_MODEL_VERSION,
           APP_AUDIO_ACTIVE_INPUT_SHAPE,
           APP_AUDIO_ACTIVE_CLASS_ORDER);
    app_model_result_monitor_print_float(APP_MODEL_DEMO_COUGH_THRESHOLD);
    printf(", event_threshold=");
    app_model_result_monitor_print_float(APP_MODEL_EVENT_THRESHOLD);
    printf(", event_min_energy=");
    app_model_result_monitor_print_float(APP_MODEL_EVENT_MIN_ENERGY);
    printf(", frontend_name=%s, APP_AUDIO_MODEL_SELECT=%lu, "
           "APP_MODEL_RUNTIME_PROFILE_ENABLE=%lu, APP_MODEL_LOG_LEVEL=%lu, "
           "APP_MODEL_LOG_RATE_LIMIT_MS=%lu, APP_MODEL_PRINT_FLOAT_ENABLE=%lu, "
           "APP_BLE_ENABLE=%lu, APP_BLE_FAKE_DATA_ENABLE=%lu, "
           "APP_BLE_STACK_ENABLE=%lu, APP_AUDIO_SPECTRUM_BACKEND=%lu, "
           "APP_PDM_PCM_BLOCK_COUNT=%lu, APP_MODEL_EVENT_MIN_HITS=%lu, "
           "APP_MODEL_EVENT_WINDOW_MS=%lu, APP_MODEL_EVENT_COOLDOWN_MS=%lu\r\n",
           APP_AUDIO_ACTIVE_FRONTEND_NAME,
           (unsigned long)APP_AUDIO_MODEL_SELECT,
           (unsigned long)APP_MODEL_RUNTIME_PROFILE_ENABLE,
           (unsigned long)APP_MODEL_LOG_LEVEL,
           (unsigned long)APP_MODEL_LOG_RATE_LIMIT_MS,
           (unsigned long)APP_MODEL_PRINT_FLOAT_ENABLE,
           (unsigned long)APP_BLE_ENABLE,
           (unsigned long)APP_BLE_FAKE_DATA_ENABLE,
           (unsigned long)APP_BLE_STACK_ENABLE,
           (unsigned long)APP_AUDIO_SPECTRUM_BACKEND,
           (unsigned long)APP_PDM_PCM_BLOCK_COUNT,
           (unsigned long)APP_MODEL_EVENT_MIN_HITS,
           (unsigned long)APP_MODEL_EVENT_WINDOW_MS,
           (unsigned long)APP_MODEL_EVENT_COOLDOWN_MS);
    app_model_result_monitor_log_end(log_start_ms);
}

static void app_model_result_monitor_update_status_counts(
    const app_model_inference_result_t *result)
{
    if (NULL == result)
    {
        return;
    }

    /* 把最近看到的结果按状态分类累计，便于快速判断当前主要故障类型。 */
    switch ((app_model_inference_status_t)result->status)
    {
        case APP_MODEL_INFERENCE_STATUS_OK:
            model_result_monitor_stats.ok_results++;
            break;

        case APP_MODEL_INFERENCE_STATUS_INVALID_INPUT:
            model_result_monitor_stats.invalid_input_results++;
            break;

        case APP_MODEL_INFERENCE_STATUS_MODEL_NOT_READY:
            model_result_monitor_stats.model_not_ready_results++;
            break;

        case APP_MODEL_INFERENCE_STATUS_MODEL_ERROR:
            model_result_monitor_stats.model_error_results++;
            break;

        default:
            break;
    }
}

static bool app_model_result_monitor_has_live_scores(
    const app_model_inference_result_t *result)
{
    /* class_count covers the two model logits. scores[2..4] are fixed
     * auxiliary slots guarded by APP_MODEL_INFERENCE_MAX_SCORES above.
     */
    return ((NULL != result) &&
            (APP_MODEL_INFERENCE_STATUS_OK == result->status) &&
            (2u <= result->class_count));
}

static void app_model_result_monitor_note_live_result(
    const app_model_inference_result_t *result,
    uint32_t result_sequence,
    uint32_t now_ms)
{
    if (NULL == result)
    {
        return;
    }

    /* 记录最近一次正式推理的关键观测值，供 1 秒统计与事件判决复用。 */
    model_result_runtime.last_input_sequence = result->input_sequence;
    model_result_runtime.last_result_sequence = result_sequence;
    model_result_runtime.last_cough_prob = result->scores[2];
    model_result_monitor_stats.has_live_result = true;
    model_result_monitor_stats.last_cough_prob = result->scores[2];
    model_result_monitor_stats.last_result_time_ms = now_ms;
    if (model_result_runtime.max_cough_prob_1s < result->scores[2])
    {
        model_result_runtime.max_cough_prob_1s = result->scores[2];
    }
    model_result_monitor_stats.max_cough_prob_1s =
        model_result_runtime.max_cough_prob_1s;

    /* 用 demo 阈值把每帧结果粗分为 cough / non_cough，
     * 后续周期统计直接基于这里累加的计数输出。
     */
    if (APP_MODEL_DEMO_COUGH_THRESHOLD <= result->scores[2])
    {
        model_result_runtime.decision_count_cough_1s++;
    }
    else
    {
        model_result_runtime.decision_count_non_cough_1s++;
    }
    model_result_monitor_stats.decision_count_cough_1s =
        model_result_runtime.decision_count_cough_1s;
    model_result_monitor_stats.decision_count_non_cough_1s =
        model_result_runtime.decision_count_non_cough_1s;

    /* 同时累加推理耗时，用于观察 CM55 侧平均耗时和峰值耗时。 */
    model_result_runtime.infer_ms_total_1s += result->inference_time_ms;
    model_result_runtime.infer_ms_count_1s++;
    if (model_result_runtime.infer_ms_max_1s < result->inference_time_ms)
    {
        model_result_runtime.infer_ms_max_1s = result->inference_time_ms;
    }
}

static void app_model_result_monitor_bridge_summary(uint32_t now_ms)
{
#if (APP_MONITOR_SUMMARY_ENABLE)
    app_monitor_audio_input_t audio;
    app_monitor_device_input_t device;
    app_model_result_monitor_stats_t stats;
    uint32_t now_ms_local = now_ms;
    uint8_t cough_prob_x100;
    uint8_t event_threshold_x100 =
        (uint8_t)((APP_MODEL_EVENT_THRESHOLD * 100.0f) + 0.5f);

    memset(&audio, 0, sizeof(audio));
    memset(&device, 0, sizeof(device));
    memset(&stats, 0, sizeof(stats));

    app_model_result_monitor_get_stats(&stats);

    if (stats.last_cough_prob <= 0.0f)
    {
        cough_prob_x100 = 0u;
    }
    else if (stats.last_cough_prob >= 1.0f)
    {
        cough_prob_x100 = 100u;
    }
    else
    {
        cough_prob_x100 = (uint8_t)((stats.last_cough_prob * 100.0f) + 0.5f);
    }

    audio.valid = stats.has_live_result &&
                  (APP_MODEL_INFERENCE_STATUS_OK == stats.last_status);
    audio.ready = stats.has_result &&
                  (APP_MODEL_INFERENCE_STATUS_MODEL_NOT_READY != stats.last_status);
    audio.stale = (!stats.has_live_result) ||
                  ((0u != stats.last_result_time_ms) &&
                   ((now_ms_local - stats.last_result_time_ms) >
                    APP_MONITOR_SUMMARY_AUDIO_STALE_MS));
    audio.cough_prob_x100 = cough_prob_x100;
    audio.event_threshold_x100 =
        (100u < event_threshold_x100) ? 100u : event_threshold_x100;
    audio.cough_confirmed = (cough_prob_x100 >= audio.event_threshold_x100);
    audio.cough_density_high =
        (stats.decision_count_cough_1s >= APP_MODEL_EVENT_MIN_HITS);
    audio.model_status = stats.last_status;
    audio.input_sequence = stats.last_input_sequence;
    audio.result_sequence = stats.last_result_sequence;
    audio.age_ms = (0u != stats.last_result_time_ms) ?
                   (now_ms_local - stats.last_result_time_ms) : 0u;
    audio.audio_quality = audio.valid ? 100u : 0u;
    audio.mic_quality_poor = false;
    audio.reason_flags = 0u;

#if (APP_BLE_ENABLE)
    device.monitor_requested = app_ble_cmd_monitor_is_requested();
    device.ble_connected = app_ble_is_connected();
    device.time_synced = app_ble_cmd_time_is_synced();
#else
    device.monitor_requested = true;
    device.ble_connected = false;
    device.time_synced = false;
#endif
    device.model_ready = stats.has_live_result &&
                         (APP_MODEL_INFERENCE_STATUS_OK == stats.last_status);
    device.shared_memory_ready = stats.has_result;
    device.session_active = stats.has_live_result;
    device.uptime_ms = now_ms_local;
    device.device_reason_flags = 0u;

    (void)app_monitor_summary_update_audio(&audio);
    (void)app_monitor_summary_update_device(&device);
    (void)app_monitor_summary_tick(now_ms_local);
#else
    (void)now_ms;
#endif
}

#if (APP_MODEL_EVENT_DUMP_CONTEXT_ENABLE)
static void app_model_result_monitor_note_event_context(
    const app_model_inference_result_t *result,
    uint32_t result_sequence,
    uint32_t now_ms)
{
    app_model_event_context_entry_t *entry;

    if (NULL == result)
    {
        return;
    }

    entry = &model_event_context[model_event_context_write_index];
    model_event_context_write_index++;
    if (APP_MODEL_EVENT_CONTEXT_DEPTH <= model_event_context_write_index)
    {
        model_event_context_write_index = 0u;
    }

    entry->t_ms = now_ms;
    entry->input_sequence = result->input_sequence;
    entry->result_sequence = result_sequence;
    entry->cough_prob = result->scores[2];
    entry->energy = result->scores[3];
    entry->threshold_pass = (APP_MODEL_EVENT_THRESHOLD <= result->scores[2]);
    entry->valid = true;
}

static void app_model_result_monitor_print_event_context(uint32_t event_seq)
{
    uint32_t log_start_ms = app_model_result_monitor_log_start();
    bool first = true;

    printf("[AUDIO_EVENT_CONTEXT] event_seq=%lu, recent=",
           (unsigned long)event_seq);
    for (uint32_t n = 0u; n < APP_MODEL_EVENT_CONTEXT_DEPTH; n++)
    {
        uint32_t index =
            (model_event_context_write_index + n) %
            APP_MODEL_EVENT_CONTEXT_DEPTH;
        const app_model_event_context_entry_t *entry =
            &model_event_context[index];

        if (!entry->valid)
        {
            continue;
        }

        if (!first)
        {
            printf(";");
        }
        first = false;
        printf("seq:%lu/result:%lu/t_ms:%lu/pass:%lu/prob:",
               (unsigned long)entry->input_sequence,
               (unsigned long)entry->result_sequence,
               (unsigned long)entry->t_ms,
               (unsigned long)(entry->threshold_pass ? 1u : 0u));
        app_model_result_monitor_print_float(entry->cough_prob);
        printf("/energy:");
        app_model_result_monitor_print_float(entry->energy);
    }
    if (first)
    {
        printf("none");
    }
    printf("\r\n");
    app_model_result_monitor_log_end(log_start_ms);
}
#endif

static void app_model_result_monitor_print_smoke_result(
    const app_model_inference_result_t *result)
{
    uint32_t log_start_ms;

    if (NULL == result)
    {
        return;
    }

    /* Smoke output is intentionally not rate-limited and always prints numeric
     * values, even if live float printing is disabled.
     */
    log_start_ms = app_model_result_monitor_log_start();
    printf("[MODEL_SMOKE] t_ms=%lu, sample=%lu, out0=",
           (unsigned long)app_model_result_monitor_now_ms(),
           (unsigned long)result->input_sequence);
    app_model_result_monitor_print_float_force(result->scores[0]);
    printf(", out1=");
    app_model_result_monitor_print_float_force(result->scores[1]);
    printf(", cough_prob=");
    app_model_result_monitor_print_float_force(result->scores[2]);
    printf(", expected_prob=");
    app_model_result_monitor_print_float_force(result->scores[5]);
    printf(", diff=");
    app_model_result_monitor_print_float_force(result->scores[6]);
    printf(", infer_ms=%lu\r\n",
           (unsigned long)result->inference_time_ms);
    app_model_result_monitor_log_end(log_start_ms);
}

#if (APP_MODEL_IPC_SMOKE_PAYLOAD_ENABLE)
static void app_model_result_monitor_check_ipc_smoke_result(
    const app_model_inference_result_t *result,
    uint32_t result_sequence,
    uint32_t now_ms)
{
    uint32_t expected_hash;
    uint32_t observed_hash;
    uint32_t cm55_expected_hash;
    uint32_t result_sequence_mismatch;
    uint32_t payload_hash_mismatch;
    uint32_t cm55_payload_mismatch;
    uint32_t cm55_sentinel_mismatch;
    uint32_t cm55_sequence_mismatch;
    uint32_t stale_result_accepted;
    uint32_t partial_result_accepted = 0u;

    if ((NULL == result) ||
        (APP_MODEL_INFERENCE_STATUS_OK != result->status) ||
        (2u > result->class_count))
    {
        return;
    }

    expected_hash = app_model_ipc_smoke_expected_hash(
        result->input_sequence,
        APP_MODEL_AUDIO_MODEL_FLOAT_COUNT);
    observed_hash = app_model_ipc_smoke_hash_from_halves(result->scores[5],
                                                         result->scores[6]);
    cm55_expected_hash = app_model_ipc_smoke_hash_from_halves(result->scores[7],
                                                              result->scores[8]);
    result_sequence_mismatch =
        (result_sequence != result->input_sequence) ? 1u : 0u;
    payload_hash_mismatch = (observed_hash != expected_hash) ? 1u : 0u;
    cm55_payload_mismatch = (uint32_t)(result->scores[9] + 0.5f);
    cm55_sentinel_mismatch = (uint32_t)(result->scores[10] + 0.5f);
    cm55_sequence_mismatch = (uint32_t)(result->scores[11] + 0.5f);
    stale_result_accepted =
        ((0u == result->input_sequence) ||
         (cm55_expected_hash != expected_hash)) ? 1u : 0u;

    if (ipc_smoke_result_check_logs < APP_MODEL_IPC_SMOKE_LOG_LIMIT)
    {
        ipc_smoke_result_check_logs++;
        app_model_ipc_smoke_log_lock();
        printf("[IPC_RESULT_CHECK] core=CM33 epoch=%lu t_ms=%lu input_seq=%lu "
               "result_seq=%lu, expected_hash32=0x%08lx, "
               "observed_hash32=0x%08lx, cm55_expected_hash32=0x%08lx, "
               "result_sequence_mismatch=%lu, payload_hash_mismatch=%lu, "
               "cm55_payload_mismatch=%lu, cm55_sentinel_mismatch=%lu, "
               "cm55_sequence_mismatch=%lu, stale_result_accepted=%lu, "
               "partial_result_accepted=%lu, partial_rejected_total=%lu\r\n",
               (unsigned long)APP_MODEL_IPC_SMOKE_BOOT_EPOCH,
               (unsigned long)now_ms,
               (unsigned long)result->input_sequence,
               (unsigned long)result_sequence,
               (unsigned long)expected_hash,
               (unsigned long)observed_hash,
               (unsigned long)cm55_expected_hash,
               (unsigned long)result_sequence_mismatch,
               (unsigned long)payload_hash_mismatch,
               (unsigned long)cm55_payload_mismatch,
               (unsigned long)cm55_sentinel_mismatch,
               (unsigned long)cm55_sequence_mismatch,
               (unsigned long)stale_result_accepted,
               (unsigned long)partial_result_accepted,
               (unsigned long)ipc_smoke_result_partial_rejected);
        fflush(stdout);
        app_model_ipc_smoke_log_unlock();
    }
}
#endif

#if (APP_MODEL_LOG_LEVEL >= APP_MODEL_LOG_LEVEL_DEBUG)
static void app_model_result_monitor_print_demo_result(
    const app_model_inference_result_t *result,
    uint32_t result_sequence)
{
    const char *decision;
    uint32_t log_start_ms;
#if (APP_MODEL_INPUT_DUMP_ENABLE)
    static uint32_t model_input_stats_printed;
#endif

    if (NULL == result)
    {
        return;
    }

    decision = (APP_MODEL_DEMO_COUGH_THRESHOLD <= result->scores[2]) ?
               "COUGH" : "NON_COUGH";

    log_start_ms = app_model_result_monitor_log_start();
    printf("[MODEL_DEMO] t_ms=%lu, input_seq=%lu, out0=",
           (unsigned long)app_model_result_monitor_now_ms(),
           (unsigned long)result->input_sequence);
    app_model_result_monitor_print_float(result->scores[0]);
    printf(", out1=");
    app_model_result_monitor_print_float(result->scores[1]);
    printf(", cough_prob=");
    app_model_result_monitor_print_float(result->scores[2]);
    printf(", decision=%s, energy=", decision);
    app_model_result_monitor_print_float(result->scores[3]);
    printf(", channel=%lu, infer_ms=%lu\r\n",
           (unsigned long)result->scores[4],
           (unsigned long)result->inference_time_ms);
#if (APP_MODEL_INPUT_DUMP_ENABLE)
    if (model_input_stats_printed < APP_MODEL_INPUT_DUMP_WINDOWS)
    {
        printf("[MODEL_INPUT_STATS] t_ms=%lu, input_seq=%lu, result_seq=%lu, "
               "feature_min=",
               (unsigned long)app_model_result_monitor_now_ms(),
               (unsigned long)result->input_sequence,
               (unsigned long)result_sequence);
        app_model_result_monitor_print_float(result->scores[5]);
        printf(", feature_max=");
        app_model_result_monitor_print_float(result->scores[6]);
        printf(", feature_mean=");
        app_model_result_monitor_print_float(result->scores[7]);
        printf(", feature_std=");
        app_model_result_monitor_print_float(result->scores[8]);
        printf(", hash_low16=%lu, floor80_count=%lu, near0_count=%lu, sample0=",
               (unsigned long)result->scores[9],
               (unsigned long)result->scores[10],
               (unsigned long)result->scores[11]);
        app_model_result_monitor_print_float(result->scores[12]);
        printf(", sample_last=");
        app_model_result_monitor_print_float(result->scores[13]);
        printf("\r\n");
        model_input_stats_printed++;
    }
#endif
    app_model_result_monitor_log_end(log_start_ms);
}
#endif

static void app_model_result_monitor_print_event(
    const app_model_inference_result_t *result,
    uint32_t result_sequence,
    const char *decision)
{
    uint32_t log_start_ms;

    if ((NULL == result) || (NULL == decision))
    {
        return;
    }

    model_result_runtime.event_id++;
    model_result_monitor_stats.events_printed++;
    model_result_monitor_stats.last_event_id = model_result_runtime.event_id;

    log_start_ms = app_model_result_monitor_log_start();
    printf("[MODEL_EVENT] t_ms=%lu, event_id=%lu, cough_prob=",
           (unsigned long)app_model_result_monitor_now_ms(),
           (unsigned long)model_result_runtime.event_id);
    app_model_result_monitor_print_float(result->scores[2]);
    printf(", threshold=");
    app_model_result_monitor_print_float(APP_MODEL_EVENT_THRESHOLD);
    printf(", input_seq=%lu, result_seq=%lu, decision=%s, energy=",
           (unsigned long)result->input_sequence,
           (unsigned long)result_sequence,
           decision);
    app_model_result_monitor_print_float(result->scores[3]);
    printf(", max_energy=");
    app_model_result_monitor_print_float(
        model_result_runtime.last_event_max_energy);
    printf("\r\n");
    app_model_result_monitor_log_end(log_start_ms);
}

static void app_model_result_monitor_print_suppress(
    const app_model_inference_result_t *result,
    uint32_t result_sequence,
    uint32_t now_ms,
    float max_energy)
{
#if (APP_MODEL_LOG_LEVEL >= APP_MODEL_LOG_LEVEL_STAT)
    uint32_t log_start_ms;

    if (NULL == result)
    {
        return;
    }

    if ((0u != model_result_runtime.last_suppress_log_ms) &&
        (0u < APP_MODEL_SUPPRESS_LOG_RATE_LIMIT_MS) &&
        ((now_ms - model_result_runtime.last_suppress_log_ms) <
         APP_MODEL_SUPPRESS_LOG_RATE_LIMIT_MS))
    {
        return;
    }

    model_result_runtime.last_suppress_log_ms = now_ms;

    log_start_ms = app_model_result_monitor_log_start();
    printf("[MODEL_SUPPRESS] t_ms=%lu, reason=low_energy, cough_prob=",
           (unsigned long)now_ms);
    app_model_result_monitor_print_float(result->scores[2]);
    printf(", threshold=");
    app_model_result_monitor_print_float(APP_MODEL_EVENT_THRESHOLD);
    printf(", max_energy=");
    app_model_result_monitor_print_float(max_energy);
    printf(", min_energy=");
    app_model_result_monitor_print_float(APP_MODEL_EVENT_MIN_ENERGY);
    printf(", input_seq=%lu, result_seq=%lu, suppress_count_total=%lu, "
           "decision=SUPPRESSED\r\n",
           (unsigned long)result->input_sequence,
           (unsigned long)result_sequence,
           (unsigned long)model_result_monitor_stats.suppress_count_total);
    app_model_result_monitor_log_end(log_start_ms);
#else
    (void)result;
    (void)result_sequence;
    (void)now_ms;
    (void)max_energy;
#endif
}

static bool app_model_result_monitor_should_print_event(
    const app_model_inference_result_t *result,
    uint32_t result_sequence,
    uint32_t now_ms)
{
    float candidate_energy;

    if ((NULL == result) ||
        (APP_MODEL_EVENT_THRESHOLD > result->scores[2]))
    {
        model_result_runtime.event_candidate_hits = 0u;
        model_result_runtime.event_candidate_start_ms = 0u;
        model_result_runtime.event_candidate_max_energy = 0.0f;
        return false;
    }

    if ((0u != model_result_runtime.last_event_ms) &&
        (0u < APP_MODEL_EVENT_COOLDOWN_MS) &&
        ((now_ms - model_result_runtime.last_event_ms) <
         APP_MODEL_EVENT_COOLDOWN_MS))
    {
        return false;
    }

    if ((0u == model_result_runtime.event_candidate_hits) ||
        ((0u < APP_MODEL_EVENT_WINDOW_MS) &&
         ((now_ms - model_result_runtime.event_candidate_start_ms) >
          APP_MODEL_EVENT_WINDOW_MS)))
    {
        model_result_runtime.event_candidate_start_ms = now_ms;
        model_result_runtime.event_candidate_hits = 0u;
        model_result_runtime.event_candidate_max_energy = 0.0f;
    }

    candidate_energy = result->scores[3];
    if ((0u == model_result_runtime.event_candidate_hits) ||
        (model_result_runtime.event_candidate_max_energy < candidate_energy))
    {
        model_result_runtime.event_candidate_max_energy = candidate_energy;
    }

    model_result_runtime.event_candidate_hits++;
    if (model_result_runtime.event_candidate_hits < APP_MODEL_EVENT_MIN_HITS)
    {
        return false;
    }

    candidate_energy = model_result_runtime.event_candidate_max_energy;
    if (candidate_energy < APP_MODEL_EVENT_MIN_ENERGY)
    {
        model_result_monitor_stats.suppress_count_total++;
        model_result_runtime.last_suppressed_energy = candidate_energy;
        if (model_result_runtime.max_suppressed_energy_1s < candidate_energy)
        {
            model_result_runtime.max_suppressed_energy_1s = candidate_energy;
        }
        app_model_result_monitor_print_suppress(
            result,
            result_sequence,
            now_ms,
            candidate_energy);
        model_result_runtime.event_candidate_hits = 0u;
        model_result_runtime.event_candidate_start_ms = 0u;
        model_result_runtime.event_candidate_max_energy = 0.0f;
        return false;
    }

    model_result_runtime.last_event_max_energy = candidate_energy;
    model_result_runtime.event_candidate_hits = 0u;
    model_result_runtime.event_candidate_start_ms = 0u;
    model_result_runtime.event_candidate_max_energy = 0.0f;
    model_result_runtime.last_event_ms = now_ms;
    return true;
}

#if (!APP_MODEL_SMOKE_TEST_ENABLE)
#if (APP_MODEL_IPC_SMOKE_EXCLUSIVE_ENABLE)
static void app_model_result_monitor_maybe_print_ipc_smoke_stat(
    const volatile app_model_shared_region_t *shared,
    uint32_t now_ms)
{
    app_audio_preprocess_stats_t audio_stats;
    app_pdm_pcm_stats_t pdm_stats;
    uint32_t dropped_delta;
    uint32_t seq_gap;

    if ((NULL == shared) ||
        (0u == APP_MODEL_LOG_RATE_LIMIT_MS) ||
        ((now_ms - model_result_runtime.last_stat_ms) <
         APP_MODEL_LOG_RATE_LIMIT_MS))
    {
        return;
    }

    app_audio_preprocess_get_stats(&audio_stats);
    app_pdm_pcm_get_stats(&pdm_stats);
    dropped_delta = pdm_stats.dropped_total -
                    model_result_runtime.prev_dropped_total;
    seq_gap = (shared->producer_sequence >= shared->result_sequence) ?
              (shared->producer_sequence - shared->result_sequence) : 0u;

    app_model_ipc_smoke_log_lock();
    printf("[MODEL_STAT] core=CM33 epoch=%lu t_ms=%lu input_seq=%lu "
           "result_seq=%lu busy=%lu dropped_delta_1s=%lu pdm_error=%lu "
           "queue_depth=%lu seq_gap=%lu model_not_ready=%lu\r\n",
           (unsigned long)APP_MODEL_IPC_SMOKE_BOOT_EPOCH,
           (unsigned long)now_ms,
           (unsigned long)shared->producer_sequence,
           (unsigned long)shared->result_sequence,
           (unsigned long)audio_stats.shared_busy,
           (unsigned long)dropped_delta,
           (unsigned long)pdm_stats.pdm_error_count,
           (unsigned long)pdm_stats.queue_depth,
           (unsigned long)seq_gap,
           (unsigned long)model_result_monitor_stats.model_not_ready_results);
    fflush(stdout);
    app_model_ipc_smoke_log_unlock();

    model_result_runtime.last_stat_ms = now_ms;
    model_result_runtime.prev_dropped_total = pdm_stats.dropped_total;
}
#else
static void app_model_result_monitor_maybe_print_stat(
    const volatile app_model_shared_region_t *shared,
    uint32_t now_ms)
{
#if (APP_MODEL_LOG_LEVEL >= APP_MODEL_LOG_LEVEL_STAT)
    app_audio_preprocess_stats_t audio_stats;
    app_pdm_pcm_stats_t pdm_stats;
    uint32_t dropped_delta;
    uint32_t suppress_count_delta;
    uint32_t mel_count_delta;
    uint32_t mel_total_delta;
    uint32_t mel_ms_avg;
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
    uint32_t condition_count_delta;
    uint32_t condition_total_delta;
    uint32_t condition_ms_avg;
    uint32_t spectrum_count_delta;
    uint32_t spectrum_total_delta;
    uint32_t spectrum_ms_avg;
    uint32_t melbank_count_delta;
    uint32_t melbank_total_delta;
    uint32_t melbank_ms_avg;
#endif
    uint32_t infer_ms_avg;
    uint32_t total_ms_avg;
    uint32_t seq_gap;
    uint32_t log_start_ms;

    if (NULL == shared)
    {
        return;
    }

    if ((0u == APP_MODEL_LOG_RATE_LIMIT_MS) ||
        ((now_ms - model_result_runtime.last_stat_ms) <
         APP_MODEL_LOG_RATE_LIMIT_MS))
    {
        return;
    }

    app_audio_preprocess_get_stats(&audio_stats);
    app_pdm_pcm_get_stats(&pdm_stats);

    dropped_delta = pdm_stats.dropped_total -
                    model_result_runtime.prev_dropped_total;
    suppress_count_delta = model_result_monitor_stats.suppress_count_total -
                           model_result_runtime.prev_suppress_count_total;
    mel_count_delta = audio_stats.mel_windows_profiled -
                      model_result_runtime.prev_mel_windows_profiled;
    mel_total_delta = audio_stats.mel_ms_total -
                      model_result_runtime.prev_mel_ms_total;
    mel_ms_avg = (0u < mel_count_delta) ?
                 (mel_total_delta / mel_count_delta) : 0u;
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
    condition_count_delta = audio_stats.condition_windows_profiled -
        model_result_runtime.prev_condition_windows_profiled;
    condition_total_delta = audio_stats.condition_ms_total -
        model_result_runtime.prev_condition_ms_total;
    condition_ms_avg = (0u < condition_count_delta) ?
        (condition_total_delta / condition_count_delta) : 0u;
    spectrum_count_delta = audio_stats.spectrum_windows_profiled -
        model_result_runtime.prev_spectrum_windows_profiled;
    spectrum_total_delta = audio_stats.spectrum_ms_total -
        model_result_runtime.prev_spectrum_ms_total;
    spectrum_ms_avg = (0u < spectrum_count_delta) ?
        (spectrum_total_delta / spectrum_count_delta) : 0u;
    melbank_count_delta = audio_stats.melbank_windows_profiled -
        model_result_runtime.prev_melbank_windows_profiled;
    melbank_total_delta = audio_stats.melbank_ms_total -
        model_result_runtime.prev_melbank_ms_total;
    melbank_ms_avg = (0u < melbank_count_delta) ?
        (melbank_total_delta / melbank_count_delta) : 0u;
#endif
    infer_ms_avg = (0u < model_result_runtime.infer_ms_count_1s) ?
                   (model_result_runtime.infer_ms_total_1s /
                    model_result_runtime.infer_ms_count_1s) : 0u;
    total_ms_avg = mel_ms_avg + infer_ms_avg;
    seq_gap = (shared->producer_sequence >= shared->result_sequence) ?
              (shared->producer_sequence - shared->result_sequence) : 0u;

    log_start_ms = app_model_result_monitor_log_start();
    printf("[MODEL_STAT] t_ms=%lu, input_seq=%lu, result_seq=%lu, "
           "last_cough_prob=",
           (unsigned long)now_ms,
           (unsigned long)shared->producer_sequence,
           (unsigned long)shared->result_sequence);
    app_model_result_monitor_print_float(model_result_runtime.last_cough_prob);
    printf(", max_cough_prob_1s=");
    app_model_result_monitor_print_float(model_result_runtime.max_cough_prob_1s);
    printf(", decision_count_cough_1s=%lu, "
           "decision_count_non_cough_1s=%lu, mel_ms_avg=%lu, "
           "mel_ms_max=%lu",
           (unsigned long)model_result_runtime.decision_count_cough_1s,
           (unsigned long)model_result_runtime.decision_count_non_cough_1s,
           (unsigned long)mel_ms_avg,
           (unsigned long)audio_stats.mel_ms_max);
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
    printf(", condition_ms_avg=%lu, condition_ms_max=%lu, "
           "spectrum_ms_avg=%lu, spectrum_ms_max=%lu, "
           "melbank_ms_avg=%lu, melbank_ms_max=%lu",
           (unsigned long)condition_ms_avg,
           (unsigned long)audio_stats.condition_ms_max,
           (unsigned long)spectrum_ms_avg,
           (unsigned long)audio_stats.spectrum_ms_max,
           (unsigned long)melbank_ms_avg,
           (unsigned long)audio_stats.melbank_ms_max);
#endif
    printf(", infer_ms_avg=%lu, infer_ms_max=%lu, "
           "total_ms_avg=%lu, dropped_total=%lu, dropped_delta_1s=%lu, "
           "dropped_queue=%lu, dropped_no_free=%lu, dropped_paused=%lu, "
           "pdm_error=%lu, ready=%lu, published=%lu, busy=%lu, gated=%lu, "
           "queue_depth=%lu, seq_gap=%lu, model_not_ready=%lu, "
           "suppress_count_total=%lu, suppress_count_delta_1s=%lu, "
           "last_suppressed_energy=",
           (unsigned long)infer_ms_avg,
           (unsigned long)model_result_runtime.infer_ms_max_1s,
           (unsigned long)total_ms_avg,
           (unsigned long)pdm_stats.dropped_total,
           (unsigned long)dropped_delta,
           (unsigned long)pdm_stats.dropped_queue_full,
           (unsigned long)pdm_stats.dropped_no_free_block,
           (unsigned long)pdm_stats.dropped_capture_paused,
           (unsigned long)pdm_stats.pdm_error_count,
           (unsigned long)audio_stats.windows_ready,
           (unsigned long)audio_stats.windows_published,
           (unsigned long)audio_stats.shared_busy,
           (unsigned long)audio_stats.windows_energy_gated,
           (unsigned long)pdm_stats.queue_depth,
           (unsigned long)seq_gap,
           (unsigned long)model_result_monitor_stats.model_not_ready_results,
           (unsigned long)model_result_monitor_stats.suppress_count_total,
           (unsigned long)suppress_count_delta);
    app_model_result_monitor_print_float(
        model_result_runtime.last_suppressed_energy);
    printf(", max_suppressed_energy_1s=");
    app_model_result_monitor_print_float(
        model_result_runtime.max_suppressed_energy_1s);
    printf(", log_ms=%lu\r\n",
           (unsigned long)model_result_monitor_stats.last_log_ms);
    app_model_result_monitor_log_end(log_start_ms);

    model_result_runtime.last_stat_ms = now_ms;
    model_result_runtime.prev_dropped_total = pdm_stats.dropped_total;
    model_result_runtime.prev_suppress_count_total =
        model_result_monitor_stats.suppress_count_total;
    model_result_runtime.prev_mel_ms_total = audio_stats.mel_ms_total;
    model_result_runtime.prev_mel_windows_profiled =
        audio_stats.mel_windows_profiled;
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
    model_result_runtime.prev_condition_ms_total =
        audio_stats.condition_ms_total;
    model_result_runtime.prev_condition_windows_profiled =
        audio_stats.condition_windows_profiled;
    model_result_runtime.prev_spectrum_ms_total =
        audio_stats.spectrum_ms_total;
    model_result_runtime.prev_spectrum_windows_profiled =
        audio_stats.spectrum_windows_profiled;
    model_result_runtime.prev_melbank_ms_total =
        audio_stats.melbank_ms_total;
    model_result_runtime.prev_melbank_windows_profiled =
        audio_stats.melbank_windows_profiled;
#endif
    model_result_runtime.max_cough_prob_1s = 0.0f;
    model_result_runtime.decision_count_cough_1s = 0u;
    model_result_runtime.decision_count_non_cough_1s = 0u;
    model_result_monitor_stats.max_cough_prob_1s = 0.0f;
    model_result_monitor_stats.decision_count_cough_1s = 0u;
    model_result_monitor_stats.decision_count_non_cough_1s = 0u;
    model_result_runtime.infer_ms_total_1s = 0u;
    model_result_runtime.infer_ms_count_1s = 0u;
    model_result_runtime.infer_ms_max_1s = 0u;
    model_result_runtime.max_suppressed_energy_1s = 0.0f;
#else
    (void)shared;
    (void)now_ms;
#endif
}
#endif
#endif

static void app_model_result_monitor_print_idle_diag(
    const volatile app_model_shared_region_t *shared)
{
    app_audio_preprocess_stats_t stats;
    app_pdm_pcm_stats_t pdm_stats;
    uint32_t log_start_ms;

    app_audio_preprocess_get_stats(&stats);
    app_pdm_pcm_get_stats(&pdm_stats);

    /* 无结果时的低频诊断。
     * 这条日志用于区分“串口/烧录没有起来”和“模型链路暂时没有输出”：
     * - blocks=0：PDM 采集任务还没有给前处理送 block；
     * - ready/published 增长但 consumer 不增长：CM55 没消费输入，优先检查 CM55 烧录；
     * - consumer 增长但没有 MODEL_DEMO：CM55 可能在模型初始化/推理阶段异常；
     * - gated 增长：音频能量低于门限，可能环境太安静或 MIC 增益偏低。
     */
    log_start_ms = app_model_result_monitor_log_start();
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
    printf(", dropped=%lu, dropped_queue=%lu, dropped_no_free=%lu, "
           "dropped_paused=%lu, queue_depth=%lu\r\n",
           (unsigned long)pdm_stats.dropped_total,
           (unsigned long)pdm_stats.dropped_queue_full,
           (unsigned long)pdm_stats.dropped_no_free_block,
           (unsigned long)pdm_stats.dropped_capture_paused,
           (unsigned long)pdm_stats.queue_depth);
    app_model_result_monitor_log_end(log_start_ms);
}

static uint32_t app_model_result_monitor_log_start(void)
{
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
    return app_model_result_monitor_now_ms();
#else
    return 0u;
#endif
}

static void app_model_result_monitor_log_end(uint32_t start_ms)
{
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
    model_result_monitor_stats.last_log_ms =
        app_model_result_monitor_now_ms() - start_ms;
#else
    (void)start_ms;
#endif
}

static uint32_t app_model_result_monitor_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

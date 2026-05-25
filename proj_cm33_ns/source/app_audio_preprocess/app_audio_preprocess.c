#include "app_audio_preprocess.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#if ((APP_AUDIO_SPECTRUM_BACKEND != APP_AUDIO_SPECTRUM_BACKEND_DFT) && \
     (APP_AUDIO_SPECTRUM_BACKEND != APP_AUDIO_SPECTRUM_BACKEND_RFFT))
#error "Unsupported APP_AUDIO_SPECTRUM_BACKEND"
#endif

#if ((APP_AUDIO_SPECTRUM_COMPARE_ENABLE != 0u) && \
     (APP_AUDIO_SPECTRUM_COMPARE_ENABLE != 1u))
#error "Unsupported APP_AUDIO_SPECTRUM_COMPARE_ENABLE"
#endif

#if ((APP_AUDIO_SPECTRUM_BACKEND == APP_AUDIO_SPECTRUM_BACKEND_RFFT) || \
     (APP_AUDIO_SPECTRUM_COMPARE_ENABLE))
#define APP_AUDIO_PREPROCESS_RFFT_REQUIRED       (1u)
#include "arm_math.h"
#else
#define APP_AUDIO_PREPROCESS_RFFT_REQUIRED       (0u)
#endif

#define APP_AUDIO_PREPROCESS_PI                 (3.14159265358979323846f)

#ifndef APP_MODEL_RUNTIME_PROFILE_ENABLE
#define APP_MODEL_RUNTIME_PROFILE_ENABLE        (0u)
#endif

#ifndef APP_AUDIO_FEATURE_DUMP_ENABLE
#define APP_AUDIO_FEATURE_DUMP_ENABLE           (0u)
#endif

#ifndef APP_AUDIO_FEATURE_DUMP_WINDOWS
#define APP_AUDIO_FEATURE_DUMP_WINDOWS          (3u)
#endif

#if (APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE && \
     (0u == APP_AUDIO_EVENT_FEATURE_DUMP_RING_DEPTH))
#error "APP_AUDIO_EVENT_FEATURE_DUMP_RING_DEPTH must be > 0"
#endif

#if (APP_AUDIO_EVENT_PCM_DUMP_ENABLE && \
     (1u != APP_AUDIO_EVENT_PCM_DUMP_RING_DEPTH))
#error "APP_AUDIO_EVENT_PCM_DUMP_RING_DEPTH currently supports only 1"
#endif

#if (APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE || APP_AUDIO_EVENT_PCM_DUMP_ENABLE)
#define APP_AUDIO_EVENT_ANY_DUMP_ENABLE          (1u)
#else
#define APP_AUDIO_EVENT_ANY_DUMP_ENABLE          (0u)
#endif

#if (APP_AUDIO_EVENT_ANY_DUMP_ENABLE)
#ifndef APP_AUDIO_EVENT_DUMP_LINE_DELAY_MS
#define APP_AUDIO_EVENT_DUMP_LINE_DELAY_MS       (1u)
#endif

#define APP_AUDIO_EVENT_DUMP_LINE_MAX_CHARS      (640u)
#endif

/* 本文件是正式业务链路的唯一 MIC 输入前处理任务实现。
 *
 * 任务边界：
 * - 输入：app_pdm_pcm 发布的 10 ms 双通道 PCM block，格式为 L,R,L,R...
 * - 输出：shared/app_model_shared.h 定义的共享内存特征帧，payload 为 float32。
 *
 * 设计原则：
 * - app_pdm_pcm 只做采集，不做算法；
 * - app_get_data 只做测试/导出，不参与正式业务；
 * - 本模块拥有滑动窗口和特征提取参数，后续模型调整优先改配置结构；
 * - 当前频谱计算仍是参考实现，但已避免在内层循环反复调用 sin/cos。
 *   若实时性能仍不足，只需要替换 app_audio_preprocess_power_spectrum()。
 */

typedef struct
{
    uint16_t window_samples;
    uint16_t window_hop_samples;
    uint16_t frame_len_samples;
    uint16_t frame_hop_samples;
    uint16_t spectrum_bins;
    uint16_t time_bins;
    uint16_t fft_window_offset;
    uint16_t mel_edges[APP_MODEL_AUDIO_FEATURE_MAX_MEL_BINS + 2u];
    float twiddle_cos[APP_AUDIO_PREPROCESS_MAX_SPECTRUM_BINS];
    float twiddle_sin[APP_AUDIO_PREPROCESS_MAX_SPECTRUM_BINS];
    float frame_window[APP_AUDIO_PREPROCESS_MAX_FRAME_SAMPLES];
} app_audio_preprocess_plan_t;

#if (APP_AUDIO_SPECTRUM_COMPARE_ENABLE)
typedef struct
{
    float power_max_abs_diff;
    float mel_max_abs_diff;
    float logmel_max_abs_diff;
    double power_abs_sum;
    double mel_abs_sum;
    double logmel_abs_sum;
    uint32_t power_count;
    uint32_t mel_count;
    uint32_t logmel_count;
    uint32_t window_index;
} app_audio_preprocess_compare_stats_t;
#endif

#if (APP_AUDIO_EVENT_ANY_DUMP_ENABLE)
typedef struct
{
    uint32_t count;
    uint32_t hash32;
    float min_value;
    float max_value;
    float mean;
    float stddev;
} app_audio_event_feature_stats_t;
#endif

#if (APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE)
typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t count;
    uint32_t payload_bytes;
    float energy;
    uint8_t selected_channel;
    bool valid;
    app_audio_event_feature_stats_t stats;
    float feature[APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS];
} app_audio_event_feature_frame_t;
#endif

#if (APP_AUDIO_EVENT_PCM_DUMP_ENABLE)
typedef struct
{
    uint32_t count;
    uint32_t hash32;
    int16_t min_value;
    int16_t max_value;
    float mean;
    float rms;
} app_audio_event_pcm_stats_t;

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t sample_rate_hz;
    uint32_t count;
    float dc_removed_energy;
    uint8_t selected_channel;
    bool valid;
    bool locked;
    bool feature_valid;
    app_audio_event_pcm_stats_t pcm_stats;
#if (APP_AUDIO_EVENT_PCM_DUMP_INCLUDE_FEATURE)
    app_audio_event_feature_stats_t feature_stats;
#endif
    int16_t pcm[APP_AUDIO_PREPROCESS_MAX_WINDOW_SAMPLES];
#if (APP_AUDIO_EVENT_PCM_DUMP_INCLUDE_FEATURE)
    float feature[APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS];
#endif
} app_audio_event_pcm_frame_t;
#endif

static TaskHandle_t audio_preprocess_task_handle = NULL;
static app_audio_preprocess_config_t audio_preprocess_config;
static app_audio_preprocess_plan_t audio_preprocess_plan;
static app_audio_preprocess_stats_t audio_preprocess_stats;
static bool audio_preprocess_config_ready;

static int16_t mono_ring[APP_AUDIO_PREPROCESS_MAX_WINDOW_SAMPLES];
static uint16_t mono_ring_write_index;
static uint32_t mono_total_samples;
static uint32_t mono_samples_since_window;

static int16_t left_delay_line[APP_AUDIO_PREPROCESS_MAX_DELAY_SAMPLES];
static int16_t right_delay_line[APP_AUDIO_PREPROCESS_MAX_DELAY_SAMPLES];
static uint16_t delay_line_index;

static float window_buffer[APP_AUDIO_PREPROCESS_MAX_WINDOW_SAMPLES];
static float frame_buffer[APP_AUDIO_PREPROCESS_MAX_FRAME_SAMPLES];
static float power_spectrum[APP_AUDIO_PREPROCESS_MAX_SPECTRUM_BINS];
static float model_input_feature[APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS];
#if (APP_AUDIO_SPECTRUM_COMPARE_ENABLE)
static float compare_power_spectrum[APP_AUDIO_PREPROCESS_MAX_SPECTRUM_BINS];
static float compare_model_input_feature[APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS];
static uint32_t compare_windows_reported;
#endif
#if (APP_AUDIO_FEATURE_DUMP_ENABLE)
static uint32_t feature_dump_windows_reported;
#endif
#if (APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE)
static app_audio_event_feature_frame_t event_feature_ring[
    APP_AUDIO_EVENT_FEATURE_DUMP_RING_DEPTH];
static app_audio_event_feature_frame_t event_feature_dump_buffer;
static uint32_t event_feature_ring_write_index;
static uint32_t event_feature_dumps_printed;
#endif
#if (APP_AUDIO_EVENT_PCM_DUMP_ENABLE)
static app_audio_event_pcm_frame_t event_pcm_snapshot;
static uint32_t event_pcm_dumps_printed;
static bool event_pcm_pending_valid;
#endif
#if (APP_AUDIO_EVENT_ANY_DUMP_ENABLE)
static char event_dump_line[APP_AUDIO_EVENT_DUMP_LINE_MAX_CHARS];
#endif
#if (APP_AUDIO_PREPROCESS_RFFT_REQUIRED)
static arm_rfft_fast_instance_f32 audio_preprocess_rfft_instance;
static float rfft_input_buffer[APP_AUDIO_PREPROCESS_MAX_FFT_SIZE];
static float rfft_output_buffer[APP_AUDIO_PREPROCESS_MAX_FFT_SIZE];
static bool audio_preprocess_rfft_ready;
#endif

static cy_rslt_t app_audio_preprocess_validate_and_plan(
    const app_audio_preprocess_config_t *config,
    app_audio_preprocess_plan_t *plan);
static void app_audio_preprocess_reset_stream_state(void);
static bool app_audio_preprocess_block_is_valid(
    const app_pdm_pcm_block_t *block);
static uint8_t app_audio_preprocess_choose_channel(
    const app_pdm_pcm_block_t *block);
static int16_t app_audio_preprocess_mix_sample(int16_t left,
                                               int16_t right,
                                               uint8_t selected_channel);
static void app_audio_preprocess_push_mono_sample(int16_t sample);
static bool app_audio_preprocess_window_ready(void);
static bool app_audio_preprocess_extract_and_publish(uint32_t timestamp_ms,
                                                     uint8_t selected_channel);
static void app_audio_preprocess_copy_ordered_window(void);
static bool app_audio_preprocess_condition_window(float *energy);
static void app_audio_preprocess_power_spectrum(const float *frame);
static void app_audio_preprocess_power_spectrum_dft(const float *frame,
                                                    float *spectrum);
#if (APP_AUDIO_PREPROCESS_RFFT_REQUIRED)
static bool app_audio_preprocess_init_rfft(uint16_t fft_size);
static void app_audio_preprocess_power_spectrum_rfft(const float *frame,
                                                     float *spectrum);
#endif
static float app_audio_preprocess_mel_energy_from(const float *spectrum,
                                                  uint16_t mel_index);
static float app_audio_preprocess_mel_energy(uint16_t mel_index);
static void app_audio_preprocess_power_to_db_and_normalize(float max_mel_energy);
static void app_audio_preprocess_power_to_db_and_normalize_buffer(
    float *feature,
    float max_mel_energy);
static bool app_audio_preprocess_is_no_norm_profile(uint8_t frontend_profile);
static bool app_audio_preprocess_rms_gain_enabled(uint8_t frontend_profile);
static bool app_audio_preprocess_feature_zscore_enabled(uint8_t frontend_profile);
static bool app_audio_preprocess_global_feature_norm_enabled(uint8_t frontend_profile);
static void app_audio_preprocess_print_resolved_config(void);
static void app_audio_preprocess_print_float_value(float value);
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
static void app_audio_preprocess_record_stage_time(uint32_t elapsed_ms,
                                                   uint32_t *last_ms,
                                                   uint32_t *total_ms,
                                                   uint32_t *max_ms,
                                                   uint32_t *count);
static void app_audio_preprocess_record_condition_time(uint32_t elapsed_ms);
static void app_audio_preprocess_record_spectrum_time(uint32_t elapsed_ms);
static void app_audio_preprocess_record_melbank_time(uint32_t elapsed_ms);
static void app_audio_preprocess_record_mel_time(uint32_t elapsed_ms);
#endif
#if (APP_AUDIO_SPECTRUM_COMPARE_ENABLE)
static void app_audio_preprocess_compare_update(float active,
                                                float reference,
                                                float *max_abs_diff,
                                                double *abs_sum,
                                                uint32_t *count);
static void app_audio_preprocess_print_compare_result(
    const app_audio_preprocess_compare_stats_t *stats);
static void app_audio_preprocess_print_float(float value);
#endif
#if (APP_AUDIO_FEATURE_DUMP_ENABLE)
static void app_audio_preprocess_maybe_dump_feature_stats(
    const app_model_audio_feature_desc_t *desc,
    const float *feature);
static void app_audio_preprocess_dump_print_float(float value);
#endif
#if (APP_AUDIO_EVENT_ANY_DUMP_ENABLE)
static bool app_audio_preprocess_event_feature_stats(
    const float *feature,
    uint32_t element_count,
    app_audio_event_feature_stats_t *stats);
static void app_audio_preprocess_event_print_hex_word(uint32_t value);
static void app_audio_preprocess_event_dump_pace(void);
static void app_audio_preprocess_event_dump_append_hex_word(size_t *used,
                                                            uint32_t value);
static void app_audio_preprocess_event_dump_append_hex_halfword(size_t *used,
                                                                uint16_t value);
static void app_audio_preprocess_event_dump_append_text(size_t *used,
                                                        const char *text);
#endif
#if (APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE)
static void app_audio_preprocess_event_feature_remember(
    const app_model_audio_feature_desc_t *desc,
    const float *feature);
static bool app_audio_preprocess_event_feature_copy_to_dump_buffer(
    uint32_t input_sequence);
#endif
#if (APP_AUDIO_EVENT_PCM_DUMP_ENABLE)
static void app_audio_preprocess_event_pcm_capture_pending(uint32_t timestamp_ms,
                                                           uint8_t selected_channel);
static bool app_audio_preprocess_event_pcm_stats(
    const int16_t *pcm,
    uint32_t sample_count,
    app_audio_event_pcm_stats_t *stats);
static void app_audio_preprocess_event_pcm_remember(
    const app_model_audio_feature_desc_t *desc,
    const float *feature);
#endif
static void app_audio_preprocess_init_shared_region(void);
static bool app_audio_preprocess_publish_feature(uint32_t timestamp_ms,
                                                 uint8_t selected_channel,
                                                 float energy);
static uint32_t app_audio_preprocess_ms_to_samples(uint32_t sample_rate_hz,
                                                   uint16_t ms);
static float app_audio_preprocess_hz_to_mel(float hz);
static float app_audio_preprocess_mel_to_hz(float mel);
static int16_t app_audio_preprocess_saturate_i16(int32_t value);
static uint32_t app_audio_preprocess_now_ms(void);

cy_rslt_t app_audio_preprocess_task_init(void)
{
    BaseType_t ret;

    if (NULL != audio_preprocess_task_handle)
    {
        return CY_RSLT_SUCCESS;
    }

    if (!audio_preprocess_config_ready)
    {
        app_audio_preprocess_config_t default_config;

        app_audio_preprocess_get_default_config(&default_config);
        if (CY_RSLT_SUCCESS != app_audio_preprocess_configure(&default_config))
        {
            return CY_RSLT_TYPE_ERROR;
        }
    }

    ret = xTaskCreate(app_audio_preprocess_task,
                      "audio_preproc",
                      APP_AUDIO_PREPROCESS_TASK_STACK_SIZE,
                      NULL,
                      APP_AUDIO_PREPROCESS_TASK_PRIORITY,
                      &audio_preprocess_task_handle);

    return (pdPASS == ret) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
}

void app_audio_preprocess_task(void *pvParameters)
{
    (void)pvParameters;

    /* 任务启动后先初始化共享区，再清空本地流状态，确保第一帧从干净状态开始。 */
    app_audio_preprocess_init_shared_region();
    app_audio_preprocess_reset_stream_state();

    for (;;)
    {
        app_pdm_pcm_block_t block;

        /* 正式业务链路阻塞等待 10 ms PCM block；不做忙轮询，避免无谓占用 CPU。 */
        if (!app_pdm_pcm_receive_block(&block, portMAX_DELAY))
        {
            continue;
        }

        audio_preprocess_stats.blocks_received++;

        /* 记录采集序号是否连续，用于排查队列拥塞、丢块或上游采集异常。 */
        if (audio_preprocess_stats.has_last_sequence &&
            (block.sequence != (audio_preprocess_stats.last_sequence + 1u)))
        {
            audio_preprocess_stats.sequence_gaps++;
        }
        audio_preprocess_stats.last_sequence = block.sequence;
        audio_preprocess_stats.has_last_sequence = true;

        if (app_audio_preprocess_block_is_valid(&block))
        {
            uint8_t selected_channel =
                app_audio_preprocess_choose_channel(&block);

            /* 把一整个双通道 block 转成单声道样本流，并依次推入 1 s 环形窗口。 */
            for (uint16_t i = 0; i < block.samples_per_channel; i++)
            {
                uint16_t base = (uint16_t)(i * NUM_CHANNELS);
                int16_t mono = app_audio_preprocess_mix_sample(
                    block.data[base],
                    block.data[base + 1u],
                    selected_channel);

                app_audio_preprocess_push_mono_sample(mono);
            }

            audio_preprocess_stats.last_selected_channel = selected_channel;
            if (app_audio_preprocess_window_ready())
            {
                uint32_t timestamp_ms = app_audio_preprocess_now_ms();

                /* 当窗口步长达到要求后，提取一帧完整模型输入并尝试发布到共享区。 */
                audio_preprocess_stats.windows_ready++;
                if (app_audio_preprocess_extract_and_publish(timestamp_ms,
                                                             selected_channel))
                {
                    audio_preprocess_stats.windows_published++;
                }
            }
        }
        else
        {
            audio_preprocess_stats.invalid_blocks++;
        }

        /* 无论 block 是否有效，最终都必须把采集块归还给上游缓冲池。 */
        app_pdm_pcm_release_block(block.block_index);
    }
}

cy_rslt_t app_audio_preprocess_configure(
    const app_audio_preprocess_config_t *config)
{
    if ((NULL == config) || (NULL != audio_preprocess_task_handle))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    /* 注意：app_audio_preprocess_plan_t 内部包含窗函数、旋转因子和 Mel
     * 边界等较大的工作表，大小约 8 KB。CM33 non-secure 工程默认 MSP
     * 只有 4 KB，如果在 main()/scheduler 启动前把它作为局部变量放到栈上，
     * 会在创建前处理任务前直接触发栈溢出或 HardFault，串口上只看到前一条
     * BOOT 日志。这里直接写入静态 plan；如果校验失败，config_ready 不置位，
     * 后续不会使用这份未完成的计划。
     */
    if (CY_RSLT_SUCCESS !=
        app_audio_preprocess_validate_and_plan(config, &audio_preprocess_plan))
    {
        return CY_RSLT_TYPE_ERROR;
    }

#if (APP_AUDIO_PREPROCESS_RFFT_REQUIRED)
    if (!app_audio_preprocess_init_rfft(config->fft_size))
    {
        return CY_RSLT_TYPE_ERROR;
    }
#endif

    audio_preprocess_config = *config;
    audio_preprocess_config_ready = true;
    app_audio_preprocess_reset_stream_state();
    app_audio_preprocess_print_resolved_config();

    return CY_RSLT_SUCCESS;
}

const app_audio_preprocess_config_t *app_audio_preprocess_get_config(void)
{
    return audio_preprocess_config_ready ? &audio_preprocess_config : NULL;
}

void app_audio_preprocess_get_default_config(
    app_audio_preprocess_config_t *config)
{
    if (NULL == config)
    {
        return;
    }

    /* 先整体清零，确保未显式赋值的兼容字段保持确定初值。 */
    memset(config, 0, sizeof(*config));

    /* 这组默认值共同定义“板端正式前处理”的基线配置，
     * 训练端若调整窗口、Mel 或 frontend profile，应同步更新这里。
     */
    config->sample_rate_hz = SAMPLE_RATE_HZ;
    config->window_ms = APP_AUDIO_PREPROCESS_DEFAULT_WINDOW_MS;
    config->window_hop_ms = APP_AUDIO_PREPROCESS_DEFAULT_WINDOW_HOP_MS;
    config->frame_len_ms = APP_AUDIO_PREPROCESS_DEFAULT_FRAME_LEN_MS;
    config->frame_hop_ms = APP_AUDIO_PREPROCESS_DEFAULT_FRAME_HOP_MS;
    config->fft_size = APP_AUDIO_PREPROCESS_DEFAULT_FFT_SIZE;
    config->window_function = APP_AUDIO_WINDOW_HANN;
    config->mel_bin_count = APP_AUDIO_PREPROCESS_DEFAULT_MEL_BINS;
    config->mel_low_hz = APP_AUDIO_PREPROCESS_DEFAULT_MEL_LOW_HZ;
    config->mel_high_hz = APP_AUDIO_PREPROCESS_DEFAULT_MEL_HIGH_HZ;

    /* 默认使用 SELECT_BEST，让每个 block 根据短时能量自动选择左右声道中较强的一路。 */
    config->channel_mix_mode = APP_AUDIO_CHANNEL_MIX_SELECT_BEST;
    config->fixed_delay_samples = 0;

    /* frontend_profile 决定后续是否执行 RMS gain、z-score 以及具体 log-mel 对齐方式。 */
    config->frontend_profile = APP_AUDIO_PREPROCESS_DEFAULT_FRONTEND_PROFILE;
    config->energy_gate_threshold = APP_AUDIO_PREPROCESS_DEFAULT_ENERGY_GATE;
    config->normalize_target_rms = APP_AUDIO_PREPROCESS_DEFAULT_TARGET_RMS;
    config->normalize_max_gain = APP_AUDIO_PREPROCESS_DEFAULT_MAX_GAIN;
    config->log_epsilon = APP_AUDIO_PREPROCESS_DEFAULT_LOG_EPSILON;
    config->feature_mean = APP_AUDIO_PREPROCESS_DEFAULT_FEATURE_MEAN;
    config->feature_std = APP_AUDIO_PREPROCESS_DEFAULT_FEATURE_STD;

    /* 当前正式链路默认输出 float32 特征；量化字段先保留统一配置接口。 */
    config->quant_type = APP_MODEL_AUDIO_QUANT_FLOAT32;
    config->quant_scale = 1.0f;
    config->quant_zero_point = APP_AUDIO_PREPROCESS_DEFAULT_QUANT_ZERO;
}

void app_audio_preprocess_get_stats(app_audio_preprocess_stats_t *stats)
{
    if (NULL != stats)
    {
        *stats = audio_preprocess_stats;
    }
}

#if (APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE)
bool app_audio_preprocess_event_feature_dump_can_emit(void)
{
    return (event_feature_dumps_printed <
            APP_AUDIO_EVENT_FEATURE_DUMP_MAX_EVENTS);
}

void app_audio_preprocess_dump_event_feature(uint32_t input_sequence,
                                             uint32_t result_sequence,
                                             uint32_t event_index,
                                             float cough_prob,
                                             float event_energy)
{
    const uint32_t words_per_line = 20u;
    uint32_t count;

    if (!app_audio_preprocess_event_feature_dump_can_emit())
    {
        return;
    }
    event_feature_dumps_printed++;

    if (!app_audio_preprocess_event_feature_copy_to_dump_buffer(
            input_sequence))
    {
        printf("[AUDIO_FEATURE_FULL_MISS] seq=%lu, result_seq=%lu, "
               "event_index=%lu, reason=not_in_recent_ring\r\n",
               (unsigned long)input_sequence,
               (unsigned long)result_sequence,
               (unsigned long)event_index);
        fflush(stdout);
        return;
    }

    count = event_feature_dump_buffer.count;
    printf("[AUDIO_FEATURE_FULL_BEGIN] seq=%lu, result_seq=%lu, "
           "event_index=%lu, cough_prob=",
           (unsigned long)event_feature_dump_buffer.sequence,
           (unsigned long)result_sequence,
           (unsigned long)event_index);
    app_audio_preprocess_print_float_value(cough_prob);
    printf(", energy=");
    app_audio_preprocess_print_float_value(event_energy);
    printf(", feature_energy=");
    app_audio_preprocess_print_float_value(event_feature_dump_buffer.energy);
    printf(", selected_channel=%lu, count=%lu, feature_min=",
           (unsigned long)event_feature_dump_buffer.selected_channel,
           (unsigned long)count);
    app_audio_preprocess_print_float_value(
        event_feature_dump_buffer.stats.min_value);
    printf(", feature_max=");
    app_audio_preprocess_print_float_value(
        event_feature_dump_buffer.stats.max_value);
    printf(", feature_mean=");
    app_audio_preprocess_print_float_value(
        event_feature_dump_buffer.stats.mean);
    printf(", feature_std=");
    app_audio_preprocess_print_float_value(
        event_feature_dump_buffer.stats.stddev);
    printf(", hash32=");
    app_audio_preprocess_event_print_hex_word(
        event_feature_dump_buffer.stats.hash32);
    printf("\r\n");
    app_audio_preprocess_event_dump_pace();

    for (uint32_t offset = 0u; offset < count; offset += words_per_line)
    {
        uint32_t line_count = count - offset;

        if (words_per_line < line_count)
        {
            line_count = words_per_line;
        }

        size_t used = (size_t)snprintf(
            event_dump_line,
            sizeof(event_dump_line),
            "[AUDIO_FEATURE_FULL_DATA_HEX] seq=%lu, offset=%lu, "
            "count=%lu, words=",
            (unsigned long)event_feature_dump_buffer.sequence,
            (unsigned long)offset,
            (unsigned long)line_count);
        for (uint32_t i = 0u; i < line_count; i++)
        {
            uint32_t word = 0u;

            memcpy(&word,
                   &event_feature_dump_buffer.feature[offset + i],
                   sizeof(word));
            if (0u < i)
            {
                app_audio_preprocess_event_dump_append_text(&used, ",");
            }
            app_audio_preprocess_event_dump_append_hex_word(&used, word);
        }
        printf("%s\r\n", event_dump_line);
        app_audio_preprocess_event_dump_pace();
    }

    printf("[AUDIO_FEATURE_FULL_END] seq=%lu, count=%lu\r\n",
           (unsigned long)event_feature_dump_buffer.sequence,
           (unsigned long)count);
    app_audio_preprocess_event_dump_pace();
    fflush(stdout);
}
#endif

#if (APP_AUDIO_EVENT_PCM_DUMP_ENABLE)
bool app_audio_preprocess_event_pcm_dump_can_emit(void)
{
    return (event_pcm_dumps_printed < APP_AUDIO_EVENT_PCM_DUMP_MAX_EVENTS);
}

void app_audio_preprocess_dump_event_pcm(uint32_t input_sequence,
                                         uint32_t result_sequence,
                                         uint32_t event_index,
                                         float cough_prob,
                                         float event_energy)
{
    const uint32_t words_per_line = 40u;
    uint32_t count;

    if (!app_audio_preprocess_event_pcm_dump_can_emit())
    {
        return;
    }

    event_pcm_dumps_printed++;
    event_pcm_snapshot.locked = true;

    if ((!event_pcm_snapshot.valid) ||
        (event_pcm_snapshot.sequence != input_sequence) ||
        (APP_AUDIO_PREPROCESS_MAX_WINDOW_SAMPLES < event_pcm_snapshot.count))
    {
        printf("[AUDIO_PCM_FULL_MISS] seq=%lu, result_seq=%lu, "
               "event_index=%lu, reason=not_in_recent_snapshot\r\n",
               (unsigned long)input_sequence,
               (unsigned long)result_sequence,
               (unsigned long)event_index);
        fflush(stdout);
        event_pcm_snapshot.locked = false;
        return;
    }

    count = event_pcm_snapshot.count;
    printf("[AUDIO_PCM_FULL_BEGIN] seq=%lu, result_seq=%lu, "
           "event_index=%lu, cough_prob=",
           (unsigned long)event_pcm_snapshot.sequence,
           (unsigned long)result_sequence,
           (unsigned long)event_index);
    app_audio_preprocess_print_float_value(cough_prob);
    printf(", energy=");
    app_audio_preprocess_print_float_value(event_energy);
    printf(", dc_removed_energy=");
    app_audio_preprocess_print_float_value(event_pcm_snapshot.dc_removed_energy);
    printf(", selected_channel=%lu, sample_rate=%lu, count=%lu, "
           "pcm_stage=mono_selected_before_dc_removal, "
           "pcm_format=int16_mono_le, pc_scale=pcm_int16/32768.0, "
           "pcm_min=%ld, pcm_max=%ld, pcm_mean=",
           (unsigned long)event_pcm_snapshot.selected_channel,
           (unsigned long)event_pcm_snapshot.sample_rate_hz,
           (unsigned long)count,
           (long)event_pcm_snapshot.pcm_stats.min_value,
           (long)event_pcm_snapshot.pcm_stats.max_value);
    app_audio_preprocess_print_float_value(event_pcm_snapshot.pcm_stats.mean);
    printf(", pcm_rms=");
    app_audio_preprocess_print_float_value(event_pcm_snapshot.pcm_stats.rms);
    printf(", hash32=");
    app_audio_preprocess_event_print_hex_word(
        event_pcm_snapshot.pcm_stats.hash32);
    printf(", same_event_feature_available=%lu\r\n",
           (unsigned long)(event_pcm_snapshot.feature_valid ? 1u : 0u));
    app_audio_preprocess_event_dump_pace();

    for (uint32_t offset = 0u; offset < count; offset += words_per_line)
    {
        uint32_t line_count = count - offset;

        if (words_per_line < line_count)
        {
            line_count = words_per_line;
        }

        size_t used = (size_t)snprintf(
            event_dump_line,
            sizeof(event_dump_line),
            "[AUDIO_PCM_FULL_DATA_HEX] seq=%lu, offset=%lu, "
            "count=%lu, words=",
            (unsigned long)event_pcm_snapshot.sequence,
            (unsigned long)offset,
            (unsigned long)line_count);
        for (uint32_t i = 0u; i < line_count; i++)
        {
            if (0u < i)
            {
                app_audio_preprocess_event_dump_append_text(&used, ",");
            }
            app_audio_preprocess_event_dump_append_hex_halfword(
                &used,
                (uint16_t)event_pcm_snapshot.pcm[offset + i]);
        }
        printf("%s\r\n", event_dump_line);
        app_audio_preprocess_event_dump_pace();
    }

    printf("[AUDIO_PCM_FULL_END] seq=%lu, count=%lu\r\n",
           (unsigned long)event_pcm_snapshot.sequence,
           (unsigned long)count);
    app_audio_preprocess_event_dump_pace();

#if (APP_AUDIO_EVENT_PCM_DUMP_INCLUDE_FEATURE)
    if (event_pcm_snapshot.feature_valid)
    {
        const uint32_t feature_words_per_line = 20u;
        uint32_t feature_count = event_pcm_snapshot.feature_stats.count;

        printf("[AUDIO_FEATURE_FULL_BEGIN] seq=%lu, result_seq=%lu, "
               "event_index=%lu, cough_prob=",
               (unsigned long)event_pcm_snapshot.sequence,
               (unsigned long)result_sequence,
               (unsigned long)event_index);
        app_audio_preprocess_print_float_value(cough_prob);
        printf(", energy=");
        app_audio_preprocess_print_float_value(event_energy);
        printf(", feature_energy=");
        app_audio_preprocess_print_float_value(
            event_pcm_snapshot.dc_removed_energy);
        printf(", selected_channel=%lu, count=%lu, feature_min=",
               (unsigned long)event_pcm_snapshot.selected_channel,
               (unsigned long)feature_count);
        app_audio_preprocess_print_float_value(
            event_pcm_snapshot.feature_stats.min_value);
        printf(", feature_max=");
        app_audio_preprocess_print_float_value(
            event_pcm_snapshot.feature_stats.max_value);
        printf(", feature_mean=");
        app_audio_preprocess_print_float_value(
            event_pcm_snapshot.feature_stats.mean);
        printf(", feature_std=");
        app_audio_preprocess_print_float_value(
            event_pcm_snapshot.feature_stats.stddev);
        printf(", hash32=");
        app_audio_preprocess_event_print_hex_word(
            event_pcm_snapshot.feature_stats.hash32);
        printf("\r\n");
        app_audio_preprocess_event_dump_pace();

        for (uint32_t offset = 0u; offset < feature_count;
             offset += feature_words_per_line)
        {
            uint32_t line_count = feature_count - offset;

            if (feature_words_per_line < line_count)
            {
                line_count = feature_words_per_line;
            }

            size_t used = (size_t)snprintf(
                event_dump_line,
                sizeof(event_dump_line),
                "[AUDIO_FEATURE_FULL_DATA_HEX] seq=%lu, offset=%lu, "
                "count=%lu, words=",
                (unsigned long)event_pcm_snapshot.sequence,
                (unsigned long)offset,
                (unsigned long)line_count);
            for (uint32_t i = 0u; i < line_count; i++)
            {
                uint32_t word = 0u;

                memcpy(&word,
                       &event_pcm_snapshot.feature[offset + i],
                       sizeof(word));
                if (0u < i)
                {
                    app_audio_preprocess_event_dump_append_text(&used, ",");
                }
                app_audio_preprocess_event_dump_append_hex_word(&used, word);
            }
            printf("%s\r\n", event_dump_line);
            app_audio_preprocess_event_dump_pace();
        }

        printf("[AUDIO_FEATURE_FULL_END] seq=%lu, count=%lu\r\n",
               (unsigned long)event_pcm_snapshot.sequence,
               (unsigned long)feature_count);
        app_audio_preprocess_event_dump_pace();
    }
#endif

    fflush(stdout);
    event_pcm_snapshot.locked = false;
}
#endif

static cy_rslt_t app_audio_preprocess_validate_and_plan(
    const app_audio_preprocess_config_t *config,
    app_audio_preprocess_plan_t *plan)
{
    uint32_t delay_abs;
    uint16_t mel_bin_count;
    uint16_t spectrum_last_bin;
    float low_mel;
    float high_mel;

    if ((NULL == config) || (NULL == plan) ||
        (SAMPLE_RATE_HZ != config->sample_rate_hz) ||
        (0u == config->window_ms) ||
        (0u == config->window_hop_ms) ||
        (0u == config->frame_len_ms) ||
        (0u == config->frame_hop_ms) ||
        (0u == config->fft_size) ||
        (0u != (config->fft_size & 1u)) ||
        (APP_AUDIO_PREPROCESS_MAX_FFT_SIZE < config->fft_size) ||
        (APP_MODEL_AUDIO_MODEL_MEL_BINS != config->mel_bin_count) ||
        (0.0f > config->mel_low_hz) ||
        (config->mel_low_hz >= config->mel_high_hz) ||
        (((float)config->sample_rate_hz * 0.5f) < config->mel_high_hz) ||
        (0.0f > config->energy_gate_threshold) ||
        (((uint8_t)APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_V2_ZSCORE != config->frontend_profile) &&
         ((uint8_t)APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_BOARD_HTK_NO_NORM_V1 != config->frontend_profile)) ||
        (0.0f >= config->normalize_target_rms) ||
        (0.0f >= config->normalize_max_gain) ||
        (0.0f >= config->log_epsilon) ||
        (0.0f >= config->feature_std) ||
        (APP_MODEL_AUDIO_QUANT_FLOAT32 != config->quant_type) ||
        (0.0f >= config->quant_scale))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    memset(plan, 0, sizeof(*plan));
    plan->window_samples =
        (uint16_t)app_audio_preprocess_ms_to_samples(config->sample_rate_hz,
                                                     config->window_ms);
    plan->window_hop_samples =
        (uint16_t)app_audio_preprocess_ms_to_samples(config->sample_rate_hz,
                                                     config->window_hop_ms);
    plan->frame_len_samples =
        (uint16_t)app_audio_preprocess_ms_to_samples(config->sample_rate_hz,
                                                     config->frame_len_ms);
    plan->frame_hop_samples =
        (uint16_t)app_audio_preprocess_ms_to_samples(config->sample_rate_hz,
                                                     config->frame_hop_ms);
    plan->spectrum_bins = (uint16_t)((config->fft_size / 2u) + 1u);

    if ((0u == plan->window_samples) ||
        (APP_AUDIO_PREPROCESS_MAX_WINDOW_SAMPLES < plan->window_samples) ||
        (0u == plan->window_hop_samples) ||
        (plan->window_samples < plan->window_hop_samples) ||
        (0u == plan->frame_len_samples) ||
        (plan->frame_len_samples > config->fft_size) ||
        (APP_AUDIO_PREPROCESS_MAX_FRAME_SAMPLES < plan->frame_len_samples) ||
        (2u > plan->frame_len_samples) ||
        (0u == plan->frame_hop_samples) ||
        (plan->window_samples < plan->frame_len_samples))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    /* 训练端 librosa.melspectrogram() 默认 center=True，会在 1 s 音频两端按
     * n_fft/2 补零。因此 16000 点、hop=160 时得到 1 + 16000 / 160 = 101 帧。
     * 这里显式按 center padding 计算时间帧数，保证输出 shape 与模型一致。
     */
    plan->time_bins =
        (uint16_t)(1u + (plan->window_samples / plan->frame_hop_samples));
    if ((0u == plan->time_bins) ||
        (APP_MODEL_AUDIO_MODEL_TIME_BINS != plan->time_bins))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    plan->fft_window_offset =
        (uint16_t)((config->fft_size - plan->frame_len_samples) / 2u);

    delay_abs = (0 > config->fixed_delay_samples) ?
                (uint32_t)(-config->fixed_delay_samples) :
                (uint32_t)config->fixed_delay_samples;
    if (APP_AUDIO_PREPROCESS_MAX_DELAY_SAMPLES < delay_abs)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    for (uint16_t i = 0; i < plan->frame_len_samples; i++)
    {
        /* librosa 默认使用 periodic Hann/Hamming 窗；分母取 N 而不是 N-1。 */
        float phase = (2.0f * APP_AUDIO_PREPROCESS_PI * (float)i) /
                      (float)plan->frame_len_samples;

        switch (config->window_function)
        {
            case APP_AUDIO_WINDOW_HAMMING:
                plan->frame_window[i] = 0.54f - (0.46f * cosf(phase));
                break;

            case APP_AUDIO_WINDOW_RECTANGULAR:
                plan->frame_window[i] = 1.0f;
                break;

            case APP_AUDIO_WINDOW_HANN:
            default:
                plan->frame_window[i] = 0.5f - (0.5f * cosf(phase));
                break;
        }
    }

    for (uint16_t k = 0; k < plan->spectrum_bins; k++)
    {
        float phase = (2.0f * APP_AUDIO_PREPROCESS_PI * (float)k) /
                      (float)config->fft_size;

        plan->twiddle_cos[k] = cosf(phase);
        plan->twiddle_sin[k] = -sinf(phase);
    }

    mel_bin_count = config->mel_bin_count;
    spectrum_last_bin = (uint16_t)(plan->spectrum_bins - 1u);
    low_mel = app_audio_preprocess_hz_to_mel(config->mel_low_hz);
    high_mel = app_audio_preprocess_hz_to_mel(config->mel_high_hz);

    for (uint16_t i = 0; i < (uint16_t)(mel_bin_count + 2u); i++)
    {
        float mel = low_mel + ((high_mel - low_mel) *
                               ((float)i / (float)(mel_bin_count + 1u)));
        float hz = app_audio_preprocess_mel_to_hz(mel);
        uint32_t bin = (uint32_t)(((float)(config->fft_size + 1u) * hz) /
                                  (float)config->sample_rate_hz);

        if (spectrum_last_bin < bin)
        {
            bin = spectrum_last_bin;
        }
        plan->mel_edges[i] = (uint16_t)bin;
    }

    return CY_RSLT_SUCCESS;
}

static void app_audio_preprocess_reset_stream_state(void)
{
    /* 每次重新配置或任务启动时，清空本地音频状态。
     * 这些缓冲都在 CM33 本地内存中，不进入 CM33/CM55 共享区。
     */
    memset(mono_ring, 0, sizeof(mono_ring));
    memset(left_delay_line, 0, sizeof(left_delay_line));
    memset(right_delay_line, 0, sizeof(right_delay_line));
    memset(&audio_preprocess_stats, 0, sizeof(audio_preprocess_stats));
    mono_ring_write_index = 0;
    mono_total_samples = 0;
    mono_samples_since_window = 0;
    delay_line_index = 0;
#if (APP_AUDIO_SPECTRUM_COMPARE_ENABLE)
    compare_windows_reported = 0u;
#endif
}

static bool app_audio_preprocess_block_is_valid(
    const app_pdm_pcm_block_t *block)
{
    /* 校验 block 描述符，防止消费者误读到不属于 recorded_data 的指针。 */
    return ((NULL != block) &&
            (NULL != block->data) &&
            (APP_PDM_PCM_BLOCK_SAMPLES == block->sample_count) &&
            (APP_PDM_PCM_SAMPLES_PER_CH_PER_BLOCK ==
             block->samples_per_channel) &&
            (APP_PDM_PCM_BLOCK_COUNT > block->block_index) &&
            (&recorded_data[block->block_index][0] == block->data));
}

static uint8_t app_audio_preprocess_choose_channel(
    const app_pdm_pcm_block_t *block)
{
    /* 选优模式：当前先用 10 ms block 内左右通道能量比较。
     * 后续如果需要更稳定，可以改成 1 s 窗口级别的能量统计或 SNR 估计。
     */
    uint64_t left_energy = 0;
    uint64_t right_energy = 0;

    if (APP_AUDIO_CHANNEL_MIX_SELECT_BEST !=
        audio_preprocess_config.channel_mix_mode)
    {
        return APP_AUDIO_PREPROCESS_SELECTED_MIXED;
    }

    for (uint16_t i = 0; i < block->samples_per_channel; i++)
    {
        uint16_t base = (uint16_t)(i * NUM_CHANNELS);
        int32_t left = block->data[base];
        int32_t right = block->data[base + 1u];

        left_energy += (uint64_t)(left * left);
        right_energy += (uint64_t)(right * right);
    }

    return (left_energy >= right_energy) ? 0u : 1u;
}

static int16_t app_audio_preprocess_mix_sample(int16_t left,
                                               int16_t right,
                                               uint8_t selected_channel)
{
    /* 双通道转单声道。
     * 平均和固定延时求和都做 1/2 缩放，避免两个满幅信号相加后溢出。
     */
    int32_t mixed;
    int16_t delayed;
    uint16_t delay_abs;
    uint16_t read_index;

    switch (audio_preprocess_config.channel_mix_mode)
    {
        case APP_AUDIO_CHANNEL_MIX_LEFT:
            return left;

        case APP_AUDIO_CHANNEL_MIX_RIGHT:
            return right;

        case APP_AUDIO_CHANNEL_MIX_SELECT_BEST:
            return (0u == selected_channel) ? left : right;

        case APP_AUDIO_CHANNEL_MIX_DELAY_SUM:
            delay_abs = (0 > audio_preprocess_config.fixed_delay_samples) ?
                        (uint16_t)(-audio_preprocess_config.fixed_delay_samples) :
                        (uint16_t)audio_preprocess_config.fixed_delay_samples;
            if (0u == delay_abs)
            {
                mixed = ((int32_t)left + (int32_t)right) / 2;
                return app_audio_preprocess_saturate_i16(mixed);
            }

            read_index = (delay_line_index + APP_AUDIO_PREPROCESS_MAX_DELAY_SAMPLES -
                          delay_abs) % APP_AUDIO_PREPROCESS_MAX_DELAY_SAMPLES;
            if (0 < audio_preprocess_config.fixed_delay_samples)
            {
                delayed = right_delay_line[read_index];
                mixed = ((int32_t)left + (int32_t)delayed) / 2;
            }
            else
            {
                delayed = left_delay_line[read_index];
                mixed = ((int32_t)delayed + (int32_t)right) / 2;
            }

            left_delay_line[delay_line_index] = left;
            right_delay_line[delay_line_index] = right;
            delay_line_index = (uint16_t)((delay_line_index + 1u) %
                                          APP_AUDIO_PREPROCESS_MAX_DELAY_SAMPLES);
            return app_audio_preprocess_saturate_i16(mixed);

        case APP_AUDIO_CHANNEL_MIX_AVERAGE:
        default:
            mixed = ((int32_t)left + (int32_t)right) / 2;
            return app_audio_preprocess_saturate_i16(mixed);
    }
}

static void app_audio_preprocess_push_mono_sample(int16_t sample)
{
    /* 单声道 1 s 环形缓冲。写指针永远指向下一次写入位置；
     * 当窗口 ready 时，从写指针开始读一圈即可得到时间顺序正确的 1 s 数据。
     */
    mono_ring[mono_ring_write_index] = sample;
    mono_ring_write_index = (uint16_t)((mono_ring_write_index + 1u) %
                                       audio_preprocess_plan.window_samples);
    mono_total_samples++;
    mono_samples_since_window++;
}

static bool app_audio_preprocess_window_ready(void)
{
    /* 默认配置下：先等累计 16000 点，再每 8000 点触发一次窗口，
     * 即 1 s 窗口、50% 重叠。窗口长度和 hop 都来自配置，不在流程里写死。
     */
    if (mono_total_samples < audio_preprocess_plan.window_samples)
    {
        return false;
    }

    if (mono_samples_since_window < audio_preprocess_plan.window_hop_samples)
    {
        return false;
    }

    mono_samples_since_window = 0;
    return true;
}

static bool app_audio_preprocess_extract_and_publish(uint32_t timestamp_ms,
                                                     uint8_t selected_channel)
{
    /* 完整特征流水线：
     * 有序窗口 -> 去直流/归一化/能量门限 -> 分帧加窗 -> 功率谱
     * -> Mel 滤波器组 -> power_to_db(ref=max) -> 本窗口标准化 -> float32 共享内存。
     */
    float energy = 0.0f;
    float max_mel_energy = 0.0f;
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
    uint32_t profile_start_ms = app_audio_preprocess_now_ms();
    uint32_t stage_start_ms;
    uint32_t spectrum_ms_total = 0u;
    uint32_t melbank_ms_total = 0u;
#endif
#if (APP_AUDIO_SPECTRUM_COMPARE_ENABLE)
    bool compare_enabled =
        (compare_windows_reported < APP_AUDIO_SPECTRUM_COMPARE_WINDOWS);
    float compare_max_mel_energy = 0.0f;
    app_audio_preprocess_compare_stats_t compare_stats;

    if (compare_enabled)
    {
        memset(&compare_stats, 0, sizeof(compare_stats));
        compare_stats.window_index = compare_windows_reported + 1u;
    }
#endif

    app_audio_preprocess_copy_ordered_window();
#if (APP_AUDIO_EVENT_PCM_DUMP_ENABLE)
    app_audio_preprocess_event_pcm_capture_pending(timestamp_ms,
                                                   selected_channel);
#endif
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
    stage_start_ms = app_audio_preprocess_now_ms();
#endif
    if (!app_audio_preprocess_condition_window(&energy))
    {
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
        app_audio_preprocess_record_condition_time(
            app_audio_preprocess_now_ms() - stage_start_ms);
#endif
        audio_preprocess_stats.windows_energy_gated++;
        audio_preprocess_stats.last_energy = energy;
        return false;
    }
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
    app_audio_preprocess_record_condition_time(
        app_audio_preprocess_now_ms() - stage_start_ms);
#endif

    for (uint16_t t = 0; t < audio_preprocess_plan.time_bins; t++)
    {
        int32_t frame_start =
            ((int32_t)t * (int32_t)audio_preprocess_plan.frame_hop_samples) -
            ((int32_t)audio_preprocess_plan.frame_len_samples / 2);

        /* center=True 对应窗口中心落在 t * hop 的位置。
         * 超出 1 s 原始窗口的左右边界按 0 补齐；这正是 PC 侧 librosa 默认行为。
         */
        memset(frame_buffer, 0,
               (size_t)audio_preprocess_config.fft_size * sizeof(frame_buffer[0]));

        for (uint16_t n = 0; n < audio_preprocess_plan.frame_len_samples; n++)
        {
            int32_t source_index = frame_start + (int32_t)n;
            uint16_t frame_index =
                (uint16_t)(audio_preprocess_plan.fft_window_offset + n);

            if ((0 <= source_index) &&
                ((int32_t)audio_preprocess_plan.window_samples > source_index))
            {
                frame_buffer[frame_index] =
                    window_buffer[source_index] *
                    audio_preprocess_plan.frame_window[n];
            }
        }

#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
        stage_start_ms = app_audio_preprocess_now_ms();
#endif
        app_audio_preprocess_power_spectrum(frame_buffer);
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
        spectrum_ms_total += app_audio_preprocess_now_ms() - stage_start_ms;
#endif

#if (APP_AUDIO_SPECTRUM_COMPARE_ENABLE)
        if (compare_enabled)
        {
#if (APP_AUDIO_SPECTRUM_BACKEND == APP_AUDIO_SPECTRUM_BACKEND_RFFT)
            app_audio_preprocess_power_spectrum_dft(frame_buffer,
                                                    compare_power_spectrum);
#else
            app_audio_preprocess_power_spectrum_rfft(frame_buffer,
                                                     compare_power_spectrum);
#endif
            for (uint16_t k = 0; k < audio_preprocess_plan.spectrum_bins; k++)
            {
                app_audio_preprocess_compare_update(
                    power_spectrum[k],
                    compare_power_spectrum[k],
                    &compare_stats.power_max_abs_diff,
                    &compare_stats.power_abs_sum,
                    &compare_stats.power_count);
            }
        }
#endif

#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
        stage_start_ms = app_audio_preprocess_now_ms();
#endif
        for (uint16_t mel = 0; mel < audio_preprocess_config.mel_bin_count; mel++)
        {
            float mel_energy = app_audio_preprocess_mel_energy(mel);
            uint16_t out_index =
                (uint16_t)((mel * audio_preprocess_plan.time_bins) + t);

            model_input_feature[out_index] = mel_energy;
            if (max_mel_energy < mel_energy)
            {
                max_mel_energy = mel_energy;
            }
        }
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
        melbank_ms_total += app_audio_preprocess_now_ms() - stage_start_ms;
#endif

#if (APP_AUDIO_SPECTRUM_COMPARE_ENABLE)
        if (compare_enabled)
        {
            for (uint16_t mel = 0; mel < audio_preprocess_config.mel_bin_count; mel++)
            {
                float active_mel_energy;
                float compare_mel_energy =
                    app_audio_preprocess_mel_energy_from(compare_power_spectrum,
                                                         mel);
                uint16_t out_index =
                    (uint16_t)((mel * audio_preprocess_plan.time_bins) + t);

                compare_model_input_feature[out_index] = compare_mel_energy;
                if (compare_max_mel_energy < compare_mel_energy)
                {
                    compare_max_mel_energy = compare_mel_energy;
                }

                active_mel_energy = model_input_feature[out_index];
                app_audio_preprocess_compare_update(
                    active_mel_energy,
                    compare_mel_energy,
                    &compare_stats.mel_max_abs_diff,
                    &compare_stats.mel_abs_sum,
                    &compare_stats.mel_count);
            }
        }
#endif
    }

    app_audio_preprocess_power_to_db_and_normalize(max_mel_energy);

#if (APP_AUDIO_SPECTRUM_COMPARE_ENABLE)
    if (compare_enabled)
    {
        uint32_t element_count =
            (uint32_t)audio_preprocess_config.mel_bin_count *
            (uint32_t)audio_preprocess_plan.time_bins;

        app_audio_preprocess_power_to_db_and_normalize_buffer(
            compare_model_input_feature,
            compare_max_mel_energy);

        for (uint32_t i = 0; i < element_count; i++)
        {
            app_audio_preprocess_compare_update(
                model_input_feature[i],
                compare_model_input_feature[i],
                &compare_stats.logmel_max_abs_diff,
                &compare_stats.logmel_abs_sum,
                &compare_stats.logmel_count);
        }

        app_audio_preprocess_print_compare_result(&compare_stats);
        compare_windows_reported++;
    }
#endif

    audio_preprocess_stats.last_energy = energy;
#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
    app_audio_preprocess_record_spectrum_time(spectrum_ms_total);
    app_audio_preprocess_record_melbank_time(melbank_ms_total);
    app_audio_preprocess_record_mel_time(
        app_audio_preprocess_now_ms() - profile_start_ms);
#endif
    return app_audio_preprocess_publish_feature(timestamp_ms,
                                                selected_channel,
                                                energy);
}

static void app_audio_preprocess_copy_ordered_window(void)
{
    /* 环形缓冲转线性窗口，同时把 int16 PCM 转为 [-1, 1) 浮点。 */
    uint16_t start = mono_ring_write_index;

    for (uint16_t i = 0; i < audio_preprocess_plan.window_samples; i++)
    {
        uint16_t index =
            (uint16_t)((start + i) % audio_preprocess_plan.window_samples);

        window_buffer[i] = (float)mono_ring[index] / 32768.0f;
    }
}

static bool app_audio_preprocess_condition_window(float *energy)
{
    /* 去直流和可选 RMS 归一化。
     * energy_gate_threshold 使用 RMS 归一化前能量，这样静音不会被
     * normalize_max_gain 放大后误判为有效语音。
     */
    double sum = 0.0;
    double square_sum = 0.0;
    float mean;
    float rms;
    float gain;

    for (uint16_t i = 0; i < audio_preprocess_plan.window_samples; i++)
    {
        sum += window_buffer[i];
    }
    mean = (float)(sum / audio_preprocess_plan.window_samples);

    for (uint16_t i = 0; i < audio_preprocess_plan.window_samples; i++)
    {
        float sample = window_buffer[i] - mean;

        window_buffer[i] = sample;
        square_sum += ((double)sample * (double)sample);
    }

    *energy = (float)(square_sum / audio_preprocess_plan.window_samples);
    if (*energy < audio_preprocess_config.energy_gate_threshold)
    {
        return false;
    }

    if (!app_audio_preprocess_rms_gain_enabled(
            audio_preprocess_config.frontend_profile))
    {
        return true;
    }

    rms = sqrtf(*energy);
    gain = audio_preprocess_config.normalize_target_rms / rms;
    if (audio_preprocess_config.normalize_max_gain < gain)
    {
        gain = audio_preprocess_config.normalize_max_gain;
    }

    for (uint16_t i = 0; i < audio_preprocess_plan.window_samples; i++)
    {
        window_buffer[i] *= gain;
    }

    return true;
}

static void app_audio_preprocess_power_spectrum(const float *frame)
{
#if (APP_AUDIO_SPECTRUM_BACKEND == APP_AUDIO_SPECTRUM_BACKEND_RFFT)
    app_audio_preprocess_power_spectrum_rfft(frame, power_spectrum);
#else
    app_audio_preprocess_power_spectrum_dft(frame, power_spectrum);
#endif
}

static void app_audio_preprocess_power_spectrum_dft(const float *frame,
                                                    float *spectrum)
{
    /* 参考实现：直接 DFT 得到单边功率谱。
     *
     * 这条路径保留为 RFFT 等价性检查和回退基准。它输出未归一化的
     * real^2 + imag^2，RFFT 后端必须维持同一尺度。
     */
    uint16_t fft_size = audio_preprocess_config.fft_size;

    for (uint16_t k = 0; k < audio_preprocess_plan.spectrum_bins; k++)
    {
        float real = 0.0f;
        float imag = 0.0f;
        float phase_cos = 1.0f;
        float phase_sin = 0.0f;
        float twiddle_cos = audio_preprocess_plan.twiddle_cos[k];
        float twiddle_sin = audio_preprocess_plan.twiddle_sin[k];

        for (uint16_t n = 0; n < fft_size; n++)
        {
            float sample = frame[n];
            float next_cos;
            float next_sin;

            real += sample * phase_cos;
            imag += sample * phase_sin;

            next_cos = (phase_cos * twiddle_cos) -
                       (phase_sin * twiddle_sin);
            next_sin = (phase_cos * twiddle_sin) +
                       (phase_sin * twiddle_cos);
            phase_cos = next_cos;
            phase_sin = next_sin;
        }

        spectrum[k] = (real * real) + (imag * imag);
    }
}

#if (APP_AUDIO_PREPROCESS_RFFT_REQUIRED)
static bool app_audio_preprocess_init_rfft(uint16_t fft_size)
{
    audio_preprocess_rfft_ready =
        (ARM_MATH_SUCCESS ==
         arm_rfft_fast_init_f32(&audio_preprocess_rfft_instance,
                                (uint16_t)fft_size));

    return audio_preprocess_rfft_ready;
}

static void app_audio_preprocess_power_spectrum_rfft(const float *frame,
                                                     float *spectrum)
{
    uint16_t fft_size = audio_preprocess_config.fft_size;
    uint16_t nyquist_bin = (uint16_t)(fft_size / 2u);

    if (!audio_preprocess_rfft_ready)
    {
        app_audio_preprocess_power_spectrum_dft(frame, spectrum);
        return;
    }

    memcpy(rfft_input_buffer,
           frame,
           (size_t)fft_size * sizeof(rfft_input_buffer[0]));
    arm_rfft_fast_f32(&audio_preprocess_rfft_instance,
                      rfft_input_buffer,
                      rfft_output_buffer,
                      0u);

    /* CMSIS-DSP fast RFFT forward output is unnormalized. For real input:
     * out[0] is DC real, out[1] is Nyquist real, and bins 1..N/2-1 are
     * interleaved real/imag pairs. Imaginary sign differences do not affect
     * power, but the DC/Nyquist packing must be handled explicitly.
     */
    spectrum[0] = rfft_output_buffer[0] * rfft_output_buffer[0];
    for (uint16_t k = 1u; k < nyquist_bin; k++)
    {
        float real = rfft_output_buffer[2u * k];
        float imag = rfft_output_buffer[(2u * k) + 1u];

        spectrum[k] = (real * real) + (imag * imag);
    }
    spectrum[nyquist_bin] = rfft_output_buffer[1] * rfft_output_buffer[1];
}
#endif

static float app_audio_preprocess_mel_energy(uint16_t mel_index)
{
    return app_audio_preprocess_mel_energy_from(power_spectrum, mel_index);
}

static float app_audio_preprocess_mel_energy_from(const float *spectrum,
                                                  uint16_t mel_index)
{
    /* 三角 Mel 滤波器。mel_edges[] 在配置阶段根据 sample_rate/fft_size/mel 范围预计算。 */
    uint16_t start = audio_preprocess_plan.mel_edges[mel_index];
    uint16_t center = audio_preprocess_plan.mel_edges[mel_index + 1u];
    uint16_t end = audio_preprocess_plan.mel_edges[mel_index + 2u];
    float energy = 0.0f;

    if ((start >= center) || (center >= end))
    {
        return 0.0f;
    }

    for (uint16_t k = start; k < center; k++)
    {
        float weight = (float)(k - start) / (float)(center - start);

        energy += spectrum[k] * weight;
    }

    for (uint16_t k = center; k <= end; k++)
    {
        float weight = (float)(end - k) / (float)(end - center);

        energy += spectrum[k] * weight;
    }

    return energy;
}

static void app_audio_preprocess_power_to_db_and_normalize(float max_mel_energy)
{
    app_audio_preprocess_power_to_db_and_normalize_buffer(model_input_feature,
                                                          max_mel_energy);
}

static void app_audio_preprocess_power_to_db_and_normalize_buffer(
    float *feature,
    float max_mel_energy)
{
    /* 对齐训练端 src/audio/features.py:
     * 1. librosa.power_to_db(mel, ref=np.max)，最大能量对应 0 dB；
     * 2. 默认 top_db=80，因此低能量区域被截到 -80 dB；
     * 3. v3 board_htk_no_norm_v1 到此为止，不做特征标准化；
     * 4. v2 兼容 profile 才对当前 40x101 特征图做本窗口 mean/std 标准化。
     *
     * 注意：这里仍是第一版板端近似实现，Mel 滤波器细节后续还要和 PC 做逐点比对。
     */
    const float amin = 1.0e-10f;
    const float top_db = 80.0f;
    uint32_t element_count =
        (uint32_t)audio_preprocess_config.mel_bin_count *
        (uint32_t)audio_preprocess_plan.time_bins;
    float ref_power = (max_mel_energy > amin) ? max_mel_energy : amin;
    float ref_db = 10.0f * log10f(ref_power);
    double sum = 0.0;
    double square_sum = 0.0;
    float mean;
    float std;

    for (uint32_t i = 0; i < element_count; i++)
    {
        float power = (feature[i] > amin) ? feature[i] : amin;
        float db = (10.0f * log10f(power)) - ref_db;

        if (-top_db > db)
        {
            db = -top_db;
        }

        feature[i] = db;
        sum += db;
        square_sum += ((double)db * (double)db);
    }

    if (!app_audio_preprocess_feature_zscore_enabled(
            audio_preprocess_config.frontend_profile))
    {
        return;
    }

    mean = (float)(sum / (double)element_count);
    std = (float)((square_sum / (double)element_count) -
                  ((double)mean * (double)mean));
    if (0.0f > std)
    {
        std = 0.0f;
    }
    std = sqrtf(std);

    for (uint32_t i = 0; i < element_count; i++)
    {
        feature[i] = (feature[i] - mean) /
                     (std + audio_preprocess_config.log_epsilon);
    }
}

static bool app_audio_preprocess_is_no_norm_profile(uint8_t frontend_profile)
{
    return ((uint8_t)APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_BOARD_HTK_NO_NORM_V1 ==
            frontend_profile);
}

static bool app_audio_preprocess_rms_gain_enabled(uint8_t frontend_profile)
{
    if (app_audio_preprocess_is_no_norm_profile(frontend_profile))
    {
        return false;
    }

    return ((uint8_t)APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_V2_ZSCORE ==
            frontend_profile);
}

static bool app_audio_preprocess_feature_zscore_enabled(uint8_t frontend_profile)
{
    if (app_audio_preprocess_is_no_norm_profile(frontend_profile))
    {
        return false;
    }

    return ((uint8_t)APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_V2_ZSCORE ==
            frontend_profile);
}

static bool app_audio_preprocess_global_feature_norm_enabled(
    uint8_t frontend_profile)
{
    (void)frontend_profile;

    return false;
}

static void app_audio_preprocess_print_resolved_config(void)
{
    uint8_t frontend_profile = audio_preprocess_config.frontend_profile;

    printf("[AUDIO_PREPROCESS_INFO] frontend_profile=%lu, frontend_name=%s, "
           "rms_gain_enable=%lu, feature_zscore_enable=%lu, "
           "global_feature_norm_enable=%lu, normalize_target_rms=",
           (unsigned long)frontend_profile,
           APP_AUDIO_ACTIVE_FRONTEND_NAME,
           (unsigned long)app_audio_preprocess_rms_gain_enabled(frontend_profile),
           (unsigned long)app_audio_preprocess_feature_zscore_enabled(
               frontend_profile),
           (unsigned long)app_audio_preprocess_global_feature_norm_enabled(
               frontend_profile));
    app_audio_preprocess_print_float_value(
        audio_preprocess_config.normalize_target_rms);
    printf(", normalize_max_gain=");
    app_audio_preprocess_print_float_value(
        audio_preprocess_config.normalize_max_gain);
    printf(", feature_mean=");
    app_audio_preprocess_print_float_value(audio_preprocess_config.feature_mean);
    printf(", feature_std=");
    app_audio_preprocess_print_float_value(audio_preprocess_config.feature_std);
    printf(", power_to_db_mode=ref_max_top_db_80, power_ref_mode=max, "
           "log_epsilon=");
    app_audio_preprocess_print_float_value(audio_preprocess_config.log_epsilon);
    printf(", energy_gate_threshold=");
    app_audio_preprocess_print_float_value(
        audio_preprocess_config.energy_gate_threshold);
    printf("\r\n");
}

static void app_audio_preprocess_print_float_value(float value)
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

#if (APP_MODEL_RUNTIME_PROFILE_ENABLE)
static void app_audio_preprocess_record_stage_time(uint32_t elapsed_ms,
                                                   uint32_t *last_ms,
                                                   uint32_t *total_ms,
                                                   uint32_t *max_ms,
                                                   uint32_t *count)
{
    *last_ms = elapsed_ms;
    *total_ms += elapsed_ms;
    (*count)++;
    if (*max_ms < elapsed_ms)
    {
        *max_ms = elapsed_ms;
    }
}

static void app_audio_preprocess_record_condition_time(uint32_t elapsed_ms)
{
    app_audio_preprocess_record_stage_time(
        elapsed_ms,
        &audio_preprocess_stats.last_condition_ms,
        &audio_preprocess_stats.condition_ms_total,
        &audio_preprocess_stats.condition_ms_max,
        &audio_preprocess_stats.condition_windows_profiled);
}

static void app_audio_preprocess_record_spectrum_time(uint32_t elapsed_ms)
{
    app_audio_preprocess_record_stage_time(
        elapsed_ms,
        &audio_preprocess_stats.last_spectrum_ms,
        &audio_preprocess_stats.spectrum_ms_total,
        &audio_preprocess_stats.spectrum_ms_max,
        &audio_preprocess_stats.spectrum_windows_profiled);
}

static void app_audio_preprocess_record_melbank_time(uint32_t elapsed_ms)
{
    app_audio_preprocess_record_stage_time(
        elapsed_ms,
        &audio_preprocess_stats.last_melbank_ms,
        &audio_preprocess_stats.melbank_ms_total,
        &audio_preprocess_stats.melbank_ms_max,
        &audio_preprocess_stats.melbank_windows_profiled);
}

static void app_audio_preprocess_record_mel_time(uint32_t elapsed_ms)
{
    app_audio_preprocess_record_stage_time(
        elapsed_ms,
        &audio_preprocess_stats.last_mel_ms,
        &audio_preprocess_stats.mel_ms_total,
        &audio_preprocess_stats.mel_ms_max,
        &audio_preprocess_stats.mel_windows_profiled);
}
#endif

#if (APP_AUDIO_SPECTRUM_COMPARE_ENABLE)
static void app_audio_preprocess_compare_update(float active,
                                                float reference,
                                                float *max_abs_diff,
                                                double *abs_sum,
                                                uint32_t *count)
{
    float diff = active - reference;

    if (0.0f > diff)
    {
        diff = -diff;
    }

    if (*max_abs_diff < diff)
    {
        *max_abs_diff = diff;
    }
    *abs_sum += (double)diff;
    (*count)++;
}

static void app_audio_preprocess_print_compare_result(
    const app_audio_preprocess_compare_stats_t *stats)
{
    uint32_t active_backend = APP_AUDIO_SPECTRUM_BACKEND;
    uint32_t reference_backend =
        (APP_AUDIO_SPECTRUM_BACKEND == APP_AUDIO_SPECTRUM_BACKEND_RFFT) ?
        APP_AUDIO_SPECTRUM_BACKEND_DFT : APP_AUDIO_SPECTRUM_BACKEND_RFFT;
    float power_mean_abs_diff =
        (0u < stats->power_count) ?
        (float)(stats->power_abs_sum / (double)stats->power_count) : 0.0f;
    float mel_mean_abs_diff =
        (0u < stats->mel_count) ?
        (float)(stats->mel_abs_sum / (double)stats->mel_count) : 0.0f;
    float logmel_mean_abs_diff =
        (0u < stats->logmel_count) ?
        (float)(stats->logmel_abs_sum / (double)stats->logmel_count) : 0.0f;

    printf("[MODEL_SPEC_COMPARE] window=%lu, active_backend=%lu, "
           "reference_backend=%lu, power_max_abs_diff=",
           (unsigned long)stats->window_index,
           (unsigned long)active_backend,
           (unsigned long)reference_backend);
    app_audio_preprocess_print_float(stats->power_max_abs_diff);
    printf(", power_mean_abs_diff=");
    app_audio_preprocess_print_float(power_mean_abs_diff);
    printf(", mel_max_abs_diff=");
    app_audio_preprocess_print_float(stats->mel_max_abs_diff);
    printf(", mel_mean_abs_diff=");
    app_audio_preprocess_print_float(mel_mean_abs_diff);
    printf(", logmel_max_abs_diff=");
    app_audio_preprocess_print_float(stats->logmel_max_abs_diff);
    printf(", logmel_mean_abs_diff=");
    app_audio_preprocess_print_float(logmel_mean_abs_diff);
    printf("\r\n");
}

static void app_audio_preprocess_print_float(float value)
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
#endif

static void app_audio_preprocess_init_shared_region(void)
{
    /* 初始化共享协议头。这里不清空原始 PCM，因为原始 PCM 不在共享区。 */
    volatile app_model_shared_region_t *shared = APP_MODEL_SHARED_REGION;

    if ((APP_MODEL_SHARED_MAGIC != shared->magic) ||
        (APP_MODEL_SHARED_VERSION != shared->version))
    {
        memset((void *)shared, 0, sizeof(*shared));
        shared->magic = APP_MODEL_SHARED_MAGIC;
        shared->version = APP_MODEL_SHARED_VERSION;
        shared->input_state = APP_MODEL_SHARED_INPUT_EMPTY;
    }
}

static bool app_audio_preprocess_publish_feature(uint32_t timestamp_ms,
                                                 uint8_t selected_channel,
                                                 float energy)
{
    /* 发布策略：共享区只有一个输入槽。
     * 如果 CM55 还没消费上一帧，则本帧丢弃并计数 shared_busy，避免覆盖正在推理的数据。
     */
    volatile app_model_shared_region_t *shared = APP_MODEL_SHARED_REGION;
    uint16_t payload_bytes =
        (uint16_t)((uint32_t)audio_preprocess_config.mel_bin_count *
                   (uint32_t)audio_preprocess_plan.time_bins *
                   (uint32_t)sizeof(model_input_feature[0]));
    app_model_audio_feature_desc_t desc;

    if ((APP_MODEL_SHARED_INPUT_READY == shared->input_state) ||
        (APP_MODEL_SHARED_INPUT_READING == shared->input_state))
    {
        audio_preprocess_stats.shared_busy++;
        return false;
    }

    memset(&desc, 0, sizeof(desc));
    desc.sequence = shared->producer_sequence + 1u;
    desc.timestamp_ms = timestamp_ms;
    desc.sample_rate_hz = (uint16_t)audio_preprocess_config.sample_rate_hz;
    desc.window_ms = audio_preprocess_config.window_ms;
    desc.window_hop_ms = audio_preprocess_config.window_hop_ms;
    desc.frame_len_ms = audio_preprocess_config.frame_len_ms;
    desc.frame_hop_ms = audio_preprocess_config.frame_hop_ms;
    desc.fft_size = audio_preprocess_config.fft_size;
    desc.mel_bin_count = audio_preprocess_config.mel_bin_count;
    desc.time_bin_count = audio_preprocess_plan.time_bins;
    desc.payload_bytes = payload_bytes;
    desc.quant_type = (uint8_t)audio_preprocess_config.quant_type;
    desc.quant_zero_point = audio_preprocess_config.quant_zero_point;
    desc.quant_scale = audio_preprocess_config.quant_scale;
    desc.energy = energy;
    desc.selected_channel = selected_channel;
    desc.valid = 1u;

    shared->input_state = APP_MODEL_SHARED_INPUT_WRITING;
    shared->audio = desc;
    memcpy((void *)&shared->audio_payload[0],
           model_input_feature,
           payload_bytes);
    shared->producer_sequence = desc.sequence;
    audio_preprocess_stats.last_published_sequence = desc.sequence;
    audio_preprocess_stats.last_published_timestamp_ms = timestamp_ms;
    shared->input_state = APP_MODEL_SHARED_INPUT_READY;
    APP_MODEL_SHARED_CLEAN_CACHE((void *)shared, sizeof(*shared));

#if (APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE)
    app_audio_preprocess_event_feature_remember(&desc, model_input_feature);
#endif

#if (APP_AUDIO_EVENT_PCM_DUMP_ENABLE)
    app_audio_preprocess_event_pcm_remember(&desc, model_input_feature);
#endif

#if (APP_AUDIO_FEATURE_DUMP_ENABLE)
    app_audio_preprocess_maybe_dump_feature_stats(&desc, model_input_feature);
#endif

    return true;
}

#if (APP_AUDIO_EVENT_ANY_DUMP_ENABLE)
static bool app_audio_preprocess_event_feature_stats(
    const float *feature,
    uint32_t element_count,
    app_audio_event_feature_stats_t *stats)
{
    const uint32_t fnv_offset_basis = 2166136261u;
    const uint32_t fnv_prime = 16777619u;
    uint32_t byte_count;
    const uint8_t *feature_bytes;
    double sum = 0.0;
    double square_sum = 0.0;
    float variance;

    if ((NULL == feature) || (NULL == stats) || (0u == element_count) ||
        (APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS < element_count))
    {
        return false;
    }

    memset(stats, 0, sizeof(*stats));
    stats->count = element_count;
    stats->hash32 = fnv_offset_basis;
    stats->min_value = feature[0];
    stats->max_value = feature[0];

    byte_count = element_count * (uint32_t)sizeof(feature[0]);
    feature_bytes = (const uint8_t *)feature;
    for (uint32_t i = 0u; i < byte_count; i++)
    {
        stats->hash32 ^= (uint32_t)feature_bytes[i];
        stats->hash32 *= fnv_prime;
    }

    for (uint32_t i = 0u; i < element_count; i++)
    {
        float value = feature[i];

        if (value < stats->min_value)
        {
            stats->min_value = value;
        }
        if (stats->max_value < value)
        {
            stats->max_value = value;
        }
        sum += value;
        square_sum += ((double)value * (double)value);
    }

    stats->mean = (float)(sum / (double)element_count);
    variance = (float)((square_sum / (double)element_count) -
                       ((double)stats->mean * (double)stats->mean));
    if (0.0f > variance)
    {
        variance = 0.0f;
    }
    stats->stddev = sqrtf(variance);

    return true;
}
#endif

#if (APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE)
static void app_audio_preprocess_event_feature_remember(
    const app_model_audio_feature_desc_t *desc,
    const float *feature)
{
    uint32_t element_count;
    app_audio_event_feature_frame_t *slot;

    if ((NULL == desc) || (NULL == feature))
    {
        return;
    }

    element_count = (uint32_t)desc->mel_bin_count *
                    (uint32_t)desc->time_bin_count;
    if ((0u == element_count) ||
        (APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS < element_count))
    {
        return;
    }

    slot = &event_feature_ring[event_feature_ring_write_index];
    event_feature_ring_write_index++;
    if (APP_AUDIO_EVENT_FEATURE_DUMP_RING_DEPTH <=
        event_feature_ring_write_index)
    {
        event_feature_ring_write_index = 0u;
    }

    slot->valid = false;
    memset(slot, 0, sizeof(*slot));
    slot->sequence = desc->sequence;
    slot->timestamp_ms = desc->timestamp_ms;
    slot->count = element_count;
    slot->payload_bytes = element_count * (uint32_t)sizeof(feature[0]);
    slot->energy = desc->energy;
    slot->selected_channel = desc->selected_channel;
    memcpy(slot->feature, feature, slot->payload_bytes);
    if (!app_audio_preprocess_event_feature_stats(
            slot->feature,
            element_count,
            &slot->stats))
    {
        return;
    }
    slot->valid = true;
}

static bool app_audio_preprocess_event_feature_copy_to_dump_buffer(
    uint32_t input_sequence)
{
    for (uint32_t i = 0u; i < APP_AUDIO_EVENT_FEATURE_DUMP_RING_DEPTH; i++)
    {
        app_audio_event_feature_frame_t *slot = &event_feature_ring[i];

        if (slot->valid && (slot->sequence == input_sequence))
        {
            memcpy(&event_feature_dump_buffer,
                   slot,
                   sizeof(event_feature_dump_buffer));
            return (event_feature_dump_buffer.valid &&
                    (event_feature_dump_buffer.sequence == input_sequence) &&
                    (event_feature_dump_buffer.count <=
                     APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS));
        }
    }

    return false;
}
#endif

#if (APP_AUDIO_EVENT_ANY_DUMP_ENABLE)
static void app_audio_preprocess_event_dump_pace(void)
{
    fflush(stdout);
#if (APP_AUDIO_EVENT_DUMP_LINE_DELAY_MS > 0u)
    vTaskDelay(pdMS_TO_TICKS(APP_AUDIO_EVENT_DUMP_LINE_DELAY_MS));
#endif
}

static void app_audio_preprocess_event_dump_append_text(size_t *used,
                                                        const char *text)
{
    int written;

    if ((NULL == used) || (NULL == text) ||
        (*used >= APP_AUDIO_EVENT_DUMP_LINE_MAX_CHARS))
    {
        return;
    }

    written = snprintf(&event_dump_line[*used],
                       APP_AUDIO_EVENT_DUMP_LINE_MAX_CHARS - *used,
                       "%s",
                       text);
    if (0 < written)
    {
        size_t advance = (size_t)written;
        size_t remaining = APP_AUDIO_EVENT_DUMP_LINE_MAX_CHARS - *used;

        *used += (advance < remaining) ? advance : (remaining - 1u);
    }
}

static void app_audio_preprocess_event_dump_append_hex_word(size_t *used,
                                                            uint32_t value)
{
    int written;

    if ((NULL == used) || (*used >= APP_AUDIO_EVENT_DUMP_LINE_MAX_CHARS))
    {
        return;
    }

    written = snprintf(&event_dump_line[*used],
                       APP_AUDIO_EVENT_DUMP_LINE_MAX_CHARS - *used,
                       "0x%08lx",
                       (unsigned long)value);
    if (0 < written)
    {
        size_t advance = (size_t)written;
        size_t remaining = APP_AUDIO_EVENT_DUMP_LINE_MAX_CHARS - *used;

        *used += (advance < remaining) ? advance : (remaining - 1u);
    }
}

static void app_audio_preprocess_event_dump_append_hex_halfword(size_t *used,
                                                                uint16_t value)
{
    int written;

    if ((NULL == used) || (*used >= APP_AUDIO_EVENT_DUMP_LINE_MAX_CHARS))
    {
        return;
    }

    written = snprintf(&event_dump_line[*used],
                       APP_AUDIO_EVENT_DUMP_LINE_MAX_CHARS - *used,
                       "0x%04lx",
                       (unsigned long)value);
    if (0 < written)
    {
        size_t advance = (size_t)written;
        size_t remaining = APP_AUDIO_EVENT_DUMP_LINE_MAX_CHARS - *used;

        *used += (advance < remaining) ? advance : (remaining - 1u);
    }
}

static void app_audio_preprocess_event_print_hex_word(uint32_t value)
{
    printf("0x%08lx", (unsigned long)value);
}
#endif

#if (APP_AUDIO_EVENT_PCM_DUMP_ENABLE)
static void app_audio_preprocess_event_pcm_capture_pending(uint32_t timestamp_ms,
                                                           uint8_t selected_channel)
{
    uint16_t start = mono_ring_write_index;

    if ((!app_audio_preprocess_event_pcm_dump_can_emit()) ||
        event_pcm_snapshot.locked ||
        (audio_preprocess_plan.window_samples >
         APP_AUDIO_PREPROCESS_MAX_WINDOW_SAMPLES))
    {
        return;
    }

    event_pcm_snapshot.valid = false;
    event_pcm_snapshot.feature_valid = false;
    event_pcm_snapshot.sequence = 0u;
    event_pcm_snapshot.timestamp_ms = timestamp_ms;
    event_pcm_snapshot.sample_rate_hz = audio_preprocess_config.sample_rate_hz;
    event_pcm_snapshot.count = audio_preprocess_plan.window_samples;
    event_pcm_snapshot.dc_removed_energy = 0.0f;
    event_pcm_snapshot.selected_channel = selected_channel;

    for (uint16_t i = 0u; i < audio_preprocess_plan.window_samples; i++)
    {
        uint16_t index =
            (uint16_t)((start + i) % audio_preprocess_plan.window_samples);

        event_pcm_snapshot.pcm[i] = mono_ring[index];
    }

    if (!app_audio_preprocess_event_pcm_stats(
            event_pcm_snapshot.pcm,
            event_pcm_snapshot.count,
            &event_pcm_snapshot.pcm_stats))
    {
        event_pcm_pending_valid = false;
        return;
    }

    event_pcm_pending_valid = true;
    event_pcm_snapshot.valid = true;
}

static bool app_audio_preprocess_event_pcm_stats(
    const int16_t *pcm,
    uint32_t sample_count,
    app_audio_event_pcm_stats_t *stats)
{
    const uint32_t fnv_offset_basis = 2166136261u;
    const uint32_t fnv_prime = 16777619u;
    uint32_t byte_count;
    const uint8_t *pcm_bytes;
    int64_t sum = 0;
    double square_sum = 0.0;

    if ((NULL == pcm) || (NULL == stats) || (0u == sample_count) ||
        (APP_AUDIO_PREPROCESS_MAX_WINDOW_SAMPLES < sample_count))
    {
        return false;
    }

    memset(stats, 0, sizeof(*stats));
    stats->count = sample_count;
    stats->hash32 = fnv_offset_basis;
    stats->min_value = pcm[0];
    stats->max_value = pcm[0];

    byte_count = sample_count * (uint32_t)sizeof(pcm[0]);
    pcm_bytes = (const uint8_t *)pcm;
    for (uint32_t i = 0u; i < byte_count; i++)
    {
        stats->hash32 ^= (uint32_t)pcm_bytes[i];
        stats->hash32 *= fnv_prime;
    }

    for (uint32_t i = 0u; i < sample_count; i++)
    {
        int16_t value = pcm[i];

        if (value < stats->min_value)
        {
            stats->min_value = value;
        }
        if (stats->max_value < value)
        {
            stats->max_value = value;
        }
        sum += value;
        square_sum += ((double)value * (double)value);
    }

    stats->mean = (float)((double)sum / (double)sample_count);
    stats->rms = sqrtf((float)(square_sum / (double)sample_count));

    return true;
}

static void app_audio_preprocess_event_pcm_remember(
    const app_model_audio_feature_desc_t *desc,
    const float *feature)
{
    uint32_t element_count;

    if ((NULL == desc) || (!event_pcm_pending_valid) ||
        event_pcm_snapshot.locked)
    {
        return;
    }

    event_pcm_snapshot.sequence = desc->sequence;
    event_pcm_snapshot.dc_removed_energy = desc->energy;
    event_pcm_snapshot.selected_channel = desc->selected_channel;

#if (APP_AUDIO_EVENT_PCM_DUMP_INCLUDE_FEATURE)
    event_pcm_snapshot.feature_valid = false;
    if (NULL != feature)
    {
        element_count = (uint32_t)desc->mel_bin_count *
                        (uint32_t)desc->time_bin_count;
        if ((0u < element_count) &&
            (APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS >= element_count))
        {
            memcpy(event_pcm_snapshot.feature,
                   feature,
                   element_count * (uint32_t)sizeof(feature[0]));
            event_pcm_snapshot.feature_valid =
                app_audio_preprocess_event_feature_stats(
                    event_pcm_snapshot.feature,
                    element_count,
                    &event_pcm_snapshot.feature_stats);
        }
    }
#else
    (void)feature;
    element_count = 0u;
    (void)element_count;
#endif

    event_pcm_snapshot.valid = true;
    event_pcm_pending_valid = false;
}

#endif

#if (APP_AUDIO_FEATURE_DUMP_ENABLE)
static void app_audio_preprocess_maybe_dump_feature_stats(
    const app_model_audio_feature_desc_t *desc,
    const float *feature)
{
    const uint32_t fnv_offset_basis = 2166136261u;
    const uint32_t fnv_prime = 16777619u;
    uint32_t element_count;
    uint32_t byte_count;
    const uint8_t *feature_bytes;
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
    uint32_t sample_last_index;

    if ((NULL == desc) || (NULL == feature) ||
        ((0u < APP_AUDIO_FEATURE_DUMP_WINDOWS) &&
         (APP_AUDIO_FEATURE_DUMP_WINDOWS <= feature_dump_windows_reported)))
    {
        return;
    }

    element_count = (uint32_t)desc->mel_bin_count *
                    (uint32_t)desc->time_bin_count;
    if (0u == element_count)
    {
        return;
    }

    byte_count = element_count * (uint32_t)sizeof(feature[0]);
    feature_bytes = (const uint8_t *)feature;
    for (uint32_t i = 0u; i < byte_count; i++)
    {
        hash ^= (uint32_t)feature_bytes[i];
        hash *= fnv_prime;
    }

    min_value = feature[0];
    max_value = feature[0];
    for (uint32_t i = 0u; i < element_count; i++)
    {
        float value = feature[i];

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
    sample_last_index = element_count - 1u;

    feature_dump_windows_reported++;
    printf("[AUDIO_FEATURE_CM33] seq=%lu, energy=",
           (unsigned long)desc->sequence);
    app_audio_preprocess_dump_print_float(desc->energy);
    printf(", selected_channel=%lu, count=%lu, feature_min=",
           (unsigned long)desc->selected_channel,
           (unsigned long)element_count);
    app_audio_preprocess_dump_print_float(min_value);
    printf(", feature_max=");
    app_audio_preprocess_dump_print_float(max_value);
    printf(", feature_mean=");
    app_audio_preprocess_dump_print_float(mean);
    printf(", feature_std=");
    app_audio_preprocess_dump_print_float(stddev);
    printf(", hash_low16=%lu, floor80_count=%lu, near0_count=%lu, f0=",
           (unsigned long)(hash & 0xffffu),
           (unsigned long)floor_count,
           (unsigned long)near_zero_count);
    app_audio_preprocess_dump_print_float(feature[0]);
    printf(", f1=");
    app_audio_preprocess_dump_print_float(feature[(1u < element_count) ?
                                                  1u : sample_last_index]);
    printf(", f2=");
    app_audio_preprocess_dump_print_float(feature[(2u < element_count) ?
                                                  2u : sample_last_index]);
    printf(", f39=");
    app_audio_preprocess_dump_print_float(feature[(39u < element_count) ?
                                                  39u : sample_last_index]);
    printf(", f40=");
    app_audio_preprocess_dump_print_float(feature[(40u < element_count) ?
                                                  40u : sample_last_index]);
    printf(", f100=");
    app_audio_preprocess_dump_print_float(feature[(100u < element_count) ?
                                                  100u : sample_last_index]);
    printf(", sample_last=");
    app_audio_preprocess_dump_print_float(feature[sample_last_index]);
    printf("\r\n");
}

static void app_audio_preprocess_dump_print_float(float value)
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
#endif

static uint32_t app_audio_preprocess_ms_to_samples(uint32_t sample_rate_hz,
                                                   uint16_t ms)
{
    return (sample_rate_hz * (uint32_t)ms) / 1000u;
}

static float app_audio_preprocess_hz_to_mel(float hz)
{
    return 2595.0f * log10f(1.0f + (hz / 700.0f));
}

static float app_audio_preprocess_mel_to_hz(float mel)
{
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

static int16_t app_audio_preprocess_saturate_i16(int32_t value)
{
    if (INT16_MAX < value)
    {
        return INT16_MAX;
    }
    if (INT16_MIN > value)
    {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static uint32_t app_audio_preprocess_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

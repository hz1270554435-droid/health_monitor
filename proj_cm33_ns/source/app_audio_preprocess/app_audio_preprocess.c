#include "app_audio_preprocess.h"

#include <math.h>
#include <string.h>

#define APP_AUDIO_PREPROCESS_PI                 (3.14159265358979323846f)

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
static float app_audio_preprocess_mel_energy(uint16_t mel_index);
static void app_audio_preprocess_power_to_db_and_normalize(float max_mel_energy);
static void app_audio_preprocess_init_shared_region(void);
static bool app_audio_preprocess_publish_feature(uint32_t timestamp_ms,
                                                 uint8_t selected_channel,
                                                 float energy);
static uint32_t app_audio_preprocess_ms_to_samples(uint32_t sample_rate_hz,
                                                   uint16_t ms);
static float app_audio_preprocess_hz_to_mel(float hz);
static float app_audio_preprocess_mel_to_hz(float mel);
static int16_t app_audio_preprocess_saturate_i16(int32_t value);

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

    app_audio_preprocess_init_shared_region();
    app_audio_preprocess_reset_stream_state();

    for (;;)
    {
        app_pdm_pcm_block_t block;

        if (!app_pdm_pcm_receive_block(&block, portMAX_DELAY))
        {
            continue;
        }

        audio_preprocess_stats.blocks_received++;

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
                uint32_t timestamp_ms =
                    (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

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

    audio_preprocess_config = *config;
    audio_preprocess_config_ready = true;
    app_audio_preprocess_reset_stream_state();

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

    memset(config, 0, sizeof(*config));
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
    config->channel_mix_mode = APP_AUDIO_CHANNEL_MIX_SELECT_BEST;
    config->fixed_delay_samples = 0;
    config->energy_gate_threshold = APP_AUDIO_PREPROCESS_DEFAULT_ENERGY_GATE;
    config->normalize_target_rms = APP_AUDIO_PREPROCESS_DEFAULT_TARGET_RMS;
    config->normalize_max_gain = APP_AUDIO_PREPROCESS_DEFAULT_MAX_GAIN;
    config->log_epsilon = APP_AUDIO_PREPROCESS_DEFAULT_LOG_EPSILON;
    config->feature_mean = APP_AUDIO_PREPROCESS_DEFAULT_FEATURE_MEAN;
    config->feature_std = APP_AUDIO_PREPROCESS_DEFAULT_FEATURE_STD;
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

    app_audio_preprocess_copy_ordered_window();
    if (!app_audio_preprocess_condition_window(&energy))
    {
        audio_preprocess_stats.windows_energy_gated++;
        audio_preprocess_stats.last_energy = energy;
        return false;
    }

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

        app_audio_preprocess_power_spectrum(frame_buffer);

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
    }

    app_audio_preprocess_power_to_db_and_normalize(max_mel_energy);

    audio_preprocess_stats.last_energy = energy;
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
    /* 去直流和 RMS 归一化。
     * energy_gate_threshold 使用归一化前能量，这样静音不会被 normalize_max_gain
     * 放大后误判为有效语音。
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
    /* 参考实现：直接 DFT 得到单边功率谱。
     *
     * 第一版为了少引入库依赖，仍使用软件 DFT；但不再在内层循环里调用 sinf/cosf，
     * 而是用配置阶段预计算的旋转因子递推。后续如果实时性能不足，应把本函数
     * 替换成 CMSIS-DSP RFFT，输出仍填 power_spectrum[]，其它前处理流程不变。
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

        power_spectrum[k] = (real * real) + (imag * imag);
    }
}

static float app_audio_preprocess_mel_energy(uint16_t mel_index)
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

        energy += power_spectrum[k] * weight;
    }

    for (uint16_t k = center; k <= end; k++)
    {
        float weight = (float)(end - k) / (float)(end - center);

        energy += power_spectrum[k] * weight;
    }

    return energy;
}

static void app_audio_preprocess_power_to_db_and_normalize(float max_mel_energy)
{
    /* 对齐训练端 src/audio/features.py:
     * 1. librosa.power_to_db(mel, ref=np.max)，最大能量对应 0 dB；
     * 2. 默认 top_db=80，因此低能量区域被截到 -80 dB；
     * 3. 对当前 40x101 特征图做本窗口 mean/std 标准化。
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
        float power = (model_input_feature[i] > amin) ?
                      model_input_feature[i] : amin;
        float db = (10.0f * log10f(power)) - ref_db;

        if (-top_db > db)
        {
            db = -top_db;
        }

        model_input_feature[i] = db;
        sum += db;
        square_sum += ((double)db * (double)db);
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
        model_input_feature[i] =
            (model_input_feature[i] - mean) /
            (std + audio_preprocess_config.log_epsilon);
    }
}

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
    shared->input_state = APP_MODEL_SHARED_INPUT_READY;
    APP_MODEL_SHARED_CLEAN_CACHE((void *)shared, sizeof(*shared));

    return true;
}

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

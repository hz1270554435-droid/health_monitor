#include "app_audio_preprocess.h"

#include <math.h>
#include <string.h>

#define APP_AUDIO_PREPROCESS_PI                 (3.14159265358979323846f)

/* 本文件是正式业务链路的唯一 MIC 输入前处理任务实现。
 *
 * 任务边界：
 * - 输入：app_pdm_pcm 发布的 10 ms 双通道 PCM block，格式为 L,R,L,R...
 * - 输出：shared/app_model_shared.h 定义的共享内存特征帧，payload 已量化。
 *
 * 设计原则：
 * - app_pdm_pcm 只做采集，不做算法；
 * - app_get_data 只做测试/导出，不参与正式业务；
 * - 本模块拥有滑动窗口、特征提取和量化参数，后续模型调整优先改配置结构；
 * - 当前频谱计算使用朴素 DFT，便于先验证端到端协议。若实时性能不足，
 *   只需要替换 app_audio_preprocess_power_spectrum()，上层接口不变。
 */

typedef struct
{
    uint16_t window_samples;
    uint16_t window_hop_samples;
    uint16_t frame_len_samples;
    uint16_t frame_hop_samples;
    uint16_t spectrum_bins;
    uint16_t time_bins;
    uint16_t mel_edges[APP_MODEL_AUDIO_FEATURE_MAX_MEL_BINS + 2u];
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
static uint8_t quantized_feature[APP_MODEL_AUDIO_FEATURE_MAX_BYTES];

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
static uint8_t app_audio_preprocess_quantize(float value);
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
    app_audio_preprocess_plan_t plan;

    if ((NULL == config) || (NULL != audio_preprocess_task_handle))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    if (CY_RSLT_SUCCESS !=
        app_audio_preprocess_validate_and_plan(config, &plan))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    audio_preprocess_config = *config;
    audio_preprocess_plan = plan;
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
    config->quant_type = APP_MODEL_AUDIO_QUANT_INT8;
    config->quant_scale = APP_AUDIO_PREPROCESS_DEFAULT_QUANT_SCALE;
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
        (0u == config->mel_bin_count) ||
        (APP_MODEL_AUDIO_FEATURE_MAX_MEL_BINS < config->mel_bin_count) ||
        (0.0f > config->mel_low_hz) ||
        (config->mel_low_hz >= config->mel_high_hz) ||
        (((float)config->sample_rate_hz * 0.5f) < config->mel_high_hz) ||
        (0.0f > config->energy_gate_threshold) ||
        (0.0f >= config->normalize_target_rms) ||
        (0.0f >= config->normalize_max_gain) ||
        (0.0f >= config->log_epsilon) ||
        (0.0f >= config->feature_std) ||
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

    plan->time_bins = (uint16_t)(1u + ((plan->window_samples -
                                        plan->frame_len_samples) /
                                       plan->frame_hop_samples));
    if ((0u == plan->time_bins) ||
        (APP_MODEL_AUDIO_FEATURE_MAX_TIME_BINS < plan->time_bins))
    {
        return CY_RSLT_TYPE_ERROR;
    }

    delay_abs = (0 > config->fixed_delay_samples) ?
                (uint32_t)(-config->fixed_delay_samples) :
                (uint32_t)config->fixed_delay_samples;
    if (APP_AUDIO_PREPROCESS_MAX_DELAY_SAMPLES < delay_abs)
    {
        return CY_RSLT_TYPE_ERROR;
    }

    for (uint16_t i = 0; i < plan->frame_len_samples; i++)
    {
        float phase = (2.0f * APP_AUDIO_PREPROCESS_PI * (float)i) /
                      (float)(plan->frame_len_samples - 1u);

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
     * -> Mel 滤波器组 -> log -> 全局归一化 -> int8/uint8 量化 -> 共享内存。
     */
    float energy = 0.0f;

    app_audio_preprocess_copy_ordered_window();
    if (!app_audio_preprocess_condition_window(&energy))
    {
        audio_preprocess_stats.windows_energy_gated++;
        audio_preprocess_stats.last_energy = energy;
        return false;
    }

    for (uint16_t t = 0; t < audio_preprocess_plan.time_bins; t++)
    {
        uint16_t frame_start =
            (uint16_t)(t * audio_preprocess_plan.frame_hop_samples);

        for (uint16_t n = 0; n < audio_preprocess_plan.frame_len_samples; n++)
        {
            frame_buffer[n] = window_buffer[frame_start + n] *
                              audio_preprocess_plan.frame_window[n];
        }

        app_audio_preprocess_power_spectrum(frame_buffer);

        for (uint16_t mel = 0; mel < audio_preprocess_config.mel_bin_count; mel++)
        {
            float mel_energy = app_audio_preprocess_mel_energy(mel);
            float logmel = logf(mel_energy + audio_preprocess_config.log_epsilon);
            float normalized =
                (logmel - audio_preprocess_config.feature_mean) /
                audio_preprocess_config.feature_std;
            uint16_t out_index =
                (uint16_t)((mel * audio_preprocess_plan.time_bins) + t);

            quantized_feature[out_index] =
                app_audio_preprocess_quantize(normalized);
        }
    }

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
     * 复杂度较高，只用于先跑通框架。后续可以在这里替换为 CMSIS-DSP RFFT，
     * 输出仍填 power_spectrum[]，其它函数无需变化。
     */
    uint16_t fft_size = audio_preprocess_config.fft_size;

    for (uint16_t k = 0; k < audio_preprocess_plan.spectrum_bins; k++)
    {
        float real = 0.0f;
        float imag = 0.0f;

        for (uint16_t n = 0; n < fft_size; n++)
        {
            float sample = (n < audio_preprocess_plan.frame_len_samples) ?
                           frame[n] : 0.0f;
            float angle = (-2.0f * APP_AUDIO_PREPROCESS_PI *
                           (float)k * (float)n) / (float)fft_size;

            real += sample * cosf(angle);
            imag += sample * sinf(angle);
        }

        power_spectrum[k] = ((real * real) + (imag * imag)) / (float)fft_size;
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

static uint8_t app_audio_preprocess_quantize(float value)
{
    /* 量化公式：
     * q = round(value / scale + zero_point)
     * INT8 模式下以 uint8_t 存储底层字节，CM55 读取时按 int8_t 解释。
     */
    float scaled = (value / audio_preprocess_config.quant_scale) +
                   (float)audio_preprocess_config.quant_zero_point;
    int32_t rounded = (0.0f <= scaled) ?
                      (int32_t)(scaled + 0.5f) :
                      (int32_t)(scaled - 0.5f);

    if (APP_MODEL_AUDIO_QUANT_UINT8 == audio_preprocess_config.quant_type)
    {
        if (0 > rounded)
        {
            rounded = 0;
        }
        else if (255 < rounded)
        {
            rounded = 255;
        }
        return (uint8_t)rounded;
    }

    if (-128 > rounded)
    {
        rounded = -128;
    }
    else if (127 < rounded)
    {
        rounded = 127;
    }

    return (uint8_t)((int8_t)rounded);
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
        (uint16_t)(audio_preprocess_config.mel_bin_count *
                   audio_preprocess_plan.time_bins);
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
           quantized_feature,
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

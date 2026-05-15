/*******************************************************************************
* File Name : app_audio_preprocess.h
*
* Description : CM33 音频输入信号前处理任务。
*
* 本任务只负责正式业务链路中的 MIC 输入前处理：
* 1. 从 app_pdm_pcm 接收 10 ms 双通道 PCM block；
* 2. 按配置执行双通道选优、平均、单通道选择或固定延时求和；
* 3. 得到 16 kHz 单声道 PCM，并写入 1 s 环形缓冲；
* 4. 按 50% overlap 或其它配置步长取窗口；
* 5. 执行去直流、归一化、能量门限、分帧、加窗、Mel、power_to_db、特征归一化；
* 6. 按真实模型要求输出 float32[40,101]，并只把处理好的模型输入写入共享内存。
*
* 注意：app_get_data 只保留测试/导出用途；正式业务不要让它和本任务同时
* 消费 app_pdm_pcm 的同一个队列。
*******************************************************************************/

#ifndef __APP_AUDIO_PREPROCESS_H__
#define __APP_AUDIO_PREPROCESS_H__

#include <stdbool.h>
#include <stdint.h>

#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"

#include "app_pdm_pcm.h"
#include "app_audio_deployment_config.h"
#include "app_model_shared.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* 静态工作缓冲上限。
 * 运行时参数都可以通过 app_audio_preprocess_config_t 调整，但不能超过这些
 * 上限；如果后续模型需要更大的窗口、FFT 或特征图，需要同时扩展这里的
 * 本地工作缓冲和 shared/app_model_shared.h 中的 CM55 输入协议。
 */
#define APP_AUDIO_PREPROCESS_MAX_WINDOW_SAMPLES     (16000u)
#define APP_AUDIO_PREPROCESS_MAX_FFT_SIZE           (1024u)
#define APP_AUDIO_PREPROCESS_MAX_SPECTRUM_BINS      ((APP_AUDIO_PREPROCESS_MAX_FFT_SIZE / 2u) + 1u)
#define APP_AUDIO_PREPROCESS_MAX_FRAME_SAMPLES      (APP_AUDIO_PREPROCESS_MAX_FFT_SIZE)
#define APP_AUDIO_PREPROCESS_MAX_DELAY_SAMPLES      (256u)

/* 默认运行参数。
 * 这些宏只是“出厂默认值”，不是算法写死参数。实际项目中应保持训练脚本、
 * CM33 前处理、CM55 模型输入三处参数一致；如果训练时改了窗口长度、Mel bin
 * 数或输入 payload 类型，应优先改这里或在启动前调用 app_audio_preprocess_configure()。
 */
#define APP_AUDIO_PREPROCESS_DEFAULT_WINDOW_MS      (1000u)
#define APP_AUDIO_PREPROCESS_DEFAULT_WINDOW_HOP_MS  (500u)
#define APP_AUDIO_PREPROCESS_DEFAULT_FRAME_LEN_MS   (64u)
#define APP_AUDIO_PREPROCESS_DEFAULT_FRAME_HOP_MS   (10u)
#define APP_AUDIO_PREPROCESS_DEFAULT_FFT_SIZE       (1024u)
#define APP_AUDIO_PREPROCESS_DEFAULT_MEL_BINS       (40u)
#define APP_AUDIO_PREPROCESS_DEFAULT_MEL_LOW_HZ     (50.0f)
#define APP_AUDIO_PREPROCESS_DEFAULT_MEL_HIGH_HZ    (7600.0f)
#define APP_AUDIO_PREPROCESS_DEFAULT_ENERGY_GATE    (0.000001f)
#define APP_AUDIO_PREPROCESS_DEFAULT_TARGET_RMS     (0.10f)
#define APP_AUDIO_PREPROCESS_DEFAULT_MAX_GAIN       (20.0f)
#define APP_AUDIO_PREPROCESS_DEFAULT_LOG_EPSILON    (0.000001f)
#define APP_AUDIO_PREPROCESS_DEFAULT_FEATURE_MEAN   (0.0f)
#define APP_AUDIO_PREPROCESS_DEFAULT_FEATURE_STD    (1.0f)
#define APP_AUDIO_PREPROCESS_DEFAULT_QUANT_SCALE    (0.03125f)
#define APP_AUDIO_PREPROCESS_DEFAULT_QUANT_ZERO     (0)

#define APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_V2_ZSCORE              (0u)
#define APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_BOARD_HTK_NO_NORM_V1   (1u)

#define APP_AUDIO_SPECTRUM_BACKEND_DFT                               (0u)
#define APP_AUDIO_SPECTRUM_BACKEND_RFFT                              (1u)

#ifndef APP_AUDIO_SPECTRUM_BACKEND
#define APP_AUDIO_SPECTRUM_BACKEND APP_AUDIO_SPECTRUM_BACKEND_DFT
#endif

#ifndef APP_AUDIO_SPECTRUM_COMPARE_ENABLE
#define APP_AUDIO_SPECTRUM_COMPARE_ENABLE                            (0u)
#endif

#ifndef APP_AUDIO_SPECTRUM_COMPARE_WINDOWS
#define APP_AUDIO_SPECTRUM_COMPARE_WINDOWS                           (3u)
#endif

#ifndef APP_AUDIO_PREPROCESS_DEFAULT_FRONTEND_PROFILE
#if (APP_AUDIO_MODEL_SELECT == APP_AUDIO_MODEL_SELECT_V3_BOARD_HTK_HARDNEG)
#define APP_AUDIO_PREPROCESS_DEFAULT_FRONTEND_PROFILE \
    APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_BOARD_HTK_NO_NORM_V1
#else
#define APP_AUDIO_PREPROCESS_DEFAULT_FRONTEND_PROFILE \
    APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_V2_ZSCORE
#endif
#endif

#define APP_AUDIO_PREPROCESS_TASK_STACK_SIZE        (4096u)
#define APP_AUDIO_PREPROCESS_TASK_PRIORITY          (APP_PDM_PCM_TASK_PRIORITY - 1u)
#define APP_AUDIO_PREPROCESS_SELECTED_MIXED         (0xFFu)

typedef enum
{
    /* 只取左 MIC。用于调试单通道质量或模型明确只吃左路时。 */
    APP_AUDIO_CHANNEL_MIX_LEFT = 0,
    /* 只取右 MIC。用于调试单通道质量或模型明确只吃右路时。 */
    APP_AUDIO_CHANNEL_MIX_RIGHT,
    /* 左右 MIC 直接平均，适合两路相位和延时差异较小时。 */
    APP_AUDIO_CHANNEL_MIX_AVERAGE,
    /* 按短时能量选择当前 block 中能量更高的一路，适合简单抗遮挡/抗弱信号。 */
    APP_AUDIO_CHANNEL_MIX_SELECT_BEST,
    /* 固定延时求和。fixed_delay_samples 用于补偿两个 MIC 的固定采样延时。 */
    APP_AUDIO_CHANNEL_MIX_DELAY_SUM
} app_audio_channel_mix_mode_t;

typedef enum
{
    /* Hann 窗，语音 log-mel 的常用选择。 */
    APP_AUDIO_WINDOW_HANN = 0,
    /* Hamming 窗，旁瓣抑制略有不同，保留给后续实验。 */
    APP_AUDIO_WINDOW_HAMMING,
    /* 矩形窗，只建议用于排查或和外部工具对齐。 */
    APP_AUDIO_WINDOW_RECTANGULAR
} app_audio_window_function_t;

typedef struct
{
    /* 采样率。当前必须与 app_pdm_pcm 的 SAMPLE_RATE_HZ 一致，除非底层采集也同步改。 */
    uint32_t sample_rate_hz;

    /* 模型输入窗口长度和窗口步长。默认 1000 ms 窗口、500 ms hop，即 50% 重叠。 */
    uint16_t window_ms;
    uint16_t window_hop_ms;

    /* STFT 分帧参数。frame_len_ms 是每一帧的时间长度，frame_hop_ms 是相邻帧步长。
     * fft_size 必须不小于 frame_len_samples，且不能超过 APP_AUDIO_PREPROCESS_MAX_FFT_SIZE。
     */
    uint16_t frame_len_ms;
    uint16_t frame_hop_ms;
    uint16_t fft_size;
    app_audio_window_function_t window_function;

    /* Mel 滤波器组参数。mel_high_hz 通常不超过 sample_rate_hz / 2。 */
    uint16_t mel_bin_count;
    float mel_low_hz;
    float mel_high_hz;

    /* 双通道转单声道策略。
     * fixed_delay_samples 仅在 APP_AUDIO_CHANNEL_MIX_DELAY_SUM 下生效：
     * 大于 0 表示右声道延后参与求和，小于 0 表示左声道延后参与求和。
     */
    app_audio_channel_mix_mode_t channel_mix_mode;
    int16_t fixed_delay_samples;

    /* Frontend profile.
     * v3 deployment uses BOARD_HTK_NO_NORM_V1: DC removal + energy gate,
     * HTK log-mel, no RMS gain, no per-window feature z-score.
     * v2 compatibility uses V2_ZSCORE: RMS gain + per-window feature z-score.
     */
    uint8_t frontend_profile;

    /* 特征提取前的信号整形参数。
     * energy_gate_threshold 使用“去直流后、RMS 归一化前”的平均能量；
     * normalize_target_rms 是目标 RMS，normalize_max_gain 限制静音附近被过度放大。
     */
    float energy_gate_threshold;
    float normalize_target_rms;
    float normalize_max_gain;

    /* log-mel 后处理参数。
     * 当前真实模型训练端使用 librosa.power_to_db(ref=np.max)，然后对每个 40x101
     * 特征图单独做 (feature - mean) / (std + 1e-6)。因此第一版 MIC demo 中
     * feature_mean/std 只保留为兼容字段，正式输出不会使用固定全局均值和方差。
     */
    float log_epsilon;
    float feature_mean;
    float feature_std;

    /* payload 类型参数。当前真实模型使用 APP_MODEL_AUDIO_QUANT_FLOAT32；
     * 旧量化字段先保留，便于后续如果换成 int8 模型时复用同一配置结构。
     */
    app_model_audio_quant_type_t quant_type;
    float quant_scale;
    int32_t quant_zero_point;
} app_audio_preprocess_config_t;

typedef struct
{
    /* 从 PDM/PCM 队列成功收到的 10 ms block 数。 */
    uint32_t blocks_received;
    /* 描述符或 block 内容校验失败次数。 */
    uint32_t invalid_blocks;
    /* block sequence 不连续次数，用于观察是否发生丢块。 */
    uint32_t sequence_gaps;
    /* 已经凑够 1 个滑动窗口的次数。 */
    uint32_t windows_ready;
    /* 因能量低于门限而没有发布给 CM55 的窗口数。 */
    uint32_t windows_energy_gated;
    /* 成功写入共享内存并标记 READY 的特征窗口数。 */
    uint32_t windows_published;
    /* CM55 尚未消费上一帧，导致本次特征无法发布的次数。 */
    uint32_t shared_busy;
    /* 最近处理到的 PDM block sequence。 */
    uint32_t last_sequence;
    /* 最近一个窗口去直流后的平均能量。 */
    float last_energy;
    /* 最近一次完整 Mel 特征提取耗时，单位 ms；profile 关闭时保持 0。 */
    uint32_t last_mel_ms;
    /* Mel 特征提取累计耗时，单位 ms；用于低频平均值。 */
    uint32_t mel_ms_total;
    /* Mel 特征提取最大耗时，单位 ms。 */
    uint32_t mel_ms_max;
    /* 已纳入 Mel 耗时统计的窗口数量。 */
    uint32_t mel_windows_profiled;
    /* 最近一次窗口整形/能量门限耗时，单位 ms；profile 关闭时保持 0。 */
    uint32_t last_condition_ms;
    /* 窗口整形/能量门限累计耗时，单位 ms。 */
    uint32_t condition_ms_total;
    /* 窗口整形/能量门限最大耗时，单位 ms。 */
    uint32_t condition_ms_max;
    /* 已纳入窗口整形/能量门限统计的窗口数量。 */
    uint32_t condition_windows_profiled;
    /* 最近一次全窗口频谱计算累计耗时，单位 ms。 */
    uint32_t last_spectrum_ms;
    /* 全窗口频谱计算累计耗时，单位 ms。 */
    uint32_t spectrum_ms_total;
    /* 全窗口频谱计算最大耗时，单位 ms。 */
    uint32_t spectrum_ms_max;
    /* 已纳入频谱统计的窗口数量。 */
    uint32_t spectrum_windows_profiled;
    /* 最近一次全窗口 Mel filterbank 累计耗时，单位 ms。 */
    uint32_t last_melbank_ms;
    /* 全窗口 Mel filterbank 累计耗时，单位 ms。 */
    uint32_t melbank_ms_total;
    /* 全窗口 Mel filterbank 最大耗时，单位 ms。 */
    uint32_t melbank_ms_max;
    /* 已纳入 Mel filterbank 统计的窗口数量。 */
    uint32_t melbank_windows_profiled;
    /* 最近一次成功发布给 CM55 的 input sequence。 */
    uint32_t last_published_sequence;
    /* 最近一次成功发布给 CM55 的 CM33 时间戳，单位 ms。 */
    uint32_t last_published_timestamp_ms;
    /* 最近一次选中的通道：0=左，1=右，0xFF=混合模式。 */
    uint8_t last_selected_channel;
    /* last_sequence 是否已经有效。 */
    bool has_last_sequence;
} app_audio_preprocess_stats_t;

cy_rslt_t app_audio_preprocess_task_init(void);
void app_audio_preprocess_task(void *pvParameters);

cy_rslt_t app_audio_preprocess_configure(
    const app_audio_preprocess_config_t *config);
const app_audio_preprocess_config_t *app_audio_preprocess_get_config(void);
void app_audio_preprocess_get_default_config(
    app_audio_preprocess_config_t *config);
void app_audio_preprocess_get_stats(app_audio_preprocess_stats_t *stats);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_AUDIO_PREPROCESS_H__ */

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

#ifndef APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE
#define APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE      (0u)
#endif

#ifndef APP_AUDIO_EVENT_FEATURE_DUMP_MAX_EVENTS
#define APP_AUDIO_EVENT_FEATURE_DUMP_MAX_EVENTS  (1u)
#endif

#ifndef APP_AUDIO_EVENT_FEATURE_DUMP_RING_DEPTH
#define APP_AUDIO_EVENT_FEATURE_DUMP_RING_DEPTH  (2u)
#endif

#ifndef APP_AUDIO_EVENT_PCM_DUMP_ENABLE
#define APP_AUDIO_EVENT_PCM_DUMP_ENABLE          (0u)
#endif

#ifndef APP_AUDIO_EVENT_PCM_DUMP_MAX_EVENTS
#define APP_AUDIO_EVENT_PCM_DUMP_MAX_EVENTS      (1u)
#endif

#ifndef APP_AUDIO_EVENT_PCM_DUMP_RING_DEPTH
#define APP_AUDIO_EVENT_PCM_DUMP_RING_DEPTH      (1u)
#endif

#ifndef APP_AUDIO_EVENT_PCM_DUMP_INCLUDE_FEATURE
#define APP_AUDIO_EVENT_PCM_DUMP_INCLUDE_FEATURE (1u)
#endif

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
#if ((APP_AUDIO_MODEL_SELECT == APP_AUDIO_MODEL_SELECT_V3_BOARD_HTK_HARDNEG) || \
     (APP_AUDIO_MODEL_SELECT == APP_AUDIO_MODEL_SELECT_V3_1_BOARD_FPFIX) || \
     (APP_AUDIO_MODEL_SELECT == APP_AUDIO_MODEL_SELECT_V3_3_2_B0_CURRENT))
#define APP_AUDIO_PREPROCESS_DEFAULT_FRONTEND_PROFILE \
    APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_BOARD_HTK_NO_NORM_V1
#else
#define APP_AUDIO_PREPROCESS_DEFAULT_FRONTEND_PROFILE \
    APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_V2_ZSCORE
#endif
#endif

#if ((APP_AUDIO_MODEL_SELECT == APP_AUDIO_MODEL_SELECT_V3_1_BOARD_FPFIX) && \
     (APP_AUDIO_PREPROCESS_DEFAULT_FRONTEND_PROFILE != \
      APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_BOARD_HTK_NO_NORM_V1))
#error "selector 4 requires APP_AUDIO_PREPROCESS_FRONTEND_PROFILE_BOARD_HTK_NO_NORM_V1"
#endif

#define APP_AUDIO_PREPROCESS_TASK_STACK_SIZE        (4096u)
#define APP_AUDIO_PREPROCESS_TASK_PRIORITY          (APP_PDM_PCM_TASK_PRIORITY - 1u)
#define APP_AUDIO_PREPROCESS_SELECTED_MIXED         (0xFFu)

typedef enum
{
    /* 只取左 MIC。用于调试单通道质量或模型明确只吃左路时。
     * 该模式会直接忽略右路采样，只保留左通道作为单声道输入，适合做通道隔离排查。
     */
    APP_AUDIO_CHANNEL_MIX_LEFT = 0,
    /* 只取右 MIC。用于调试单通道质量或模型明确只吃右路时。
     * 与 LEFT 模式对称，便于比较左右麦克风硬件质量、安装位置或遮挡差异。
     */
    APP_AUDIO_CHANNEL_MIX_RIGHT,
    /* 左右 MIC 直接平均，适合两路相位和延时差异较小时。
     * 该模式实现简单、计算量低，但若两路存在明显延时差，可能产生相消而削弱目标信号。
     */
    APP_AUDIO_CHANNEL_MIX_AVERAGE,
    /* 按短时能量选择当前 block 中能量更高的一路，适合简单抗遮挡/抗弱信号。
     * 它相当于在双通道之间做逐块的“择优单路”决策，避免弱路把强路平均稀释掉。
     */
    APP_AUDIO_CHANNEL_MIX_SELECT_BEST,
    /* 固定延时求和。fixed_delay_samples 用于补偿两个 MIC 的固定采样延时。
     * 适合硬件布局固定、两路到达时间差近似恒定的场景；只有延时补偿正确时，求和才可能提升信噪比。
     */
    APP_AUDIO_CHANNEL_MIX_DELAY_SUM
} app_audio_channel_mix_mode_t;

typedef enum
{
    /* Hann 窗，语音 log-mel 的常用选择。
     * 在频谱泄漏控制与主瓣宽度之间较均衡，是语音 STFT 中最常见的默认窗函数之一。
     */
    APP_AUDIO_WINDOW_HANN = 0,
    /* Hamming 窗，旁瓣抑制略有不同，保留给后续实验。
     * 适合在需要与外部工具或历史训练配置精确对齐时切换测试。
     */
    APP_AUDIO_WINDOW_HAMMING,
    /* 矩形窗，只建议用于排查或和外部工具对齐。
     * 该模式几乎不做加窗，频谱泄漏通常更明显，因此不建议作为正式部署默认值。
     */
    APP_AUDIO_WINDOW_RECTANGULAR
} app_audio_window_function_t;

/* 音频前处理运行配置。
 * 这是 CM33 侧从双通道 PCM 到模型输入特征全过程的参数集合。
 * 它既决定信号处理行为，也承担“与训练端配置对齐”的自描述作用，因此其中很多字段
 * 不仅影响算法执行本身，也会被写入共享内存描述符，供 CM55 和调试工具验证一致性。
 */
typedef struct
{
    /* 采样率。当前必须与 app_pdm_pcm 的 SAMPLE_RATE_HZ 一致，除非底层采集也同步改。
     * 这是整个时域与频域换算的基础参数；一旦与底层实际采样率不一致，后续所有时间窗口、
     * FFT 频率刻度和 mel 频带都会整体偏移。
     */
    uint32_t sample_rate_hz;

    /* 模型输入窗口长度和窗口步长。默认 1000 ms 窗口、500 ms hop，即 50% 重叠。
     * window_ms 决定单次推理覆盖多长历史音频；window_hop_ms 决定两次推理之间的更新频率。
     * 两者共同决定系统时延、重叠率以及事件被重复观察的概率。
     */
    uint16_t window_ms;
    uint16_t window_hop_ms;

    /* STFT 分帧参数。frame_len_ms 是每一帧的时间长度，frame_hop_ms 是相邻帧步长。
     * fft_size 必须不小于 frame_len_samples，且不能超过 APP_AUDIO_PREPROCESS_MAX_FFT_SIZE。
     * 这组参数共同决定 time_bin_count，并直接影响最终 40x101 特征图是否能与模型对齐。
     */
    uint16_t frame_len_ms;
    uint16_t frame_hop_ms;
    uint16_t fft_size;
    /* 频谱分析使用的窗函数类型。
     * 它决定每个 STFT 帧在进入 FFT 前采用哪一种加窗方式，对频谱泄漏和能量分布有直接影响。
     */
    app_audio_window_function_t window_function;

    /* Mel 滤波器组参数。mel_high_hz 通常不超过 sample_rate_hz / 2。
     * mel_bin_count 必须与共享协议和 CM55 模型输入约定保持一致；
     * mel_low_hz / mel_high_hz 决定频带覆盖范围。
     */
    uint16_t mel_bin_count;
    /* Mel 频带下边界频率，单位 Hz。
     * 低于该频率的能量通常会被忽略，可用于抑制极低频环境噪声或直流附近干扰。
     */
    float mel_low_hz;
    /* Mel 频带上边界频率，单位 Hz。
     * 它与 sample_rate_hz 一起限制最高可分析频段；设置过高通常没有意义，超过 Nyquist 的部分也无法保留。
     */
    float mel_high_hz;

    /* 双通道转单声道策略。
     * fixed_delay_samples 仅在 APP_AUDIO_CHANNEL_MIX_DELAY_SUM 下生效：
     * 大于 0 表示右声道延后参与求和，小于 0 表示左声道延后参与求和。
     * 其余模式下该字段保留但不参与运算。
     */
    app_audio_channel_mix_mode_t channel_mix_mode;
    /* 固定延时求和模式使用的样本级延时补偿量。
     * 单位是“采样点”，不是毫秒；该值的正负决定哪一路被延后对齐。
     */
    int16_t fixed_delay_samples;

    /* Frontend profile.
     * v3 deployment uses BOARD_HTK_NO_NORM_V1: DC removal + energy gate,
     * HTK log-mel, no RMS gain, no per-window feature z-score.
     * v2 compatibility uses V2_ZSCORE: RMS gain + per-window feature z-score.
     * 该字段本质上是在“同一份采集流”上切换不同训练前端约定。
     */
    uint8_t frontend_profile;

    /* 特征提取前的信号整形参数。
     * energy_gate_threshold 使用“去直流后、RMS 归一化前”的平均能量；
     * normalize_target_rms 是目标 RMS，normalize_max_gain 限制静音附近被过度放大。
     * 如果 frontend_profile 禁用了 RMS gain，这两项会只保留配置记录语义。
     */
    /* 能量门限阈值。
     * 当窗口平均能量低于该值时，可认为该窗口过于安静，不值得继续提取特征并送入模型。
     */
    float energy_gate_threshold;
    /* 目标 RMS。
     * 仅在启用 RMS 归一化的前端配置下参与运算，用于把不同响度样本拉到相近幅度尺度。
     */
    float normalize_target_rms;
    /* 最大允许增益。
     * 防止在接近静音时为了追赶目标 RMS 而把底噪极度放大，起到安全限幅作用。
     */
    float normalize_max_gain;

    /* log-mel 后处理参数。
     * 当前真实模型训练端使用 librosa.power_to_db(ref=np.max)，然后对每个 40x101
     * 特征图单独做 (feature - mean) / (std + 1e-6)。因此第一版 MIC demo 中
     * feature_mean/std 只保留为兼容字段，正式输出不会使用固定全局均值和方差。
     * log_epsilon 用于避免对极小功率直接取对数时出现数值问题。
     */
    /* 对数运算保护项。
     * 在功率极小时提供最小下界，避免出现 log(0) 或数值过度下溢。
     */
    float log_epsilon;
    /* 兼容保留的全局均值参数。
     * 某些历史前端可能会使用固定均值做归一化；当前主链路主要保留该字段用于配置对齐与记录。
     */
    float feature_mean;
    /* 兼容保留的全局标准差参数。
     * 与 feature_mean 配对存在，便于与旧模型或旧导出链路进行参数映射。
     */
    float feature_std;

    /* payload 类型参数。当前真实模型使用 APP_MODEL_AUDIO_QUANT_FLOAT32；
     * 旧量化字段先保留，便于后续如果换成 int8 模型时复用同一配置结构。
     * 在 float32 模式下，quant_scale/quant_zero_point 主要用于协议自描述。
     */
    /* 共享内存 payload 的量化/编码类型。
     * 它决定 CM33 最终向共享区写入的是整数张量还是 float32 张量，并影响 CM55 的解释方式。
     */
    app_model_audio_quant_type_t quant_type;
    /* 量化缩放因子。
     * 当 quant_type 为 INT8/UINT8 时用于反量化；float32 模式下通常保留为自描述信息。
     */
    float quant_scale;
    /* 量化零点。
     * 与 quant_scale 共同定义整数到浮点的映射关系；float32 模式下一般保持为 0。
     */
    int32_t quant_zero_point;
} app_audio_preprocess_config_t;

/* 音频前处理运行统计。
 * 该结构用于汇总 CM33 前处理任务在采集、窗口构造、特征提取、共享内存发布等阶段的运行情况。
 * 它既包含正确性相关计数，也包含耗时 profile 指标，可用于判断问题出在数据源、算法参数、
 * 共享区握手，还是性能不足。
 */
typedef struct
{
    /* 从 PDM/PCM 队列成功收到的 10 ms block 数。
     * 反映采集链路向前处理任务实际输送了多少基础音频块。
     */
    uint32_t blocks_received;
    /* 描述符或 block 内容校验失败次数。
     * 该计数增长通常意味着底层块元数据异常、长度不匹配或数据完整性存在问题。
     */
    uint32_t invalid_blocks;
    /* block sequence 不连续次数，用于观察是否发生丢块。
     * 若该值上升，说明采集到前处理之间可能存在队列溢出、任务饥饿或生产消费节奏失衡。
     */
    uint32_t sequence_gaps;
    /* 已经凑够 1 个滑动窗口的次数。
     * 表示环形缓冲中已经足够形成一次完整 window_ms 分析窗口的次数。
     */
    uint32_t windows_ready;
    /* 因能量低于门限而没有发布给 CM55 的窗口数。
     * 该值体现 energy gate 的筛除强度，可帮助判断门限是否设置过严或过松。
     */
    uint32_t windows_energy_gated;
    /* 成功写入共享内存并标记 READY 的特征窗口数。
     * 这是正式进入跨核推理链路的窗口数量，通常应小于等于 windows_ready。
     */
    uint32_t windows_published;
    /* CM55 尚未消费上一帧，导致本次特征无法发布的次数。
     * 该值升高说明共享输入槽成为瓶颈，CM33 生成特征的速度快于 CM55 消费速度。
     */
    uint32_t shared_busy;
    /* 最近处理到的 PDM block sequence。
     * 用于定位前处理当前已经跟进到底层采集流的哪个位置。
     */
    uint32_t last_sequence;
    /* 最近一个窗口去直流后的平均能量。
     * 它可用于快速观察最近一帧声音强弱，并与 energy_gate_threshold 直接对比。
     */
    float last_energy;
    /* 最近一次完整 Mel 特征提取耗时，单位 ms；profile 关闭时保持 0。
     * 这是单窗口级别的最近耗时快照，便于观察瞬时性能波动。
     */
    uint32_t last_mel_ms;
    /* Mel 特征提取累计耗时，单位 ms；用于低频平均值。
     * 可与 mel_windows_profiled 配合计算长期平均耗时。
     */
    uint32_t mel_ms_total;
    /* Mel 特征提取最大耗时，单位 ms。
     * 用于捕捉最差时延情况，评估是否存在偶发的实时性尖峰。
     */
    uint32_t mel_ms_max;
    /* 已纳入 Mel 耗时统计的窗口数量。
     * 是 mel_ms_total 的分母基础，用于求平均值时必须一并参考。
     */
    uint32_t mel_windows_profiled;
    /* 最近一次窗口整形/能量门限耗时，单位 ms；profile 关闭时保持 0。
     * 覆盖去直流、RMS 归一化、能量门限等窗口级前置处理。
     */
    uint32_t last_condition_ms;
    /* 窗口整形/能量门限累计耗时，单位 ms。
     * 用于评估特征提取前信号整形部分的长期成本。
     */
    uint32_t condition_ms_total;
    /* 窗口整形/能量门限最大耗时，单位 ms。
     * 可帮助判断异常高耗时是否出现在特征提取前的整形阶段。
     */
    uint32_t condition_ms_max;
    /* 已纳入窗口整形/能量门限统计的窗口数量。
     * 用于计算 condition 阶段的均值并和 mel 阶段耗时拆分对比。
     */
    uint32_t condition_windows_profiled;
    /* 最近一次全窗口频谱计算累计耗时，单位 ms。
     * 这里的“频谱计算”通常对应分帧后 FFT / DFT 主体部分的处理成本。
     */
    uint32_t last_spectrum_ms;
    /* 全窗口频谱计算累计耗时，单位 ms。
     * 用于长期统计 FFT 相关部分的时间开销。
     */
    uint32_t spectrum_ms_total;
    /* 全窗口频谱计算最大耗时，单位 ms。
     * 有助于识别频谱计算是否是实时链路中的峰值耗时来源。
     */
    uint32_t spectrum_ms_max;
    /* 已纳入频谱统计的窗口数量。
     * 应与 spectrum_ms_total 一起解读，以避免把累计值误当单次值。
     */
    uint32_t spectrum_windows_profiled;
    /* 最近一次全窗口 Mel filterbank 累计耗时，单位 ms。
     * 反映功率谱映射到 mel 频带阶段的最近一次执行成本。
     */
    uint32_t last_melbank_ms;
    /* 全窗口 Mel filterbank 累计耗时，单位 ms。
     * 可用于和 FFT 耗时分开分析，判断性能瓶颈是在频谱还是 mel 映射阶段。
     */
    uint32_t melbank_ms_total;
    /* 全窗口 Mel filterbank 最大耗时，单位 ms。
     * 用于观察 mel 滤波阶段的最坏时延表现。
     */
    uint32_t melbank_ms_max;
    /* 已纳入 Mel filterbank 统计的窗口数量。
     * 是计算该阶段平均耗时时的有效计数。
     */
    uint32_t melbank_windows_profiled;
    /* 最近一次成功发布给 CM55 的 input sequence。
     * 它对应共享区 producer_sequence 最近一次成功推进到的值。
     */
    uint32_t last_published_sequence;
    /* 最近一次成功发布给 CM55 的 CM33 时间戳，单位 ms。
     * 可与 CM55 写回结果时间对比，用于估算跨核推理链路端到端延迟。
     */
    uint32_t last_published_timestamp_ms;
    /* 最近一次选中的通道：0=左，1=右，0xFF=混合模式。
     * 用于回看最近发布窗口采用了哪一路或哪种混合结果。
     */
    uint8_t last_selected_channel;
    /* last_sequence 是否已经有效。
     * 用于区分“当前序号恰好是 0”和“任务尚未处理过任何有效 block”。
     */
    bool has_last_sequence;
} app_audio_preprocess_stats_t;

/* CM33_NS owns app-model shared boot/reset initialization.
 * These helpers are intentionally not private audio-task initialization.
 */
cy_rslt_t app_model_shared_boot_init_cm33_owner(void);
bool app_model_shared_boot_is_ready(void);

cy_rslt_t app_audio_preprocess_task_init(void);
void app_audio_preprocess_task(void *pvParameters);

cy_rslt_t app_audio_preprocess_configure(
    const app_audio_preprocess_config_t *config);
const app_audio_preprocess_config_t *app_audio_preprocess_get_config(void);
void app_audio_preprocess_get_default_config(
    app_audio_preprocess_config_t *config);
void app_audio_preprocess_get_stats(app_audio_preprocess_stats_t *stats);

#if (APP_AUDIO_EVENT_FEATURE_DUMP_ENABLE)
bool app_audio_preprocess_event_feature_dump_can_emit(void);
void app_audio_preprocess_dump_event_feature(uint32_t input_sequence,
                                             uint32_t result_sequence,
                                             uint32_t event_index,
                                             float cough_prob,
                                             float event_energy);
#endif

#if (APP_AUDIO_EVENT_PCM_DUMP_ENABLE)
bool app_audio_preprocess_event_pcm_dump_can_emit(void);
void app_audio_preprocess_dump_event_pcm(uint32_t input_sequence,
                                         uint32_t result_sequence,
                                         uint32_t event_index,
                                         float cough_prob,
                                         float event_energy);
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __APP_AUDIO_PREPROCESS_H__ */

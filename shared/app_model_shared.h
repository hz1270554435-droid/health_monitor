/*******************************************************************************
* File Name : app_model_shared.h
*
* Description : CM33/CM55 之间传递“处理后模型输入”和“模型结果”的固定共享内存协议。
*
* 本头文件只定义共享内存中的协议和地址，不定义原始采集缓冲。
* CM33 负责把 log-mel float32 特征写入输入槽；CM55 按同一份头文件读取输入槽，
* 并把推理结果写回结果槽。当前结果槽采用“最新结果覆盖旧结果”的调试友好语义。
* 原始 PDM/PCM 采集数据不得放入这块协议区，避免和模型输入互相干扰。
*******************************************************************************/

#ifndef __APP_MODEL_SHARED_H__
#define __APP_MODEL_SHARED_H__

#include <stdint.h>

#include "cy_pdl.h"
#if defined(COMPONENT_CM55)
#include "cymem_CM55_0.h"
#else
#include "cymem_CM33_0.h"
#endif

#if defined(__cplusplus)
extern "C" {
#endif

/* "MFEA" = Model FEAture，用于 CM55 判断共享区是否已经由 CM33 初始化。 */
#define APP_MODEL_SHARED_MAGIC                 (0x4D464541u)
#define APP_MODEL_SHARED_VERSION               (3u)

/* 共享区固定偏移。
 * m33_m55_shared 开头已经有历史共享数据使用者，例如雷达帧缓冲。
 * 这里预留前 32 KiB，音频模型输入从固定偏移开始。后续如果新增共享生产者，
 * 应在本协议中继续划分明确、互不重叠的偏移，不要依赖 linker section 的排列顺序。
 */
#define APP_MODEL_SHARED_OFFSET_BYTES          (0x00008000u)

#if defined(COMPONENT_CM55)
#define APP_MODEL_SHARED_BASE_ADDR             (CYMEM_CM55_0_m33_m55_shared_START)
#define APP_MODEL_SHARED_REGION_SIZE_BYTES     (CYMEM_CM55_0_m33_m55_shared_SIZE)
#else
#define APP_MODEL_SHARED_BASE_ADDR             (CYMEM_CM33_0_m33_m55_shared_START)
#define APP_MODEL_SHARED_REGION_SIZE_BYTES     (CYMEM_CM33_0_m33_m55_shared_SIZE)
#endif

#define APP_MODEL_SHARED_ADDR                  (APP_MODEL_SHARED_BASE_ADDR + \
                                                APP_MODEL_SHARED_OFFSET_BYTES)

/* 当前真实音频模型的固定输入形状。
 *
 * CM55 已导入的 AUDIO_compute() 入口要求输入 float[1,40,101]，PC 测试向量也按
 * NCHW 顺序展平成 40 * 101 个 float32。第一版 MIC demo 先把共享协议固定到这个
 * shape，避免 CM33 前处理和 CM55 模型各自解释一套尺寸。
 */
#define APP_MODEL_AUDIO_MODEL_MEL_BINS         (40u)
#define APP_MODEL_AUDIO_MODEL_TIME_BINS        (101u)
#define APP_MODEL_AUDIO_MODEL_FLOAT_COUNT      (APP_MODEL_AUDIO_MODEL_MEL_BINS * \
                                                APP_MODEL_AUDIO_MODEL_TIME_BINS)
#define APP_MODEL_AUDIO_MODEL_INPUT_BYTES      (APP_MODEL_AUDIO_MODEL_FLOAT_COUNT * \
                                                sizeof(float))

/* 音频特征 payload 上限。
 * 当前上限直接等于真实模型输入大小；如果后续更换模型 shape，需要同步扩展
 * CM33 前处理、CM55 模型输入和本协议版本号。
 */
#define APP_MODEL_AUDIO_FEATURE_MAX_MEL_BINS   (APP_MODEL_AUDIO_MODEL_MEL_BINS)
#define APP_MODEL_AUDIO_FEATURE_MAX_TIME_BINS  (APP_MODEL_AUDIO_MODEL_TIME_BINS)
#define APP_MODEL_AUDIO_FEATURE_MAX_ELEMENTS   (APP_MODEL_AUDIO_MODEL_FLOAT_COUNT)
#define APP_MODEL_AUDIO_FEATURE_MAX_BYTES      (APP_MODEL_AUDIO_MODEL_INPUT_BYTES)

/* smoke test 使用独立 result_sequence 区间，避免和正式 MIC 输入序号冲突。 */
#define APP_MODEL_RESULT_SEQUENCE_SMOKE_BASE   (0x80000000UL)

/* CM55 推理结果分数槽上限。
 * 当前真实模型只输出两个 logits；scores[] 额外保留 cough_prob、能量等调试值。
 * 后续如果类别数或输出头超过该值，需要同步扩展本协议以及 CM55 推理任务。
 */
#define APP_MODEL_INFERENCE_MAX_SCORES         (16u)

typedef enum
{
    /* 没有可读输入，或上一帧已经被 CM55 消费。 */
    APP_MODEL_SHARED_INPUT_EMPTY = 0,
    /* CM33 正在写入描述符和 payload，CM55 不应读取。 */
    APP_MODEL_SHARED_INPUT_WRITING,
    /* CM33 已经写完一帧模型输入，CM55 可以读取。 */
    APP_MODEL_SHARED_INPUT_READY,
    /* CM55 正在读取，CM33 不应覆盖。 */
    APP_MODEL_SHARED_INPUT_READING
} app_model_shared_input_state_t;

typedef enum
{
    /* 暂无推理结果，或结果已经被未来的 CM33 结果消费者清空。 */
    APP_MODEL_SHARED_RESULT_EMPTY = 0,
    /* CM55 正在写入 result/result_sequence，CM33 不应读取。 */
    APP_MODEL_SHARED_RESULT_WRITING,
    /* CM55 已写入最新一次推理结果。当前框架采用“保留最新结果”语义。 */
    APP_MODEL_SHARED_RESULT_READY
} app_model_shared_result_state_t;

typedef enum
{
    /* payload 中每个字节按 int8_t 解释，范围 -128 到 127。 */
    APP_MODEL_AUDIO_QUANT_INT8 = 0,
    /* payload 中每个字节按 uint8_t 解释，范围 0 到 255。 */
    APP_MODEL_AUDIO_QUANT_UINT8,
    /* payload 中每 4 个字节按 little-endian float32 解释，当前真实模型使用该格式。 */
    APP_MODEL_AUDIO_QUANT_FLOAT32
} app_model_audio_quant_type_t;

typedef enum
{
    /* 模型推理成功执行。 */
    APP_MODEL_INFERENCE_STATUS_OK = 0,
    /* CM55 发现共享内存中的输入描述符不合法。 */
    APP_MODEL_INFERENCE_STATUS_INVALID_INPUT,
    /* 模型尚未导入；当前仅完成数据流框架。 */
    APP_MODEL_INFERENCE_STATUS_MODEL_NOT_READY,
    /* 模型初始化、soft reset 或运行时接口返回错误。 */
    APP_MODEL_INFERENCE_STATUS_MODEL_ERROR
} app_model_inference_status_t;

typedef struct
{
    /* 音频特征序号，由 CM33 每发布一帧递增一次。 */
    uint32_t sequence;
    /* CM33 生成该特征时的系统 tick 时间，单位 ms。 */
    uint32_t timestamp_ms;
    /* 以下字段描述本帧特征对应的前处理参数，方便 CM55 和调试工具校验。 */
    uint16_t sample_rate_hz;
    uint16_t window_ms;
    uint16_t window_hop_ms;
    uint16_t frame_len_ms;
    uint16_t frame_hop_ms;
    uint16_t fft_size;
    uint16_t mel_bin_count;
    uint16_t time_bin_count;
    /* payload 有效字节数。float32 模式下等于 mel_bin_count * time_bin_count * 4。 */
    uint16_t payload_bytes;
    /* 量化类型，取 app_model_audio_quant_type_t。 */
    uint8_t quant_type;
    /* 反量化公式：float_value = (q - quant_zero_point) * quant_scale。
     * float32 模式下保留为 zero=0/scale=1，CM55 直接把 payload 解释为 float。
     */
    int32_t quant_zero_point;
    float quant_scale;
    /* 去直流后、归一化前的平均能量；可用于观察能量门限效果。 */
    float energy;
    /* 0=左 MIC，1=右 MIC，0xFF=平均/延时求和等混合模式。 */
    uint8_t selected_channel;
    /* 1 表示本帧通过能量门限并可供推理；0 保留给未来无效帧协议。 */
    uint8_t valid;
    uint8_t reserved[2];
} app_model_audio_feature_desc_t;

typedef struct
{
    /* 本结果对应的输入特征序号，等于 audio.sequence。 */
    uint32_t input_sequence;
    /* CM55 写入结果时的系统 tick 时间，单位 ms。 */
    uint32_t timestamp_ms;
    /* 本次推理处理耗时，单位 ms。 */
    uint32_t inference_time_ms;
    /* 有效类别分数个数；模型未接入时为 0。 */
    uint16_t class_count;
    /* 取 app_model_inference_status_t。 */
    uint8_t status;
    uint8_t reserved;
    /* 通用浮点分数槽。当前约定：
     * scores[0]/[1]=两个 logits，scores[2]=cough_prob，
     * scores[3]=CM33 窗口能量，scores[4]=选中通道。
     */
    float scores[APP_MODEL_INFERENCE_MAX_SCORES];
} app_model_inference_result_t;

typedef struct
{
    /* 共享区基础信息和生产/消费状态。
     * input_state 只描述 CM33->CM55 的输入槽；result_state 只描述 CM55->CM33 的结果槽。
     * 两个槽位互相独立，避免 CM55 推理耗时阻塞 CM33 发布下一帧输入。
     */
    volatile uint32_t magic;
    volatile uint32_t version;
    volatile uint32_t input_state;
    volatile uint32_t result_state;
    volatile uint32_t producer_sequence;
    volatile uint32_t consumer_sequence;
    volatile uint32_t result_sequence;
    app_model_audio_feature_desc_t audio;
    app_model_inference_result_t result;

    /* 特征 payload 按 mel-major 展平：
     * audio_payload[mel * time_bin_count + time]。
     * 当前真实模型要求 float32，因此每 4 字节组成一个 float；保留 uint8_t 数组
     * 是为了协议区按字节计数和 memcpy 更直接。
     */
    uint8_t audio_payload[APP_MODEL_AUDIO_FEATURE_MAX_BYTES];
} app_model_shared_region_t;

#define APP_MODEL_SHARED_REGION \
    ((volatile app_model_shared_region_t *)APP_MODEL_SHARED_ADDR)

/* Cache 一致性钩子。
 * 如果 m33_m55_shared 在任一核心上是 cacheable，需要在工程级别覆盖这两个宏，
 * 在 CM33 写完后 clean，在 CM55 读前 invalidate。当前默认实现为空操作。
 */
#ifndef APP_MODEL_SHARED_CLEAN_CACHE
#define APP_MODEL_SHARED_CLEAN_CACHE(address, size) \
    do { (void)(address); (void)(size); } while (0)
#endif

#ifndef APP_MODEL_SHARED_INVALIDATE_CACHE
#define APP_MODEL_SHARED_INVALIDATE_CACHE(address, size) \
    do { (void)(address); (void)(size); } while (0)
#endif

typedef char app_model_shared_region_size_check[
    (sizeof(app_model_shared_region_t) <=
     (APP_MODEL_SHARED_REGION_SIZE_BYTES - APP_MODEL_SHARED_OFFSET_BYTES)) ? 1 : -1];

#if defined(__cplusplus)
}
#endif

#endif /* __APP_MODEL_SHARED_H__ */

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
    /* 没有可读输入，或上一帧已经被 CM55 消费。
     * 进入条件：系统初始化后默认状态，或 CM55 完成读取并由对应逻辑清空槽位后。
     * 使用约束：CM33 只有在该状态下才应开始写入新一帧输入，避免覆盖 CM55 尚未消费的数据。
     */
    APP_MODEL_SHARED_INPUT_EMPTY = 0,
    /* CM33 正在写入描述符和 payload，CM55 不应读取。
     * 该状态表示输入槽内容处于“构建中”，其中 sequence、尺寸字段、量化参数、payload
     * 可能只写入了一部分；任何消费者在此阶段读取都可能拿到不一致的半帧数据。
     */
    APP_MODEL_SHARED_INPUT_WRITING,
    /* CM33 已经写完一帧模型输入，CM55 可以读取。
     * 含义：描述符字段与 audio_payload 已完成写入，并且必要的 cache clean 已完成。
     * CM55 观察到该状态后，可将状态切换为 READING 并开始取数。
     */
    APP_MODEL_SHARED_INPUT_READY,
    /* CM55 正在读取，CM33 不应覆盖。
     * 该状态用于显式标识“消费者已接管此帧输入”；在 CM55 完成复制/解释输入前，
     * CM33 必须避免复用或覆盖共享槽位，防止出现读写并发冲突。
     */
    APP_MODEL_SHARED_INPUT_READING
} app_model_shared_input_state_t;

typedef enum
{
    /* 暂无推理结果，或结果已经被未来的 CM33 结果消费者清空。
     * 该状态表示结果槽当前没有一份可供上层业务读取和展示的有效结果。
     */
    APP_MODEL_SHARED_RESULT_EMPTY = 0,
    /* CM55 正在写入 result/result_sequence，CM33 不应读取。
     * 该阶段 result 描述信息、scores 数组与序号字段可能尚未完全一致，
     * 因此 CM33 只能等待其切换到 READY 后再解析。
     */
    APP_MODEL_SHARED_RESULT_WRITING,
    /* CM55 已写入最新一次推理结果。当前框架采用“保留最新结果”语义。
     * 含义：结果槽始终保存最近一次完成的推理输出，新的结果会直接覆盖旧结果，
     * 便于调试端或监控任务读取“最新状态”，而不是积压一个结果队列。
     */
    APP_MODEL_SHARED_RESULT_READY
} app_model_shared_result_state_t;

typedef enum
{
    /* payload 中每个字节按 int8_t 解释，范围 -128 到 127。
     * 适用于对称或近似对称量化模型；读取侧需结合 quant_zero_point 与 quant_scale
     * 进行反量化，恢复近似浮点特征值。
     */
    APP_MODEL_AUDIO_QUANT_INT8 = 0,
    /* payload 中每个字节按 uint8_t 解释，范围 0 到 255。
     * 常用于非对称量化输入；零点通常不为 0，CM55 必须按描述符中的量化参数解释。
     */
    APP_MODEL_AUDIO_QUANT_UINT8,
    /* payload 中每 4 个字节按 little-endian float32 解释，当前真实模型使用该格式。
     * 这是当前主链路的真实部署格式：CM33 直接写入 float32 log-mel，CM55 直接按
     * 浮点输入张量读取，无需额外反量化步骤。
     */
    APP_MODEL_AUDIO_QUANT_FLOAT32
} app_model_audio_quant_type_t;

typedef enum
{
    /* 模型推理成功执行。
     * 表示本次输入已被模型正常处理，result 中的 class_count 与 scores[] 可按约定解释。
     */
    APP_MODEL_INFERENCE_STATUS_OK = 0,
    /* CM55 发现共享内存中的输入描述符不合法。
     * 典型触发原因包括：payload_bytes 与尺寸不匹配、量化类型非法、mel/time bin 越界、
     * 或输入元数据与当前模型 ABI 不一致。出现该状态时，本次结果通常不应参与业务判定。
     */
    APP_MODEL_INFERENCE_STATUS_INVALID_INPUT,
    /* 模型尚未导入；当前仅完成数据流框架。
     * 该状态用于板端链路联调阶段，表示共享内存和任务框架已打通，但真正的模型执行入口
     * 还未准备好，因此结果槽只提供“未就绪”状态而不是分类分数。
     */
    APP_MODEL_INFERENCE_STATUS_MODEL_NOT_READY,
    /* 模型初始化、soft reset 或运行时接口返回错误。
     * 说明问题发生在模型组件内部，而非输入描述符本身；通常需要结合日志继续排查
     * 模型加载、运行时状态机或底层推理库错误。
     */
    APP_MODEL_INFERENCE_STATUS_MODEL_ERROR
} app_model_inference_status_t;

/* 音频特征描述符。
 * 作用：为 audio_payload 这块原始字节数组提供完整的“如何解释这份输入”的元数据。
 * 读取者不能只看 payload 本身，必须联合本结构中的尺寸、量化、时间戳和前处理参数
 * 才能正确恢复出模型输入张量，并判断这份输入是否来自预期的音频前处理链路。
 */
typedef struct
{
    /* 音频特征序号，由 CM33 每发布一帧递增一次。
     * 用途：作为跨核链路的主关联键，CM55 读取输入、写回结果、调试日志比对时都依赖它
     * 确认“这份结果对应哪一帧输入”。
     */
    uint32_t sequence;
    /* CM33 生成该特征时的系统 tick 时间，单位 ms。
     * 语义：记录输入产生时刻，而不是 CM55 开始推理的时间；可用于估算端到端时延、
     * 观察实时性抖动，以及与采集侧日志对时。
     */
    uint32_t timestamp_ms;
    /* 以下字段描述本帧特征对应的前处理参数，方便 CM55 和调试工具校验。 */
    /* 原始音频采样率，单位 Hz。
     * 该值用于说明本帧 mel 特征是基于哪一个时域采样率计算得到的；若与模型训练假设不符，
     * 即使 mel_bin/time_bin 数量一致，也可能导致推理分布失真。
     */
    uint16_t sample_rate_hz;
    /* 上层分析窗口时长，单位 ms。
     * 通常表示一次送入特征提取流程的整体音频时间跨度，用于辅助确认当前帧的时间上下文。
     */
    uint16_t window_ms;
    /* 上层分析窗口滑移步长，单位 ms。
     * 当系统按固定周期发布特征时，该字段描述相邻两帧特征在时间轴上的前进间隔。
     */
    uint16_t window_hop_ms;
    /* 短时帧长度，单位 ms。
     * 对应 STFT / mel 提取内部单帧分析窗大小，是 time_bin_count 与频率分辨率的重要来源之一。
     */
    uint16_t frame_len_ms;
    /* 短时帧步长，单位 ms。
     * 描述相邻分析帧之间的 hop，大多数 time_bin_count 的形成都直接取决于该参数。
     */
    uint16_t frame_hop_ms;
    /* FFT 点数。
     * 该值决定频谱离散化精度，也间接影响 mel 滤波器组的输入频谱分辨率。
     */
    uint16_t fft_size;
    /* 当前 payload 中每个时间步对应的 mel 频带数量。
     * 对于当前主模型应等于 40；若将来模型版本升级，可通过该字段与协议版本共同区分。
     */
    uint16_t mel_bin_count;
    /* 当前 payload 中时间轴上的帧数。
     * 对于当前主模型应等于 101；读取侧应结合 mel_bin_count 验证总元素数是否合法。
     */
    uint16_t time_bin_count;
    /* payload 有效字节数。float32 模式下等于 mel_bin_count * time_bin_count * 4。
     * 这是消费侧做边界检查的第一道保护：既不能超过协议上限，也应与量化类型和尺寸匹配。
     */
    uint16_t payload_bytes;
    /* 量化类型，取 app_model_audio_quant_type_t。
     * 它决定 audio_payload 的字节应按 int8 / uint8 / float32 哪种方式解释。
     */
    uint8_t quant_type;
    /* 反量化公式：float_value = (q - quant_zero_point) * quant_scale。
     * 对量化模型而言，这是把整数特征恢复到近似实数域所需的零点参数。
     * float32 模式下保留为 zero=0/scale=1，表示无需量化反变换。
     */
    int32_t quant_zero_point;
    /* 量化缩放因子。
     * 与 quant_zero_point 配套使用；其数值越大，单个量化步长映射回实数域的间隔越大。
     */
    float quant_scale;
    /* 去直流后、归一化前的平均能量；可用于观察能量门限效果。
     * 这是对原始音频强弱的一个摘要指标，可辅助判断静音、弱音或异常高能量样本。
     */
    float energy;
    /* 0=左 MIC，1=右 MIC，0xFF=平均/延时求和等混合模式。
     * 用于记录当前特征来自哪个通道或何种通道融合策略，便于问题定位与多麦配置扩展。
     */
    uint8_t selected_channel;
    /* 1 表示本帧通过能量门限并可供推理；0 保留给未来无效帧协议。
     * 当前主链路通常只发布可推理帧；该字段为后续引入“保留帧但显式标无效”机制预留。
     */
    uint8_t valid;
    /* 预留字节。
     * 目的：保持结构体对齐，并为后续新增小字段预留空间；当前读取侧应忽略其内容。
     */
    uint8_t reserved[2];
} app_model_audio_feature_desc_t;

/* 模型推理结果描述符。
 * 作用：承载一次 CM55 推理完成后的状态、耗时与分数输出。
 * 结果消费者必须优先检查 status、class_count 和 input_sequence，确认这份结果既是成功结果，
 * 又确实对应自己关心的那一帧输入，然后再按约定解释 scores[] 中的槽位。
 */
typedef struct
{
    /* 本结果对应的输入特征序号，等于 audio.sequence。
     * 这是输入输出关联的核心字段，用于确认当前结果究竟是针对哪一帧特征产生的。
     */
    uint32_t input_sequence;
    /* CM55 写入结果时的系统 tick 时间，单位 ms。
     * 与 audio.timestamp_ms 结合可近似估算“输入产生到结果落盘”的端到端延迟。
     */
    uint32_t timestamp_ms;
    /* 本次推理处理耗时，单位 ms。
     * 通常表示模型执行路径自身消耗的时间，不一定包含输入等待、任务调度或结果轮询延迟。
     */
    uint32_t inference_time_ms;
    /* 有效类别分数个数；模型未接入时为 0。
     * 该值描述 scores[] 前多少项可按“类别输出”解释；其后槽位可以保留给调试信息或扩展字段。
     */
    uint16_t class_count;
    /* 取 app_model_inference_status_t。
     * 读取侧应先检查该字段；只有 status=OK 时，类别结果通常才有业务上的可解释性。
     */
    uint8_t status;
    /* 预留字节。
     * 当前无业务语义，主要用于结构体布局对齐以及未来状态位扩展。
     */
    uint8_t reserved;
    /* 通用浮点分数槽。
     * 设计原则：在固定 ABI 下保留一组可扩展的浮点槽位，既能装模型原始输出，
     * 也能附带调试和监控所需的派生值，避免为每次调试修改共享内存结构。
     * 当前约定：
     * scores[0]/[1]=两个 logits，scores[2]=cough_prob，
     * scores[3]=CM33 窗口能量，scores[4]=选中通道。
     * 其余槽位当前未定义，读取侧不得擅自赋予固定业务含义。
     */
    float scores[APP_MODEL_INFERENCE_MAX_SCORES];
} app_model_inference_result_t;

/* CM33/CM55 共享区总结构。
 * 这是板端链路协议的核心内存布局定义：头部状态字段 + 输入描述符 + 结果描述符 + 输入 payload。
 * 任一核心都必须严格按照本结构的字段顺序、大小和语义访问共享区；如果需要变更布局或字段含义，
 * 应同步更新协议版本，并确保所有生产者、消费者以及测试向量一起升级。
 */
typedef struct
{
    /* 共享区基础信息和生产/消费状态。
     * input_state 只描述 CM33->CM55 的输入槽；result_state 只描述 CM55->CM33 的结果槽。
     * 两个槽位互相独立，避免 CM55 推理耗时阻塞 CM33 发布下一帧输入。
     * producer_sequence 表示 CM33 最近一次成功发布的输入序号；
     * consumer_sequence 表示 CM55 最近一次成功取走的输入序号；
     * result_sequence 表示结果槽中当前这份结果对应的输入序号或 smoke 序号。
     */
    /* 协议魔数。
     * 用于快速判断共享区是否已经被正确初始化，避免 CM55 在随机内存内容上误判为有效协议区。
     */
    volatile uint32_t magic;
    /* 协议版本号。
     * 当共享内存布局、字段含义或结果解释方式发生不兼容变化时，必须递增该值，
     * 以便不同固件组件能够在运行时检测版本不匹配。
     */
    volatile uint32_t version;
    /* 输入槽状态，取值见 app_model_shared_input_state_t。
     * 只用于协调 CM33 发布输入与 CM55 消费输入之间的握手，不影响结果槽状态机。
     */
    volatile uint32_t input_state;
    /* 结果槽状态，取值见 app_model_shared_result_state_t。
     * 只用于协调 CM55 发布结果与 CM33/调试任务读取结果之间的可见性。
     */
    volatile uint32_t result_state;
    /* CM33 最近一次成功发布到共享区的输入序号。
     * 它反映“生产侧最新进度”，即使 CM55 还未消费，也能用于观察输入是否持续产生。
     */
    volatile uint32_t producer_sequence;
    /* CM55 最近一次成功取走并开始处理的输入序号。
     * 它反映“消费侧已跟进到哪一帧”，可用于判断 CM55 是否落后于 CM33 发布节奏。
     */
    volatile uint32_t consumer_sequence;
    /* 当前结果槽对应的结果序号。
     * 正常情况下它与 result.input_sequence 一致；在 smoke test 场景下，也可能落在专用的
     * smoke 序号区间，用于与正式 MIC 流程区分。
     */
    volatile uint32_t result_sequence;
    /* 输入特征描述符。
     * 为紧随其后的 audio_payload 提供尺寸、量化、时间戳、能量等解释信息。
     */
    app_model_audio_feature_desc_t audio;
    /* 推理结果描述符。
     * 保存 CM55 针对最近一次已完成推理写回的结果状态和分数。
     */
    app_model_inference_result_t result;

    /* 特征 payload 按 mel-major 展平：
     * audio_payload[mel * time_bin_count + time]。
     * 当前真实模型要求 float32，因此每 4 字节组成一个 float；保留 uint8_t 数组
     * 是为了协议区按字节计数和 memcpy 更直接。
     * CM33 在 input_state=WRITING 时写入该数组，CM55 在 READY->READING 后读取。
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

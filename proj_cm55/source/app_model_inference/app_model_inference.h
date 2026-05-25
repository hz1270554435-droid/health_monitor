/*******************************************************************************
* File Name : app_model_inference.h
*
* Description : CM55 模型推理任务框架。
*
* 本模块只负责 CM55 侧正式业务链路：
* 1. 轮询 shared/app_model_shared.h 定义的共享内存；
* 2. 读取 CM33 已完成前处理的 float32[40,101] 音频特征；
* 3. 调用真实 AUDIO_compute() 模型推理入口；
* 4. 把 logits、cough_prob 和调试信息写回共享内存结果区。
*
* smoke test 仍保留为可选 baseline，但第一版 MIC demo 默认直接进入正式推理链路。
*******************************************************************************/

#ifndef __APP_MODEL_INFERENCE_H__
#define __APP_MODEL_INFERENCE_H__

#include <stdbool.h>
#include <stdint.h>

#include "cy_pdl.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"

#include "app_model_shared.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* CM55 推理任务参数。
 * 这些只是框架默认值：正式模型导入后，如果模型栈、arena 或后处理变大，
 * 需要根据 map 文件和栈水位调整 APP_MODEL_INFERENCE_TASK_STACK_SIZE。
 */
#define APP_MODEL_INFERENCE_TASK_STACK_SIZE     (2048u)
#define APP_MODEL_INFERENCE_TASK_PRIORITY       (3u)
#define APP_MODEL_INFERENCE_POLL_DELAY_MS       (5u)

typedef struct
{
    /* 成功从共享内存取走的输入帧数。
     * 含义：CM55 已经完成对 input_state=READY 输入槽的接管，并把该帧作为一次有效输入开始处理。
     * 该计数可用于观察 CM55 是否持续跟上 CM33 发布特征的节奏。
     */
    uint32_t inputs_consumed;
    /* 共享区 magic/version 尚未初始化或版本不匹配的次数。
     * 该值持续增长通常说明共享区尚未由 CM33 正确初始化，或者 CM33/CM55 两侧编译时
     * 使用的协议头版本不一致，导致 CM55 拒绝继续读取输入。
     */
    uint32_t shared_not_ready;
    /* 输入描述符不合法的次数。
     * 例如 payload 大小、mel/time bin、量化类型、序号或其它元数据不满足当前模型 ABI 要求时，
     * CM55 会拒绝执行推理并累计该计数。
     */
    uint32_t invalid_inputs;
    /* 已经调用正式推理入口的次数。
     * 该值只统计真正进入 AUDIO_compute() 或等价正式模型入口的次数，可与 inputs_consumed 对比
     * 判断有多少输入在模型执行前就被判为无效。
     */
    uint32_t inference_runs;
    /* 最近一次消费的 CM33 特征序号。
     * 用于快速确认 CM55 当前已经处理到哪一帧输入，也方便和共享内存中的 producer_sequence
     * 做差值比较，评估是否存在处理滞后。
     */
    uint32_t last_input_sequence;
    /* 最近一次正式推理耗时，单位 ms。
     * 该值描述最近一次模型执行路径的耗时快照，可用于实时调试和性能回归观察。
     */
    uint32_t last_inference_time_ms;
    /* 最近一次推理状态，取 app_model_inference_status_t。
     * 该字段用于快速判断最近一次失败属于共享区未就绪、输入非法还是模型执行异常。
     * 查看统计时应优先结合该状态解释 last_input_sequence 和 last_inference_time_ms。
     */
    uint8_t last_status;
} app_model_inference_stats_t;

cy_rslt_t app_model_inference_task_init(void);
void app_model_inference_task(void *pvParameters);
void app_model_inference_get_stats(app_model_inference_stats_t *stats);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_MODEL_INFERENCE_H__ */

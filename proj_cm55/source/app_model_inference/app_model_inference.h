/*******************************************************************************
* File Name : app_model_inference.h
*
* Description : CM55 模型推理任务框架。
*
* 本模块只负责 CM55 侧正式业务链路：
* 1. 轮询 shared/app_model_shared.h 定义的共享内存；
* 2. 读取 CM33 已完成前处理和量化的音频特征；
* 3. 调用模型推理入口；
* 4. 把占位结果写回共享内存结果区。
*
* 当前项目还没有导入模型，因此推理入口先返回 MODEL_NOT_READY。后续接入模型时，
* 应只替换 app_model_inference.c 内部的占位函数，不需要改变 CM33 的采集和前处理任务。
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
    /* 成功从共享内存取走的输入帧数。 */
    uint32_t inputs_consumed;
    /* 共享区 magic/version 尚未初始化或版本不匹配的次数。 */
    uint32_t shared_not_ready;
    /* 输入描述符不合法的次数。 */
    uint32_t invalid_inputs;
    /* 已经调用占位/正式推理入口的次数。 */
    uint32_t inference_runs;
    /* 最近一次消费的 CM33 特征序号。 */
    uint32_t last_input_sequence;
    /* 最近一次占位/正式推理耗时，单位 ms。 */
    uint32_t last_inference_time_ms;
    /* 最近一次推理状态，取 app_model_inference_status_t。 */
    uint8_t last_status;
} app_model_inference_stats_t;

cy_rslt_t app_model_inference_task_init(void);
void app_model_inference_task(void *pvParameters);
void app_model_inference_get_stats(app_model_inference_stats_t *stats);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_MODEL_INFERENCE_H__ */

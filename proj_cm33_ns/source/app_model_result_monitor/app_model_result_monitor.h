/*******************************************************************************
* File Name : app_model_result_monitor.h
*
* Description : CM33 侧模型结果观察任务。
*
* 本模块只用于调试/联调：读取 CM55 写回 shared/app_model_shared.h 的结果区，
* 并通过 debug UART 打印简短摘要。它不消费 PDM 队列、不读取原始 PCM、不参与
* 音频前处理，因此可以和正式 app_audio_preprocess 任务并行。
*
* 默认 main.c 不启动该任务；需要观察 CM55 是否已经取走特征并写回结果时，再打开
* APP_MODEL_RESULT_MONITOR_ENABLE。
*******************************************************************************/

#ifndef __APP_MODEL_RESULT_MONITOR_H__
#define __APP_MODEL_RESULT_MONITOR_H__

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

/* 结果观察任务默认较低优先级，只做慢速调试打印，避免影响音频前处理。 */
#define APP_MODEL_RESULT_MONITOR_TASK_STACK_SIZE    (1024u)
#define APP_MODEL_RESULT_MONITOR_TASK_PRIORITY      (tskIDLE_PRIORITY + 1u)

/* 轮询间隔。当前结果区是“最新结果覆盖旧结果”语义，不保证调试任务看到每一帧。 */
#define APP_MODEL_RESULT_MONITOR_POLL_MS            (100u)

/* 是否在打印后把 result_state 清成 EMPTY。
 * 默认保留最新结果，方便调试器随时查看共享内存；如果未来 CM33 业务层需要
 * “只消费一次”的语义，可以把该宏改为 1。
 */
#ifndef APP_MODEL_RESULT_MONITOR_CLEAR_AFTER_READ
#define APP_MODEL_RESULT_MONITOR_CLEAR_AFTER_READ   (0u)
#endif

typedef struct
{
    /* 共享区 magic/version 尚未就绪或版本不匹配的次数。 */
    uint32_t shared_not_ready;
    /* 观察到的 READY 结果数。同一 result_sequence 只计一次。 */
    uint32_t results_seen;
    /* result_state 为 WRITING 时跳过读取的次数。 */
    uint32_t result_writing;
    /* 最近一次看到的 CM55 result_sequence。 */
    uint32_t last_result_sequence;
    /* 最近一次结果状态，取 app_model_inference_status_t。 */
    uint8_t last_status;
    /* 是否已经看到过至少一个结果。 */
    bool has_result;
} app_model_result_monitor_stats_t;

cy_rslt_t app_model_result_monitor_task_init(void);
void app_model_result_monitor_task(void *pvParameters);
void app_model_result_monitor_get_stats(
    app_model_result_monitor_stats_t *stats);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_MODEL_RESULT_MONITOR_H__ */

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

/* 结果观察任务优先级。
 *
 * 当前阶段需要观察 CM55 smoke test 连续写回的多条结果。共享 result 区是
 * “最新结果覆盖旧结果”的单槽语义，如果本任务优先级太低，正式音频前处理忙时
 * 可能只看到第一条 smoke，后续结果会在打印前被覆盖。
 *
 * 该任务只在 APP_MODEL_RESULT_MONITOR_ENABLE 打开时启动，属于联调工具；正式部署
 * 或实时性能测试时应关闭监控任务，避免 debug UART 打印影响音频链路。
 */
#define APP_MODEL_RESULT_MONITOR_TASK_STACK_SIZE    (1024u)
#define APP_MODEL_RESULT_MONITOR_TASK_PRIORITY      (configMAX_PRIORITIES - 1u)

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
    /* 共享区 magic/version 尚未就绪或版本不匹配的次数。
     * 用于统计结果观察任务因为共享协议未初始化或协议版本不一致而无法继续读取结果槽的情况。
     */
    uint32_t shared_not_ready;
    /* 最近一次看到共享区 ready 的时间戳，单位 ms。
     * 仅在结果观察任务看到新的 READY 结果副本并接受后更新，用于旁路 summary
     * bridge 估计“最近 live 观测距离当前多久”。
     */
    uint32_t last_result_time_ms;
    /* 观察到的 READY 结果数。同一 result_sequence 只计一次。
     * 该计数反映监控任务实际“看见过多少条新结果”，而不是 CM55 总共产生了多少次推理输出。
     */
    uint32_t results_seen;
    /* result_state 为 WRITING 时跳过读取的次数。
     * 该值升高说明监控任务较频繁撞上 CM55 正在写结果的窗口，属于正常并发保护现象。
     */
    uint32_t result_writing;
    /* 最近一次看到的 CM55 result_sequence。
     * 用于判断监控任务当前追踪到哪一条结果，也可用于去重避免重复打印同一结果。
     */
    uint32_t last_result_sequence;
    /* 最近一次看到的输入序号。
     * 表示最近被监控到的结果对应的是哪一帧输入特征，便于与 CM33 发布端序号对齐。
     */
    uint32_t last_input_sequence;
    /* OK 结果数量。
     * 统计 status=APP_MODEL_INFERENCE_STATUS_OK 的结果数，用于观察链路中成功推理的占比。
     */
    uint32_t ok_results;
    /* INVALID_INPUT 结果数量。
     * 统计 CM55 因输入描述符非法而拒绝推理的次数，有助于排查共享内存协议或前处理参数不一致问题。
     */
    uint32_t invalid_input_results;
    /* MODEL_NOT_READY 结果数量。
     * 主要出现在正式模型尚未接入或模型入口未准备好的联调阶段。
     */
    uint32_t model_not_ready_results;
    /* MODEL_ERROR 结果数量。
     * 表示模型运行时内部发生异常，而不是输入本身格式错误。
     */
    uint32_t model_error_results;
    /* 已打印的 cough 事件数量。
     * 这是监控任务最终输出到 debug UART 的事件数，不等于所有看到的 OK 结果数。
     */
    uint32_t events_printed;
    /* 被 event-level energy gate 抑制的候选事件数量。
     * 表示虽然结果满足一定候选条件，但因为能量或附加规则不达标而未打印/未上报。
     */
    uint32_t suppress_count_total;
    /* 最近一次日志 printf 耗时，单位 ms；profile 关闭时保持 0。
     * 用于评估串口打印对实时链路的扰动大小，帮助决定是否需要降低打印频率。
     */
    uint32_t last_log_ms;
    /* 最近一次结果状态，取 app_model_inference_status_t。
     * 是一份“最近结果快照”，查看监控统计时通常应优先参考该字段。
     */
    uint8_t last_status;
    /* 最近一次正式 live 推理的 cough 概率，来自 result.scores[2]。
     * smoke/fixed-vector 结果不会更新该字段，避免 display 把离线 smoke
     * 误当作真实业务状态。
     */
    float last_cough_prob;
    /* 当前统计窗口内观察到的最大 cough 概率。该值随 MODEL_STAT 周期重置。 */
    float max_cough_prob_1s;
    /* 当前统计窗口内按 demo threshold 判为 cough / non_cough 的帧数。
     * 这些是低频展示/诊断计数，不是正式 1min/5min cough summary。
     */
    uint32_t decision_count_cough_1s;
    uint32_t decision_count_non_cough_1s;
    /* 最近一次打印的 cough event id。仅用于只读展示/诊断。 */
    uint32_t last_event_id;
    /* 是否已经看到过至少一个结果。
     * 用于区分“当前结果状态为空”和“监控任务启动以来还从未看到过任何结果”。
     */
    bool has_result;
    /* 是否已经看到至少一个正式 live 结果。 */
    bool has_live_result;
} app_model_result_monitor_stats_t;

cy_rslt_t app_model_result_monitor_task_init(void);
void app_model_result_monitor_task(void *pvParameters);
void app_model_result_monitor_get_stats(
    app_model_result_monitor_stats_t *stats);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_MODEL_RESULT_MONITOR_H__ */

/******************************************************************************
* File Name : app_pdm_pcm.h
*
* Description : PDM PCM 采集模块接口。
*               本模块把 PDM 硬件转换出的左右声道 PCM 数据整理成固定 10 ms
*               数据块，并通过 FreeRTOS 队列把“块描述符”交给推理任务。
********************************************************************************
* Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#ifndef __APP_PDM_PCM_H__
#define __APP_PDM_PCM_H__


#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
* 头文件
*******************************************************************************/
#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cybsp.h"
#include "app_i2s.h"
#include "FreeRTOS.h"
#include "queue.h"
/*******************************************************************************
* 宏定义
*******************************************************************************/
/* 当前使用双声道采集，数据在内存中按 L,R,L,R... 交错排列。 */
#define NUM_CHANNELS                   (2u)
#define LEFT_CH_INDEX                  (2u)
#define RIGHT_CH_INDEX                 (3u)

/* 右声道中断作为本模块的统一触发源，左右声道在同一次 ISR 中一起读取。 */
#define PDM_IRQ                        CYBSP_PDM_CHANNEL_3_IRQ
#define LEFT_CH_CONFIG                 channel_2_config
#define RIGHT_CH_CONFIG                channel_3_config

/* PDM 硬件 FIFO 深度为 64，触发阈值取半 FIFO。
 * 每次 RX trigger 到来时，ISR 从左右声道各读 32 个采样点。
 */
#define PDM_HW_FIFO_SIZE               (64u)
#define RX_FIFO_TRIG_LEVEL             (PDM_HW_FIFO_SIZE/2)
#define PDM_HALF_FIFO_SIZE             (PDM_HW_FIFO_SIZE/2)

/* 录音缓存时长，单位：秒 */
#define RECORDING_DURATION_SEC         (4u)
/* 旧版线性录音缓存大小，保留给兼容代码使用 */
#define BUFFER_SIZE                    (RECORDING_DURATION_SEC * SAMPLE_RATE_HZ)

/* 录音开始阶段需要忽略的采样点数量 */
#define IGNORED_SAMPLES                (PDM_HW_FIFO_SIZE)

/* PDM PCM 中断优先级。FreeRTOS 中断安全 API 要求该优先级不能高于系统允许范围。 */
#define PDM_PCM_ISR_PRIORITY            (7u)

/* 面向流式推理的音频块配置：
 * 16 kHz 采样率下，10 ms 音频等于每声道 160 个采样点。
 * 双声道交错后，一个 block 内共有 320 个 int16_t。
 * APP_PDM_PCM_BLOCK_COUNT 是背压缓冲深度，推理任务短时变慢时可吸收抖动。 
 */
#define APP_PDM_PCM_BLOCK_MS                       (10u)
#define APP_PDM_PCM_BLOCK_COUNT                    (8u)
#define APP_PDM_PCM_SAMPLES_PER_CH_PER_BLOCK       ((SAMPLE_RATE_HZ * APP_PDM_PCM_BLOCK_MS) / 1000u)
#define APP_PDM_PCM_BLOCK_SAMPLES                  (NUM_CHANNELS * APP_PDM_PCM_SAMPLES_PER_CH_PER_BLOCK)
#define APP_PDM_PCM_TASK_STACK_SIZE                (1024u)
#define APP_PDM_PCM_TASK_PRIORITY                  (configMAX_PRIORITIES - 2u)

/* EVK 板载 PDM 麦克风增益范围 */
#define PDM_PCM_MIN_GAIN                        (-103.0)
#define PDM_PCM_MAX_GAIN                        (83.0)
#define PDM_MIC_GAIN_VALUE                      (20)

/* 增益到 PDM scale 的映射 */

#define PDM_PCM_SEL_GAIN_83DB                   (83.0)
#define PDM_PCM_SEL_GAIN_77DB                   (77.0)
#define PDM_PCM_SEL_GAIN_71DB                   (71.0)
#define PDM_PCM_SEL_GAIN_65DB                   (65.0)
#define PDM_PCM_SEL_GAIN_59DB                   (59.0)
#define PDM_PCM_SEL_GAIN_53DB                   (53.0)
#define PDM_PCM_SEL_GAIN_47DB                   (47.0)
#define PDM_PCM_SEL_GAIN_41DB                   (41.0)
#define PDM_PCM_SEL_GAIN_35DB                   (35.0)
#define PDM_PCM_SEL_GAIN_29DB                   (29.0)
#define PDM_PCM_SEL_GAIN_23DB                   (23.0)
#define PDM_PCM_SEL_GAIN_17DB                   (17.0)
#define PDM_PCM_SEL_GAIN_11DB                   (11.0)
#define PDM_PCM_SEL_GAIN_5DB                    (5.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_1DB           (-1.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_7DB           (-7.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_13DB          (-13.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_19DB          (-19.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_25DB          (-25.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_31DB          (-31.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_37DB          (-37.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_43DB          (-43.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_49DB          (-49.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_55DB          (-55.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_61DB          (-61.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_67DB          (-67.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_73DB          (-73.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_79DB          (-79.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_85DB          (-85.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_91DB          (-91.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_97DB          (-97.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_103DB         (-103.0)

/* 推理任务从队列中收到的是这个描述符，而不是一整块 PCM 数据副本。
 * 真正的 PCM 数据仍在 recorded_data 中；消费者处理完后必须释放 block_index。
 */
typedef struct
{
    /* 指向 recorded_data 中一个完整 10 ms 双声道 PCM 块，格式为 L,R,L,R...。 */
    const int16_t *data;

    /* 该块内 int16_t 总数，包含左右两个声道。当前为 320。 */
    uint16_t sample_count;

    /* 该块内每个声道的采样点数量。当前 16 kHz/10 ms 下为 160。 */
    uint16_t samples_per_channel;

    /* recorded_data 的行索引；推理任务处理完后用它归还该 block。 */
    uint8_t block_index;

    /* 单调递增的块序号，测试或推理侧可用它检查是否发生跳块。 */
    uint32_t sequence;

    /* 发布该块时的累计丢块计数快照，用于观察消费者是否跟不上采集。 */
    uint32_t dropped_count;
} app_pdm_pcm_block_t;

/* 共享内存中的 PCM 环形缓冲区。
 * 每一行是一个完整 10 ms 双声道块；数据所有权由 block 描述符和 release 接口管理。
 */
extern int16_t recorded_data[APP_PDM_PCM_BLOCK_COUNT][APP_PDM_PCM_BLOCK_SAMPLES];


/*******************************************************************************
* 函数声明
*******************************************************************************/
/* 创建 PDM PCM 采集任务和内部队列。应在 vTaskStartScheduler() 前调用。 */
cy_rslt_t app_pdm_pcm_task_init(void);

/* 初始化 PDM/PCM 硬件、声道、增益和中断。通常由 app_pdm_pcm_task() 内部调用。 */
void app_pdm_pcm_init(void);

/* 激活左右声道，开始由硬件中断驱动采集。 */
void app_pdm_pcm_activate(void);

/* 将 dB 增益值映射为 PDM 驱动支持的离散 gain scale。 */
cy_en_pdm_pcm_gain_sel_t convert_db_to_pdm_scale(double db);

/* 同时设置左右声道的 PDM 增益。 */
void set_pdm_pcm_gain(cy_en_pdm_pcm_gain_sel_t gain);

/* PDM RX 中断处理函数，由 NVIC 调用，应用层不要直接调用。 */
void pdm_interrupt_handler(void);

/* 停止左右声道采集。 */
void app_pdm_pcm_deactivate(void);

/* PDM PCM 采集任务入口，负责启动硬件，之后采集由 ISR 驱动。 */
void app_pdm_pcm_task(void *pvParameters);

/* 获取内部 block 队列句柄，主要用于调试或兼容旧代码。优先使用 receive/release 接口。 */
QueueHandle_t app_pdm_pcm_get_queue(void);

/* 接收一个已准备好的 10 ms PCM 块；使用完成后调用者必须释放 block_index。 */
bool app_pdm_pcm_receive_block(app_pdm_pcm_block_t *block, TickType_t ticks_to_wait);

/* 将已经消费完成的 PCM 块归还给 ISR 可写的空闲池。 */
void app_pdm_pcm_release_block(uint8_t block_index);

/* 因消费者释放不及时而被丢弃的音频块数量。 */
uint32_t app_pdm_pcm_get_dropped_count(void);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __APP_PDM_PCM_H__ */
/* [] END OF FILE */

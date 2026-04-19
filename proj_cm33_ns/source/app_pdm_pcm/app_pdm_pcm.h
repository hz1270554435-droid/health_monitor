/******************************************************************************
* File Name : app_pdm_pcm.h
*
* Description : Header file for PDM PCM containing function ptototypes.
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
* Macros
*******************************************************************************/
/* 声道数量 */
#define NUM_CHANNELS                   (2u)
#define LEFT_CH_INDEX                  (2u)
#define RIGHT_CH_INDEX                 (3u)
#define PDM_IRQ                        CYBSP_PDM_CHANNEL_3_IRQ
#define LEFT_CH_CONFIG                 channel_2_config
#define RIGHT_CH_CONFIG                channel_3_config

#define PDM_HW_FIFO_SIZE               (64u)
#define RX_FIFO_TRIG_LEVEL             (PDM_HW_FIFO_SIZE/2)
/* PDM Half FIFO Size */
#define PDM_HALF_FIFO_SIZE             (PDM_HW_FIFO_SIZE/2)

/* 录音缓存时长，单位：秒 */
#define RECORDING_DURATION_SEC         (4u)
/* 旧版线性录音缓存大小，保留给兼容代码使用 */
#define BUFFER_SIZE                    (RECORDING_DURATION_SEC * SAMPLE_RATE_HZ)

/* 录音开始阶段需要忽略的采样点数量 */
#define IGNORED_SAMPLES                (PDM_HW_FIFO_SIZE)

/* PDM PCM interrupt priority */
#define PDM_PCM_ISR_PRIORITY            (7u)

/* 面向流式推理的音频块配置。
 * 16 kHz 采样率下，10 ms 音频等于每声道 160 个采样点。
 * PCM 数据按双声道交错格式保存：L0, R0, L1, R1, ...
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

/*******************************************************************************
* 全局变量
*******************************************************************************/
typedef struct
{
    /* 指向 recorded_data 中一个完整 10 ms 双声道音频块。 */
    const int16_t *data;
    /* 该块内 int16_t 总数，包含左右两个声道。 */
    uint16_t sample_count;
    /* 该块内每个声道的采样点数量。 */
    uint16_t samples_per_channel;
    /* 环形缓冲区块索引；推理任务处理完后需要释放这个索引。 */
    uint8_t block_index;
    /* 单调递增的块序号，用于检测是否发生跳块。 */
    uint32_t sequence;
    /* 发布该块时的丢块计数快照。 */
    uint32_t dropped_count;
} app_pdm_pcm_block_t;

/* 共享内存中的环形缓冲区；每一行是一个完整 10 ms 双声道 PCM 块。 */
extern int16_t recorded_data[APP_PDM_PCM_BLOCK_COUNT][APP_PDM_PCM_BLOCK_SAMPLES];


/*******************************************************************************
* Functions Prototypes
*******************************************************************************/
cy_rslt_t app_pdm_pcm_task_init(void);
void app_pdm_pcm_init(void);
void app_pdm_pcm_activate(void);
cy_en_pdm_pcm_gain_sel_t convert_db_to_pdm_scale(double db);
void set_pdm_pcm_gain(cy_en_pdm_pcm_gain_sel_t gain);
void pdm_interrupt_handler(void);
void app_pdm_pcm_deactivate(void);
void app_pdm_pcm_task(void *pvParameters);
QueueHandle_t app_pdm_pcm_get_queue(void);
/* 接收一个已准备好的 10 ms 音频块；使用完成后调用者必须释放 block_index。 */
bool app_pdm_pcm_receive_block(app_pdm_pcm_block_t *block, TickType_t ticks_to_wait);
/* 将已经消费完成的音频块归还给 ISR 可写的空闲池。 */
void app_pdm_pcm_release_block(uint8_t block_index);
/* 因消费者释放不及时而被丢弃的音频块数量。 */
uint32_t app_pdm_pcm_get_dropped_count(void);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __APP_PDM_PCM_H__ */
/* [] END OF FILE */

/*******************************************************************************
* File Name        : main.c
*
* Description      : This source file contains the main routine for non-secure
*                    application in the CM33 CPU
*
* Related Document : See README.md
*
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
* obtained this Software ("EULA"). If no EULA applies, Cypress hereby grants
* you a personal, non-exclusive, non-transferable license to copy, modify, and
* compile the Software source code solely for use in connection with Cypress's
* integrated circuit products. Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified above is
* prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of this
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "app_pdm_pcm.h"
#include "app_get_data.h"
#include "app_audio_preprocess.h"
#include "app_model_result_monitor.h"
#include "app_uart_radar.h"
#include "app_csv_export.h"
#include "app_ble_config.h"
#if (APP_BLE_ENABLE)
#include "app_ble_stream.h"
#endif
#include "retarget_io_init.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

/* The timeout value in microsecond used to wait for core to be booted. */
#define CM55_BOOT_WAIT_TIME_USEC          (10u)
/* App boot address for CM55 project. */
#define CM55_APP_BOOT_ADDR                (CYMEM_CM33_0_m55_nvm_START + \
                                           CYBSP_MCUBOOT_HEADER_SIZE)

/* 应用运行模式。
 *
 * 注意：app_pdm_pcm 内部只有一个 PDM block 队列。CSV 导出、MIC 自检和正式
 * 音频前处理都属于这个队列的“消费者”，同一时间只能启用其中一个，否则同一个
 * PCM block 会被其中一个任务抢走，另一个任务就会看到不连续数据。
 *
 * 默认使用正式业务链路：CM33 做音频输入前处理，把处理好的 float32 特征写入
 * m33_m55_shared，后续由 CM55 推理任务读取。需要采集训练数据时，把
 * APP_RUNTIME_MODE 改成 APP_RUNTIME_MODE_CSV_EXPORT；需要只看 MIC 数据质量时，
 * 改成 APP_RUNTIME_MODE_MIC_SELF_TEST。
 */
#define APP_RUNTIME_MODE_AUDIO_PREPROCESS (0u)
#define APP_RUNTIME_MODE_CSV_EXPORT       (1u)
#define APP_RUNTIME_MODE_MIC_SELF_TEST    (2u)

#ifndef APP_RUNTIME_MODE
#define APP_RUNTIME_MODE                  APP_RUNTIME_MODE_AUDIO_PREPROCESS
#endif

/* Debug UART 波特率选择接口。
 *
 * 这是应用层唯一需要关心的串口速率入口；retarget_io_init.c 只负责把这里选定
 * 的速率转换成硬件 divider，不再根据业务场景自行判断。
 *
 * 默认规则：
 * - CSV_EXPORT：用于训练/采集数据导出，数据量大，默认使用 2 Mbps；
 * - AUDIO_PREPROCESS/MIC_SELF_TEST：用于正式链路、模型 smoke test 或普通调试日志，
 *   默认使用串口助手常用的 115200。
 *
 * 临时强制指定速率时，可以在这里定义 APP_DEBUG_UART_BAUD_RATE，或通过编译宏覆盖。
 * 只允许使用 retarget_io_init.h 中声明的 RETARGET_IO_BAUD_115200 /
 * RETARGET_IO_BAUD_2000000，避免传入底层尚未校准的任意波特率。
 */
#ifndef APP_DEBUG_UART_BAUD_RATE
#if (APP_RUNTIME_MODE == APP_RUNTIME_MODE_CSV_EXPORT)
#define APP_DEBUG_UART_BAUD_RATE          RETARGET_IO_BAUD_2000000
#else
#define APP_DEBUG_UART_BAUD_RATE          RETARGET_IO_BAUD_115200
#endif
#endif

/* CM55 结果观察开关。
 * 0：正式部署时可关闭，不打印 CM55 结果，避免 debug UART 对实时链路产生干扰；
 * 1：在正式音频前处理模式下额外启动结果观察任务，只读共享内存 result 区。
 * 该任务不消费 PDM 队列，因此不会影响 app_audio_preprocess。
 * 当前为了上板 baseline 烟雾测试默认打开；模型链路验证通过后可改回 0。
 */
#ifndef APP_MODEL_RESULT_MONITOR_ENABLE
#define APP_MODEL_RESULT_MONITOR_ENABLE   (1u)
#endif

/* BLE bring-up 调试开关。
 * 0：正常运行现有音频前处理 + CM55 推理链路；
 * 1：临时暂停 AUDIO_PREPROCESS 模式下的 PDM、音频前处理、结果监控和 CM55 boot，
 *    让串口和 CPU 资源优先留给 BLE stack/advertising 诊断。
 */
#ifndef APP_BLE_DEBUG_DISABLE_INFERENCE
#define APP_BLE_DEBUG_DISABLE_INFERENCE   (0u)
#endif

#if ((APP_RUNTIME_MODE != APP_RUNTIME_MODE_AUDIO_PREPROCESS) && \
     (APP_RUNTIME_MODE != APP_RUNTIME_MODE_CSV_EXPORT) && \
     (APP_RUNTIME_MODE != APP_RUNTIME_MODE_MIC_SELF_TEST))
#error "Unsupported APP_RUNTIME_MODE"
#endif

#if ((APP_MODEL_RESULT_MONITOR_ENABLE != 0u) && \
     (APP_MODEL_RESULT_MONITOR_ENABLE != 1u))
#error "Unsupported APP_MODEL_RESULT_MONITOR_ENABLE"
#endif

#if ((APP_BLE_DEBUG_DISABLE_INFERENCE != 0u) && \
     (APP_BLE_DEBUG_DISABLE_INFERENCE != 1u))
#error "Unsupported APP_BLE_DEBUG_DISABLE_INFERENCE"
#endif

/* 编译期限制 Debug UART 只使用已经计算并验证过 divider 的速率。 */
#if ((APP_DEBUG_UART_BAUD_RATE != RETARGET_IO_BAUD_115200) && \
     (APP_DEBUG_UART_BAUD_RATE != RETARGET_IO_BAUD_2000000))
#error "Unsupported APP_DEBUG_UART_BAUD_RATE"
#endif

int main(void)
{
    cy_rslt_t result;

    result = cybsp_init();
    handle_app_error(result);

    __enable_irq();

    /* 初始化 debug UART 重定向。
     * 这里传入 main.c 选出的 APP_DEBUG_UART_BAUD_RATE，使采集模式和普通调试
     * 模式可以共用同一套 printf/retarget-io 初始化代码。
     */
    init_retarget_io(APP_DEBUG_UART_BAUD_RATE);
    printf("[BOOT] CM33 alive, mode=%lu, uart_baud=%lu, shared_ver=%lu\r\n",
           (unsigned long)APP_RUNTIME_MODE,
           (unsigned long)APP_DEBUG_UART_BAUD_RATE,
           (unsigned long)APP_MODEL_SHARED_VERSION);
    fflush(stdout);

    /* 创建 PDM/PCM 采集任务。
     * 该任务只负责启动硬件并由 ISR 持续产出 10 ms 双通道 PCM block；
     * 后续由下面按运行模式选择的唯一消费者取走这些 block。
     */
#if ((APP_RUNTIME_MODE != APP_RUNTIME_MODE_CSV_EXPORT) || \
     (APP_CSV_EXPORT_MIC_CAPTURE_ENABLE))
#if !((APP_BLE_DEBUG_DISABLE_INFERENCE) && \
      (APP_RUNTIME_MODE == APP_RUNTIME_MODE_AUDIO_PREPROCESS))
    result = app_pdm_pcm_task_init();
    handle_app_error(result);
    printf("[BOOT] PDM PCM task created\r\n");
    fflush(stdout);
#else
    printf("[BOOT] BLE debug disabled PDM/inference path\r\n");
    fflush(stdout);
#endif
#endif

#if (((APP_RUNTIME_MODE == APP_RUNTIME_MODE_CSV_EXPORT) && \
      (APP_CSV_EXPORT_RADAR_CAPTURE_ENABLE)) || \
     (APP_RUNTIME_MODE == APP_RUNTIME_MODE_MIC_SELF_TEST))
    /* 测试/导出模式下启动雷达接收任务。
     * CSV 导出按 APP_CSV_EXPORT_CAPTURE_MODE 决定是否消费雷达队列；
     * MIC 自检模式下也保留雷达自检入口，
     * 方便单板联调两个传感器。正式音频前处理模式暂不启动雷达任务，避免无人
     * 消费雷达队列时产生无意义背压。
     */
    result = app_uart_radar_task_init();
    handle_app_error(result);
#endif

#if ((APP_RUNTIME_MODE == APP_RUNTIME_MODE_AUDIO_PREPROCESS) && \
     (!APP_BLE_DEBUG_DISABLE_INFERENCE))
    /* 正式业务模式：只创建一个音频输入前处理任务作为 PDM 队列消费者。
     * 该任务完成双通道转单声道、滑窗、log-mel 和归一化，然后把 float32 特征
     * 写入 CM33/CM55 共享内存。不要在该模式下再启动 app_get_data 或
     * app_csv_export，否则会抢同一个 PDM block 队列。
     */
    printf("[BOOT] audio preprocess task init begin\r\n");
    fflush(stdout);
    result = app_audio_preprocess_task_init();
    if (CY_RSLT_SUCCESS != result)
    {
        /* 这里先打印错误码再进入统一错误处理，避免任务创建失败时串口只停在
         * 上一条 BOOT 日志，无法判断是参数校验失败、heap 不足还是其它错误。
         */
        printf("[BOOT] audio preprocess task init failed, result=0x%08lx\r\n",
               (unsigned long)result);
        fflush(stdout);
    }
    handle_app_error(result);
    printf("[BOOT] audio preprocess task created\r\n");
    fflush(stdout);

#if (APP_MODEL_RESULT_MONITOR_ENABLE)
    /* 可选调试任务：观察 CM55 写回的 result 区。
     * 它不读取 audio_payload，不消费 PDM block，只适合联调阶段确认 CM55 是否已
     * 消费特征并运行到模型入口。默认关闭。
     */
    result = app_model_result_monitor_task_init();
    handle_app_error(result);
    printf("[BOOT] model result monitor task created\r\n");
    fflush(stdout);
#endif
#elif (APP_RUNTIME_MODE == APP_RUNTIME_MODE_CSV_EXPORT)
    /* 训练/采集数据模式：CSV 导出任务按 APP_CSV_EXPORT_CAPTURE_MODE 消费 MIC
     * 和/或雷达队列，并通过 debug UART 输出带 device 标签的数据流。该模式用于
     * PC 端采集，不运行正式前处理任务。
     */
    result = app_csv_export_task_init();
    handle_app_error(result);
#elif (APP_RUNTIME_MODE == APP_RUNTIME_MODE_MIC_SELF_TEST)
#if (APP_UART_RADAR_TEST_ENABLE)
    /* 雷达自检任务只消费雷达队列，并通过 debug UART 打印摘要。
     * 它不会消费 PDM 队列，因此可以和下面的 MIC 自检任务并行。
     */
    result = app_uart_radar_test_task_init();
    handle_app_error(result);
#endif

    /* MIC 自检任务只消费 PDM 队列，用于观察 block 连续性、幅值、均值等采集质量。
     * 它是测试任务，不属于正式业务前处理链路。
     */
    result = app_get_data_test_task_init();
    handle_app_error(result);
#endif

#if (APP_BLE_ENABLE)
    result = app_ble_stream_init();
    if (CY_RSLT_SUCCESS != result)
    {
        printf("[BOOT] BLE Stage 1 init failed, result=0x%08lx\r\n",
               (unsigned long)result);
        fflush(stdout);
    }
    handle_app_error(result);
    printf("[BOOT] BLE Stage 1 task created, fake=%u, stack=%u\r\n",
           (unsigned int)APP_BLE_FAKE_DATA_ENABLE,
           (unsigned int)APP_BLE_STACK_ENABLE);
    fflush(stdout);
#endif

#if (APP_BLE_DEBUG_DISABLE_INFERENCE)
    printf("[BOOT] BLE debug inference chain disabled, CM55 boot skipped\r\n");
    fflush(stdout);
#else
    /* 启动 CM55。
     * CM55 侧模型推理任务会读取 app_audio_preprocess 写入的共享内存特征。
     * 测试/导出模式下 CM33 不发布正式特征，CM55 任务会保持轮询等待。
     */
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);
    printf("[BOOT] CM55 boot requested, addr=0x%08lx\r\n",
           (unsigned long)CM55_APP_BOOT_ADDR);
    fflush(stdout);
#endif

    vTaskStartScheduler();
    configASSERT(0);

    for (;;)
    {
    }
}

/* [] END OF FILE */

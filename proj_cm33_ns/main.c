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
#include "app_audio_replay.h"
#include "app_get_data.h"
#include "app_audio_preprocess.h"
#include "app_model_result_monitor.h"
#include "app_model_ipc_smoke.h"
#include "app_uart_radar.h"
#include "app_radar_bridge.h"
#include "app_csv_export.h"
#include "app_ble_config.h"
#include "app_board_time.h"
#include "app_build_config.h"
#include "app_monitor_summary.h"
#if (APP_BLE_ENABLE)
#include "app_ble_stream.h"
#endif
#if (APP_DISPLAY_ENABLE)
#include "app_display.h"
#endif
#include "retarget_io_init.h"
#include "app_display_diag.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

/* The timeout value in microsecond used to wait for core to be booted. */
#define CM55_BOOT_WAIT_TIME_USEC          (10u)
/* App boot address for CM55 project. */
#define CM55_APP_BOOT_ADDR                (CYMEM_CM33_0_m55_nvm_START + \
                                           CYBSP_MCUBOOT_HEADER_SIZE)

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

#ifndef APP_UART_SANITY_OUTPUT_METHOD
#define APP_UART_SANITY_OUTPUT_METHOD     (1u)
#endif

#ifndef APP_UART_SANITY_LINE_DELAY_MS
#define APP_UART_SANITY_LINE_DELAY_MS     (10u)
#endif

#define APP_UART_SANITY_METHOD_PRINTF     (0u)
#define APP_UART_SANITY_METHOD_BLOCKING   (1u)
#define APP_UART_SANITY_LINE_COUNT        (100u)

/* 编译期限制 Debug UART 只使用已经计算并验证过 divider 的速率。 */
#if ((APP_DEBUG_UART_BAUD_RATE != RETARGET_IO_BAUD_115200) && \
     (APP_DEBUG_UART_BAUD_RATE != RETARGET_IO_BAUD_230400) && \
     (APP_DEBUG_UART_BAUD_RATE != RETARGET_IO_BAUD_460800) && \
     (APP_DEBUG_UART_BAUD_RATE != RETARGET_IO_BAUD_921600) && \
     (APP_DEBUG_UART_BAUD_RATE != RETARGET_IO_BAUD_2000000))
#error "Unsupported APP_DEBUG_UART_BAUD_RATE"
#endif

#if ((APP_UART_SANITY_TEST_ENABLE != 0u) && \
     (APP_UART_SANITY_TEST_ENABLE != 1u))
#error "Unsupported APP_UART_SANITY_TEST_ENABLE"
#endif

#if ((APP_UART_SANITY_OUTPUT_METHOD != APP_UART_SANITY_METHOD_PRINTF) && \
     (APP_UART_SANITY_OUTPUT_METHOD != APP_UART_SANITY_METHOD_BLOCKING))
#error "Unsupported APP_UART_SANITY_OUTPUT_METHOD"
#endif

#if (APP_UART_SANITY_TEST_ENABLE)
static void app_uart_sanity_run(void)
{
    char line[32];
    uint32_t sequence;

    for (sequence = 0u; sequence < APP_UART_SANITY_LINE_COUNT; ++sequence)
    {
        int length = snprintf(line,
                              sizeof(line),
                              "[UART_SANITY] seq=%03lu\r\n",
                              (unsigned long)sequence);

        if ((length <= 0) || ((size_t)length >= sizeof(line)))
        {
            handle_app_error(CY_RSLT_TYPE_ERROR);
        }

#if (APP_UART_SANITY_OUTPUT_METHOD == APP_UART_SANITY_METHOD_BLOCKING)
        retarget_io_write_blocking((const uint8_t *)line, (uint32_t)length);
#else
        printf("%s", line);
        fflush(stdout);
#endif

        Cy_SysLib_Delay(APP_UART_SANITY_LINE_DELAY_MS);
    }
}
#endif

int main(void)
{
    cy_rslt_t result;

    result = cybsp_init();
    handle_app_error(result);

    __enable_irq();

#if ((APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE) && \
     (!APP_CM55_INFERENCE_ENABLE))
    /* Display-only proof mode mirrors the official LVGL demo launcher:
     * CM33_NS releases CM55 and stays out of UART, shared memory, audio, radar,
     * BLE, and local display paths. Optional launcher UART is diagnostic-only.
     */
#if (APP_DISPLAY_CM33_LAUNCHER_UART_ENABLE)
    init_retarget_io(APP_DEBUG_UART_BAUD_RATE);
    printf("[CM33_DISPLAY_ONLY] launcher_start baud=%lu cm55_addr=0x%08lx\r\n",
           (unsigned long)APP_DEBUG_UART_BAUD_RATE,
           (unsigned long)CM55_APP_BOOT_ADDR);
    fflush(stdout);
#endif

#if (APP_DISPLAY_DIAG_ENABLE)
    app_display_diag_reset();
    app_display_diag_mark(APP_DISPLAY_DIAG_STAGE_CM33_LAUNCHER_START,
                          0u,
                          (uint32_t)__LINE__,
                          0u);
#if (APP_DISPLAY_CM33_DIAG_POLL_ENABLE)
    init_retarget_io(APP_DEBUG_UART_BAUD_RATE);
    printf("[DISPLAY_DIAG] cm33_launcher_start addr=0x%08lx\r\n",
           (unsigned long)CM55_APP_BOOT_ADDR);
    fflush(stdout);
#endif
#endif

    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);

#if (APP_DISPLAY_DIAG_ENABLE)
    app_display_diag_mark(APP_DISPLAY_DIAG_STAGE_CM33_CM55_RELEASED,
                          0u,
                          (uint32_t)__LINE__,
                          0u);
#endif

#if (APP_DISPLAY_CM33_LAUNCHER_UART_ENABLE)
    printf("[CM33_DISPLAY_ONLY] cm55_released\r\n");
    fflush(stdout);
#endif

#if ((APP_DISPLAY_DIAG_ENABLE) && (APP_DISPLAY_CM33_DIAG_POLL_ENABLE))
    for (;;)
    {
        const volatile app_display_diag_region_t *diag = APP_DISPLAY_DIAG_REGION;

        Cy_SysLib_Delay(500u);
        printf("[DISPLAY_DIAG] magic=0x%08lx ver=%lu stage=%lu(%s) "
               "result=0x%08lx line=%lu heartbeat=%lu flags=0x%08lx "
               "detail0=0x%08lx detail1=0x%08lx "
               "detail2=0x%08lx detail3=0x%08lx\r\n",
               (unsigned long)diag->magic,
               (unsigned long)diag->version,
               (unsigned long)diag->stage,
               app_display_diag_stage_name((uint32_t)diag->stage),
               (unsigned long)diag->result,
               (unsigned long)diag->line,
               (unsigned long)diag->heartbeat,
               (unsigned long)diag->flags,
               (unsigned long)diag->detail0,
               (unsigned long)diag->detail1,
               (unsigned long)diag->detail2,
               (unsigned long)diag->detail3);
        fflush(stdout);
    }
#endif

#if (APP_DISPLAY_CM33_LAUNCHER_HEARTBEAT_ENABLE)
    for (;;)
    {
        Cy_SysLib_Delay(5000u);
        printf("[CM33_DISPLAY_ONLY] heartbeat cm55_released=1\r\n");
        fflush(stdout);
    }
#else
    for (;;)
    {
        Cy_SysLib_Delay(1000u);
    }
#endif
#endif

    /* 初始化 debug UART 重定向。
     * 这里传入 main.c 选出的 APP_DEBUG_UART_BAUD_RATE，使采集模式和普通调试
     * 模式可以共用同一套 printf/retarget-io 初始化代码。
     */
    init_retarget_io(APP_DEBUG_UART_BAUD_RATE);

#if (APP_UART_SANITY_TEST_ENABLE)
    app_uart_sanity_run();
    for (;;)
    {
        Cy_SysLib_Delay(1000u);
    }
#endif

    app_model_ipc_smoke_log_init();
    result = app_board_time_init();
    if (CY_RSLT_SUCCESS != result)
    {
        printf("[BOOT] board time init failed, result=0x%08lx\r\n",
               (unsigned long)result);
        fflush(stdout);
    }
    handle_app_error(result);

    printf("[BOOT] CM33 alive, mode=%lu, uart_baud=%lu, shared_ver=%lu\r\n",
           (unsigned long)APP_RUNTIME_MODE,
           (unsigned long)APP_DEBUG_UART_BAUD_RATE,
           (unsigned long)APP_MODEL_SHARED_VERSION);
    fflush(stdout);

    /* CM33_NS is the sole app-model shared boot/reset owner. This must finish
     * before any task is created and before CM55 is released.
     */
    result = app_model_shared_boot_init_cm33_owner();
    if (CY_RSLT_SUCCESS != result)
    {
        printf("[BOOT] app-model shared boot owner failed, result=0x%08lx\r\n",
               (unsigned long)result);
        fflush(stdout);
    }
    handle_app_error(result);

#if (APP_MONITOR_SUMMARY_ENABLE)
    result = app_monitor_summary_init();
    if (CY_RSLT_SUCCESS != result)
    {
        printf("[BOOT] monitor summary init failed, result=0x%08lx\r\n",
               (unsigned long)result);
        fflush(stdout);
    }
    handle_app_error(result);
    printf("[BOOT] monitor summary owner ready, mock=%lu, ring=%lu\r\n",
           (unsigned long)APP_MONITOR_SUMMARY_MOCK_ENABLE,
           (unsigned long)APP_MONITOR_SUMMARY_EVENT_RING_SIZE);
    fflush(stdout);
#endif

#if (APP_DISPLAY_ENABLE)
    result = app_display_init();
    if (CY_RSLT_SUCCESS != result)
    {
        printf("[BOOT] display init failed, result=0x%08lx\r\n",
               (unsigned long)result);
        fflush(stdout);
    }
    handle_app_error(result);

    result = app_display_start();
    if (CY_RSLT_SUCCESS != result)
    {
        printf("[BOOT] display start failed, result=0x%08lx\r\n",
               (unsigned long)result);
        fflush(stdout);
    }
    handle_app_error(result);
    printf("[BOOT] display backend=%s lcd=%lu\r\n",
#if (APP_DISPLAY_LCD_ENABLE)
           "lcd",
#else
           "null",
#endif
           (unsigned long)APP_DISPLAY_LCD_ENABLE);
    fflush(stdout);
#endif

    /* 创建 PDM/PCM 采集任务。
     * 该任务只负责启动硬件并由 ISR 持续产出 10 ms 双通道 PCM block；
     * 后续由下面按运行模式选择的唯一消费者取走这些 block。
     */
#if ((APP_RUNTIME_MODE != APP_RUNTIME_MODE_CSV_EXPORT) || \
     (APP_CSV_EXPORT_MIC_CAPTURE_ENABLE))
#if !((APP_BLE_DEBUG_DISABLE_INFERENCE) && \
      (APP_RUNTIME_MODE == APP_RUNTIME_MODE_AUDIO_PREPROCESS))
#if (APP_AUDIO_REPLAY_TEST_ENABLE)
    result = app_audio_replay_init();
    handle_app_error(result);
    printf("[BOOT] replay provider init complete\r\n");
    fflush(stdout);
#else
    result = app_pdm_pcm_task_init();
    handle_app_error(result);
    printf("[BOOT] PDM PCM task created\r\n");
    fflush(stdout);
#endif
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

#if (APP_RADAR_BRIDGE_ENABLE)
    /* Real radar bridge profile: start radar parser and bridge task.
     * Bridge is the sole consumer of the radar queue.
     * Test decoder MUST be disabled (APP_UART_RADAR_TEST_ENABLE=0).
     */
    result = app_uart_radar_task_init();
    handle_app_error(result);
    printf("[BOOT] radar parser task created (bridge profile)\r\n");
    fflush(stdout);

    result = app_radar_bridge_init();
    handle_app_error(result);
    printf("[BOOT] radar bridge task created\r\n");
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

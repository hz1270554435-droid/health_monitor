/*
 * FreeRTOS Kernel V10.6.2
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * (c) 2019-2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 *
 * CM55 本地 FreeRTOS 配置。
 *
 * 当前 CM55 只运行模型推理任务框架：从 m33_m55_shared 读取 CM33 已经完成
 * 前处理和量化的音频特征。正式模型导入后，如果需要 MVE/更大堆/更高优先级，
 * 优先从这里调整 RTOS 参数。
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#if defined (__ICCARM__) || (__GNUC__)
#include "cy_utils.h"
#include "cycfg_system.h"
#include "cy_device_headers.h"
#endif

#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#if defined (__ICCARM__) || (__GNUC__)
extern uint32_t SystemCoreClock;
#endif
#define configCPU_CLOCK_HZ                      SystemCoreClock
#define configTICK_RATE_HZ                      ((TickType_t ) 1000)
#define configMAX_PRIORITIES                    7
#define configMINIMAL_STACK_SIZE                256
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               10
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  1
#define configENABLE_BACKWARD_COMPATIBILITY     0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5

/* 当前占位推理不使用 Helium/MVE。后续模型若启用 MVE 优化，需要把这里改为 1。 */
#define configENABLE_MVE                        0

/* CM55 带 FPU；如果工程切到 softfloat，构建系统会定义 MTB_SOFTFLOAT。 */
#if defined(MTB_SOFTFLOAT)
#define configENABLE_FPU                        0
#else
#define configENABLE_FPU                        1
#endif
#define configENABLE_MPU                        0
#define configENABLE_TRUSTZONE                  0
#define configRUN_FREERTOS_SECURE_ONLY          0

/* 当前 CM55 任务都用 xTaskCreate 动态创建，先关闭 static allocation，避免必须实现
 * vApplicationGetIdleTaskMemory/vApplicationGetTimerTaskMemory 钩子。
 */
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ((size_t )(32 * 1024))
#define configAPPLICATION_ALLOCATED_HEAP        0

#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         1

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               3
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )

/* CM55 FreeRTOS 端口要求该值不能为 0。 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    0x20
#define configMAX_API_CALL_INTERRUPT_PRIORITY   configMAX_SYSCALL_INTERRUPT_PRIORITY

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xResumeFromISR                  1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     0
#define INCLUDE_xTaskGetIdleTaskHandle          0
#define INCLUDE_eTaskGetState                   0
#define INCLUDE_xEventGroupSetBitFromISR        1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 0
#define INCLUDE_xTaskGetHandle                  0
#define INCLUDE_xTaskResumeFromISR              1

#if defined(NDEBUG)
#define configASSERT( x ) CY_UNUSED_PARAMETER( x )
#else
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); CY_HALT(); }
#endif

#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#define HEAP_ALLOCATION_TYPE1                   (1)
#define HEAP_ALLOCATION_TYPE2                   (2)
#define HEAP_ALLOCATION_TYPE3                   (3)
#define HEAP_ALLOCATION_TYPE4                   (4)
#define HEAP_ALLOCATION_TYPE5                   (5)
#define NO_HEAP_ALLOCATION                      (0)

/* 使用 heap_3，和 CM33 工程保持一致，由 C library malloc/free 承担动态内存。 */
#define configHEAP_ALLOCATION_SCHEME            (HEAP_ALLOCATION_TYPE3)

/* CM55 当前只轮询共享内存。先关闭 tickless，减少低功耗钩子的链接依赖。 */
#define configUSE_TICKLESS_IDLE                 0

#if defined(__llvm__) && !defined(__ARMCC_VERSION)
#define configUSE_PICOLIBC_TLS                  1
#else
#define configUSE_NEWLIB_REENTRANT              1
#endif

#endif /* FREERTOS_CONFIG_H */

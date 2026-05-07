/*******************************************************************************
* File Name : app_model_smoke.h
*
* Description : CM55 模型部署烟雾测试。
*
* 本模块只用于验证“PC 已生成 Log-Mel 特征 -> 板端 AUDIO_compute() -> 输出
* 与 PC expected_output.json 对齐”这条最短模型链路。它不依赖 CM33 音频前处理，
* 也不读取实时 MIC 数据。
*******************************************************************************/

#ifndef __APP_MODEL_SMOKE_H__
#define __APP_MODEL_SMOKE_H__

#include <stdbool.h>
#include <stdint.h>

#include "cy_pdl.h"
#include "cybsp.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* 1：CM55 启动后先运行 PC Log-Mel 测试向量。
 *
 * baseline 已经在板端验证通过；第一版 MIC demo 默认关闭 smoke，让 CM55 启动后
 * 直接等待 CM33 前处理发布的实时 MIC float32 特征。后续需要复测模型 baseline 时，
 * 可在 Makefile 中定义 APP_MODEL_SMOKE_TEST_ENABLE=1 或临时改这里。
 */
#ifndef APP_MODEL_SMOKE_TEST_ENABLE
#define APP_MODEL_SMOKE_TEST_ENABLE    (0u)
#endif

bool app_model_smoke_run_once(void);

#if defined(__cplusplus)
}
#endif

#endif /* __APP_MODEL_SMOKE_H__ */

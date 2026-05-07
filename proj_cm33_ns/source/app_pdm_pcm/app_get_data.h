/*******************************************************************************
* File Name : app_get_data.h
*
* Description : PDM/PCM 采集链路测试接口。
*
* 本模块只用于 MIC 数据自检：从 app_pdm_pcm 的 PDM block 队列取数，检查序号、
* 幅值和 block 描述符是否正常。它不是正式音频前处理任务，也不是 CM55 模型
* 推理任务。正式业务链路请使用 app_audio_preprocess + CM55 app_model_inference。
*******************************************************************************/

#ifndef __APP_GET_DATA_H__
#define __APP_GET_DATA_H__


#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "app_pdm_pcm.h"
/*******************************************************************************
* Macros
*******************************************************************************/
#define APP_GET_DATA_TEST_TASK_STACK_SIZE       (1024u)
#define APP_GET_DATA_TEST_TASK_PRIORITY         (APP_PDM_PCM_TASK_PRIORITY - 1u)

/*******************************************************************************
* Global Variables
*******************************************************************************/

/*******************************************************************************
* Functions Prototypes
*******************************************************************************/
cy_rslt_t app_get_data_test_task_init(void);
void app_get_data_test_task(void *pvParameters);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __APP_GET_DATA_H__ */
/* [] END OF FILE */

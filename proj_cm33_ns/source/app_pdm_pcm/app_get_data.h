/******************************************************************************
* File Name : app_get_data.h
*
* Description : Header file for GET DATA containing function prototypes.
********************************************************************************/

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
#define APP_INFERENCE_TASK_STACK_SIZE       (1024u)
#define APP_INFERENCE_TASK_PRIORITY         (APP_PDM_PCM_TASK_PRIORITY - 1u)

/*******************************************************************************
* Global Variables
*******************************************************************************/

/*******************************************************************************
* Functions Prototypes
*******************************************************************************/
cy_rslt_t inference_task_init(void);
void inference_task(void *pvParameters);
void mic_data_test(void);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __APP_GET_DATA_H__ */
/* [] END OF FILE */

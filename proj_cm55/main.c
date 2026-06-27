/*******************************************************************************
* File Name        : main.c
*
* Description      : This source file contains the main routine for CM55 CPU
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

/*******************************************************************************
* Header File
*******************************************************************************/

#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_build_config.h"
#include "app_display_diag.h"
#include <stdio.h>

#if (APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE)
#include "app_cm55_display_bringup.h"
#include "app_cm55_display_retarget.h"
#endif

#if (APP_CM55_INFERENCE_ENABLE)
#include "app_model_inference.h"
#endif

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM55 application. 
* 
* CM33 application enables the CM55 CPU. CM55 then starts a FreeRTOS task
* that reads prepared model input from shared memory.
* 
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if ((APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE) && (APP_DISPLAY_DIAG_ENABLE))
    app_display_diag_mark(APP_DISPLAY_DIAG_STAGE_CM55_MAIN_ENTER,
                          0u,
                          (uint32_t)__LINE__,
                          0u);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
#if ((APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE) && (APP_DISPLAY_DIAG_ENABLE))
        app_display_diag_mark(APP_DISPLAY_DIAG_STAGE_CM55_CYBSP_INIT_FAIL,
                              (uint32_t)result,
                              (uint32_t)__LINE__,
                              0u);
#endif
        CY_ASSERT(0);
    }

#if ((APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE) && (APP_DISPLAY_DIAG_ENABLE))
    app_display_diag_mark(APP_DISPLAY_DIAG_STAGE_CM55_CYBSP_INIT_OK,
                          (uint32_t)result,
                          (uint32_t)__LINE__,
                          0u);
#endif

#if (APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE)
#if (APP_DISPLAY_CM55_UART_LOG_ENABLE)
    result = app_cm55_display_retarget_init();
    if (CY_RSLT_SUCCESS != result)
    {
#if (APP_DISPLAY_DIAG_ENABLE)
        app_display_diag_mark(APP_DISPLAY_DIAG_STAGE_CM55_RETARGET_FAIL,
                              (uint32_t)result,
                              (uint32_t)__LINE__,
                              0u);
#endif
#if (APP_CM55_INFERENCE_ENABLE)
        CY_ASSERT(0);
#else
        result = CY_RSLT_SUCCESS;
#endif
    }
#if (APP_DISPLAY_DIAG_ENABLE)
    else
    {
        app_display_diag_mark(APP_DISPLAY_DIAG_STAGE_CM55_RETARGET_OK,
                              (uint32_t)result,
                              (uint32_t)__LINE__,
                              0u);
    }
#endif
#else
#if (APP_DISPLAY_DIAG_ENABLE)
    app_display_diag_mark(APP_DISPLAY_DIAG_STAGE_CM55_RETARGET_SKIP,
                          0u,
                          (uint32_t)__LINE__,
                          0u);
#endif
#endif
#endif

    /* Enable global interrupts */
    __enable_irq();

#if (APP_DISPLAY_OFFICIAL_CM55_BRINGUP_ENABLE)
#if (APP_DISPLAY_CM55_UART_LOG_ENABLE)
    printf("[CM55_BOOT] display_bringup_enable=1 inference_enable=%lu\r\n",
           (unsigned long)APP_CM55_INFERENCE_ENABLE);
    fflush(stdout);
#endif
    result = app_cm55_display_bringup_init();
#if (APP_CM55_INFERENCE_ENABLE)
    if (CY_RSLT_SUCCESS == result)
    {
        result = app_model_inference_task_init();
    }
#endif
#elif (APP_CM55_INFERENCE_ENABLE)
    result = app_model_inference_task_init();
#else
    result = CY_RSLT_SUCCESS;
#endif
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    vTaskStartScheduler();
    CY_ASSERT(0);

    for (;;)
    {
    }
}

/* [] END OF FILE */

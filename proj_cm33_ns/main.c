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
#include "app_uart_radar.h"
#include "app_csv_export.h"
#include "retarget_io_init.h"
#include "FreeRTOS.h"
#include "task.h"

/* The timeout value in microsecond used to wait for core to be booted. */
#define CM55_BOOT_WAIT_TIME_USEC          (10u)
/* App boot address for CM55 project. */
#define CM55_APP_BOOT_ADDR                (CYMEM_CM33_0_m55_nvm_START + \
                                           CYBSP_MCUBOOT_HEADER_SIZE)

int main(void)
{
    cy_rslt_t result;

    result = cybsp_init();
    handle_app_error(result);

    __enable_irq();

    /* Initialize retarget-io before any printf from FreeRTOS tasks. */
    init_retarget_io();

    /* Create the PDM/PCM capture task. It initializes the microphone stream
     * after the scheduler starts and publishes 10 ms blocks for inference.
     */
    result = app_pdm_pcm_task_init();
    handle_app_error(result);

    /* Create the LD6002 radar UART task. It collects complete TF frames from
     * SCB5 and publishes them as queued blocks for inference.
     */
    result = app_uart_radar_task_init();
    handle_app_error(result);

#if (APP_CSV_EXPORT_ENABLE)
    /* CSV export task consumes both PDM and radar queues and prints one
     * tagged CSV stream through the debug UART.
     */
    result = app_csv_export_task_init();
    handle_app_error(result);
#else
#if (APP_UART_RADAR_TEST_ENABLE)
    /* Radar self-test task. It consumes only the radar frame queue and prints
     * summaries through the debug UART, not through SCB5.
     */
    result = app_uart_radar_test_task_init();
    handle_app_error(result);
#endif

    /* Temporary self-test task. Replace this when the real inference task
     * consumes the PDM/radar queues directly.
     */
    result = inference_task_init();
    handle_app_error(result);
#endif

    /* Enable CM55. Keep this if the multi-core project still uses the CM55 app. */
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);

    vTaskStartScheduler();
    configASSERT(0);

    for (;;)
    {
    }
}

/* [] END OF FILE */

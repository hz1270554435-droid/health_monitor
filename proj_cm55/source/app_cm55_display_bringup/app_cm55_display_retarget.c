/*******************************************************************************
* File Name : app_cm55_display_retarget.c
*
* Description : Initializes debug UART retarget-io from CM55 display proof mode.
*******************************************************************************/

#include "app_cm55_display_retarget.h"

#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_scb_uart.h"
#include "mtb_hal.h"

static cy_stc_scb_uart_context_t debug_uart_context;
static mtb_hal_uart_t debug_uart_hal_obj;

cy_rslt_t app_cm55_display_retarget_init(void)
{
    cy_rslt_t result;

    result = (cy_rslt_t)Cy_SCB_UART_Init(CYBSP_DEBUG_UART_HW,
                                         &CYBSP_DEBUG_UART_config,
                                         &debug_uart_context);
    if (CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    Cy_SCB_UART_Enable(CYBSP_DEBUG_UART_HW);

    result = mtb_hal_uart_setup(&debug_uart_hal_obj,
                                &CYBSP_DEBUG_UART_hal_config,
                                &debug_uart_context,
                                NULL);
    if (CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    return cy_retarget_io_init(&debug_uart_hal_obj);
}


/*******************************************************************************
 * File Name:   retarget_io_init.c
 *
 * Description: This file contains the initialization routine for the 
 *              retarget-io middleware
 *
 * Related Document: See README.md
 *
 *******************************************************************************
* Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
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
* Header Files
*******************************************************************************/
#include "retarget_io_init.h"

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* For the RetargetIO (Debug UART) usage */
static cy_stc_scb_uart_context_t    DEBUG_UART_context;  
static mtb_hal_uart_t               DEBUG_UART_hal_obj;  

/* Debug UART 分频表。
 *
 * 这里的数值和 BSP 当前生成配置绑定：CYBSP_DEBUG_UART_config.oversample = 10，
 * debug UART 的 peripheral clock root = 100 MHz。PSoC 的 divider 寄存器值比
 * 实际分频小 1，因此：
 * - 2 Mbps：100 MHz / 5 / 10 = 2000000，寄存器值为 4；
 * - 115200：100 MHz / 87 / 10 = 114942.5，寄存器值为 86，误差约 -0.22%。
 *
 * 如果以后用 Device Configurator 改了 UART oversample 或 clock root，必须同步
 * 重新计算这里的 divider，否则上位机会继续出现乱码。
 */
#define DEBUG_UART_115200_DIVIDER_VALUE    (86U)
#define DEBUG_UART_2000000_DIVIDER_VALUE   (4U)

/* 将应用层选择的固定波特率映射为硬件 divider 寄存器值。 */
static uint32_t debug_uart_get_divider_value(uint32_t baud_rate);

/* Retarget-io deepsleep callback parameters  */
#if (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP)

/* Context reference structure for Debug UART */
static mtb_syspm_uart_deepsleep_context_t retarget_io_syspm_ds_context =
{
    .uart_context       = &DEBUG_UART_context,
    .async_context      = NULL,
    .tx_pin =
    {
        .port           = CYBSP_DEBUG_UART_TX_PORT,
        .pinNum         = CYBSP_DEBUG_UART_TX_PIN,
        .hsiom          = CYBSP_DEBUG_UART_TX_HSIOM
    },
    .rts_pin = 
    {
        .port           = DEBUG_UART_RTS_PORT,
        .pinNum         = DEBUG_UART_RTS_PIN,
        .hsiom          = HSIOM_SEL_GPIO
    }
};

/* SysPm callback parameter structure for Debug UART */
static cy_stc_syspm_callback_params_t retarget_io_syspm_cb_params =
{
    .context            = &retarget_io_syspm_ds_context,
    .base               = CYBSP_DEBUG_UART_HW
};

/* SysPm callback structure for Debug UART */
static cy_stc_syspm_callback_t retarget_io_syspm_cb =
{
    .callback           = &mtb_syspm_scb_uart_deepsleep_callback,
    .skipMode           = SYSPM_SKIP_MODE,
    .type               = CY_SYSPM_DEEPSLEEP,
    .callbackParams     = &retarget_io_syspm_cb_params,
    .prevItm            = NULL,
    .nextItm            = NULL,
    .order              = SYSPM_CALLBACK_ORDER
};
#endif /* (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP) */

/*******************************************************************************
* Function Name: init_retarget_io
********************************************************************************
* Summary:
*  初始化 debug UART，并把 printf/scanf 重定向到该串口。
*
* Parameters:
*  baud_rate - 只能传 RETARGET_IO_BAUD_115200 或 RETARGET_IO_BAUD_2000000。
*              该参数由 main.c 根据运行模式统一选择。
*
* Return:
*  void
*
*******************************************************************************/
void init_retarget_io(uint32_t baud_rate)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* 安全镜像或 BSP 可能已经按默认值初始化过 debug UART 时钟。
     * 这里在 CM33_NS 侧按 main.c 传入的应用模式重新设置，避免采集模式和
     * 普通日志模式的串口助手波特率不一致。
     */
    Cy_SysClk_PeriPclkDisableDivider((en_clk_dst_t)CYBSP_DEBUG_UART_CLK_DIV_GRP_NUM,
                                     CYBSP_DEBUG_UART_CLK_DIV_HW,
                                     CYBSP_DEBUG_UART_CLK_DIV_NUM);
    Cy_SysClk_PeriPclkSetDivider((en_clk_dst_t)CYBSP_DEBUG_UART_CLK_DIV_GRP_NUM,
                                 CYBSP_DEBUG_UART_CLK_DIV_HW,
                                 CYBSP_DEBUG_UART_CLK_DIV_NUM,
                                 debug_uart_get_divider_value(baud_rate));
    Cy_SysClk_PeriPclkEnableDivider((en_clk_dst_t)CYBSP_DEBUG_UART_CLK_DIV_GRP_NUM,
                                    CYBSP_DEBUG_UART_CLK_DIV_HW,
                                    CYBSP_DEBUG_UART_CLK_DIV_NUM);

    /* 初始化 SCB UART 本体。波特率由上面的 divider 决定，数据位/校验/停止位仍使用
     * BSP 生成的 CYBSP_DEBUG_UART_config，即 8N1。
     */
    result = (cy_rslt_t)Cy_SCB_UART_Init(CYBSP_DEBUG_UART_HW, 
                                        &CYBSP_DEBUG_UART_config, 
                                        &DEBUG_UART_context);
    
    /* UART 初始化失败时停机，避免后续调试日志看似正常但实际没有串口输出。 */
    handle_app_error(result);

    /* 使能 SCB UART。 */
    Cy_SCB_UART_Enable(CYBSP_DEBUG_UART_HW);

    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, 
                                &CYBSP_DEBUG_UART_hal_config, 
                                &DEBUG_UART_context, NULL);
    
    /* HAL UART 绑定失败时停机，避免 retarget-io 挂到无效 UART 对象。 */
    handle_app_error(result);

    /* 初始化 retarget-io，使 printf/scanf 走 debug UART。 */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

    /* retarget-io 初始化失败时停机，避免后续 printf 输出丢失且难以定位。 */
    handle_app_error(result);

#if (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP)
    /* 低功耗模式下注册 UART SysPm 回调，保证 deepsleep 进出时串口状态一致。 */
    Cy_SysPm_RegisterCallback(&retarget_io_syspm_cb);
#endif /* (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP) */
}

static uint32_t debug_uart_get_divider_value(uint32_t baud_rate)
{
    /* 只接受头文件公开的两个固定速率。若传入其它值，说明 main.c 或编译宏配置
     * 已经越过了当前已校准范围，直接停机比使用错误波特率输出乱码更容易定位。
     */
    switch (baud_rate)
    {
        case RETARGET_IO_BAUD_115200:
            return DEBUG_UART_115200_DIVIDER_VALUE;

        case RETARGET_IO_BAUD_2000000:
            return DEBUG_UART_2000000_DIVIDER_VALUE;

        default:
            handle_app_error(CY_RSLT_TYPE_ERROR);
            return DEBUG_UART_115200_DIVIDER_VALUE;
    }
}

/* [] END OF FILE */

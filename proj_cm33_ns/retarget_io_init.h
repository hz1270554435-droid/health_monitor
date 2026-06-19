/*******************************************************************************
 * File Name:   retarget_io_init.h
 *
 * Description:  This file is the public interface of retarget_io_init.c and 
 *               contains the necessary UART configuration parameters.
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

#ifndef _RETARGET_IO_INIT_H_
#define _RETARGET_IO_INIT_H_

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "mtb_hal.h"
#include "cy_retarget_io.h"
#include "mtb_syspm_callbacks.h"

/*******************************************************************************
* Macros
*******************************************************************************/

/* retarget-io deepsleep callback macros */
#define DEBUG_UART_RTS_PORT     (NULL)
#define DEBUG_UART_RTS_PIN      (0U)

/* Default syspm callback configuration elements */
#define SYSPM_SKIP_MODE         (0U)
#define SYSPM_CALLBACK_ORDER    (1U)

/* Debug UART 支持的应用层波特率。
 *
 * 注意：这里不是开放的任意 baud 配置，而是列出已经在 retarget_io_init.c 中
 * 计算过 divider 的固定速率：
 * - 115200：普通串口助手、模型 smoke test、正式链路调试日志；
 * - 230400 / 460800 / 921600：UART transport sanity matrix；
 * - 2000000：CSV/PCM 等高吞吐采集导出，减少串口阻塞和丢帧概率。
 *
 * 新增其它速率时，应同步在 retarget_io_init.c 中增加 divider 映射和注释。
 */
#define RETARGET_IO_BAUD_115200     (115200UL)
#define RETARGET_IO_BAUD_230400     (230400UL)
#define RETARGET_IO_BAUD_460800     (460800UL)
#define RETARGET_IO_BAUD_921600     (921600UL)
#define RETARGET_IO_BAUD_2000000    (2000000UL)


/*******************************************************************************
* Function prototypes
*******************************************************************************/
/* 初始化 debug UART 和 retarget-io。
 *
 * 参数 baud_rate 必须为上面列出的固定速率之一。
 * 业务层应在 main.c 中根据 APP_RUNTIME_MODE 选择波特率；底层只负责按该速率
 * 设置 UART 时钟分频并完成 printf 重定向。
 */
void init_retarget_io(uint32_t baud_rate);

/* Task/early-boot context only. This bypasses stdio and waits until the UART
 * FIFO and shifter are empty before returning.
 */
void retarget_io_write_blocking(const uint8_t *data, uint32_t size);

/*******************************************************************************
* Function Name: handle_app_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  uint32_t status - status indicates success or failure
*
* Return:
*  void
*
*******************************************************************************/
__STATIC_INLINE void handle_app_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        /* Disable all interrupts. */
        __disable_irq();
        
        CY_ASSERT(0);
        
        /* Infinite loop */
        while(true);
    }
}

#endif /* _RETARGET_IO_INIT_H_ */

/* [] END OF FILE */

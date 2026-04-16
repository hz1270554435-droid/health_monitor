/******************************************************************************
* File Name : i2s_playback.c
*
* Description : Source file for Audio Playback via I2S.
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
* Header Files
*******************************************************************************/
#include "app_i2s.h"

/*******************************************************************************
* Global Variables
*******************************************************************************/
mtb_hal_i2c_t MW_I2C_hal_obj;
cy_stc_scb_i2c_context_t MW_CYBSP_I2C_CONTROLLER_0_context;
mtb_hal_i2c_cfg_t i2c_config = 
{
    .is_target = false,
    .address = I2C_ADDRESS,
    .frequency_hz = I2C_FREQUENCY_HZ,
    .address_mask = MTB_HAL_I2C_DEFAULT_ADDR_MASK,
    .enable_address_callback = false
};

const cy_stc_sysint_t i2s_isr_txcfg = 
{
    .intrSrc = (IRQn_Type) tdm_0_interrupts_tx_0_IRQn,
    .intrPriority = I2S_ISR_PRIORITY
};

/* Audio playback tracking variables */
uint32_t i2s_txcount = 0;
volatile bool i2s_flag = false;

uint16_t zeros_data[HW_FIFO_HALF_SIZE/2] = {0};
/*******************************************************************************
 * Function Name: app_i2s_init
 *******************************************************************************
* Summary: Initializes I2S and registers I2S interrupt handler
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void app_i2s_init(void)
{
    /* Initialize the I2S interrupt */
    Cy_SysInt_Init(&i2s_isr_txcfg, i2s_tx_interrupt_handler);
    NVIC_EnableIRQ(i2s_isr_txcfg.intrSrc);

   /* Initialize the I2S */
    cy_en_tdm_status_t volatile return_status = Cy_AudioTDM_Init(TDM_STRUCT0, 
                                                &CYBSP_TDM_CONTROLLER_0_config);
    if (CY_TDM_SUCCESS != return_status)
    {
        CY_ASSERT(0);
    }

    /* Clear TX interrupts */
    Cy_AudioTDM_ClearTxInterrupt(TDM_STRUCT0_TX, CY_TDM_INTR_TX_MASK);
    Cy_AudioTDM_SetTxInterruptMask(TDM_STRUCT0_TX, CY_TDM_INTR_TX_MASK);
}

/*******************************************************************************
 * Function Name: app_tlv_codec_init
 *******************************************************************************
* Summary: Initializes the I2C and TLV codec. 
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void app_tlv_codec_init(void)
{
    /* Initialize I2C used to configure TLV codec */
    tlv_codec_i2c_init();
    /* TLV codec/ MW init */
    mtb_tlv320dac3100_init(&MW_I2C_hal_obj);
    /* Configure internal clock dividers to achieve desired sample rate */
    mtb_tlv320dac3100_configure_clocking(MCLK_HZ, SAMPLE_RATE_HZ, I2S_WORD_LENGTH, 
                                         TLV320DAC3100_SPK_AUDIO_OUTPUT);
    /* Activate TLV320DAC3100 */
    mtb_tlv320dac3100_activate();

}

/*******************************************************************************
 * Function Name: tlv_codec_i2c_init
 *******************************************************************************
* Summary: Initialize I2C used for the TLV codec configurations
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void tlv_codec_i2c_init(void)
{
    cy_en_scb_i2c_status_t result;
    cy_rslt_t hal_result;

    /* Initialize and enable the I2C in controller mode. */
    result = Cy_SCB_I2C_Init(CYBSP_I2C_CONTROLLER_HW, 
                             &CYBSP_I2C_CONTROLLER_config, 
                             &MW_CYBSP_I2C_CONTROLLER_0_context);
    if(result != CY_SCB_I2C_SUCCESS)
    {
        CY_ASSERT(0);
    }
    /* Enable I2C hardware. */
    Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_HW);

    /* I2C HAL init */
    hal_result = mtb_hal_i2c_setup(&MW_I2C_hal_obj, 
                                   &CYBSP_I2C_CONTROLLER_hal_config, 
                                &MW_CYBSP_I2C_CONTROLLER_0_context, NULL);
    handle_app_error(hal_result);

    /* Configure the I2C block. */
    hal_result = mtb_hal_i2c_configure(&MW_I2C_hal_obj, &i2c_config);
    handle_app_error(hal_result);
}

/*******************************************************************************
 * Function Name: i2s_tx_interrupt_handler
 *******************************************************************************
* Summary: I2S transmit interrupt handler function.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void i2s_tx_interrupt_handler(void)
{
    /*get intr status and look for errors*/
    uint32_t intr = Cy_AudioTDM_GetTxInterruptStatusMasked(TDM_STRUCT0_TX);

    if(CY_TDM_INTR_TX_FIFO_TRIGGER & intr)
    {

        for(int i=0; i < HW_FIFO_HALF_SIZE/2; i++)
        {
            /* Write same data for L,R channels(dual mono) in FIFO */
            Cy_AudioTDM_WriteTxData(TDM_STRUCT0_TX, 
                                    (uint32_t) (audio_data_ptr[i2s_txcount++]));
            Cy_AudioTDM_WriteTxData(TDM_STRUCT0_TX, 
                                    (uint32_t) (audio_data_ptr[i2s_txcount++]));
        }
        /* If the end of the wave data is reached, reset i2s_txcount
        check and set end of playback */
        if (i2s_txcount >= (recorded_data_size))
        {
            i2s_txcount = 0;
            i2s_flag = true;
        }
    }
    else if(CY_TDM_INTR_TX_FIFO_UNDERFLOW & intr)
    {
        printf("Error: I2S transmit underflowed");
    }

    /* Clear all Tx I2S Interrupt */
    Cy_AudioTDM_ClearTxInterrupt(TDM_STRUCT0_TX, CY_TDM_INTR_TX_MASK);
}

/*******************************************************************************
 * Function Name: app_i2s_disable
 *******************************************************************************
* Summary: Clear Tx FIFO and disable I2S
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void app_i2s_disable(void)
{
    Cy_AudioTDM_DisableTx(TDM_STRUCT0_TX);
}

/*******************************************************************************
 * Function Name: app_i2s_activate
 *******************************************************************************
* Summary: Activate I2S Tx interrupt
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void app_i2s_activate(void)
{
    /* Activate and enable I2S TX interrupts */
    Cy_AudioTDM_ActivateTx(TDM_STRUCT0_TX);
}

/*******************************************************************************
 * Function Name: app_i2s_enable
 *******************************************************************************
* Summary: Enable I2S and fill TX HW FIFO
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void app_i2s_enable(void)
{
    /* Clear TX interrupts */
    Cy_AudioTDM_ClearTxInterrupt(TDM_STRUCT0_TX, CY_TDM_INTR_TX_MASK);
    Cy_AudioTDM_SetTxInterruptMask(TDM_STRUCT0_TX, CY_TDM_INTR_TX_MASK);

    /* Start the I2S TX */                                
    Cy_AudioTDM_EnableTx(TDM_STRUCT0_TX);

    /* Fill TX FIFO before it is activated with Zeros */
    for(int i=0; i < HW_FIFO_HALF_SIZE/2; i++)
    {
        /* Write data in FIFO */
        Cy_AudioTDM_WriteTxData(TDM0_TDM_STRUCT0_TDM_TX_STRUCT, 
                                (uint32_t) (zeros_data[i]));
        Cy_AudioTDM_WriteTxData(TDM0_TDM_STRUCT0_TDM_TX_STRUCT, 
                                (uint32_t) (zeros_data[i]));
    }
    
}

/*******************************************************************************
 * Function Name: app_i2s_activate
 *******************************************************************************
* Summary: Activate I2S Tx interrupt
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void app_i2s_deactivate(void)
{
    /* Deactivate and enable I2S TX interrupts */
    Cy_AudioTDM_DeActivateTx(TDM_STRUCT0_TX);
}
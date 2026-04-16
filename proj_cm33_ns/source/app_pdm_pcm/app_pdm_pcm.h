/******************************************************************************
* File Name : app_pdm_pcm.h
*
* Description : Header file for PDM PCM containing function ptototypes.
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
#ifndef __APP_PDM_PCM_H__
#define __APP_PDM_PCM_H__


#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cybsp.h"
#include "app_i2s.h"
/*******************************************************************************
* Macros
*******************************************************************************/
/* Number of channels */
#define NUM_CHANNELS                   (2u)
#define LEFT_CH_INDEX                  (2u)
#define RIGHT_CH_INDEX                 (3u)
#define PDM_IRQ                        CYBSP_PDM_CHANNEL_3_IRQ
#define LEFT_CH_CONFIG                 channel_2_config
#define RIGHT_CH_CONFIG                channel_3_config

#define PDM_HW_FIFO_SIZE               (64u)
#define RX_FIFO_TRIG_LEVEL             (PDM_HW_FIFO_SIZE/2)
/* PDM Half FIFO Size */
#define PDM_HALF_FIFO_SIZE             (PDM_HW_FIFO_SIZE/2)

/* Recording time in seconds */
#define RECORDING_DURATION_SEC         (4u)
/* Size of the recorded buffer */
#define BUFFER_SIZE                    (RECORDING_DURATION_SEC * SAMPLE_RATE_HZ)

/* Number of samples to ignore in the beginning of a recording */
#define IGNORED_SAMPLES                (PDM_HW_FIFO_SIZE)

/* PDM PCM interrupt priority */
#define PDM_PCM_ISR_PRIORITY            (7u)

/* Gain range for EVK kit PDM mic */
#define PDM_PCM_MIN_GAIN                        (-103.0)
#define PDM_PCM_MAX_GAIN                        (83.0)
#define PDM_MIC_GAIN_VALUE                      (20)

/* Gain to Scale mapping */

#define PDM_PCM_SEL_GAIN_83DB                   (83.0)
#define PDM_PCM_SEL_GAIN_77DB                   (77.0)
#define PDM_PCM_SEL_GAIN_71DB                   (71.0)
#define PDM_PCM_SEL_GAIN_65DB                   (65.0)
#define PDM_PCM_SEL_GAIN_59DB                   (59.0)
#define PDM_PCM_SEL_GAIN_53DB                   (53.0)
#define PDM_PCM_SEL_GAIN_47DB                   (47.0)
#define PDM_PCM_SEL_GAIN_41DB                   (41.0)
#define PDM_PCM_SEL_GAIN_35DB                   (35.0)
#define PDM_PCM_SEL_GAIN_29DB                   (29.0)
#define PDM_PCM_SEL_GAIN_23DB                   (23.0)
#define PDM_PCM_SEL_GAIN_17DB                   (17.0)
#define PDM_PCM_SEL_GAIN_11DB                   (11.0)
#define PDM_PCM_SEL_GAIN_5DB                    (5.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_1DB           (-1.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_7DB           (-7.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_13DB          (-13.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_19DB          (-19.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_25DB          (-25.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_31DB          (-31.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_37DB          (-37.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_43DB          (-43.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_49DB          (-49.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_55DB          (-55.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_61DB          (-61.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_67DB          (-67.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_73DB          (-73.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_79DB          (-79.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_85DB          (-85.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_91DB          (-91.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_97DB          (-97.0)
#define PDM_PCM_SEL_GAIN_NEGATIVE_103DB         (-103.0)

/*******************************************************************************
* Global Variables
*******************************************************************************/
extern int16_t recorded_data[NUM_CHANNELS * BUFFER_SIZE];


/*******************************************************************************
* Functions Prototypes
*******************************************************************************/
void app_pdm_pcm_init(void);
void app_pdm_pcm_activate(void);
cy_en_pdm_pcm_gain_sel_t convert_db_to_pdm_scale(double db);
void set_pdm_pcm_gain(cy_en_pdm_pcm_gain_sel_t gain);
void pdm_interrupt_handler(void);
void app_pdm_pcm_deactivate(void);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __APP_PDM_PCM_H__ */
/* [] END OF FILE */

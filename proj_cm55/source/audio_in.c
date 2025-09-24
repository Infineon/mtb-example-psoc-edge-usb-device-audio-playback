/*****************************************************************************
* File Name        : audio_in.c
*
* Description      : This file contains the Audio In path configuration and
*                    processing code.
*
* Related Document : See README.md
*
******************************************************************************
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
*****************************************************************************/
#include "audio_in.h"
#include "audio_app.h"
#include "audio.h"
#include "emusbdev_audio_config.h"
#include "retarget_io_init.h"
#include "rtos.h"


/*****************************************************************************
* Macros
*****************************************************************************/
/*Macro to select STEREO or MONO mode */
#define AUDIO_DATA_INTERLEAVING     (1u)

#define LSB_MASK                    (0x0000FFFF)

/*****************************************************************************
* Global Variables
*****************************************************************************/
const cy_stc_pdm_pcm_channel_config_t pdm_pcm_channel_2_config =
{
    .sampledelay = 1,
    .wordSize = CY_PDM_PCM_WSIZE_16_BIT,
    .signExtension = true,
    .rxFifoTriggerLevel = 31,
    .fir0_enable = false,
    .fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_1,
    .fir0_scale = 0,
#if (AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_16KHZ)
    .cic_decim_code  = CY_PDM_PCM_CHAN_CIC_DECIM_32,
    .fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_3,
    .fir1_scale = 10,
#elif(AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_32KHZ)
    .cic_decim_code  = CY_PDM_PCM_CHAN_CIC_DECIM_16,
    .fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_3,
    .fir1_scale = 8,
#elif(AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_48KHZ)
    .cic_decim_code  = CY_PDM_PCM_CHAN_CIC_DECIM_16,
    .fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_2,
    .fir1_scale = 5,
#elif(AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_22KHZ)
    .cic_decim_code  = CY_PDM_PCM_CHAN_CIC_DECIM_16,
    .fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_4,
    .fir1_scale = 8,
#elif(AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_44KHZ)
    .cic_decim_code  = CY_PDM_PCM_CHAN_CIC_DECIM_16,
    .fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_2,
    .fir1_scale = 5,
#endif
    .dc_block_disable = false,
    .dc_block_code = CY_PDM_PCM_CHAN_DCBLOCK_CODE_16,
};

const cy_stc_pdm_pcm_channel_config_t pdm_pcm_channel_3_config =
{
    .sampledelay = 5,
    .wordSize = CY_PDM_PCM_WSIZE_16_BIT,
    .signExtension = true,
    .rxFifoTriggerLevel = 31,
    .fir0_enable = false,
    .fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_1,
    .fir0_scale = 0,
#if (AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_16KHZ)
    .cic_decim_code  = CY_PDM_PCM_CHAN_CIC_DECIM_32,
    .fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_3,
    .fir1_scale = 10,
#elif(AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_32KHZ)
    .cic_decim_code  = CY_PDM_PCM_CHAN_CIC_DECIM_16,
    .fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_3,
    .fir1_scale = 8,
#elif(AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_48KHZ)
    .cic_decim_code  = CY_PDM_PCM_CHAN_CIC_DECIM_16,
    .fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_2,
    .fir1_scale = 5,
#elif(AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_22KHZ)
    .cic_decim_code  = CY_PDM_PCM_CHAN_CIC_DECIM_16,
    .fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_4,
    .fir1_scale = 8,
#elif(AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_44KHZ)
    .cic_decim_code  = CY_PDM_PCM_CHAN_CIC_DECIM_16,
    .fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_2,
    .fir1_scale = 5,
#endif
    .dc_block_disable = false,
    .dc_block_code = CY_PDM_PCM_CHAN_DCBLOCK_CODE_16,
};

/* PCM buffer data (16-bits) */
uint16_t audio_in_pcm_buffer_ping[(MAX_AUDIO_IN_PACKET_SIZE_WORDS)];
uint16_t audio_in_pcm_buffer_pong[(MAX_AUDIO_IN_PACKET_SIZE_WORDS)];

/* Audio IN flags */
volatile bool audio_in_start_recording = false;
volatile bool audio_in_is_recording    = false;

/* Mic mute status */
U8 mic_mute;

/*****************************************************************************
* Static const data
*****************************************************************************/
const unsigned char silent_frame[MAX_AUDIO_IN_PACKET_SIZE_BYTES] = {0};

/*****************************************************************************
* Function Name: audio_in_init
******************************************************************************
* Summary:
*  Initialize the PDM/PCM block and create the "Audio In Task" which will
*  process the Audio IN endpoint transactions.
*
* Parameters:
*  None
*
* Return:
*  None
*
*****************************************************************************/
void audio_in_init(void)
{
    BaseType_t rtos_task_status;

    /* Initialize PDM/PCM block */
    cy_en_pdm_pcm_status_t volatile status = Cy_PDM_PCM_Init(CYBSP_PDM_HW, &CYBSP_PDM_config);
    
    if(CY_PDM_PCM_SUCCESS != status)
    {
        handle_app_error();
    }

    /* Initialize PDM/PCM channel 0 -Left, 1 -Right */
    /* Enable PDM channel, we will activate channel for record later */
    Cy_PDM_PCM_Channel_Enable(CYBSP_PDM_HW, LEFT_CH_INDEX);
    Cy_PDM_PCM_Channel_Enable(CYBSP_PDM_HW, RIGHT_CH_INDEX);

    Cy_PDM_PCM_Channel_Init(CYBSP_PDM_HW, &pdm_pcm_channel_2_config, (uint8_t) LEFT_CH_INDEX);
    Cy_PDM_PCM_Channel_Init(CYBSP_PDM_HW, &pdm_pcm_channel_3_config, (uint8_t) RIGHT_CH_INDEX);

    /* Create the AUDIO Write RTOS task */
    rtos_task_status = xTaskCreate(audio_in_process, "Audio In Task", AUDIO_TASK_STACK_DEPTH, NULL,
            AUDIO_IN_TASK_PRIORITY, &rtos_audio_in_task);
    if (pdPASS != rtos_task_status)
    {
        handle_app_error();
    }
}


/*****************************************************************************
* Function Name: audio_in_enable
******************************************************************************
* Summary:
*  Start a recording session.
*
* Parameters:
*  None
*
* Return:
*  None
*
*****************************************************************************/
void audio_in_enable(void)
{
    audio_in_start_recording = true;

    /* Activate recording from channel after init Activate Channel */
    Cy_PDM_PCM_Activate_Channel(CYBSP_PDM_HW, LEFT_CH_INDEX);
    Cy_PDM_PCM_Activate_Channel(CYBSP_PDM_HW, RIGHT_CH_INDEX);

    USBD_AUDIO_Start_Play(usb_audio_context, NULL);

    /* Turn ON the kit LED to indicate start of a recording session */
    Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_ON);
}


/*****************************************************************************
* Function Name: audio_in_disable
******************************************************************************
* Summary:
*  Stop a recording session.
*
* Parameters:
*  None
*
* Return:
*  None
*
*****************************************************************************/
void audio_in_disable(void)
{
    audio_in_is_recording = false;

    Cy_PDM_PCM_DeActivate_Channel(CYBSP_PDM_HW, LEFT_CH_INDEX);
    Cy_PDM_PCM_DeActivate_Channel(CYBSP_PDM_HW, RIGHT_CH_INDEX);

    USBD_AUDIO_Stop_Play(usb_audio_context);

    /* Turn OFF the kit LED to indicate the end of the recording session */
    Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_OFF);
}


/*****************************************************************************
* Function Name: audio_in_process
******************************************************************************
* Summary:
*  Wrapper task for USBD_AUDIO_Write_Task (audio in endpoint).
*
* Parameters:
*  arg
*
* Return:
*  None
*
*****************************************************************************/
void audio_in_process(void *arg)
{
    CY_UNUSED_PARAMETER(arg);

    USBD_AUDIO_Write_Task();

    for (;;)
    {
    }
}


/*****************************************************************************
* Function Name: audio_in_endpoint_callback
******************************************************************************
* Summary:
*  Callback called in the context of USBD_AUDIO_Write_Task.
*  Handles data sent to the host (IN direction).
*
* Parameters:
*  user_context: User context which is passed to the callback.
*  next_buffer: Buffer containing audio samples which should match the
*                configuration from microphone USBD_AUDIO_IF_CONF.
*  packet_size: Size of the next buffer.
*
* Return:
*  None
*
*****************************************************************************/
void audio_in_endpoint_callback(void *user_context,
                                const U8 **next_buffer,
                                U32 *packet_size)
{
    static uint16_t *audio_in_pcm_buffer = NULL;

    CY_UNUSED_PARAMETER(user_context);

    if (audio_in_start_recording)
    {
        audio_in_start_recording = false;
        audio_in_is_recording = true;

        /* Clear Audio In buffer */
        memset(audio_in_pcm_buffer_ping, 0, (MAX_AUDIO_IN_PACKET_SIZE_BYTES));
        memset(audio_in_pcm_buffer_pong, 0, (MAX_AUDIO_IN_PACKET_SIZE_BYTES));

        audio_in_pcm_buffer = audio_in_pcm_buffer_ping;

        /* Start a transfer to the Audio IN endpoint */
        *next_buffer = (uint8_t *) audio_in_pcm_buffer;
        *packet_size = MAX_AUDIO_IN_PACKET_SIZE_BYTES;
    }
    else if (audio_in_is_recording) /* Check if should keep recording */
    {
        if (audio_in_pcm_buffer == audio_in_pcm_buffer_ping)
        {
            audio_in_pcm_buffer = audio_in_pcm_buffer_pong;
        }
        else
        {
            audio_in_pcm_buffer = audio_in_pcm_buffer_ping;
        }
        
        /* Read audio data from PDM-PCM FIFO */
        for(uint8_t i=0; i < MAX_AUDIO_IN_PACKET_SIZE_WORDS; i++)
        {
            int32_t data = (int32_t)Cy_PDM_PCM_Channel_ReadFifo(CYBSP_PDM_HW, LEFT_CH_INDEX);
            #if AUDIO_DATA_INTERLEAVING
            *(audio_in_pcm_buffer + i++) = (uint16_t)(data);
            #else
                audio_data_left[l] = (int16_t)(data);
            #endif
            data = (int32_t)Cy_PDM_PCM_Channel_ReadFifo(CYBSP_PDM_HW, RIGHT_CH_INDEX);
            #if AUDIO_DATA_INTERLEAVING
            *(audio_in_pcm_buffer + i) = (uint16_t)(data);
            #else
                audio_data_right[l] = (int16_t)(data);
            #endif
        }
        if (1U == mic_mute)
        {
            /* Send silent frames in case of mute */
            *next_buffer = silent_frame;
        }
        else
        {
        /* Send captured audio samples to the Audio IN endpoint */
            *next_buffer = (uint8_t *) audio_in_pcm_buffer;
        }
        *packet_size = MAX_AUDIO_IN_PACKET_SIZE_BYTES;
    }
}

/* [] END OF FILE */
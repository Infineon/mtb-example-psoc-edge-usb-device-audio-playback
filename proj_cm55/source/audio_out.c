/*******************************************************************************
* File Name: audio_out.c
*
*  Description: This file contains the Audio Out path configuration and
*               processing code
*
*******************************************************************************
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

/*****************************************************************************
* Headers
*****************************************************************************/
#include "audio_out.h"
#include "audio_app.h"
#include "audio.h"
#include "rtos.h"
#include "retarget_io_init.h"

/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/
/* PCM buffer data (16-bits) */
uint16_t audio_out_pcm_buffer_ping[MAX_AUDIO_OUT_PACKET_SIZE_WORDS];
uint16_t audio_out_pcm_buffer_pong[MAX_AUDIO_OUT_PACKET_SIZE_WORDS];

int32_t list_stat;

/* Audio OUT flags */
volatile bool audio_out_is_streaming    = false;
volatile bool audio_start_streaming    = false;

TaskHandle_t rtos_audio_out_task = NULL;


/*******************************************************************************
* Function Name: audio_out_init
********************************************************************************
* Summary:
*  Initialize the audio OUT flow by setting up the I2S block 
*  and scheduling "Audio Out Task"
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void audio_out_init(void)
{
    BaseType_t rtos_task_status;

    /* Initialize the I2S */
    cy_en_tdm_status_t volatile return_status = Cy_AudioTDM_Init(TDM_STRUCT0, 
                                                &CYBSP_TDM_CONTROLLER_0_config);

    if (CY_TDM_SUCCESS != return_status)
    {
        handle_app_error();
    }

    /* Start the I2S TX */                                
    Cy_AudioTDM_EnableTx(TDM_STRUCT0_TX);

    rtos_task_status = xTaskCreate(audio_out_process, "Audio Out Task",
        AUDIO_OUT_TASK_STACK_DEPTH, NULL, AUDIO_OUT_TASK_PRIORITY,
                        &rtos_audio_out_task);

    if (pdPASS != rtos_task_status)
    {
        handle_app_error();
    }

    configASSERT(rtos_audio_out_task);  
}

/*******************************************************************************
* Function Name: audio_out_enable
********************************************************************************
* Summary:
*  Start a playing session.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void audio_out_enable(void)
{
    if(audio_start_streaming == false)
    {
        /* Activate and enable I2S TX interrupts */
        Cy_AudioTDM_ActivateTx(TDM_STRUCT0_TX);

        audio_start_streaming = true;
        list_stat = USBD_AUDIO_Start_Listen(usb_audio_context, NULL);
        configASSERT(0 == list_stat);
    }
}

/*******************************************************************************
* Function Name: audio_out_disable
********************************************************************************
* Summary:
*   Stop a playing session.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void audio_out_disable(void)
{
    audio_out_is_streaming = false;
    USBD_AUDIO_Stop_Listen(usb_audio_context);
}

/*******************************************************************************
* Function Name: audio_out_process
********************************************************************************
* Summary:
*   Main task for the audio out endpoint. 
*
* Parameters:
*  void *arg - arguments for audio input processing function
*
* Return:
*  None
*******************************************************************************/
void audio_out_process(void *arg)
{
    (void) arg;

    USBD_AUDIO_Read_Task();

    while (1)
    {

    }
}

/*******************************************************************************
* Function Name: audio_out_endpoint_callback
********************************************************************************
* Summary:
*   Audio OUT endpoint callback implementation. It enables transfer of
*   audio frame from USB OUT endpoint buffer to I2S TX FIFO buffer.
* Parameters:
*  void * user_context -
*  int num_bytes_received -
*  U8 ** next_buffer -
*  U32 * packet_size -
*
* Return:
*  None
*
*******************************************************************************/
void audio_out_endpoint_callback(void *user_context, int num_bytes_received, 
                                 U8 **next_buffer, U32 *packet_size)
{
    CY_UNUSED_PARAMETER(user_context);
    unsigned int data_to_write;
    static uint16_t *audio_out_to_i2s_tx = NULL;

    if (audio_start_streaming)
    {
        audio_start_streaming = false;
        audio_out_is_streaming = true;

        /* Clear Audio Out buffer */
        memset(audio_out_pcm_buffer_ping, 0, (MAX_AUDIO_OUT_PACKET_SIZE_BYTES));
        memset(audio_out_pcm_buffer_pong, 0, (MAX_AUDIO_OUT_PACKET_SIZE_BYTES));

        audio_out_to_i2s_tx = audio_out_pcm_buffer_ping;

        /* Start a transfer to the Audio OUT endpoint */
        *next_buffer = (uint8_t *) audio_out_to_i2s_tx;
    }
    else if(audio_out_is_streaming)
    {
        if(num_bytes_received != 0)
        {
            /* Number of samples*/
            data_to_write = num_bytes_received / AUDIO_OUT_SUB_FRAME_SIZE ;

            /* Write data to I2S Tx */
            for(int i=0; i < data_to_write; i++)
            {
                /* Write same data for L,R channels in FIFO */
                Cy_AudioTDM_WriteTxData(TDM_STRUCT0_TX, 
                                        (uint32_t) *(audio_out_to_i2s_tx + i));
                Cy_AudioTDM_WriteTxData(TDM_STRUCT0_TX, 
                                        (uint32_t) *(audio_out_to_i2s_tx + i));
            }

             /* Start a transfer to OUT endpoint */
            *next_buffer = (uint8_t *) audio_out_to_i2s_tx;
        }
    }
}

/* [] END OF FILE */
/*******************************************************************************
* File Name: audio_out.c
*
*  Description: This file contains the Audio Out path configuration and
*               processing code
*
*******************************************************************************
* (c) 2025-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
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
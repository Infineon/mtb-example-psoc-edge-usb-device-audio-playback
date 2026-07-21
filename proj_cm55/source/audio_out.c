/*******************************************************************************
* File Name: audio_out.c
*
*  Description: Audio OUT (speaker) hardware initialization (I2S/TDM).
*               Stream lifecycle, TDM FIFO writes, and feedback rate
*               computation are handled by audio_app.c via the USBD_AC API.
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
#include "audio.h"
#include "rtos.h"
#include "retarget_io_init.h"

/*******************************************************************************
* Function Name: audio_out_init
********************************************************************************
* Summary:
*  Initializes the TDM/I2S controller for the speaker (OUT) stream and enables
*  the transmit path used to drive audio samples to the codec.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void audio_out_init(void)
{
    cy_en_tdm_status_t volatile return_status = Cy_AudioTDM_Init(TDM_STRUCT0,
                                                &CYBSP_TDM_CONTROLLER_0_config);
    if (CY_TDM_SUCCESS != return_status)
    {
        handle_app_error();
    }

    Cy_AudioTDM_EnableTx(TDM_STRUCT0_TX);
}

/* [] END OF FILE */

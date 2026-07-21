/*******************************************************************************
* File Name   : usbd_ac_config.h
*
* Description : USB Audio Class (USBD_AC) descriptor configuration header.
*               Adapted from SEGGER emUSBD Audio Device Generator output for
*               the PSoC Edge as audio device.
*
*               Device topology:
*                 Speaker:    48 kHz, mono,   16-bit, asynchronous with feedback EP
*                 Microphone: 48 kHz, stereo, 16-bit, synchronous
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
#ifndef USBD_AC_CFG_H
#define USBD_AC_CFG_H

#include "USB_AC.h"

/*******************************************************************************
* Audio version
*******************************************************************************/
#define USBD_AC_AUDIO_VERSION                            1

/*******************************************************************************
* Control unit / entity IDs (matches descriptor byte layout)
*******************************************************************************/
#define USBD_AC_ID_CONTROL                         0x00000u
#define USBD_AC_ID_UNIT_USBIn                      0x00100u  /* IT  1 - USB Streaming -> Speaker */
#define USBD_AC_ID_UNIT_SpeakerControl             0x00200u  /* FU  2 - Speaker Mute/Volume     */
#define USBD_AC_ID_UNIT_Speaker                    0x00300u  /* OT  3 - Speaker output           */
#define USBD_AC_ID_UNIT_Microphone                 0x00400u  /* IT  4 - Microphone input          */
#define USBD_AC_ID_UNIT_MicControl                 0x00500u  /* FU  5 - Mic Mute/Volume           */
#define USBD_AC_ID_UNIT_USBOut                     0x00600u  /* OT  6 - USB Streaming <- Mic       */
#define USBD_AC_ID_AS_Speaker                      0x10000u
#define USBD_AC_ID_EP_Speaker                      0x1ff00u
#define USBD_AC_ID_AS_Microphone                   0x20000u
#define USBD_AC_ID_EP_Microphone                   0x2ff00u

/*******************************************************************************
* Interface indices
*******************************************************************************/
#define USBD_AC_INTERFACE_Control                        0u
#define USBD_AC_INTERFACE_Speaker                        1u
#define USBD_AC_INTERFACE_Microphone                     2u

/*******************************************************************************
* Endpoint indices (into the _Endpoints[] table)
*******************************************************************************/
#define USBD_AC_DATA_EP_Speaker                          0u
#define USBD_AC_FDBCK_EP_Speaker                         1u
#define USBD_AC_DATA_EP_Microphone                       2u

/*******************************************************************************
* Configuration pointer
*******************************************************************************/
#define USB_AC_CONFIGURATION    (&USB_AC_Config_audio)

extern const USBD_AC_CONFIG USB_AC_Config_audio;

/*******************************************************************************
* Frequency macros (used by audio_app.c for control get/set callbacks)
*******************************************************************************/
#define SPEAKER_FREQUENCIES            48000
#define MICROPHONE_FREQUENCIES         48000

#endif /* USBD_AC_CFG_H */

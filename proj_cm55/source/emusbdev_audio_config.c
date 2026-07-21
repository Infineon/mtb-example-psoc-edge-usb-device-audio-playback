/*****************************************************************************
* File Name   : emusbdev_audio_config.c
*
* Description : HID report descriptor for the USB Audio HID control interface.
*               The USBD_AC audio class configuration is in usbd_ac_config.c.
*               USB_DEVICE_INFO is now in audio_app.c.
*
******************************************************************************
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
*****************************************************************************/

#include "emusbdev_audio_config.h"

/* HID report descriptor (Consumer Control page - volume, mute, transport) */
const U8 hid_report[] = {
    0x05, 0x0c,                    /* USAGE_PAGE (Consumer Devices) */
    0x09, 0x01,                    /* USAGE (Consumer Control) */
    0xa1, 0x01,                    /* COLLECTION (Application) */
    0x85, 0x01,                    /* REPORT_ID (1) */
    0x15, 0x00,                    /* LOGICAL_MINIMUM (0) */
    0x25, 0x01,                    /* LOGICAL_MAXIMUM (1) */
    0x09, 0xe9,                    /* USAGE (Volume Up) */
    0x09, 0xea,                    /* USAGE (Volume Down) */
    0x09, 0xe2,                    /* USAGE (Mute) */
    0x09, 0xcd,                    /* USAGE (Play/Pause) */
    0x09, 0xb5,                    /* USAGE (Scan Next Track) */
    0x09, 0xb6,                    /* USAGE (Scan Previous Track) */
    0x09, 0xbc,                    /* USAGE (Repeat) */
    0x09, 0xb9,                    /* USAGE (Random Play) */
    0x75, 0x01,                    /* REPORT_SIZE (1) */
    0x95, 0x08,                    /* REPORT_COUNT (8) */
    0x81, 0x02,                    /* INPUT (Data,Var,Abs) */
    0xc0                           /* END_COLLECTION */
};

/* [] END OF FILE */

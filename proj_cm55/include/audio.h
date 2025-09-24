/******************************************************************************
* File Name   : audio.h
*
* Description : This file contains the constants mapped to the USB descriptor.
*
* Note        : See README.md
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
******************************************************************************/
#ifndef AUDIO_H
#define AUDIO_H

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/******************************************************************************
* Macros
******************************************************************************/
#define AUDIO_SAMPLING_RATE_16KHZ               (16000U)
#define AUDIO_SAMPLING_RATE_22KHZ               (22050U)
#define AUDIO_SAMPLING_RATE_32KHZ               (32000U)
#define AUDIO_SAMPLING_RATE_44KHZ               (44100U)
#define AUDIO_SAMPLING_RATE_48KHZ               (48000U)

/* Initialization data for a single audio format */
#define AUDIO_IN_NUM_CHANNELS                   (2U)
#define AUDIO_IN_SUB_FRAME_SIZE                 (2U)   /* In bytes */
#define AUDIO_IN_BIT_RESOLUTION                 (16U)
#define AUDIO_IN_SAMPLE_FREQ                    AUDIO_SAMPLING_RATE_48KHZ

#define AUDIO_OUT_NUM_CHANNELS      (1U)
#define AUDIO_OUT_SUB_FRAME_SIZE    (2U)    /* In bytes */
#define AUDIO_OUT_BIT_RESOLUTION    (16U)
#define AUDIO_OUT_SAMPLE_FREQ       AUDIO_SAMPLING_RATE_48KHZ

/* Each report consists of 2 bytes:
 * 1. The report ID (0x01) and a
 * 2. bit mask containing 8 control events: */

#define AUDIO_HID_REPORT_VOLUME_UP            (0x01u)
#define AUDIO_HID_REPORT_VOLUME_DOWN          (0x02u)
#define AUDIO_HID_REPORT_PLAY_PAUSE           (0x08u)
#define AUDIO_HID_CTRL_MUTE                   (0x04u)
#define AUDIO_HID_REPORT_ID0                  (0x1u)
#define AUDIO_HID_REPORT_ID1                  (0x0u)

#define AUDIO_VOLUME_SIZE     (2U)
/**< Volume minimum value MSB */
#define AUDIO_VOLUME_MIN_MSB  (0x00U)
/**< Volume minimum value LSB */
#define AUDIO_VOLUME_MIN_LSB  (0x1AU)
/**< Volume maximum value MSB */
#define AUDIO_VOLUME_MAX_MSB  (0x00U)
/**< Volume maximum value LSB */
#define AUDIO_VOLUME_MAX_LSB  (0x7FU)
/**< Volume resolution MSB */
#define AUDIO_VOLUME_RES_MSB  (0x00U)
/**< Volume resolution LSB */
#define AUDIO_VOLUME_RES_LSB  (0x01U)

/* VendorID */
#define AUDIO_DEVICE_VENDOR_ID                  (0x058B)

/* ProductIDs */
#if (AUDIO_SAMPLING_RATE_16KHZ == AUDIO_IN_SAMPLE_FREQ)
#define CODEC_SAMPLE_RATE_HZ                    (TLV320DAC3100_DAC_SAMPLE_RATE_16_KHZ)
#define AUDIO_DEVICE_PRODUCT_ID                 (0x0289)
#elif (AUDIO_SAMPLING_RATE_22KHZ == AUDIO_IN_SAMPLE_FREQ)
#define CODEC_SAMPLE_RATE_HZ                    (TLV320DAC3100_DAC_SAMPLE_RATE_22_05_KHZ)
#define AUDIO_DEVICE_PRODUCT_ID                 (0x028A)
#elif (AUDIO_SAMPLING_RATE_32KHZ == AUDIO_IN_SAMPLE_FREQ)
#define CODEC_SAMPLE_RATE_HZ                    (TLV320DAC3100_DAC_SAMPLE_RATE_32_KHZ)
#define AUDIO_DEVICE_PRODUCT_ID                 (0x028B)
#elif (AUDIO_SAMPLING_RATE_44KHZ == AUDIO_IN_SAMPLE_FREQ)
#define CODEC_SAMPLE_RATE_HZ                    (TLV320DAC3100_DAC_SAMPLE_RATE_44_1_KHZ)
#define AUDIO_DEVICE_PRODUCT_ID                 (0x028C)
#elif (AUDIO_SAMPLING_RATE_48KHZ == AUDIO_IN_SAMPLE_FREQ)
#define CODEC_SAMPLE_RATE_HZ                    (TLV320DAC3100_DAC_SAMPLE_RATE_48_KHZ)
#define AUDIO_DEVICE_PRODUCT_ID                 (0x028D)
#else
#error "Sample rate not supported in this code example."
#endif /* AUDIO_SAMPLING_RATE_16KHZ */


/******************************************************************************
* Has to match the configured values in Microphone and Speaker Configuration
* For Example:

* For a sample rate of 44100, 16 bits per sample, 2 channels:
* (44100 * ((16/8) * 2)) / 1000 = 176 bytes
* Additional sample size is added to make sure we can send 
* odd sized frames if necessary:
* 176 bytes + ((16/8) * 2) = 180
* For 
******************************************************************************/

/* USB IN Endpoint Audio maximum packet size (in bytes) */
/* Packet size = ( Sampling frequency * (Bit resolution / 8) * Num of channels ) / (frame duration in ms) */
#define MAX_AUDIO_IN_PACKET_SIZE_BYTES          ((((AUDIO_IN_SAMPLE_FREQ) * (((AUDIO_IN_BIT_RESOLUTION) / 8U) * (AUDIO_IN_NUM_CHANNELS))) / 1000U))

/* USB IN Endpoint Audio maximum packet size (in words) */
/* Number of Words = (Number of bytes / Audio sub-frame size) */
#define MAX_AUDIO_IN_PACKET_SIZE_WORDS          ((MAX_AUDIO_IN_PACKET_SIZE_BYTES) / (AUDIO_IN_SUB_FRAME_SIZE))


/* USB OUT Endpoint Audio maximum packet size (in bytes) */
/* Packet size = ( Sampling frequency * (Bit resolution / 8) * Num of channels ) / (frame duration in ms) */
#define MAX_AUDIO_OUT_PACKET_SIZE_BYTES          ((((AUDIO_OUT_SAMPLE_FREQ) * (((AUDIO_OUT_BIT_RESOLUTION) / 8U) * (AUDIO_OUT_NUM_CHANNELS))) / 1000U)) /* In bytes */

/* USB OUT Endpoint Audio maximum packet size (in words) */
/* Number of Words = (Number of bytes / Audio sub-frame size) */
#define MAX_AUDIO_OUT_PACKET_SIZE_WORDS          ((MAX_AUDIO_OUT_PACKET_SIZE_BYTES) / (AUDIO_OUT_SUB_FRAME_SIZE)) /* In words */


#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* AUDIO_H */

/* [] END OF FILE */
/******************************************************************************
* File Name   : pdm_pcm.h
*
* Description : PDM/PCM interface declarations.
*
* Note        : See README.md
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
#ifndef PDM_PCM_H
#define PDM_PCM_H

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdint.h>

/******************************************************************************
* Configurable options
******************************************************************************/
/* Maximum frame size in bytes, used for internal buffer allocation.
 * Consider the following equation when providing this number: 
 *
 *    Total value = Number of channels *
 *                  Window size per channel *
 *                  Sample size in bytes *  
 *                  Headroom factor 
 *
 * Where:
 *   - Number of channels: the total number of PDM channels
 *   - Window size per channel: the size of the audio window for each channel,
 *                              which is usually pass as argument for the 
 *                              pdm_pcm_read() function
 *   - Sample size in bytes: 1 byte for 8-bit samples, 
 *                           2 bytes for 16-bit samples, 
 *                           4 bytes for 24-bit or 32-bit samples
 *   - Headroom factor: a factor to provide additional buffer space, 
 *                      usually set to at least 1.5
 *
 * Important Note:
 *   This value should be power of 2 to efficiently implement a ring buffer.
 *   Currently, it supports up to 6 channels.
 */
#define PDM_PCM_MAX_FRAME_SIZE_IN_BYTES   (4096U) 

/* Set the pointer of the PDM/PCM component from the device-configurator.
 * Alternatively, set the pointer to the PDM/PCM hardware instance. */
#define PDM_PCM_HW                        (CYBSP_PDM_HW)

/* Set the config structure of the PDM/PCM component from the device-configurator.
 * Alternatively, define your own config structure. */
#define PDM_PCM_config                    (CYBSP_PDM_config)

/* Set the PDM/PCM related clock settings from the device-configurator.
 * Alternatively, set the values manually of the clock instance. */
#define PDM_PCM_GRP_NUM                   (CYBSP_PDM_CLK_DIV_GRP_NUM)
#define PDM_PCM_DIV_NUM                   (CYBSP_PDM_CLK_DIV_NUM)
#define PDM_PCM_DIV_TYPE                  (CYBSP_PDM_CLK_DIV_HW)

/* Set the name of the channel index and PDM channel configuration based on
 * the device-configurator settings. If a channel is not used, comment it out. */
#define PDM_PCM_CHANNEL_A_INDEX             (2U)
#define PDM_PCM_CHANNEL_A_CONFIG            (channel_2_config)
#define PDM_PCM_CHANNEL_A_IRQ               (CYBSP_PDM_CHANNEL_2_IRQ)

#define PDM_PCM_CHANNEL_B_INDEX             (3U)
#define PDM_PCM_CHANNEL_B_CONFIG            (channel_3_config)

//#define PDM_PCM_CHANNEL_C_INDEX             
//#define PDM_PCM_CHANNEL_C_CONFIG            

//#define PDM_PCM_CHANNEL_D_INDEX             
//#define PDM_PCM_CHANNEL_D_CONFIG                  

//#define PDM_PCM_CHANNEL_E_INDEX             
//#define PDM_PCM_CHANNEL_E_CONFIG            

//#define PDM_PCM_CHANNEL_F_INDEX             
//#define PDM_PCM_CHANNEL_F_CONFIG    

/******************************************************************************
* Constants and macros
******************************************************************************/
/* Mask config options */
#define PDM_PCM_CFG_NONBLOCKING           (1UL << 0)
#define PDM_PCM_CFG_START_ON_INIT         (1UL << 1)
#define PDM_PCM_CFG_INTERLEAVED           (1UL << 2)

#define PDM_PCM_CFG_DEFAULT_CONFIG        (PDM_PCM_CFG_NONBLOCKING | \
                                           PDM_PCM_CFG_START_ON_INIT | \
                                           PDM_PCM_CFG_INTERLEAVED)

/* Maximum timeout value */
#define PDM_PCM_MAX_TIMEOUT_MS            (0xFFFFFFFFUL)

/* Maximum and minimum gain values in dB */
#define PDM_PCM_GAIN_MIN_DB               (-103)
#define PDM_PCM_GAIN_MAX_DB               (83)

/* Supported oversample ratios */
#define PDM_PCM_OVERSAMPLE_RATIO_32       (32U)
#define PDM_PCM_OVERSAMPLE_RATIO_48       (48U)
#define PDM_PCM_OVERSAMPLE_RATIO_64       (64U)
#define PDM_PCM_OVERSAMPLE_RATIO_96       (96U)

typedef enum
{
    PDM_PCM_SUCCESS = 0,
    PDM_PCM_BAD_PARAM,
    PDM_PCM_BAD_STATE,
    PDM_PCM_BAD_CLOCK,
    PDM_PCM_TIMEOUT,
    PDM_PCM_NO_DATA,
    PDM_PCM_NOT_ENOUGH_DATA,
    PDM_PCM_WRONG_FRAME_SIZE,
    PDM_PCM_HW_ERROR,
    PDM_PCM_OVERFLOW
} pdm_pcm_status_t;

pdm_pcm_status_t pdm_pcm_init(uint8_t channels, uint8_t sample_size_bits);

pdm_pcm_status_t pdm_pcm_init_adv(uint8_t channels,
                                  uint8_t sample_size_bits,
                                  uint32_t sample_rate_hz,
                                  uint16_t oversample_ratio,
                                  uint32_t config_mask);

pdm_pcm_status_t pdm_pcm_start(void);

pdm_pcm_status_t pdm_pcm_stop(void);

pdm_pcm_status_t pdm_pcm_deinit(void);

pdm_pcm_status_t pdm_pcm_set_hw_gain(int8_t gain_in_db);

pdm_pcm_status_t pdm_pcm_set_soft_gain(int8_t gain_in_db);

pdm_pcm_status_t pdm_pcm_get_buffered_sample_count(uint32_t *sample_count_per_channel);

pdm_pcm_status_t pdm_pcm_set_trigger_callback(uint32_t level_per_channel, void (*callback)(void *arg), void *arg);

pdm_pcm_status_t pdm_pcm_blocking_read(void *buffer,
                                       uint32_t samples_per_channel,
                                       uint32_t timeout_ms);

pdm_pcm_status_t pdm_pcm_nonblocking_read(void *buffer,
                                          uint32_t samples_per_channel,
                                          uint32_t *samples_read_per_channel);

pdm_pcm_status_t pdm_pcm_read(void *buffer, uint32_t samples_per_channel);

#if defined(__cplusplus)
}
#endif

#endif /* PDM_PCM_H */

/* [] END OF FILE */

/******************************************************************************
* File Name   : audio.h
*
* Description : This file contains the constants mapped to the USB descriptor.
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
******************************************************************************/
#ifndef AUDIO_H
#define AUDIO_H

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/******************************************************************************
* Macros
******************************************************************************/
/* Supported audio sampling rates */
#define AUDIO_SAMPLING_RATE_16KHZ               (16000U)
#define AUDIO_SAMPLING_RATE_22KHZ               (22050U)
#define AUDIO_SAMPLING_RATE_32KHZ               (32000U)
#define AUDIO_SAMPLING_RATE_44KHZ               (44100U)
#define AUDIO_SAMPLING_RATE_48KHZ               (48000U)

/* Common audio sampling rate. The microphone (IN) and speaker (OUT) streams
 * MUST run at the same rate. To change the rate, edit ONLY this macro and set
 * it to one of the AUDIO_SAMPLING_RATE_* values defined above. */
#define AUDIO_SAMPLE_FREQ                       AUDIO_SAMPLING_RATE_48KHZ

/* Common audio bit resolution. The microphone (IN) and speaker (OUT) streams
 * MUST use the same bit resolution. DO NOT CHANGE: the firmware, USB
 * descriptor, and PDM/I2S configuration are hard-wired to this value. */
#define AUDIO_BIT_RESOLUTION                    (16U)  /* DO NOT CHANGE */

/* Audio IN (microphone) format - fixed stereo 16-bit PDM capture.
 * DO NOT CHANGE the channel count or sub-frame size: the firmware, USB
 * descriptor, and PDM/PCM configuration are hard-wired to these values. */
#define AUDIO_IN_NUM_CHANNELS                   (2U)   /* DO NOT CHANGE */
#define AUDIO_IN_SUB_FRAME_SIZE                 (2U)   /* In bytes - DO NOT CHANGE */
#define AUDIO_IN_BIT_RESOLUTION                 AUDIO_BIT_RESOLUTION   /* DO NOT CHANGE, refer to AUDIO_BIT_RESOLUTION */
#define AUDIO_IN_SAMPLE_FREQ                    AUDIO_SAMPLE_FREQ      /* DO NOT CHANGE, refer to AUDIO_SAMPLE_FREQ */

/* Audio OUT (speaker) format parameters - fixed mono 16-bit playback.
 * DO NOT CHANGE the channel count or sub-frame size: the firmware, USB
 * descriptor, and I2S/TDM configuration are hard-wired to these values. */
#define AUDIO_OUT_NUM_CHANNELS      (1U)   /* DO NOT CHANGE */
#define AUDIO_OUT_SUB_FRAME_SIZE    (2U)   /* In bytes - DO NOT CHANGE */
#define AUDIO_OUT_BIT_RESOLUTION    AUDIO_BIT_RESOLUTION   /* DO NOT CHANGE, refer to AUDIO_BIT_RESOLUTION */
#define AUDIO_OUT_SAMPLE_FREQ       AUDIO_SAMPLE_FREQ      /* DO NOT CHANGE, refer to AUDIO_SAMPLE_FREQ */

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
/*
 * Important: Use a unique PID after ANY change to the USB descriptor layout
 * (channels, sample rate, bit depth, endpoint count). Windows caches audio
 * settings keyed by VID:PID and silently refuses to stream if the cached
 * settings don't match. See:
 *   https://wiki.segger.com/USB_Audio#Audio_class_issues_on_Windows
 */
#if (AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_16KHZ)
#define CODEC_SAMPLE_RATE_HZ                    (TLV320DAC3100_DAC_SAMPLE_RATE_16_KHZ)
#define AUDIO_DEVICE_PRODUCT_ID                 (0x0290)
#elif (AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_22KHZ)
#define CODEC_SAMPLE_RATE_HZ                    (TLV320DAC3100_DAC_SAMPLE_RATE_22_05_KHZ)
#define AUDIO_DEVICE_PRODUCT_ID                 (0x0291)
#elif (AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_32KHZ)
#define CODEC_SAMPLE_RATE_HZ                    (TLV320DAC3100_DAC_SAMPLE_RATE_32_KHZ)
#define AUDIO_DEVICE_PRODUCT_ID                 (0x0292)
#elif (AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_44KHZ)
#define CODEC_SAMPLE_RATE_HZ                    (TLV320DAC3100_DAC_SAMPLE_RATE_44_1_KHZ)
#define AUDIO_DEVICE_PRODUCT_ID                 (0x0293)
#elif (AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_48KHZ)
#define CODEC_SAMPLE_RATE_HZ                    (TLV320DAC3100_DAC_SAMPLE_RATE_48_KHZ)
#define AUDIO_DEVICE_PRODUCT_ID                 (0x0294)
#else
#error "Sample rate not supported in this code example."
#endif /* AUDIO_IN_SAMPLE_FREQ */


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

/* Extra frames (samples per channel) the IN stream may add on top of the
 * nominal packet to drain the PDM FIFO when the capture clock runs faster than
 * the USB SOF. The microphone IN endpoint is sized to accommodate this. */
#define AUDIO_IN_FEEDBACK_HEADROOM_SAMPLES       (1U)

/* Max IN EP packet size including headroom for the drift-adjusted sample count.
 * = nominal packet bytes + headroom samples * (bit resolution / 8 * channels) */
#define MAX_AUDIO_IN_EP_PACKET_SIZE_BYTES        ((MAX_AUDIO_IN_PACKET_SIZE_BYTES) + \
                                                  ((AUDIO_IN_FEEDBACK_HEADROOM_SAMPLES) * \
                                                   (((AUDIO_IN_BIT_RESOLUTION) / 8U) * (AUDIO_IN_NUM_CHANNELS))))


/* USB OUT Endpoint Audio maximum packet size (in bytes) */
/* Packet size = ( Sampling frequency * (Bit resolution / 8) * Num of channels ) / (frame duration in ms) */
#define MAX_AUDIO_OUT_PACKET_SIZE_BYTES          ((((AUDIO_OUT_SAMPLE_FREQ) * (((AUDIO_OUT_BIT_RESOLUTION) / 8U) * (AUDIO_OUT_NUM_CHANNELS))) / 1000U)) /* In bytes */

/* Extra headroom (in samples) reserved on top of the nominal packet so the
 * feedback endpoint can request a higher playback rate (up to ~+1 kHz). */
#define AUDIO_OUT_FEEDBACK_HEADROOM_SAMPLES      (2U)

/* Max EP packet size including headroom for the feedback-adjusted rate.
 * = nominal packet bytes + headroom samples * (bit resolution / 8 * channels) */
#define MAX_AUDIO_OUT_EP_PACKET_SIZE_BYTES       ((MAX_AUDIO_OUT_PACKET_SIZE_BYTES) + \
                                                  ((AUDIO_OUT_FEEDBACK_HEADROOM_SAMPLES) * \
                                                   (((AUDIO_OUT_BIT_RESOLUTION) / 8U) * (AUDIO_OUT_NUM_CHANNELS))))

/* USB OUT Endpoint Audio maximum packet size (in words) */
/* Number of Words = (Number of bytes / Audio sub-frame size) */
#define MAX_AUDIO_OUT_PACKET_SIZE_WORDS          ((MAX_AUDIO_OUT_EP_PACKET_SIZE_BYTES) / (AUDIO_OUT_SUB_FRAME_SIZE)) /* In words */


/*******************************************************************************
 * Feedback Endpoint Configuration
 *
 * These constants define the USB isochronous feedback endpoint behaviour.
 * The feedback EP reports the device's actual TDM/I2S sample rate to the USB
 * host so it can adjust its data rate and avoid buffer over/underflow.
 ******************************************************************************/

/* How many Hz to deviate when speeding up / slowing down the reported rate */
#define AUDIO_FEEDBACK_SAMPLE_RATE_ADJUST_HZ    (1000U)

/* TDM TX FIFO depth on PSE84 - 128 entries (cy_tdm.h) */
#define AUDIO_OUT_TDM_FIFO_DEPTH_WORDS          (128U)

/* Worst-case FIFO entries written per USB OUT frame (incl. feedback headroom).
 * A whole USB packet is burst-written in a tight CPU loop, far faster than the
 * FIFO drains, so the level effectively jumps by this amount on every frame.
 * Each mono 16-bit sample (AUDIO_OUT_SUB_FRAME_SIZE bytes) is duplicated to the
 * L and R slots -> 2 FIFO entries per sample:
 *   entries = (MAX_AUDIO_OUT_EP_PACKET_SIZE_BYTES / AUDIO_OUT_SUB_FRAME_SIZE) * 2
 *           = (100 / 2) * 2 = 100 entries. */
#define AUDIO_OUT_TDM_FIFO_BURST_WORDS          (((MAX_AUDIO_OUT_EP_PACKET_SIZE_BYTES) / \
                                                  (AUDIO_OUT_SUB_FRAME_SIZE)) * 2U)

/* TDM TX FIFO thresholds for the 3-level feedback algorithm.
 *
 * Because a full burst lands almost instantaneously, the post-write peak level
 * is (pre-write level + burst).  To guarantee the next worst-case burst can
 * never overflow the FIFO, the "slow down" high watermark must satisfy
 *   HIGH_MARK + BURST <= DEPTH
 * which bounds the usable operating band to 0..HIGH_MARK.  The target sits at
 * the midpoint of that band and the low watermark below it triggers "speed up"
 * while a few entries still remain, avoiding underflow. */
#define AUDIO_OUT_TDM_FIFO_HIGH_MARK_WORDS      ((AUDIO_OUT_TDM_FIFO_DEPTH_WORDS) - \
                                                 (AUDIO_OUT_TDM_FIFO_BURST_WORDS))      /* 128 - 100 = 28 */
#define AUDIO_OUT_TDM_FIFO_TARGET_WORDS         ((AUDIO_OUT_TDM_FIFO_HIGH_MARK_WORDS) / 2U)  /* 14 */
#define AUDIO_OUT_TDM_FIFO_LOW_MARK_WORDS       ((AUDIO_OUT_TDM_FIFO_TARGET_WORDS) / 2U)      /* 7  */

/* ---- Microphone (IN) drift adaptation -------------------------------------
 * The IN endpoint sends a variable sample count per USB frame to keep the PDM
 * ring buffer level near a target level, compensating for drift between the PDM capture
 * clock and the USB SOF (implements review comment: no feedback EP on the IN
 * path - adjust the number of samples sent based on the PDM/PCM ring buffer level).
 *
 * PDM ring buffer depth is 64 words/channel (range 0-63, cy_pdm_pcm_v2.h). Each USB
 * frame the ring buffer gains AUDIO_IN_NOMINAL_WORDS_PER_CH words and is then drained.
 * For the level to never underflow it must stay >= nominal (enough to supply a
 * full packet); to never overflow it must stay < depth. The target is therefore
 * the MIDPOINT of [nominal .. depth], giving an equal cushion on both sides so
 * normal sub-sample jitter cannot cause an underflow. Draining to ~0 (target at
 * the nominal burst) leaves no cushion and underflows on the slightest drift. */
#define AUDIO_IN_PDM_LEVEL_DEPTH_WORDS          (64U)

/* Nominal words drained per channel each USB frame (== samples/ch per 1 ms) */
#define AUDIO_IN_NOMINAL_WORDS_PER_CH           ((MAX_AUDIO_IN_PACKET_SIZE_WORDS) / (AUDIO_IN_NUM_CHANNELS))

/* Steady-state ring buffer level target: midpoint of the safe band [nominal .. depth] */
#define AUDIO_IN_PDM_LEVEL_TARGET_WORDS         (((AUDIO_IN_NOMINAL_WORDS_PER_CH) + (AUDIO_IN_PDM_LEVEL_DEPTH_WORDS)) / 2U)

/* Dead-band around the target before a +/-1 frame correction is applied */
#define AUDIO_IN_PDM_LEVEL_DEADBAND_WORDS       (((AUDIO_IN_PDM_LEVEL_DEPTH_WORDS) - (AUDIO_IN_NOMINAL_WORDS_PER_CH)) / 4U)

#define AUDIO_IN_PDM_LEVEL_HIGH_MARK_WORDS      ((AUDIO_IN_PDM_LEVEL_TARGET_WORDS) + (AUDIO_IN_PDM_LEVEL_DEADBAND_WORDS))  /* drain one extra frame above this */
#define AUDIO_IN_PDM_LEVEL_LOW_MARK_WORDS       ((AUDIO_IN_PDM_LEVEL_TARGET_WORDS) - (AUDIO_IN_PDM_LEVEL_DEADBAND_WORDS))  /* send one fewer frame below this  */


#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* AUDIO_H */

/* [] END OF FILE */

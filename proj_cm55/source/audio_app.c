/******************************************************************************
* File Name   : audio_app.c
*
* Description : USB Audio application using USBD_AC (Audio Class) API with
*               explicit feedback endpoint for asynchronous USB/I2S clock
*               synchronization.
*
*               Architecture (message-queue driven):
*                 - Set-alternate-interface callback (ISR) posts SPEAKER/MIC
*                   ON/OFF messages to the queue.
*                 - RX callback (ISR) posts SPEAKER_DATA on each packet.
*                 - TX callback (ISR) posts MIC_DATA when a send completes.
*                 - SOF callback computes feedback rate from TDM FIFO level.
*                 - Main task loop processes messages and handles buttons.
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

/******************************************************************************
* Headers
*******************************************************************************/
#include "audio_app.h"
#include "audio_out.h"
#include "audio.h"
#include "pdm_pcm.h"
#include "usbd_ac_config.h"
#include "emusbdev_audio_config.h"
#include "USB_HID.h"
#include "rtos.h"
#include "mtb_tlv320dac3100.h"
#include "retarget_io_init.h"
#include "FreeRTOS.h"
#include "queue.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define USB_CONFIG_DELAY             (50u)
#define DELAY_TICKS_MS               (50u)
#define USB_SUSPENDED                (0u)
#define USB_CONNECTED                (1u)
#define BTN_IRQ_PRIORITY             (7u)
#define MAX_VOL_ROLLOVER             (0u)

/* I2C controller address */
#define I2C_ADDRESS                  (0x18)

/* I2C frequency in Hz */
#define I2C_FREQUENCY_HZ             (400000u)

#define SPEAKER_DEFAULT_VOL          (0x64u)

/* Feedback EP diagnostics - Set to 1 to enable
 * periodic statistics printed from the task loop.*/
#define AUDIO_FEEDBACK_DIAG_ENABLE   (0U)

/* How often (in SOF callbacks ≈ ms) to latch statistics for printing */
#define AUDIO_FEEDBACK_DIAG_INTERVAL (3000U)

/* PDM/PCM hardware/software gains in dB */
#define PDM_PCM_HW_GAIN_DB           (20u)
#define PDM_PCM_SW_GAIN_DB           (20u)

/* MCLK Value */
#define MCLK_HZ                      (2048000)

/* I2S word length parameter */
#define I2S_WORD_LENGTH              (TLV320DAC3100_I2S_WORD_SIZE_16)

/* Queue depth for inter-ISR/task messages */
#define MSG_QUEUE_DEPTH              (8u)

/* DPLL config */
#if ((AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_16KHZ))
    #define DPLL_LP_FREQ                 (49152000ul)
    #define PDM_PCM_OVERSAMPLE_RATE      (96u)
#elif ((AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_32KHZ))
    #define DPLL_LP_FREQ                 (49152000ul)
    #define PDM_PCM_OVERSAMPLE_RATE      (64u)
#elif ((AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_48KHZ))
    #define DPLL_LP_FREQ                 (49152000ul)
    #define PDM_PCM_OVERSAMPLE_RATE      (64u)
#elif((AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_22KHZ) || \
      (AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_44KHZ))
    #define DPLL_LP_FREQ                 (45158400ul)
    #define PDM_PCM_OVERSAMPLE_RATE      (64u)
#endif

#if (AUDIO_OUT_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_16KHZ)
    #define TDM_CLK_DIV_INT              (23u)
#elif (AUDIO_OUT_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_32KHZ)
    #define TDM_CLK_DIV_INT              (11u)
#elif (AUDIO_OUT_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_48KHZ)
    #define TDM_CLK_DIV_INT              (7u)
#elif (AUDIO_OUT_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_22KHZ)
    #define TDM_CLK_DIV_INT              (15u)
#elif (AUDIO_OUT_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_44KHZ)
    #define TDM_CLK_DIV_INT              (7u)
#endif

/* DPLL delay in milliseconds */
#define DPLL_DELAY_MS                (2000ul)

/*******************************************************************************
* Types - message queue events
*******************************************************************************/
typedef enum {
    MSG_SPEAKER_ON,
    MSG_SPEAKER_OFF,
    MSG_SPEAKER_DATA,
    MSG_MIC_ON,
    MSG_MIC_OFF,
    MSG_MIC_DATA,
} msg_type_t;

typedef struct {
    msg_type_t Event;
    U32        NumBytes;
    void      *pBuff;
} message_t;

/*******************************************************************************
* Forward declarations
*******************************************************************************/
static void audio_app_update_codec_volume(void);

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* RTOS task handles */
TaskHandle_t rtos_audio_app_task;

/* HID */
USB_HID_HANDLE    usb_hid_control_context;
static USB_HID_INIT_DATA    hid_init_data;

/* Message queue */
static QueueHandle_t  msg_queue;
static StaticQueue_t  msg_queue_static;
static message_t      msg_queue_storage[MSG_QUEUE_DEPTH];

/* USBD_AC stream contexts */
static USBD_AC_RX_CTX rx_ctx;   /* Speaker (host -> device) */
static USBD_AC_TX_CTX tx_ctx;   /* Microphone (device -> host) */

/* Speaker / Microphone current frequencies */
static const U32  speaker_frequencies[] = { SPEAKER_FREQUENCIES };
static const U32  mic_frequencies[]     = { MICROPHONE_FREQUENCIES };

/* AC global state (control values from the host) */
static struct {
    U32  CurrSpeakerFreq;
    U32  CurrMicFreq;
    U16  SpeakerVolume;
    U8   SpeakerMute[3];
    U8   SpeakerAltSetting;
    U8   MicrophoneMute;
    U16  MicrophoneVolume;
    U8   MicrophoneAltSetting;
    USBD_AC_STREAM_INTF_INFO SpeakerInfo;
    USBD_AC_STREAM_INTF_INFO MicInfo;
} ac_global;

/*******************************************************************************
 * Feedback EP diagnostics - ISR-safe accumulators
 *
 * These are updated inside the SOF callback (ISR) and the SPEAKER_DATA handler
 * (task), then snapshotted and printed periodically from the task loop.
 ******************************************************************************/
#if AUDIO_FEEDBACK_DIAG_ENABLE
/* --- Accumulators written from SOF ISR --- */
static volatile uint32_t fb_sof_count;          /* total SOF callbacks        */
static volatile uint32_t fb_slow_count;         /* times rate was decreased   */
static volatile uint32_t fb_fast_count;         /* times rate was increased   */
static volatile uint32_t fb_fifo_sum;           /* running sum for average    */
static volatile uint32_t fb_fifo_min;           /* minimum FIFO level seen    */
static volatile uint32_t fb_fifo_max;           /* maximum FIFO level seen    */

/* --- Accumulators written from task context (SPEAKER_DATA) --- */
static volatile uint32_t fb_overflow_drop_count;/* times samples were dropped */
static volatile uint32_t fb_underflow_count;    /* times FIFO was empty       */
static volatile uint32_t fb_data_msg_count;     /* total SPEAKER_DATA msgs    */

/* --- IN (microphone) drift/adaptation accumulators (task context) --- */
static volatile uint32_t mic_data_msg_count;    /* total MIC_DATA msgs        */
static volatile uint32_t mic_fast_count;        /* times +1 frame was sent    */
static volatile uint32_t mic_slow_count;        /* times -1 frame was sent    */
static volatile uint32_t mic_fifo_sum;          /* running sum for average    */
static volatile uint32_t mic_fifo_min;          /* min PDM FIFO level seen     */
static volatile uint32_t mic_fifo_max;          /* max PDM FIFO level seen     */
static volatile uint32_t mic_overflow_count;    /* PDM FIFO full (samples lost)*/
static volatile uint32_t mic_underflow_count;   /* PDM FIFO short of request    */
#endif /* AUDIO_FEEDBACK_DIAG_ENABLE */

/* Audio buffers - ping-pong for speaker to avoid USB overwriting in-flight data */
static U32  speaker_audio_buffer_0[MAX_AUDIO_OUT_EP_PACKET_SIZE_BYTES / 4];
static U32  speaker_audio_buffer_1[MAX_AUDIO_OUT_EP_PACKET_SIZE_BYTES / 4];
static volatile uint8_t speaker_buf_idx;
static U16  mic_audio_buffer[MAX_AUDIO_IN_EP_PACKET_SIZE_BYTES / 2];

/* I2C MTB HAL objects used by audio codec middleware */
mtb_hal_i2c_t MW_I2C_hal_obj;
cy_stc_scb_i2c_context_t MW_CYBSP_I2C_CONTROLLER_context;
mtb_hal_i2c_cfg_t i2c_config =
{
    .is_target = false,
    .address = I2C_ADDRESS,
    .frequency_hz = I2C_FREQUENCY_HZ,
    .address_mask = MTB_HAL_I2C_DEFAULT_ADDR_MASK,
    .enable_address_callback = false
};

/* User button events for volume increment/decrement */
typedef enum
{
    SWITCH_NO_EVENT,
    SWITCH_VOLUME_INCR,
    SWITCH_VOLUME_DECR,
} en_switch_event_t;

en_switch_event_t button_status;

/* Playback volume variables */
uint8_t   audio_app_volume;
uint8_t   audio_app_prev_volume;

static uint8_t  audio_app_control_report[2] = {AUDIO_HID_REPORT_ID0, AUDIO_HID_REPORT_ID1};
uint8_t usb_comm_cur_volume[AUDIO_VOLUME_SIZE] = {AUDIO_VOLUME_MAX_LSB, AUDIO_VOLUME_MAX_MSB};
uint8_t usb_comm_min_volume[AUDIO_VOLUME_SIZE] = {AUDIO_VOLUME_MIN_LSB, AUDIO_VOLUME_MIN_MSB};
uint8_t usb_comm_max_volume[AUDIO_VOLUME_SIZE] = {AUDIO_VOLUME_MAX_LSB, AUDIO_VOLUME_MAX_MSB};
uint8_t usb_comm_res_volume[AUDIO_VOLUME_SIZE] = {AUDIO_VOLUME_RES_LSB, AUDIO_VOLUME_RES_MSB};

/* Device info (HID report is from emusbdev_audio_config.c) */
static const USB_DEVICE_INFO usb_device_info = {
    AUDIO_DEVICE_VENDOR_ID,
    AUDIO_DEVICE_PRODUCT_ID,
    "Infineon Technologies",
    "USB Audio Playback",
    ""
};


/*******************************************************************************
* Function Name: gpio_isr_handler
********************************************************************************
* Summary:
*  GPIO interrupt handler for the two user buttons. Sets the pending button
*  status to a volume-increment or volume-decrement request and clears the
*  triggering pin interrupt.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void gpio_isr_handler(void)
{
    if(Cy_GPIO_GetInterruptStatus(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN))
    {
        button_status = SWITCH_VOLUME_INCR;
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN);
    }

    if(Cy_GPIO_GetInterruptStatus(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_PIN))
    {
        button_status = SWITCH_VOLUME_DECR;
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_PIN);
    }
}


/* ========================================================================== */
/*  USBD_AC callbacks (called in ISR context - must not block)                */
/* ========================================================================== */

/*******************************************************************************
* Function Name: audio_set_alt_interface_cb
********************************************************************************
* Summary:
*  USBD_AC set-alternate-interface callback (ISR context). Tracks the active
*  alternate setting and sample rate for the speaker and microphone streams and
*  posts a stream on/off event to the audio application task.
*
* Parameters:
*  InterfaceNo   : Audio streaming interface being updated (speaker or mic).
*  NewAltSetting : New alternate setting from the host (0 = zero-bandwidth).
*
* Return:
*  void
*
*******************************************************************************/
static void audio_set_alt_interface_cb(unsigned InterfaceNo, unsigned NewAltSetting)
{
    message_t msg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (InterfaceNo) {
    case USBD_AC_INTERFACE_Speaker:
        if (NewAltSetting > 0) {
            ac_global.CurrSpeakerFreq = speaker_frequencies[NewAltSetting - 1];
        }
        ac_global.SpeakerInfo = *USBD_AC_GetStreamInfo(USBD_AC_INTERFACE_Speaker, NewAltSetting);
        ac_global.SpeakerAltSetting = NewAltSetting;
        msg.Event = (NewAltSetting == 0) ? MSG_SPEAKER_OFF : MSG_SPEAKER_ON;
        xQueueSendFromISR(msg_queue, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        break;

    case USBD_AC_INTERFACE_Microphone:
        if (NewAltSetting > 0) {
            ac_global.CurrMicFreq = mic_frequencies[NewAltSetting - 1];
        }
        ac_global.MicInfo = *USBD_AC_GetStreamInfo(USBD_AC_INTERFACE_Microphone, NewAltSetting);
        ac_global.MicrophoneAltSetting = NewAltSetting;
        msg.Event = (NewAltSetting == 0) ? MSG_MIC_OFF : MSG_MIC_ON;
        xQueueSendFromISR(msg_queue, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        break;
    }
}

/*******************************************************************************
* Function Name: audio_out_rx_callback
********************************************************************************
* Summary:
*  USBD_AC receive callback for the speaker (OUT) stream (ISR context). Posts
*  the received audio buffer to the audio application task and re-arms the
*  endpoint with the other ping-pong buffer.
*
* Parameters:
*  Event   : USBD_AC event type (data-received expected).
*  pRxData : Receive descriptor holding the filled buffer and re-arm fields.
*
* Return:
*  void
*
*******************************************************************************/
static void audio_out_rx_callback(USBD_AC_EVENT Event, USBD_AC_RX_DATA *pRxData)
{
    if (Event == USBD_AC_EVENT_DATA_RECEIVED) {
        message_t msg;
        msg.Event    = MSG_SPEAKER_DATA;
        msg.NumBytes = pRxData->NumBytes;
        msg.pBuff    = pRxData->pBuffer;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(msg_queue, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        /* Re-arm with the OTHER buffer (ping-pong) */
        speaker_buf_idx ^= 1U;
        pRxData->pBuffer  = (speaker_buf_idx == 0)
                            ? speaker_audio_buffer_0
                            : speaker_audio_buffer_1;
        pRxData->NumBytes   = sizeof(speaker_audio_buffer_0);
        pRxData->NumPackets = 0;
    }
}

/*******************************************************************************
* Function Name: audio_in_tx_callback
********************************************************************************
* Summary:
*  USBD_AC transmit-complete callback for the microphone (IN) stream (ISR
*  context). Posts a MIC_DATA event so the audio application task can prepare
*  and send the next microphone packet.
*
* Parameters:
*  Event        : USBD_AC event type (data-send expected).
*  pData        : Unused transmit data pointer.
*  pUserContext : Unused user context pointer.
*
* Return:
*  void
*
*******************************************************************************/
static void audio_in_tx_callback(USBD_AC_EVENT Event, const void *pData, void *pUserContext)
{
    USB_USE_PARA(pData);
    USB_USE_PARA(pUserContext);

    if (Event == USBD_AC_EVENT_DATA_SEND) {
        message_t msg;
        msg.Event = MSG_MIC_DATA;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(msg_queue, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*******************************************************************************
* Function Name: audio_feedback_sof_callback
********************************************************************************
* Summary:
*  USBD_AC Start-of-Frame feedback callback for the speaker (OUT) stream. Reads
*  the TDM/I2S FIFO level and reports an adjusted feedback data rate to the host
*  so the incoming sample rate tracks the codec clock, preventing buffer
*  overflow or underflow.
*
* Parameters:
*  pCtx : USBD_AC receive context used to report the feedback data rate.
*
* Return:
*  void
*
*******************************************************************************/
static void audio_feedback_sof_callback(void *pCtx)
{
    USBD_AC_RX_CTX *pRX = (USBD_AC_RX_CTX *)pCtx;
    uint32_t data_rate;
    uint32_t fifo_level = Cy_AudioTDM_GetNumInTxFifo(TDM_STRUCT0_TX);

    if (fifo_level > AUDIO_OUT_TDM_FIFO_HIGH_MARK_WORDS) {
        data_rate = ((((AUDIO_OUT_SAMPLE_FREQ - AUDIO_FEEDBACK_SAMPLE_RATE_ADJUST_HZ)
                       << ac_global.SpeakerInfo.IntervalExp) << 13) + 500) / 1000;
#if AUDIO_FEEDBACK_DIAG_ENABLE
        fb_slow_count++;
#endif
    } else if (fifo_level < AUDIO_OUT_TDM_FIFO_LOW_MARK_WORDS) {
        data_rate = ((((AUDIO_OUT_SAMPLE_FREQ + AUDIO_FEEDBACK_SAMPLE_RATE_ADJUST_HZ)
                       << ac_global.SpeakerInfo.IntervalExp) << 13) + 500) / 1000;
#if AUDIO_FEEDBACK_DIAG_ENABLE
        fb_fast_count++;
#endif
    } else {
        data_rate = (((ac_global.CurrSpeakerFreq
                       << ac_global.SpeakerInfo.IntervalExp) << 13) + 500) / 1000;
    }

#if AUDIO_FEEDBACK_DIAG_ENABLE
    fb_sof_count++;
    fb_fifo_sum += fifo_level;
    if (fifo_level < fb_fifo_min) { fb_fifo_min = fifo_level; }
    if (fifo_level > fb_fifo_max) { fb_fifo_max = fifo_level; }
#endif

    USBD_AC_SetFeedbackDataRate(pRX, data_rate);
}


/* ========================================================================== */
/*  USBD_AC Audio Control callbacks (GET / SET)                               */
/* ========================================================================== */

#if USBD_AC_AUDIO_VERSION == 1
/*******************************************************************************
* Function Name: audio_control_get_cb
********************************************************************************
* Summary:
*  USBD_AC GET callback. Returns the current/min/max/resolution value for an
*  audio control request (sampling frequency, volume, or mute) addressed to the
*  speaker or microphone endpoint/feature unit.
*
* Parameters:
*  pReqInfo : Decoded audio control request (target ID and request type).
*  pBuffer  : Output buffer where the requested value is stored (LE encoded).
*
* Return:
*  Number of bytes written to pBuffer, or -1 if the request is unsupported.
*******************************************************************************/
static int audio_control_get_cb(const USBD_AC_CONTROL_INFO *pReqInfo, U8 *pBuffer)
{
    U32 Value;

    switch (pReqInfo->ID) {
    /* --- Speaker endpoint: sampling frequency --- */
    case USBD_AC_ID_EP_Speaker + USB_AC_SAMPLING_FREQ_CONTROL:
        switch (pReqInfo->bRequest) {
        case USB_AC_REQ_MIN:
        case USB_AC_REQ_MAX:
        case USB_AC_REQ_CUR:
            Value = ac_global.CurrSpeakerFreq;
            break;
        case USB_AC_REQ_RES:
            Value = 1;
            break;
        default:
            return -1;
        }
        USBD_StoreU24LE(pBuffer, Value);
        return 3;

    /* --- Microphone endpoint: sampling frequency --- */
    case USBD_AC_ID_EP_Microphone + USB_AC_SAMPLING_FREQ_CONTROL:
        switch (pReqInfo->bRequest) {
        case USB_AC_REQ_MIN:
        case USB_AC_REQ_MAX:
        case USB_AC_REQ_CUR:
            Value = ac_global.CurrMicFreq;
            break;
        case USB_AC_REQ_RES:
            Value = 1;
            break;
        default:
            return -1;
        }
        USBD_StoreU24LE(pBuffer, Value);
        return 3;

    /* --- Speaker Feature Unit: volume --- */
    case USBD_AC_ID_UNIT_SpeakerControl + USB_AC_FU_VOLUME_CONTROL:
        switch (pReqInfo->bRequest) {
        case USB_AC_REQ_CUR:
            Value = ac_global.SpeakerVolume;
            break;
        case USB_AC_REQ_MIN:
            Value = (usb_comm_min_volume[1] << 8) | usb_comm_min_volume[0];
            break;
        case USB_AC_REQ_MAX:
            Value = (usb_comm_max_volume[1] << 8) | usb_comm_max_volume[0];
            break;
        case USB_AC_REQ_RES:
            Value = (usb_comm_res_volume[1] << 8) | usb_comm_res_volume[0];
            break;
        default:
            return -1;
        }
        USBD_StoreU16LE(pBuffer, Value);
        return 2;

    /* --- Microphone Feature Unit: volume --- */
    case USBD_AC_ID_UNIT_MicControl + USB_AC_FU_VOLUME_CONTROL:
        switch (pReqInfo->bRequest) {
        case USB_AC_REQ_CUR:
            Value = ac_global.MicrophoneVolume;
            break;
        case USB_AC_REQ_MIN:
            Value = (usb_comm_min_volume[1] << 8) | usb_comm_min_volume[0];
            break;
        case USB_AC_REQ_MAX:
            Value = (usb_comm_max_volume[1] << 8) | usb_comm_max_volume[0];
            break;
        case USB_AC_REQ_RES:
            Value = (usb_comm_res_volume[1] << 8) | usb_comm_res_volume[0];
            break;
        default:
            return -1;
        }
        USBD_StoreU16LE(pBuffer, Value);
        return 2;

    /* --- Speaker Feature Unit: mute --- */
    case USBD_AC_ID_UNIT_SpeakerControl + USB_AC_FU_MUTE_CONTROL:
        if (pReqInfo->ChannelNumber == 0) {
            pBuffer[0] = ac_global.SpeakerMute[1];
            return 1;
        }
        if (pReqInfo->ChannelNumber <= 1) {
            pBuffer[0] = ac_global.SpeakerMute[pReqInfo->ChannelNumber];
            return 1;
        }
        break;

    /* --- Microphone Feature Unit: mute --- */
    case USBD_AC_ID_UNIT_MicControl + USB_AC_FU_MUTE_CONTROL:
        pBuffer[0] = ac_global.MicrophoneMute;
        return 1;
    }
    return -1;
}

/*******************************************************************************
* Function Name: audio_control_set_cb
********************************************************************************
* Summary:
*  USBD_AC SET callback. Applies a host-issued audio control request. Volume
*  changes are forwarded to the codec; sampling-frequency and mute requests are
*  cached. Unsupported requests return an error.
*
* Parameters:
*  pReqInfo : Decoded audio control request (target ID and request type).
*  NumBytes : Number of valid bytes in pBuffer.
*  pBuffer  : Input buffer holding the new value (LE encoded).
*
* Return:
*  0 on success, or -1 if the request is unsupported.
*******************************************************************************/
static int audio_control_set_cb(const USBD_AC_CONTROL_INFO *pReqInfo, U32 NumBytes, const U8 *pBuffer)
{
    switch (pReqInfo->ID) {
    case USBD_AC_ID_EP_Speaker + USB_AC_SAMPLING_FREQ_CONTROL:
        return 0;

    case USBD_AC_ID_EP_Microphone + USB_AC_SAMPLING_FREQ_CONTROL:
        return 0;

    case USBD_AC_ID_UNIT_SpeakerControl + USB_AC_FU_VOLUME_CONTROL:
        if (pReqInfo->bRequest == USB_AC_REQ_CUR && NumBytes == 2) {
            memcpy(usb_comm_cur_volume, pBuffer, sizeof(usb_comm_cur_volume));
            ac_global.SpeakerVolume = USBD_GetU16LE(pBuffer);
            audio_app_update_codec_volume();
        }
        return 0;

    case USBD_AC_ID_UNIT_MicControl + USB_AC_FU_VOLUME_CONTROL:
        if (pReqInfo->bRequest == USB_AC_REQ_CUR && NumBytes == 2) {
            ac_global.MicrophoneVolume = USBD_GetU16LE(pBuffer);
        }
        return 0;

    case USBD_AC_ID_UNIT_SpeakerControl + USB_AC_FU_MUTE_CONTROL:
        if (pReqInfo->ChannelNumber <= 1) {
            ac_global.SpeakerMute[pReqInfo->ChannelNumber] = *pBuffer;
            return 0;
        }
        break;

    case USBD_AC_ID_UNIT_MicControl + USB_AC_FU_MUTE_CONTROL:
        ac_global.MicrophoneMute = *pBuffer;
        return 0;
    }
    return -1;
}
#endif /* USBD_AC_AUDIO_VERSION == 1 */


/* ========================================================================== */
/*  USB class init                                                            */
/* ========================================================================== */

/*******************************************************************************
* Function Name: audio_class_init
********************************************************************************
* Summary:
*  Registers the USB Audio Class with the stack (control GET/SET and set-
*  alternate callbacks) and creates the static message queue used to pass
*  stream events from the USB ISR callbacks to the audio application task.
*******************************************************************************/
static void audio_class_init(void)
{
    USBD_AC_INIT_DATA init_data;

    USB_MEMSET(&init_data, 0, sizeof(init_data));
    init_data.pACConfig      = USB_AC_CONFIGURATION;
    init_data.pfControlGet   = audio_control_get_cb;
    init_data.pfControlSet   = audio_control_set_cb;
    init_data.pfSetAlternate = audio_set_alt_interface_cb;

    if (USBD_AC_Add(&init_data) != 0) {
        printf("APP_LOG: USBD_AC_Add() failed!\r\n");
        handle_app_error();
    }

    /* Create the message queue */
    msg_queue = xQueueCreateStatic(MSG_QUEUE_DEPTH, sizeof(message_t),
                                   (uint8_t *)msg_queue_storage, &msg_queue_static);
}

/*******************************************************************************
* Function Name: add_hid_control
********************************************************************************
* Summary:
*  Adds a HID interface with a single interrupt IN endpoint used to report
*  media/volume control events to the host.
*
* Return:
*  Handle to the created HID instance.
*******************************************************************************/
static USB_HID_HANDLE add_hid_control(void)
{
    USB_HID_HANDLE hInst;
    USB_ADD_EP_INFO EPIntIn;

    memset(&hid_init_data, 0, sizeof(hid_init_data));
    EPIntIn.Flags           = 0;
    EPIntIn.InDir           = USB_DIR_IN;
    EPIntIn.Interval        = 8;
    EPIntIn.MaxPacketSize   = 2;
    EPIntIn.TransferType    = USB_TRANSFER_TYPE_INT;
    hid_init_data.EPIn      = USBD_AddEPEx(&EPIntIn, NULL, 0);

    hid_init_data.pReport = hid_report;
    hid_init_data.NumBytesReport = sizeof(hid_report);
    hInst = USBD_HID_Add(&hid_init_data);

    return hInst;
}


/* ========================================================================== */
/*  Hardware init (codec, clock, buttons)                                     */
/* ========================================================================== */

/*******************************************************************************
* Function Name: audio_codec_init
********************************************************************************
* Summary:
*  Initializes the I2C master, brings up the TLV320DAC3100 audio codec, and
*  configures its clocking for the selected sample rate before activating the
*  speaker output at the default volume.
*******************************************************************************/
void audio_codec_init(void)
{
    cy_en_scb_i2c_status_t result;
    cy_rslt_t hal_result;

    result = Cy_SCB_I2C_Init(CYBSP_I2C_CONTROLLER_HW,
                             &CYBSP_I2C_CONTROLLER_config,
                             &MW_CYBSP_I2C_CONTROLLER_context);
    if(result != CY_SCB_I2C_SUCCESS) { handle_app_error(); }

    Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_HW);

    hal_result = mtb_hal_i2c_setup(&MW_I2C_hal_obj,
                                   &CYBSP_I2C_CONTROLLER_hal_config,
                                   &MW_CYBSP_I2C_CONTROLLER_context, NULL);
    if(CY_RSLT_SUCCESS != hal_result) { handle_app_error(); }

    hal_result = mtb_hal_i2c_configure(&MW_I2C_hal_obj, &i2c_config);
    if(CY_RSLT_SUCCESS != hal_result) { handle_app_error(); }

    mtb_tlv320dac3100_init(&MW_I2C_hal_obj);
    mtb_tlv320dac3100_configure_clocking(MCLK_HZ, CODEC_SAMPLE_RATE_HZ,
                                         I2S_WORD_LENGTH, TLV320DAC3100_SPK_AUDIO_OUTPUT);
    mtb_tlv320dac3100_activate();
    mtb_tlv320dac3100_adjust_speaker_output_volume(SPEAKER_DEFAULT_VOL);
}

/*******************************************************************************
* Function Name: audio_app_update_codec_volume
********************************************************************************
* Summary:
*  Converts the latest USB volume control value into a codec volume setting and
*  applies it to the speaker output, but only when the value has changed.
*******************************************************************************/
static void audio_app_update_codec_volume(void)
{
    int16_t vol_usb = ((int16_t)usb_comm_cur_volume[1]) * 256 +
                      ((int16_t)usb_comm_cur_volume[0]);

    if (vol_usb != MAX_VOL_ROLLOVER)
    {
        audio_app_volume = (uint8_t)(vol_usb);
        if (audio_app_volume != audio_app_prev_volume)
        {
            mtb_tlv320dac3100_adjust_speaker_output_volume(audio_app_volume);
            audio_app_prev_volume = audio_app_volume;
        }
    }
}

/*******************************************************************************
* Function Name: app_clock_init
********************************************************************************
* Summary:
*  Configures the application clock tree: locks the low-power DPLL to the
*  required frequency, routes it to CLK_HF7, and programs the fractional
*  dividers that generate the PDM (microphone) and TDM/I2S (speaker) clocks.
*******************************************************************************/
void app_clock_init(void)
{
    uint32_t source_freq;

    source_freq = Cy_SysClk_ClkPathMuxGetFrequency(SRSS_DPLL_LP_1_PATH_NUM);

    cy_stc_pll_config_t pll_config =
    {
        .inputFreq = source_freq,
        .outputFreq = DPLL_LP_FREQ,
        .lfMode = true,
        .outputMode = CY_SYSCLK_FLLPLL_OUTPUT_OUTPUT,
    };

    if(CY_SYSCLK_SUCCESS == Cy_SysClk_PllDisable(SRSS_DPLL_LP_1_PATH_NUM))
    {
        if(CY_SYSCLK_SUCCESS != Cy_SysClk_DpllLpConfigure(SRSS_DPLL_LP_1_PATH_NUM, &pll_config))
            handle_app_error();
        if(CY_SYSCLK_SUCCESS != Cy_SysClk_DpllLpEnable(SRSS_DPLL_LP_1_PATH_NUM, DPLL_DELAY_MS))
            handle_app_error();
        if(!Cy_SysClk_DpllLpLocked(SRSS_DPLL_LP_1_PATH_NUM))
            handle_app_error();
    }

    if(CY_SYSCLK_SUCCESS == Cy_SysClk_ClkHfDisable(CY_CFG_SYSCLK_CLKHF7))
    {
        if(CY_SYSCLK_SUCCESS != Cy_SysClk_ClkHfSetSource(CY_CFG_SYSCLK_CLKHF7, CY_SYSCLK_CLKHF_IN_CLKPATH1))
            handle_app_error();
        if(CY_SYSCLK_SUCCESS != Cy_SysClk_ClkHfEnable(CY_CFG_SYSCLK_CLKHF7))
            handle_app_error();
    }

    if(CY_SYSCLK_SUCCESS == Cy_SysClk_PeriPclkDisableDivider((en_clk_dst_t)CYBSP_TDM_CONTROLLER_0_CLK_DIV_GRP_NUM,
                                                             CY_SYSCLK_DIV_16_5_BIT, 0U))
    {
        if(CY_SYSCLK_SUCCESS != Cy_SysClk_PeriPclkSetFracDivider((en_clk_dst_t)CYBSP_TDM_CONTROLLER_0_CLK_DIV_GRP_NUM,
                                                                 CY_SYSCLK_DIV_16_5_BIT, 0U, TDM_CLK_DIV_INT, 0U))
            handle_app_error();
        if(CY_SYSCLK_SUCCESS != Cy_SysClk_PeriPclkEnableDivider((en_clk_dst_t)CYBSP_TDM_CONTROLLER_0_CLK_DIV_GRP_NUM,
                                                                CY_SYSCLK_DIV_16_5_BIT, 0U))
            handle_app_error();
    }

}


/* ========================================================================== */
/*  Audio App init & task                                                     */
/* ========================================================================== */

/*******************************************************************************
* Function Name: audio_app_init
********************************************************************************
* Summary:
*  Initializes the audio application: locks deep sleep, brings up the clocks and
*  codec, configures the user-button GPIO interrupt, and creates the audio
*  application RTOS task.
*******************************************************************************/
void audio_app_init(void)
{
    BaseType_t rtos_task_status;
    cy_en_sysint_status_t int_status;

    cy_stc_sysint_t gpio_int_config =
    {
        .intrSrc = ioss_interrupts_gpio_8_IRQn,
        .intrPriority = BTN_IRQ_PRIORITY,
    };

    mtb_hal_syspm_lock_deepsleep();
    app_clock_init();
    audio_codec_init();

    int_status = Cy_SysInt_Init(&gpio_int_config, gpio_isr_handler);
    if(CY_SYSINT_SUCCESS != int_status) {
        printf("Interrupt initialization failed\r\n");
    }
    NVIC_EnableIRQ(gpio_int_config.intrSrc);

    rtos_task_status = xTaskCreate(audio_app_task, "Audio App Task",
                                   AUDIO_TASK_STACK_DEPTH, NULL,
                                   AUDIO_APP_TASK_PRIORITY, &rtos_audio_app_task);
    if (pdPASS != rtos_task_status) { handle_app_error(); }
}


/*******************************************************************************
* Function Name: audio_app_task
********************************************************************************
* Summary:
*  Main audio application RTOS task. Initializes the USB device and audio
*  hardware, starts the USB stack, and then services stream events from the
*  message queue: routing speaker (OUT) data to the codec, generating adaptive
*  microphone (IN) packets from the PDM FIFO, and handling volume/mute changes.
*
* Parameters:
*  arg : Unused FreeRTOS task argument.
*
* Return:
*  void  (does not return)
*
*******************************************************************************/
void audio_app_task(void *arg)
{
    uint8_t usb_status = USB_SUSPENDED;
    I8 speaker_active = 0;
    I8 mic_active = 0;
    message_t msg;
    
    CY_UNUSED_PARAMETER(arg);

    /* ---- USB init ---- */
    USBD_Init();
    USBD_EnableIAD();
    audio_class_init();
    usb_hid_control_context = add_hid_control();
    USBD_SetDeviceInfo(&usb_device_info);

    /* ---- Hardware init ---- */
    pdm_pcm_init_adv(AUDIO_IN_NUM_CHANNELS, 
                     AUDIO_BIT_RESOLUTION,
                     AUDIO_SAMPLE_FREQ,
                     PDM_PCM_OVERSAMPLE_RATE,
                     PDM_PCM_CFG_NONBLOCKING | PDM_PCM_CFG_INTERLEAVED);

    pdm_pcm_set_hw_gain(PDM_PCM_HW_GAIN_DB);
    pdm_pcm_set_soft_gain(PDM_PCM_SW_GAIN_DB);
    

    audio_out_init();

    /* ---- Start USB ---- */
    USBD_Start();
    audio_app_update_codec_volume();

    for (;;)
    {
        /* ---- Wait for USB enumeration ---- */
        USB_MEMSET(&ac_global, 0, sizeof(ac_global));
        while (USB_STAT_CONFIGURED != (USBD_GetState() & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)))
        {
            Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
            usb_status = USB_SUSPENDED;
            USB_OS_Delay(USB_CONFIG_DELAY);
        }

        if (USB_SUSPENDED == usb_status) {
            usb_status = USB_CONNECTED;
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, 0U);
            printf("APP_LOG: USB Audio Device Connected\r\n");
        }

        /* ---- Process messages while configured ---- */
        while ((USBD_GetState() & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)) == USB_STAT_CONFIGURED)
        {
            if (xQueueReceive(msg_queue, &msg, pdMS_TO_TICKS(DELAY_TICKS_MS)) == pdTRUE)
            {
                switch (msg.Event)
                {
                /* ---- Speaker stream lifecycle ---- */
                case MSG_SPEAKER_ON:
                    if (speaker_active) {
                        USBD_AC_CloseRXStream(&rx_ctx);
                        Cy_AudioTDM_DeActivateTx(TDM_STRUCT0_TX);
                        speaker_active = 0;
                    }

                    /* Reset the TDM TX block so the FIFO starts empty.
                     * A Disable->Enable cycle clears the FIFO pointers,
                     * discarding any stale data from a previous session. */
                    Cy_AudioTDM_DeActivateTx(TDM_STRUCT0_TX);
                    Cy_AudioTDM_DisableTx(TDM_STRUCT0_TX);
                    Cy_AudioTDM_EnableTx(TDM_STRUCT0_TX);
                    Cy_AudioTDM_ActivateTx(TDM_STRUCT0_TX);

                    memset(&rx_ctx, 0, sizeof(rx_ctx));
                    speaker_buf_idx            = 0;
                    rx_ctx.Interface           = USBD_AC_INTERFACE_Speaker;
                    rx_ctx.RxData.pBuffer      = speaker_audio_buffer_0;
                    rx_ctx.RxData.NumBytes     = sizeof(speaker_audio_buffer_0);
                    rx_ctx.RxData.NumPackets   = 0;
                    rx_ctx.RxData.Timeout      = 5000;
                    rx_ctx.pfCallback          = audio_out_rx_callback;
                    rx_ctx.pfSOFCallback       = audio_feedback_sof_callback;
                    rx_ctx.FeedbackInterval    = 10;

                    if (USBD_AC_OpenRXStream(&rx_ctx) == 0) {
                        speaker_active = 1;
#if AUDIO_FEEDBACK_DIAG_ENABLE
                        /* Reset diagnostics on new stream */
                        fb_sof_count = 0;
                        fb_slow_count = 0;
                        fb_fast_count = 0;
                        fb_fifo_sum = 0;
                        fb_fifo_min = AUDIO_OUT_TDM_FIFO_DEPTH_WORDS;
                        fb_fifo_max = 0;
                        fb_overflow_drop_count = 0;
                        fb_underflow_count = 0;
                        fb_data_msg_count = 0;
#endif
                    }
                    break;

                case MSG_SPEAKER_OFF:
                    if (speaker_active) {
                        USBD_AC_CloseRXStream(&rx_ctx);
                        Cy_AudioTDM_DeActivateTx(TDM_STRUCT0_TX);
                        speaker_active = 0;
                    }
                    break;

                case MSG_SPEAKER_DATA:
                    if (msg.NumBytes > 0 && speaker_active) {
                        unsigned data_to_write = msg.NumBytes / AUDIO_OUT_SUB_FRAME_SIZE;
                        uint16_t *samples = (uint16_t *)msg.pBuff;

#if AUDIO_FEEDBACK_DIAG_ENABLE
                        fb_data_msg_count++;
                        {
                            uint32_t fifo_used = Cy_AudioTDM_GetNumInTxFifo(TDM_STRUCT0_TX);

                            /* Detect overflow: FIFO is at capacity */
                            if (fifo_used >= AUDIO_OUT_TDM_FIFO_DEPTH_WORDS) {
                                fb_overflow_drop_count++;
                            }

                            /* Detect underflow: FIFO ran empty while streaming */
                            if (fifo_used == 0U && fb_data_msg_count > 1) {
                                fb_underflow_count++;
                            }
                        }
#endif /* AUDIO_FEEDBACK_DIAG_ENABLE */

                        /* Write samples to the TDM FIFO.  Each mono sample is
                         * duplicated to L+R.  If the FIFO is full, WriteTxData
                         * is a safe no-op (hardware silently ignores the write).
                         * The feedback EP adjusts the USB host rate to keep the
                         * FIFO near its target level long-term. */
                        for (unsigned i = 0; i < data_to_write; i++) {
                            Cy_AudioTDM_WriteTxData(TDM_STRUCT0_TX, (uint32_t)samples[i]);
                            Cy_AudioTDM_WriteTxData(TDM_STRUCT0_TX, (uint32_t)samples[i]);
                        }

#if AUDIO_FEEDBACK_DIAG_ENABLE
                        /* Periodic diagnostic snapshot - every AUDIO_FEEDBACK_DIAG_INTERVAL SOFs.
                         * Stats are reset each interval for a clear per-window view. */
                        {
                            static uint32_t last_print_sof;
                            uint32_t sof_now = fb_sof_count;
                            if (sof_now - last_print_sof >= AUDIO_FEEDBACK_DIAG_INTERVAL) {
                                uint32_t interval_sofs = sof_now - last_print_sof;
                                uint32_t avg = (interval_sofs > 0)
                                               ? (fb_fifo_sum / interval_sofs) : 0;
                                printf("[FB-DIAG] SOFs=%lu  FIFO avg=%lu min=%lu max=%lu  "
                                       "adj(+/-)=%lu/%lu  OVF=%lu UNF=%lu\r\n",
                                       (unsigned long)interval_sofs,
                                       (unsigned long)avg,
                                       (unsigned long)fb_fifo_min,
                                       (unsigned long)fb_fifo_max,
                                       (unsigned long)fb_fast_count,
                                       (unsigned long)fb_slow_count,
                                       (unsigned long)fb_overflow_drop_count,
                                       (unsigned long)fb_underflow_count);

                                /* Reset accumulators for the next interval */
                                last_print_sof = sof_now;
                                fb_fifo_sum = 0;
                                fb_fifo_min = AUDIO_OUT_TDM_FIFO_DEPTH_WORDS;
                                fb_fifo_max = 0;
                                fb_fast_count = 0;
                                fb_slow_count = 0;
                                fb_overflow_drop_count = 0;
                                fb_underflow_count = 0;
                                fb_data_msg_count = 0;
                            }
                        }
#endif /* AUDIO_FEEDBACK_DIAG_ENABLE */
                    }
                    break;

                /* ---- Microphone stream lifecycle ---- */
                case MSG_MIC_ON:
                    if (mic_active) {
                        USBD_AC_CloseTXStream(&tx_ctx);
                        mic_active = 0;
                    }

                    /* Activate PDM channels */
                    pdm_pcm_start();

                    /* Open TX stream */
                    memset(&tx_ctx, 0, sizeof(tx_ctx));
                    tx_ctx.Interface   = USBD_AC_INTERFACE_Microphone;
                    tx_ctx.Timeout     = 5000;
                    tx_ctx.pfCallback  = audio_in_tx_callback;

                    if (USBD_AC_OpenTXStream(&tx_ctx) == 0) {
                        memset(mic_audio_buffer, 0, sizeof(mic_audio_buffer));
                        mic_active = 1;
#if AUDIO_FEEDBACK_DIAG_ENABLE
                        /* Reset IN-path diagnostics on new stream */
                        mic_data_msg_count = 0;
                        mic_fast_count = 0;
                        mic_slow_count = 0;
                        mic_fifo_sum = 0;
                        mic_fifo_min = AUDIO_IN_PDM_LEVEL_DEPTH_WORDS;
                        mic_fifo_max = 0;
                        mic_overflow_count = 0;
                        mic_underflow_count = 0;
#endif
                        USBD_AC_Send(&tx_ctx, 1, MAX_AUDIO_IN_PACKET_SIZE_BYTES, mic_audio_buffer);
                        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_ON);
                    }
                    break;

                case MSG_MIC_OFF:
                    if (mic_active) {
                        USBD_AC_CloseTXStream(&tx_ctx);
                        mic_active = 0;

                        pdm_pcm_stop();

                        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_OFF);
                    }
                    break;

                case MSG_MIC_DATA:
                {
                    /* Adapt the number of frames sent this USB frame based on the
                     * PDM ring buffer level to compensate for drift between the PDM
                     * capture clock and the USB SOF.  Mirrors the speaker feedback
                     * EP, but for the synchronous IN path:
                     *   Level above HIGH -> capture faster than USB -> send +1 frame
                     *   Level below LOW  -> capture slower than USB -> send -1 frame
                     * The count is finally clamped to what the ring buffer actually holds
                     * so an empty ring buffer is never popped. */
                    uint32_t ring_buffer_level;
                    uint16_t nominal    = AUDIO_IN_NOMINAL_WORDS_PER_CH;
                    uint16_t frames     = nominal;

                    pdm_pcm_get_buffered_sample_count(&ring_buffer_level);           
                    
                    if (ring_buffer_level > AUDIO_IN_PDM_LEVEL_HIGH_MARK_WORDS) {
                        frames = nominal + AUDIO_IN_FEEDBACK_HEADROOM_SAMPLES;
#if AUDIO_FEEDBACK_DIAG_ENABLE
                        mic_fast_count++;
#endif
                    } else if (ring_buffer_level < AUDIO_IN_PDM_LEVEL_LOW_MARK_WORDS) {
                        frames = nominal - AUDIO_IN_FEEDBACK_HEADROOM_SAMPLES;
#if AUDIO_FEEDBACK_DIAG_ENABLE
                        mic_slow_count++;
#endif
                    }

#if AUDIO_FEEDBACK_DIAG_ENABLE
                    /* Requested frame count before the FIFO-availability clamp */
                    uint16_t target = frames;
#endif

                    /* Never read more frames than the FIFO currently holds */
                    if (frames > ring_buffer_level) {
                        frames = (uint16_t) ring_buffer_level;
                    }

#if AUDIO_FEEDBACK_DIAG_ENABLE
                    mic_data_msg_count++;
                    mic_fifo_sum += ring_buffer_level;
                    if (ring_buffer_level < mic_fifo_min) { mic_fifo_min = ring_buffer_level; }
                    if (ring_buffer_level > mic_fifo_max) { mic_fifo_max = ring_buffer_level; }
                    if (ring_buffer_level >= (AUDIO_IN_PDM_LEVEL_DEPTH_WORDS - 1U)) { mic_overflow_count++; }
                    if (frames < target) { mic_underflow_count++; }
#endif /* AUDIO_FEEDBACK_DIAG_ENABLE */

                    pdm_pcm_read(mic_audio_buffer, frames);

                    USBD_AC_Send(&tx_ctx, 1, frames * AUDIO_IN_NUM_CHANNELS * AUDIO_IN_SUB_FRAME_SIZE, mic_audio_buffer);

#if AUDIO_FEEDBACK_DIAG_ENABLE
                    /* Periodic IN-path diagnostic snapshot - paced by the mic
                     * stream itself (one MIC_DATA msg ~= 1 ms), so it prints
                     * whenever (and only when) the mic stream is active.
                     * Stats are reset each interval for a per-window view. */
                    if (mic_data_msg_count >= AUDIO_FEEDBACK_DIAG_INTERVAL) {
                        uint32_t mic_avg = mic_fifo_sum / mic_data_msg_count;
                        printf("[IN-DIAG] msgs=%lu  FIFO avg=%lu min=%lu max=%lu  "
                               "adj(+/-)=%lu/%lu  OVF=%lu UNF=%lu\r\n",
                               (unsigned long)mic_data_msg_count,
                               (unsigned long)mic_avg,
                               (unsigned long)mic_fifo_min,
                               (unsigned long)mic_fifo_max,
                               (unsigned long)mic_fast_count,
                               (unsigned long)mic_slow_count,
                               (unsigned long)mic_overflow_count,
                               (unsigned long)mic_underflow_count);
                        /* Reset accumulators for the next interval */
                        mic_data_msg_count = 0;
                        mic_fast_count = 0;
                        mic_slow_count = 0;
                        mic_fifo_sum = 0;
                        mic_fifo_min = AUDIO_IN_PDM_LEVEL_DEPTH_WORDS;
                        mic_fifo_max = 0;
                        mic_overflow_count = 0;
                        mic_underflow_count = 0;
                    }
#endif /* AUDIO_FEEDBACK_DIAG_ENABLE */
                    break;
                }

                default:
                    break;
                }
            }

            /* ---- Button handling (HID volume up/down) ---- */
            if (button_status != SWITCH_NO_EVENT)
            {
                switch (button_status)
                {
                case SWITCH_VOLUME_INCR:
                    button_status = SWITCH_NO_EVENT;
                    audio_app_control_report[0] = AUDIO_HID_REPORT_ID0;
                    audio_app_control_report[1] = AUDIO_HID_REPORT_VOLUME_UP;
                    USBD_HID_Write(usb_hid_control_context, audio_app_control_report, 2, 0);
                    audio_app_control_report[1] = AUDIO_HID_REPORT_ID1;
                    USBD_HID_Write(usb_hid_control_context, audio_app_control_report, 2, 0);
                    audio_app_update_codec_volume();
                    break;

                case SWITCH_VOLUME_DECR:
                    button_status = SWITCH_NO_EVENT;
                    audio_app_control_report[0] = AUDIO_HID_REPORT_ID0;
                    audio_app_control_report[1] = AUDIO_HID_REPORT_VOLUME_DOWN;
                    USBD_HID_Write(usb_hid_control_context, audio_app_control_report, 2, 0);
                    audio_app_control_report[1] = AUDIO_HID_REPORT_ID1;
                    USBD_HID_Write(usb_hid_control_context, audio_app_control_report, 2, 0);
                    audio_app_update_codec_volume();
                    break;

                default:
                    break;
                }
            }
        }

        /* USB disconnected/suspended - close active streams */
        if (speaker_active) {
            USBD_AC_CloseRXStream(&rx_ctx);
            Cy_AudioTDM_DeActivateTx(TDM_STRUCT0_TX);
            speaker_active = 0;
        }
        if (mic_active)     { USBD_AC_CloseTXStream(&tx_ctx); mic_active = 0; }
    }
}

/* [] END OF FILE */

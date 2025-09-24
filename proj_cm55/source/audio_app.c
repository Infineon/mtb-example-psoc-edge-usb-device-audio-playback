/******************************************************************************
* File Name   : audio_app.c
*
* Description : This file contains the implementation of adding audio interface
*               and main audio process task.
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

/******************************************************************************
* Headers
*******************************************************************************/
#include "audio_app.h"
#include "audio_in.h"
#include "audio_out.h"
#include "audio.h"
#include "emusbdev_audio_config.h"
#include "USB_HID.h"
#include "rtos.h"
#include "mtb_tlv320dac3100.h"
#include "retarget_io_init.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Polling interval for the endpoint */
#define EP_IN_INTERVAL               (8u)
#define EP_OUT_INTERVAL              (8u)

#define USB_CONFIG_DELAY             (50u)
#define ONE_BYTE                     (1u)
#define THREE_BYTES                  (3u)
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

/* MLCK Value for 16KHz playback */
#define MCLK_HZ                      (2048000)

/* I2S word length parameter */
#define I2S_WORD_LENGTH              (TLV320DAC3100_I2S_WORD_SIZE_16)

#define RESET_VAL                    (0u)
#define USB_DEFAULT_RET_VAL          (1u)
#define USB_NUM_BYTES_TWO            (2u)
#define USB_NUM_BYTES_THREE          (3u)   

#if ((AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_16KHZ) || \
     (AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_32KHZ) || \
     (AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_48KHZ))
    #define DPLL_LP_FREQ                 (49152000ul)
    #define PDM_CLK_DIV_INT              (3u)
#elif((AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_22KHZ) || \
      (AUDIO_IN_SAMPLE_FREQ == AUDIO_SAMPLING_RATE_44KHZ))
    #define DPLL_LP_FREQ                 (45158400ul)
    #define PDM_CLK_DIV_INT              (3u)
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

#define DPLL_DELAY_MS                (2000ul)
/*******************************************************************************
* Function prototypes
********************************************************************************/


/*******************************************************************************
* Global Variables
********************************************************************************/
/* RTOS task handles */
TaskHandle_t rtos_audio_app_task;
TaskHandle_t rtos_audio_in_task;

USBD_AUDIO_HANDLE usb_audio_context;
static USBD_AUDIO_INIT_DATA init_data;

USB_HID_HANDLE    usb_hid_control_context;
static USB_HID_INIT_DATA    hid_init_data;

/* Microphone configurations */
static USBD_AUDIO_IF_CONF *microphone_config = (USBD_AUDIO_IF_CONF *) &audio_interfaces[1];
static uint8_t              current_microphone_format_index;

/* Speaker configurations */
static USBD_AUDIO_IF_CONF *speaker_config = (USBD_AUDIO_IF_CONF *) &audio_interfaces[0];
static uint8_t              current_speaker_format_index;

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

/* Audio playback tracking variables */
uint32_t i2s_txcount = RESET_VAL;
volatile bool i2s_flag = false;

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


/*******************************************************************************
* Function Name: gpio_isr_handler
********************************************************************************
* Summary:
*  Interrupt serive routine for USER BUTTON 1 and 2.
*  This function checks which user button was pressed and set the
*  corresponding button_status flag.
*
* Parameter:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void gpio_isr_handler(void)
{
    /* Check if USER_BTN1 was pressed */
    if(Cy_GPIO_GetInterruptStatus(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN))
    {
        button_status = SWITCH_VOLUME_INCR;
        /* Clear the USER_BTN1 interrupt */
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN);
    }

    /* Check if USER_BTN2 was pressed */
    if(Cy_GPIO_GetInterruptStatus(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_PIN))
    {
        button_status = SWITCH_VOLUME_DECR;
        /* Clear the USER_BTN2 interrupt */
        Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_PIN);
    }
}


/*******************************************************************************
* Function Name: audio_control_callback
********************************************************************************
* Summary:
*  Callback called in ISR context.
*  Receives audio class control commands and sends appropriate responses
*  where necessary.
*
* Parameters:
*  user_context: User context which is passed to the callback.
*  event: Audio event ID.
*  unit: ID of the feature unit. In case of USB_AUDIO_PLAYBACK_*
*        and USB_AUDIO_RECORD_*: 0.
*  control_selector: ID of the control. In case of USB_AUDIO_PLAYBACK_* and
*                   USB_AUDIO_RECORD_*: 0.
*  buffer_pointer: In case of GET events: pointer to a buffer into which the
*           callback should write the reply. In case of SET events: pointer
*           to a buffer containing the command value. In case of
*           USB_AUDIO_PLAYBACK_* and USB_AUDIO_RECORD_*: NULL.
*  num_bytes: In case of GET events: requested size of the reply in bytes.
*            In case of SET events: number of bytes in buffer_pointer. In case
*            of USB_AUDIO_PLAYBACK_* and USB_AUDIO_RECORD_*: 0.
*  interface_num: The number of the USB interface for which the event was issued.
*  alt_setting: The alternative setting number of the USB interface for
*              which the event was issued.
*
* Return:
*  int: =0 Audio command was handled by the callback, ≠ 0 Audio command was
*       not handled by the callback. The stack will STALL the request.
*
*******************************************************************************/
static int audio_control_callback(void *user_context, U8 event, 
                                  U8 unit, U8 control_selector, 
                                  U8 *buffer_pointer, U32 num_bytes, 
                                  U8 interface_num, U8 alt_setting)
{
    int ret_val;

    (void)user_context;
    (void)interface_num;
    ret_val = RESET_VAL;

    switch (event) {

        case USB_AUDIO_PLAYBACK_START:

            /* Host enabled transmission */
            audio_out_enable();
            break;

        case USB_AUDIO_PLAYBACK_STOP:

            /* Host disabled transmission. Some hosts do not always send this! */
            audio_out_disable();
            break;

        case USB_AUDIO_RECORD_START:

            /* Host enabled reception */
            audio_in_enable();
            break;

        case USB_AUDIO_RECORD_STOP:

            /* Host disabled reception. Some hosts do not always send this! */
            audio_in_disable();
            break;

        case USB_AUDIO_SET_CUR:

            switch (control_selector)
            {

                case USB_AUDIO_MUTE_CONTROL:
                    break;

                case USB_AUDIO_VOLUME_CONTROL:
                    if(USB_NUM_BYTES_TWO == num_bytes)
                    {
                        memcpy(usb_comm_cur_volume, buffer_pointer, sizeof(usb_comm_cur_volume));
                    }
                    break;

                case USB_AUDIO_SAMPLING_FREQ_CONTROL:

                if (USB_NUM_BYTES_THREE == num_bytes)
                {
                    if (unit == speaker_config->pUnits->FeatureUnitID)
                    {
                        if ((alt_setting > RESET_VAL) && (alt_setting < speaker_config->NumFormats))
                        {
                            current_speaker_format_index = alt_setting - 1;
                        }
                    }
                    if (unit == microphone_config->pUnits->FeatureUnitID) 
                    {
                        if ((alt_setting > RESET_VAL) && (alt_setting < microphone_config->NumFormats))
                        {
                            current_microphone_format_index = alt_setting - 1;
                        }
                    }
                }

                    break;

                default:
                    ret_val = USB_DEFAULT_RET_VAL;
                    break;
            }

            break;

        case USB_AUDIO_GET_CUR:
            switch (control_selector)
            {
                case USB_AUDIO_MUTE_CONTROL:
                    buffer_pointer[0] = RESET_VAL;
                    break;

                case USB_AUDIO_VOLUME_CONTROL:
                    buffer_pointer[0] = RESET_VAL;
                    buffer_pointer[1] = RESET_VAL;
                    break;

                case USB_AUDIO_SAMPLING_FREQ_CONTROL:
                    if (unit == speaker_config->pUnits->FeatureUnitID)
                    {
                        buffer_pointer[0] =  speaker_config->paFormats[current_speaker_format_index].SamFreq & 0xff;
                        buffer_pointer[1] = (speaker_config->paFormats[current_speaker_format_index].SamFreq >> 8) & 0xff;
                        buffer_pointer[2] = (speaker_config->paFormats[current_speaker_format_index].SamFreq >> 16) & 0xff;
                    }

                    if (unit == microphone_config->pUnits->FeatureUnitID)
                    {
                        buffer_pointer[0] =  microphone_config->paFormats[current_microphone_format_index].SamFreq & 0xff;
                        buffer_pointer[1] = (microphone_config->paFormats[current_microphone_format_index].SamFreq >> 8) & 0xff;
                        buffer_pointer[2] = (microphone_config->paFormats[current_microphone_format_index].SamFreq >> 16) & 0xff;
                    }
                    break;

                default:
                    buffer_pointer[0] = RESET_VAL;
                    buffer_pointer[1] = RESET_VAL;
                    break;
            }
            break;

        case USB_AUDIO_SET_MIN:
            break;

        case USB_AUDIO_GET_MIN:
            switch (control_selector)
            {
                case USB_AUDIO_VOLUME_CONTROL:
                    buffer_pointer[0] = usb_comm_min_volume[0];
                    buffer_pointer[1] = usb_comm_min_volume[1];
                    break;

                default:
                    buffer_pointer[0] = RESET_VAL;
                    buffer_pointer[1] = RESET_VAL;
                    break;
            }
            break;

        case USB_AUDIO_SET_MAX:
            break;

        case USB_AUDIO_GET_MAX:
            switch (control_selector)
            {
                case USB_AUDIO_VOLUME_CONTROL:
                    buffer_pointer[0] = usb_comm_max_volume[0];
                    buffer_pointer[1] = usb_comm_max_volume[1];
                    break;

                default:
                    buffer_pointer[0] = RESET_VAL;
                    buffer_pointer[1] = RESET_VAL;
                    break;
            }
            break;

        case USB_AUDIO_SET_RES:
            break;

        case USB_AUDIO_GET_RES:
            switch (control_selector)
            {
                case USB_AUDIO_VOLUME_CONTROL:
                    buffer_pointer[0] = usb_comm_res_volume[0];
                    buffer_pointer[1] = usb_comm_res_volume[1];
                    break;

                default:
                    buffer_pointer[0] = RESET_VAL;
                    buffer_pointer[1] = RESET_VAL;
                    break;
            }

            break;

        default:
            ret_val = USB_DEFAULT_RET_VAL;
            break;
    }

    return ret_val;
}


/*******************************************************************************
* Function Name: add_audio
********************************************************************************
* Summary:
*  Add a USB Audio interface to the USB stack.
*
* Parameters:
*  None
*
* Return:
*  USBD_AUDIO_HANDLE
*
*******************************************************************************/
static USBD_AUDIO_HANDLE add_audio(void)
{
    USB_ADD_EP_INFO       ep_in;
    USB_ADD_EP_INFO       ep_out;
    USBD_AUDIO_HANDLE     handle;

    memset(&ep_in, 0x0, sizeof(ep_in));
    memset(&ep_out, 0x0, sizeof(ep_out));
    memset(&init_data, 0x0, sizeof(init_data));

    /* OUT endpoint configurations */
    ep_out.Flags                     = USB_ADD_EP_FLAG_USE_ISO_SYNC_TYPES;
    ep_out.InDir                     = USB_DIR_OUT;
    ep_out.Interval                  = EP_OUT_INTERVAL;
    ep_out.MaxPacketSize             = MAX_AUDIO_OUT_PACKET_SIZE_BYTES;
    ep_out.TransferType              = USB_TRANSFER_TYPE_ISO;
    ep_out.ISO_Type                  = USB_ISO_SYNC_TYPE_ASYNCHRONOUS;

    /* IN endpoint configurations */
    ep_in.MaxPacketSize              = MAX_AUDIO_IN_PACKET_SIZE_BYTES;       /* Max packet size for IN endpoint (in bytes) */
    ep_in.Interval                   = EP_IN_INTERVAL;                       /* Interval of 1 ms (8 * 125us) */
    ep_in.Flags                      = USB_ADD_EP_FLAG_USE_ISO_SYNC_TYPES;   /* Optional parameters */
    ep_in.InDir                      = USB_DIR_IN;                           /* IN direction (Device to Host) */
    ep_in.TransferType               = USB_TRANSFER_TYPE_ISO;                /* Endpoint type - Isochronous. */
    ep_in.ISO_Type                   = USB_ISO_SYNC_TYPE_ASYNCHRONOUS;       /* Async for isochronous endpoints */

    /* Initalization data for USB audio class */
    init_data.EPIn                   = USBD_AddEPEx(&ep_in, NULL, 0);
    init_data.EPOut                  = USBD_AddEPEx(&ep_out, NULL, 0);;
    init_data.OutPacketSize          = MAX_AUDIO_OUT_PACKET_SIZE_BYTES;
    init_data.pfOnOut                = &audio_out_endpoint_callback;
    init_data.pfOnIn                 = &audio_in_endpoint_callback;
    init_data.pfOnControl            = audio_control_callback;
    init_data.pControlUserContext    = NULL;
    init_data.NumInterfaces          = SEGGER_COUNTOF(audio_interfaces);
    init_data.paInterfaces           = audio_interfaces;
    init_data.pOutUserContext        = NULL;
    init_data.pInUserContext         = NULL;

    handle = USBD_AUDIO_Add(&init_data);

    return handle;
}


/*********************************************************************
* Function name: add_hid_control
**********************************************************************
* Summary:
*  Add HID mouse class to USB stack
*
* Parameters:
*  None
*
* Return:
*  USBD_AUDIO_HANDLE - Handle for added hid instance
**********************************************************************/
static USB_HID_HANDLE add_hid_control(void) {

    USB_HID_HANDLE hInst;

    USB_ADD_EP_INFO   EPIntIn;

    memset(&hid_init_data, 0, sizeof(hid_init_data));
    EPIntIn.Flags           = 0;                             // Flags not used.
    EPIntIn.InDir           = USB_DIR_IN;                    // IN direction (Device to Host)
    EPIntIn.Interval        = 8;                             // Interval of 1 ms (125 us * 8)
    EPIntIn.MaxPacketSize   = 2;                             // Report size.
    EPIntIn.TransferType    = USB_TRANSFER_TYPE_INT;         // Endpoint type - Interrupt.
    hid_init_data.EPIn      = USBD_AddEPEx(&EPIntIn, NULL, 0);

    hid_init_data.pReport = hid_report;
    hid_init_data.NumBytesReport = sizeof(hid_report);
    hInst = USBD_HID_Add(&hid_init_data);

    return hInst;
}


/*********************************************************************
* Function name: audio_codec_init
**********************************************************************
* Summary:
*  Initializes the TLV320dac3100 codec to output audio to the speaker.
*
* Parameters:
*  None
*
* Return:
*  None
**********************************************************************/
void audio_codec_init(void)
{
    cy_en_scb_i2c_status_t result;
    cy_rslt_t hal_result;

    /* Initialize and enable the I2C in controller mode. */
    result = Cy_SCB_I2C_Init(CYBSP_I2C_CONTROLLER_HW, 
                             &CYBSP_I2C_CONTROLLER_config, 
                             &MW_CYBSP_I2C_CONTROLLER_context);

    if(result != CY_SCB_I2C_SUCCESS)
    {
        handle_app_error();
    }

    /* Enable I2C hardware. */
    Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_HW);

    /* I2C HAL init */
    hal_result = mtb_hal_i2c_setup(&MW_I2C_hal_obj, 
                                   &CYBSP_I2C_CONTROLLER_hal_config, 
                                &MW_CYBSP_I2C_CONTROLLER_context, NULL);

    if(CY_RSLT_SUCCESS != hal_result)
    {
        handle_app_error();
    }

    /* Configure the I2C block. */
    hal_result = mtb_hal_i2c_configure(&MW_I2C_hal_obj, &i2c_config);

    if(CY_RSLT_SUCCESS != hal_result)
    {
        handle_app_error();
    }

    /* TLV codec/ MW init */
    mtb_tlv320dac3100_init(&MW_I2C_hal_obj);
    /* Configure internal clock dividers to achieve desired sample rate */
    mtb_tlv320dac3100_configure_clocking(MCLK_HZ, CODEC_SAMPLE_RATE_HZ, I2S_WORD_LENGTH, TLV320DAC3100_SPK_AUDIO_OUTPUT);
    /* Activate TLV320DAC3100 */
    mtb_tlv320dac3100_activate();

    mtb_tlv320dac3100_adjust_speaker_output_volume(SPEAKER_DEFAULT_VOL);
}


/*******************************************************************************
* Function Name: audio_app_update_codec_volume
********************************************************************************
* Summary:
*   Update the audio codec volume by sending an I2C message.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
static void audio_app_update_codec_volume(void)
{
    int16_t  vol_usb = ((int16_t) usb_comm_cur_volume[1])*256 +
                            ((int16_t) usb_comm_cur_volume[0]);

    if(vol_usb != MAX_VOL_ROLLOVER)
    {
        audio_app_volume = (uint8_t)(vol_usb);

        /* Check if the volume changed */
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
*  Setup clock tree for PDM-PCM block
*
* Parameters:
*  None
*
* Return:
*  void
*
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
        {
            handle_app_error();
        }
    
        if(CY_SYSCLK_SUCCESS != Cy_SysClk_DpllLpEnable(SRSS_DPLL_LP_1_PATH_NUM, DPLL_DELAY_MS))
        {
            handle_app_error();
        }
    
        if(!Cy_SysClk_DpllLpLocked(SRSS_DPLL_LP_1_PATH_NUM))
        {
            handle_app_error();
        }
    }

    if(CY_SYSCLK_SUCCESS == Cy_SysClk_ClkHfDisable(CY_CFG_SYSCLK_CLKHF7))
    {
        if(CY_SYSCLK_SUCCESS != Cy_SysClk_ClkHfSetSource(CY_CFG_SYSCLK_CLKHF7, CY_SYSCLK_CLKHF_IN_CLKPATH1))
        {
            handle_app_error(); 
        }
    
        if(CY_SYSCLK_SUCCESS != Cy_SysClk_ClkHfEnable(CY_CFG_SYSCLK_CLKHF7))
        {
            handle_app_error(); 
        }
    }

    if(CY_SYSCLK_SUCCESS == Cy_SysClk_PeriPclkDisableDivider((en_clk_dst_t)CYBSP_PDM_CLK_DIV_GRP_NUM,
                                                             CY_SYSCLK_DIV_16_5_BIT, 1U))
    {
        if(CY_SYSCLK_SUCCESS != Cy_SysClk_PeriPclkSetFracDivider((en_clk_dst_t)CYBSP_PDM_CLK_DIV_GRP_NUM,
                                                                 CY_SYSCLK_DIV_16_5_BIT, 1U, PDM_CLK_DIV_INT, 0U))
        {
            handle_app_error();
        }

        if(CY_SYSCLK_SUCCESS != Cy_SysClk_PeriPclkEnableDivider((en_clk_dst_t)CYBSP_PDM_CLK_DIV_GRP_NUM,
                                                                CY_SYSCLK_DIV_16_5_BIT, 1U))
        {
            handle_app_error();
        }
    }

    if(CY_SYSCLK_SUCCESS == Cy_SysClk_PeriPclkDisableDivider((en_clk_dst_t)CYBSP_TDM_CONTROLLER_0_CLK_DIV_GRP_NUM,
                                                             CY_SYSCLK_DIV_16_5_BIT, 0U))
    {
        if(CY_SYSCLK_SUCCESS != Cy_SysClk_PeriPclkSetFracDivider((en_clk_dst_t)CYBSP_TDM_CONTROLLER_0_CLK_DIV_GRP_NUM,
                                                                 CY_SYSCLK_DIV_16_5_BIT, 0U, TDM_CLK_DIV_INT, 0U))
        {
            handle_app_error();
        }

        if(CY_SYSCLK_SUCCESS != Cy_SysClk_PeriPclkEnableDivider((en_clk_dst_t)CYBSP_TDM_CONTROLLER_0_CLK_DIV_GRP_NUM,
                                                                CY_SYSCLK_DIV_16_5_BIT, 0U))
        {
            handle_app_error();
        }
    }
}


/*******************************************************************************
* Function Name: audio_app_init
********************************************************************************
* Summary:
*  Invokes "audio_clock_init" to initialize the audio subsystem clock and create
*  the RTOS task "Audio App Task".
*
* Parameters:
*  None
*
* Return:
*  None
*
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

    /* Prevents this core from entering deepsleep */
    mtb_hal_syspm_lock_deepsleep();

    /* Setup clock tree for required sampling frequency */
    app_clock_init();

    /* Initialize audio codec */
    audio_codec_init();

    int_status = Cy_SysInt_Init(&gpio_int_config, gpio_isr_handler);
    if(CY_SYSINT_SUCCESS != int_status)
    {
        printf("Interrupt initialization failed\r\n");
    }

    NVIC_EnableIRQ(gpio_int_config.intrSrc);

    /* Create the AUDIO APP RTOS task */
    rtos_task_status = xTaskCreate(audio_app_task, "Audio App Task", AUDIO_TASK_STACK_DEPTH, NULL,
            AUDIO_APP_TASK_PRIORITY, &rtos_audio_app_task);

    if (pdPASS != rtos_task_status)
    {
        handle_app_error();
    }
}


/*******************************************************************************
* Function Name: audio_app_task
********************************************************************************
* Summary:
*  Main audio task. Initializes the USB communication and the audio application.
*  In the main loop, checks USB device connectivity and based on that start/stop
*  providing audio data to the host.
*
* Parameters:
*  arg
*
* Return:
*  None
*
*******************************************************************************/
void audio_app_task(void *arg)
{
    uint8_t usb_status = USB_SUSPENDED;

    CY_UNUSED_PARAMETER(arg);

    USBD_Init();

    usb_audio_context = add_audio();

    usb_hid_control_context = add_hid_control();

    USBD_SetDeviceInfo(&usb_deviceInfo);

    USBD_AUDIO_Set_Timeouts(usb_audio_context, READ_TIMEOUT_MS, WRITE_TIMEOUT_MS);

    /* Initialize the audio IN components */
    audio_in_init();

    /* Init the audio OUT components */
    audio_out_init();

    /* Start the USB stack */
    USBD_Start();

    audio_app_update_codec_volume();

    for (;;)
    {
        /* Make device appear on the bus. This function call is blocking,
        * toggle the kit user LED until device gets enumerated. */
        while (USB_STAT_CONFIGURED != (USBD_GetState() & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)))
        {
            Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
            usb_status = USB_SUSPENDED;

            USB_OS_Delay(USB_CONFIG_DELAY);
        }

        if(USB_SUSPENDED == usb_status)
        {
            usb_status = USB_CONNECTED;
            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, 0U);
            printf("APP_LOG: USB Audio Device Connected\r\n");
        }

        if(button_status != SWITCH_NO_EVENT)
        {
            switch (button_status)
            {
                case SWITCH_VOLUME_INCR:

                button_status = SWITCH_NO_EVENT;

                audio_app_control_report[0] = AUDIO_HID_REPORT_ID0;
                audio_app_control_report[1] = AUDIO_HID_REPORT_VOLUME_UP;

                USBD_HID_Write(usb_hid_control_context, audio_app_control_report, 2, 0);
                audio_app_control_report[0] = AUDIO_HID_REPORT_ID0;
                audio_app_control_report[1] = AUDIO_HID_REPORT_ID1;
                /* Send out report data */
                USBD_HID_Write(usb_hid_control_context, audio_app_control_report, 2, 0);

                /* Update codec volume based on current USB volume */
                audio_app_update_codec_volume();
                break;

                case SWITCH_VOLUME_DECR:

                button_status = SWITCH_NO_EVENT;

                audio_app_control_report[0] = AUDIO_HID_REPORT_ID0;
                audio_app_control_report[1] = AUDIO_HID_REPORT_VOLUME_DOWN;

                USBD_HID_Write(usb_hid_control_context, audio_app_control_report, 2, 0);
                /* configASSERT(2 == usbd_stat); */
                audio_app_control_report[0] = AUDIO_HID_REPORT_ID0;
                audio_app_control_report[1] = AUDIO_HID_REPORT_ID1;
                /* Send out report data */
                USBD_HID_Write(usb_hid_control_context, audio_app_control_report, 2, 0);

                /* Update codec volume based on current USB volume */
                audio_app_update_codec_volume();
                break;

                default:
                /* Do nothing */
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_TICKS_MS));
    }
}

/* [] END OF FILE */

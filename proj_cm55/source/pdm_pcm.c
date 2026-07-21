/******************************************************************************
* File Name        : pdm_pcm.c
*
* Description      : This file contains the PDM/PCM interface implementation.
*
* Related Document : See README.md
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
#include "pdm_pcm.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "cybsp.h"
#include "cycfg_peripherals.h"
#include "cy_pdl.h"

#if (defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE))
#include "cyabs_rtos.h"
#endif

#define IS_POWER_OF_TWO(n) (((n) & ((n) - 1)) == 0 && (n) != 0)

/****************************************************************************
* Macros
*****************************************************************************/
#define PDM_PCM_MAX_BITS_PER_SAMPLE            (32U)
#define PDM_PCM_MAX_CHANNELS                   (6U)
#define PDM_PCM_IRQ_PRIORITY                   (3U)
#define PDM_PCM_INTR_MASK                      (CY_PDM_PCM_INTR_RX_TRIGGER | \
                                                CY_PDM_PCM_INTR_RX_OVERFLOW | \
                                                CY_PDM_PCM_INTR_RX_FIR_OVERFLOW | \
                                                CY_PDM_PCM_INTR_RX_IF_OVERFLOW)
#define PDM_PCM_IRQ_SOURCE                     ((IRQn_Type)PDM_PCM_CHANNEL_A_IRQ)

#define PDM_PCM_SAMPLE_FORMAT_8_BIT            (1U << 0)
#define PDM_PCM_SAMPLE_FORMAT_16_BIT           (1U << 1)
#define PDM_PCM_SAMPLE_FORMAT_32_BIT           (1U << 2)

#define PDM_PCM_FIFO_TRIGGER_LEVEL             (32U)

/* Maximum and minimum gain values in FIR scale */
#define PDM_PCM_GAIN_MIN_SCALE                 (31)
#define PDM_PCM_GAIN_MAX_SCALE                 (0)

#define PDM_PCM_SOFT_GAIN_INVALID              (-128)

/****************************************************************************
* Type definitions
*****************************************************************************/
typedef struct
{
    bool initialized;
    uint8_t channels;
    uint8_t sample_format;
    int8_t soft_gain_db;
    uint32_t config_mask;
    uint8_t bits_to_shift;
    volatile bool irq_overflow;
    volatile uint32_t overflow_count;
    uint32_t rd_idx;
    uint32_t wr_idx;
    uint32_t trigger_level_per_channel;
    bool trigger_callback_armed;
    void (*trigger_callback)(void *arg);
    void *trigger_callback_arg;
} pdm_pcm_context_t;

/* Ring buffer write function typedef */
typedef void (*ring_write_func_t)(int32_t);
/* Ring buffer read function typedef */
typedef void* (*ring_read_func_t)(void*);

/****************************************************************************
* Global Variables
*****************************************************************************/
static pdm_pcm_context_t pdm_pcm_context;
static uint8_t pdm_pcm_ring[PDM_PCM_MAX_FRAME_SIZE_IN_BYTES];
static const cy_stc_sysint_t pdm_irq_cfg =
{
    .intrSrc = PDM_PCM_IRQ_SOURCE,
    .intrPriority = PDM_PCM_IRQ_PRIORITY
};

/* FIR0 Filter Coefficients*/
/* Sample rate = 16 KHz and oversample ratio = 32x */
const cy_stc_pdm_pcm_fir_coeff_t pdm_pcm_16khz_32x_fir0_coefficients[8] =
{
    {1,    26},  { 57,    14},   {-138, -239},  {-14,   490},
    {664, -111}, {-1423, -1646}, { 641,  4823}, { 8191, 8191}
};

/* Sample rate = 48 KHz and oversample ratio = 32x */
const cy_stc_pdm_pcm_fir_coeff_t pdm_pcm_48khz_32x_fir0_coefficients[8] =
{
    {-3,  -9},   { 6,    49},   { 43, -105},  {-237,  19},
    { 580, 537}, {-719, -1873}, {-428, 4025}, { 8191, 8191}
};

/* Sample rate = 16 KHz and oversample ratio = 48x */
const cy_stc_pdm_pcm_fir_coeff_t pdm_pcm_16khz_48x_fir0_coefficients[8] =
{
    {5,    12},  {-2,    -57},   {-106, -23},   {244,  465},
    {201, -684}, {-1507, -1001}, { 1569, 5358}, {8191, 8191}
};

/* Sample rate = 16 KHz and oversample ratio = 64x */
const cy_stc_pdm_pcm_fir_coeff_t pdm_pcm_16khz_64x_fir0_coefficients[8] =
{
    {1,    26},  { 57,    15},   {-139, -239},  {-14,   491},
    {665, -110}, {-1425, -1650}, { 636,  4820}, { 8191, 8191}
};

/* Sample rate = 44.1 KHz and oversample ratio = 64x */
const cy_stc_pdm_pcm_fir_coeff_t pdm_pcm_44khz_64x_fir0_coefficients[8] =
{
    {-10,  6}, {13,   36}, {-7,  -33},    {-44,  -92},
    { 92, 73}, {264, 400}, {-2249, -271}, { 8191, 8191}
};

/* Sample rate = 48 KHz and oversample ratio = 64x */
const cy_stc_pdm_pcm_fir_coeff_t pdm_pcm_48khz_64x_fir0_coefficients[8] =
{
    {-3,  -9},   { 6,    49},   { 43, -105},  {-238,  18},
    { 581, 539}, {-719, -1877}, {-434, 4021}, { 8191, 8191}
};

/* Sample rate = 16 KHz and oversample ratio = 96x */
const cy_stc_pdm_pcm_fir_coeff_t pdm_pcm_16khz_96x_fir0_coefficients[8] =
{
    {5,    12},  {-2,    -57},   {-106, -23},   {245,  466},
    {202, -684}, {-1509, -1005}, { 1566, 5357}, {8191, 8191}
};


/* FIR0 Filter Coefficients*/
/* Sample rate = 16 KHz and any oversample ratio */
const cy_stc_pdm_pcm_fir_coeff_t pdm_pcm_16khz_fir1_coefficients[14] =
{
    { 190,  258}, {-106, -87},  { 16,  169}, {-35,  -182},
    {-5,    226}, { 40,  -261}, {-97,  300}, { 173, -336},
    {-276,  369}, { 418, -398}, {-630, 422}, { 986, -439},
    {-1764, 450}, { 5480, 8191}
};

/* Sample rate = 44.1 KHz and any oversample ratio */
const cy_stc_pdm_pcm_fir_coeff_t pdm_pcm_44khz_fir1_coefficients[14] =
{
    { 3,    6},   {0,   -13},  {-6,  20},  {15, -34},
    {-33,   50},  {61,  -70},  {-105, 92}, {168, -116},
    {-260,  140}, {394, -163}, {-598, 183}, {945, -199},
    {-1706, 208}, {5323, 8191}
};

/* Sample rate = 48 KHz and any oversample ratio */
const cy_stc_pdm_pcm_fir_coeff_t pdm_pcm_48khz_fir1_coefficients[14] =
{
    {-1,    -1},  {4,    3},  {-10,  -6},  {20, 11},
    {-38,   -17}, {65,   26}, {-107, -36}, {167, 48},
    {-255,  -61}, {383,  73}, {-578, -84}, {911, 93},
    {-1643, -99}, {5126, 8191}
};

/****************************************************************************
* Static function declarations
*****************************************************************************/
static void pdm_pcm_irq_handler(void);
static pdm_pcm_status_t pdm_pcm_apply_word_size(cy_stc_pdm_pcm_channel_config_t *cfg, uint8_t sample_size_bits);
static pdm_pcm_status_t pdm_pcm_apply_oversample_ratio(cy_stc_pdm_pcm_channel_config_t *cfg, uint32_t sample_rate_hz, uint16_t oversample_ratio);
static pdm_pcm_status_t pdm_pcm_apply_fir_coefficients(cy_stc_pdm_pcm_config_v2_t *cfg, uint32_t sample_rate_hz, uint16_t oversample_ratio);
static uint8_t pdm_pcm_get_shift_bits_for_sample_format(cy_stc_pdm_pcm_channel_config_t *cfg, uint8_t sample_size_bits);
static int32_t pdm_pcm_get_available_samples_per_channel(void);

/* Ring buffer access functions */
static void pdm_pcm_write_ring_32bit_value(int32_t value);
static void pdm_pcm_write_ring_16bit_value(int32_t value);
static void pdm_pcm_write_ring_8bit_value(int32_t value);
static void* pdm_pcm_read_ring_32bit_value(void *buf);
static void* pdm_pcm_read_ring_16bit_value(void *buf);
static void* pdm_pcm_read_ring_8bit_value(void *buf);

/****************************************************************************
* Function Name: pdm_pcm_get_available_samples_per_channel
******************************************************************************
* Summary:
*  Returns the number of buffered samples available per channel.
*
* Parameters:
*  None
*
* Return:
*  Number of samples available per channel in the ring buffer. Returns -1
*  to indicate ring buffer overflow (producer has written data that exceeded
*  the ring buffer capacity, corrupting unread data).
*
*****************************************************************************/
static int32_t pdm_pcm_get_available_samples_per_channel(void)
{
    /* Calculate total bytes currently in the ring buffer */
    uint32_t bytes_in_ring = pdm_pcm_context.wr_idx - pdm_pcm_context.rd_idx;

    /* Detect overflow: if bytes in ring exceeds or equals the total ring buffer size,
       the producer has written past the buffer capacity and data is corrupted */
    if (bytes_in_ring >= PDM_PCM_MAX_FRAME_SIZE_IN_BYTES)
    {
        return -1;
    }

    int32_t num_samples_in_ring = ((int32_t) bytes_in_ring) / pdm_pcm_context.channels;

    if (pdm_pcm_context.sample_format == PDM_PCM_SAMPLE_FORMAT_32_BIT)
    {
        num_samples_in_ring /= (int32_t) sizeof(int32_t);
    }
    else if (pdm_pcm_context.sample_format == PDM_PCM_SAMPLE_FORMAT_16_BIT)
    {
        num_samples_in_ring /= (int32_t) sizeof(int16_t);
    }

    return num_samples_in_ring;
}

/****************************************************************************
* Function Name: pdm_pcm_write_ring_32bit_value
******************************************************************************
* Summary:
*  Writes a 32-bit value into the internal ring buffer at the current write index 
*  and advances the index.
*
* Parameters:
*  value: 32-bit value to be written into the ring buffer.
*
* Return:
*  None
*
*****************************************************************************/
static void pdm_pcm_write_ring_32bit_value(int32_t value)
{
    int32_t *ring_32bit = (int32_t *) &pdm_pcm_ring[pdm_pcm_context.wr_idx & ((PDM_PCM_MAX_FRAME_SIZE_IN_BYTES) - 1U)];
    (void) Cy_PDM_PCM_ApplyPCM_Gain(&value, pdm_pcm_context.soft_gain_db, CY_PDM_PCM_24BIT, &value);
    *ring_32bit = value;
    pdm_pcm_context.wr_idx += sizeof(int32_t);
}

/****************************************************************************
* Function Name: pdm_pcm_write_ring_16bit_value
******************************************************************************
* Summary:
*  Writes a 16-bit value into the internal ring buffer at the current write index 
*  and advances the index.
*
* Parameters:
*  value: 16-bit value to be written into the ring buffer.
*
* Return:
*  None
*
*****************************************************************************/
static void pdm_pcm_write_ring_16bit_value(int32_t value)
{
    int16_t *ring_16bit = (int16_t *) &pdm_pcm_ring[pdm_pcm_context.wr_idx & ((PDM_PCM_MAX_FRAME_SIZE_IN_BYTES) - 1U)];
    (void)Cy_PDM_PCM_ApplyPCM_Gain(&value, pdm_pcm_context.soft_gain_db, CY_PDM_PCM_24BIT, &value);
    *ring_16bit = (int16_t) (value >> pdm_pcm_context.bits_to_shift);
    pdm_pcm_context.wr_idx += sizeof(int16_t);
}

/****************************************************************************
* Function Name: pdm_pcm_write_ring_8bit_value
******************************************************************************
* Summary:
*  Writes an 8-bit value into the internal ring buffer at the current write index 
*  and advances the index.
*
* Parameters:
*  value: 8-bit value to be written into the ring buffer.
*
* Return:
*  None
*
*****************************************************************************/
static void pdm_pcm_write_ring_8bit_value(int32_t value)
{
    int8_t *ring_8bit = (int8_t *) &pdm_pcm_ring[pdm_pcm_context.wr_idx & ((PDM_PCM_MAX_FRAME_SIZE_IN_BYTES) - 1U)];
    (void)Cy_PDM_PCM_ApplyPCM_Gain(&value, pdm_pcm_context.soft_gain_db, CY_PDM_PCM_24BIT, &value);
    *ring_8bit = (int8_t) (value >> pdm_pcm_context.bits_to_shift);
    pdm_pcm_context.wr_idx += sizeof(int8_t);
}

/****************************************************************************
* Function Name: pdm_pcm_read_ring_32bit_value
******************************************************************************
* Summary:
*  Reads a 32-bit value from the internal ring buffer at the current read index
*  and advances the index.
*
* Parameters:
*  None
*
* Return:
*  Pointer to the next position in the buffer after reading the 32-bit value.
*
*****************************************************************************/
static void* pdm_pcm_read_ring_32bit_value(void* buf)
{
    int32_t *buf_ptr = (int32_t *) buf;
    int32_t *ring_32bit = (int32_t *) &pdm_pcm_ring[pdm_pcm_context.rd_idx & ((PDM_PCM_MAX_FRAME_SIZE_IN_BYTES) - 1U)];
    pdm_pcm_context.rd_idx += sizeof(int32_t);
    *buf_ptr = *ring_32bit;
    buf_ptr++;
    return (void *) buf_ptr;
}

/****************************************************************************
* Function Name: pdm_pcm_read_ring_16bit_value
******************************************************************************
* Summary:
*  Reads a 16-bit value from the internal ring buffer at the current read index
*  and advances the index.
*
* Parameters:
*  None
*
* Return:
*  Pointer to the next position in the buffer after reading the 16-bit value.
*
*****************************************************************************/
static void* pdm_pcm_read_ring_16bit_value(void *buf)
{
    int16_t *buf_ptr = (int16_t *) buf;
    int16_t *ring_16bit = (int16_t *) &pdm_pcm_ring[pdm_pcm_context.rd_idx & ((PDM_PCM_MAX_FRAME_SIZE_IN_BYTES) - 1U)];
    pdm_pcm_context.rd_idx += sizeof(int16_t);
    *buf_ptr = *ring_16bit;
    buf_ptr++;
    return (void *) buf_ptr;
}

/****************************************************************************
* Function Name: pdm_pcm_read_ring_8bit_value
******************************************************************************
* Summary:
*  Reads an 8-bit value from the internal ring buffer at the current read index
*  and advances the index.
*
* Parameters:
*  None
*
* Return:
*  Pointer to the next position in the buffer after reading the 8-bit value.
*
*****************************************************************************/
static void* pdm_pcm_read_ring_8bit_value(void *buf)
{
    int8_t *buf_ptr = (int8_t *) buf;
    int8_t *ring_8bit = (int8_t *) &pdm_pcm_ring[pdm_pcm_context.rd_idx & ((PDM_PCM_MAX_FRAME_SIZE_IN_BYTES) - 1U)];
    pdm_pcm_context.rd_idx += sizeof(int8_t);
    *buf_ptr = *ring_8bit;
    buf_ptr++;
    return (void *) buf_ptr;
}

/****************************************************************************
* Function Name: pdm_pcm_irq_handler
******************************************************************************
* Summary:
*  Handles PDM/PCM channel interrupts and updates internal status flags.
*
* Parameters:
*  None
*
* Return:
*  None
*
*****************************************************************************/
static void pdm_pcm_irq_handler(void)
{
    uint32_t intr_status = Cy_PDM_PCM_Channel_GetInterruptStatusMasked(PDM_PCM_HW, PDM_PCM_CHANNEL_A_INDEX);
    int32_t pdm_pcm_sample;
    int32_t available_samples_per_channel;
    ring_write_func_t write_func;

    if (pdm_pcm_context.sample_format == PDM_PCM_SAMPLE_FORMAT_32_BIT)
    {
        write_func = pdm_pcm_write_ring_32bit_value;
    }
    else if (pdm_pcm_context.sample_format == PDM_PCM_SAMPLE_FORMAT_16_BIT)
    {
        write_func = pdm_pcm_write_ring_16bit_value;
    }
    else
    {
        write_func = pdm_pcm_write_ring_8bit_value;
    }
    
    if ((intr_status & CY_PDM_PCM_INTR_RX_TRIGGER) != 0UL)
    {
        for (int32_t i = 0; i < PDM_PCM_FIFO_TRIGGER_LEVEL; i++)
        {
            /* Get sample from PDM/PCM FIFO */
            pdm_pcm_sample = (int32_t) Cy_PDM_PCM_Channel_ReadFifo(PDM_PCM_HW, PDM_PCM_CHANNEL_A_INDEX);
            /* Write sample to ring buffer */
            write_func(pdm_pcm_sample); 
        
#ifdef PDM_PCM_CHANNEL_B_CONFIG
            if (pdm_pcm_context.channels > 1U)
            {
                pdm_pcm_sample = (int32_t) Cy_PDM_PCM_Channel_ReadFifo(PDM_PCM_HW, PDM_PCM_CHANNEL_B_INDEX);
                write_func(pdm_pcm_sample);
            }
#endif 
#ifdef PDM_PCM_CHANNEL_C_CONFIG
            if (pdm_pcm_context.channels > 2U)
            {
                pdm_pcm_sample = (int32_t) Cy_PDM_PCM_Channel_ReadFifo(PDM_PCM_HW, PDM_PCM_CHANNEL_C_INDEX);
                write_func(pdm_pcm_sample);
            }
#endif
#ifdef PDM_PCM_CHANNEL_D_CONFIG
            if (pdm_pcm_context.channels > 3U)
            {
                pdm_pcm_sample = (int32_t) Cy_PDM_PCM_Channel_ReadFifo(PDM_PCM_HW, PDM_PCM_CHANNEL_D_INDEX);
                write_func(pdm_pcm_sample);
            }
#endif
#ifdef PDM_PCM_CHANNEL_E_CONFIG
            if (pdm_pcm_context.channels > 4U)
            {
                pdm_pcm_sample = (int32_t) Cy_PDM_PCM_Channel_ReadFifo(PDM_PCM_HW, PDM_PCM_CHANNEL_E_INDEX);
                write_func(pdm_pcm_sample);
            }
#endif
#ifdef PDM_PCM_CHANNEL_F_CONFIG
            if (pdm_pcm_context.channels > 5U)
            {
                pdm_pcm_sample = (int32_t) Cy_PDM_PCM_Channel_ReadFifo(PDM_PCM_HW, PDM_PCM_CHANNEL_F_INDEX);
                write_func(pdm_pcm_sample);
            }
#endif
        }

        Cy_PDM_PCM_Channel_ClearInterrupt(PDM_PCM_HW, PDM_PCM_CHANNEL_A_INDEX, CY_PDM_PCM_INTR_RX_TRIGGER);
    }

    if ((intr_status & (CY_PDM_PCM_INTR_RX_OVERFLOW | CY_PDM_PCM_INTR_RX_FIR_OVERFLOW | CY_PDM_PCM_INTR_RX_IF_OVERFLOW)) != 0UL)
    {
        /* Hardware FIFO overflow detected. This occurs when the hardware FIFO overflows before
           the ISR can drain it. Ring buffer overflow is separately detected in
           pdm_pcm_get_available_samples_per_channel() by comparing fill level against buffer size. */
        pdm_pcm_context.irq_overflow = true;
        Cy_PDM_PCM_Channel_ClearInterrupt(PDM_PCM_HW, PDM_PCM_CHANNEL_A_INDEX, CY_PDM_PCM_INTR_MASK);
    }

    if ((!pdm_pcm_context.irq_overflow) &&
        (pdm_pcm_context.trigger_callback != NULL) &&
        pdm_pcm_context.trigger_callback_armed)
    {
        available_samples_per_channel = pdm_pcm_get_available_samples_per_channel();
        if (available_samples_per_channel >= (int32_t) pdm_pcm_context.trigger_level_per_channel)
        {
            pdm_pcm_context.trigger_callback_armed = false;
            pdm_pcm_context.trigger_callback(pdm_pcm_context.trigger_callback_arg);
        }
    }

    

}

/****************************************************************************
* Function Name: pdm_pcm_apply_word_size
******************************************************************************
* Summary:
*  Applies the requested sample size into the channel configuration.
*
* Parameters:
*  cfg: Channel configuration object.
*  sample_size_bits: PCM sample width in bits.
*
* Return:
*  PDM_PCM_SUCCESS or PDM_PCM_BAD_PARAM.
*
*****************************************************************************/
static pdm_pcm_status_t pdm_pcm_apply_word_size(cy_stc_pdm_pcm_channel_config_t *cfg,
                                                 uint8_t sample_size_bits)
{
    switch (sample_size_bits)
    {
        case 8:  cfg->wordSize = CY_PDM_PCM_WSIZE_8_BIT;  break;
        case 10: cfg->wordSize = CY_PDM_PCM_WSIZE_10_BIT; break;
        case 12: cfg->wordSize = CY_PDM_PCM_WSIZE_12_BIT; break;
        case 14: cfg->wordSize = CY_PDM_PCM_WSIZE_14_BIT; break;
        case 16: cfg->wordSize = CY_PDM_PCM_WSIZE_16_BIT; break;
        case 18: cfg->wordSize = CY_PDM_PCM_WSIZE_18_BIT; break;
        case 20: cfg->wordSize = CY_PDM_PCM_WSIZE_20_BIT; break;
        case 24: cfg->wordSize = CY_PDM_PCM_WSIZE_24_BIT; break;
        default: return PDM_PCM_BAD_PARAM;
    }
    return PDM_PCM_SUCCESS;
}

/****************************************************************************
* Function Name: pdm_pcm_apply_oversample_ratio
******************************************************************************
* Summary:
*  Applies filter settings associated with the requested PCM oversample ratio.
*
* Parameters:
*  cfg: Channel configuration object.
*  sample_rate_hz: Output sample rate in Hz 
*  oversample_ratio: Requested PCM oversample ratio.
*
* Return:
*  PDM_PCM_SUCCESS or PDM_PCM_BAD_PARAM.
*
*****************************************************************************/
static pdm_pcm_status_t pdm_pcm_apply_oversample_ratio(cy_stc_pdm_pcm_channel_config_t *cfg, 
                                                       uint32_t sample_rate_hz,
                                                       uint16_t oversample_ratio)
{
    switch (sample_rate_hz)
    {
        case 16000U:
            switch (oversample_ratio)
            {
                case PDM_PCM_OVERSAMPLE_RATIO_32:
                    cfg->cic_decim_code = CY_PDM_PCM_CHAN_CIC_DECIM_8;
                    cfg->fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_2;
                    cfg->fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_2;
                    break;
                case PDM_PCM_OVERSAMPLE_RATIO_48:
                    cfg->cic_decim_code = CY_PDM_PCM_CHAN_CIC_DECIM_8;
                    cfg->fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_3;
                    cfg->fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_2;
                    break;
                case PDM_PCM_OVERSAMPLE_RATIO_64:
                    cfg->cic_decim_code = CY_PDM_PCM_CHAN_CIC_DECIM_16;
                    cfg->fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_2;
                    cfg->fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_2;
                    break;
                case PDM_PCM_OVERSAMPLE_RATIO_96:
                    cfg->cic_decim_code = CY_PDM_PCM_CHAN_CIC_DECIM_16;
                    cfg->fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_3;
                    cfg->fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_2;                  
                    break;
                default:
                    return PDM_PCM_BAD_CLOCK;
            }
            cfg->fir0_enable = true;
            cfg->fir0_scale = CY_PDM_PCM_SEL_GAIN_5DB; // Apply default gain
            break;

        case 44100U:
            switch (oversample_ratio)
            {
                case PDM_PCM_OVERSAMPLE_RATIO_64:
                    cfg->cic_decim_code = CY_PDM_PCM_CHAN_CIC_DECIM_16;
                    cfg->fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_2;
                    cfg->fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_2;
                    break;
                default:
                    return PDM_PCM_BAD_CLOCK;
            }
            cfg->fir0_enable = true;
            cfg->fir0_scale = CY_PDM_PCM_SEL_GAIN_5DB; // Apply default gain
            break;

        case 48000U:
            switch (oversample_ratio)
            {
                case PDM_PCM_OVERSAMPLE_RATIO_32:
                    cfg->cic_decim_code = CY_PDM_PCM_CHAN_CIC_DECIM_8;
                    cfg->fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_2;
                    cfg->fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_2;
                    break;
                case PDM_PCM_OVERSAMPLE_RATIO_64:
                    cfg->cic_decim_code = CY_PDM_PCM_CHAN_CIC_DECIM_16;
                    cfg->fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_2;
                    cfg->fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_2;
                    break;
                default:
                    return PDM_PCM_BAD_CLOCK;
            }
            cfg->fir0_enable = true;
            cfg->fir0_scale = CY_PDM_PCM_SEL_GAIN_5DB; // Apply default gain
            break;
        default:
            switch (oversample_ratio)
            {
                case PDM_PCM_OVERSAMPLE_RATIO_32:
                    cfg->cic_decim_code = CY_PDM_PCM_CHAN_CIC_DECIM_16;
                    cfg->fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_1;
                    cfg->fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_2;
                    break;
                case PDM_PCM_OVERSAMPLE_RATIO_48:
                    cfg->cic_decim_code = CY_PDM_PCM_CHAN_CIC_DECIM_16;
                    cfg->fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_1;
                    cfg->fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_3;
                    break;
                case PDM_PCM_OVERSAMPLE_RATIO_64:
                    cfg->cic_decim_code = CY_PDM_PCM_CHAN_CIC_DECIM_16;
                    cfg->fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_1;
                    cfg->fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_4;
                    break;
                case PDM_PCM_OVERSAMPLE_RATIO_96:
                    cfg->cic_decim_code = CY_PDM_PCM_CHAN_CIC_DECIM_32;
                    cfg->fir0_decim_code = CY_PDM_PCM_CHAN_FIR0_DECIM_1;
                    cfg->fir1_decim_code = CY_PDM_PCM_CHAN_FIR1_DECIM_3;
                    break;
                default:
                    return PDM_PCM_BAD_CLOCK;
            }
            /* Disable FIR0 for unsupported configurations */
            cfg->fir0_enable = false;
            break;
    }
    return PDM_PCM_SUCCESS;
}

/****************************************************************************
* Function Name: pdm_pcm_apply_fir_coefficients
******************************************************************************
* Summary:
*  Applies the appropriate FIR filter coefficients based on the sample rate
*  and oversample ratio. If sample rate is not supported, it keeps the default 
*  coefficients.
*
* Parameters:
*  cfg: PDM/PCM configuration object.
*  sample_rate_hz: Output sample rate in Hz (16000, 44100 or 48000).
*  oversample_ratio: Oversample ratio (32, 48, 64, 96).
*
* Return:
*  PDM_PCM_SUCCESS or PDM_PCM_BAD_PARAM if invalid parameters.
*
*****************************************************************************/
static pdm_pcm_status_t pdm_pcm_apply_fir_coefficients(cy_stc_pdm_pcm_config_v2_t *cfg,
                                                      uint32_t sample_rate_hz,
                                                      uint16_t oversample_ratio)
{
    if (cfg == NULL)
    {
        return PDM_PCM_BAD_PARAM;
    }

    /* Select FIR0 coefficients based on sample rate and oversample ratio */
    switch (sample_rate_hz)
    {
        case 16000U:
            switch (oversample_ratio)
            {
                case PDM_PCM_OVERSAMPLE_RATIO_32:
                    memcpy(cfg->fir0_coeff, pdm_pcm_16khz_32x_fir0_coefficients, 8 * sizeof(cy_stc_pdm_pcm_fir_coeff_t));
                    break;
                case PDM_PCM_OVERSAMPLE_RATIO_48:
                    memcpy(cfg->fir0_coeff, pdm_pcm_16khz_48x_fir0_coefficients, 8 * sizeof(cy_stc_pdm_pcm_fir_coeff_t));
                    break;
                case PDM_PCM_OVERSAMPLE_RATIO_64:
                    memcpy(cfg->fir0_coeff, pdm_pcm_16khz_64x_fir0_coefficients, 8 * sizeof(cy_stc_pdm_pcm_fir_coeff_t));
                    break;
                case PDM_PCM_OVERSAMPLE_RATIO_96:
                    memcpy(cfg->fir0_coeff, pdm_pcm_16khz_96x_fir0_coefficients, 8 * sizeof(cy_stc_pdm_pcm_fir_coeff_t));
                    break;
                default:
                    return PDM_PCM_BAD_PARAM;
            }
            /* Apply FIR1 coefficients for 16 kHz (same for all oversample ratios) */
            memcpy(cfg->fir1_coeff, pdm_pcm_16khz_fir1_coefficients, 14 * sizeof(cy_stc_pdm_pcm_fir_coeff_t));

            cfg->fir0_coeff_user_value = true;
            cfg->fir1_coeff_user_value = true;
            break;

        case 44100U:
            switch (oversample_ratio)
            {
                case PDM_PCM_OVERSAMPLE_RATIO_64:
                    memcpy(cfg->fir0_coeff, pdm_pcm_44khz_64x_fir0_coefficients, 8 * sizeof(cy_stc_pdm_pcm_fir_coeff_t));
                    break;
                default:
                    return PDM_PCM_BAD_PARAM;
            }
            /* Apply FIR1 coefficients for 44.1 kHz (same for all oversample ratios) */
            memcpy(cfg->fir1_coeff, pdm_pcm_44khz_fir1_coefficients, 14 * sizeof(cy_stc_pdm_pcm_fir_coeff_t));

            cfg->fir0_coeff_user_value = true;
            cfg->fir1_coeff_user_value = true;
            break;


        case 48000U:
            switch (oversample_ratio)
            {
                case PDM_PCM_OVERSAMPLE_RATIO_32:
                    memcpy(cfg->fir0_coeff, pdm_pcm_48khz_32x_fir0_coefficients, 8 * sizeof(cy_stc_pdm_pcm_fir_coeff_t));
                    break;
                case PDM_PCM_OVERSAMPLE_RATIO_64:
                    memcpy(cfg->fir0_coeff, pdm_pcm_48khz_64x_fir0_coefficients, 8 * sizeof(cy_stc_pdm_pcm_fir_coeff_t));
                    break;
                default:
                    return PDM_PCM_BAD_PARAM;
            }
            /* Apply FIR1 coefficients for 48 kHz (same for all oversample ratios) */
            memcpy(cfg->fir1_coeff, pdm_pcm_48khz_fir1_coefficients, 14 * sizeof(cy_stc_pdm_pcm_fir_coeff_t));

            cfg->fir0_coeff_user_value = true;
            cfg->fir1_coeff_user_value = true;
            break;

        default:
            /* Not supported custom coefficients */
            cfg->fir0_coeff_user_value = false;
            cfg->fir1_coeff_user_value = false;
            break;
    }

    return PDM_PCM_SUCCESS;
}

/****************************************************************************
* Function Name: pdm_pcm_get_shift_bits_for_sample_format
******************************************************************************
* Summary:
*  Compares the BSP-configured word size against the user-requested sample
*  size and returns how many bits the BSP sample must be shifted.
*
* Parameters:
*  cfg: Channel configuration object.
*  sample_size_bits: User-requested PCM sample width in bits.
*
* Return:
*  Number of bits to shift. Returns 0 when the user-requested format is
*  greater than or equal to the BSP-configured format.
*
*****************************************************************************/
static uint8_t pdm_pcm_get_shift_bits_for_sample_format(cy_stc_pdm_pcm_channel_config_t *cfg,
                                                        uint8_t sample_size_bits)
{
    uint8_t bsp_sample_size_bits;

    switch (cfg->wordSize)
    {
        case CY_PDM_PCM_WSIZE_8_BIT:
            bsp_sample_size_bits = 8U;
            break;
        case CY_PDM_PCM_WSIZE_10_BIT:
            bsp_sample_size_bits = 10U;
            break;
        case CY_PDM_PCM_WSIZE_12_BIT:
            bsp_sample_size_bits = 12U;
            break;
        case CY_PDM_PCM_WSIZE_14_BIT:
            bsp_sample_size_bits = 14U;
            break;
        case CY_PDM_PCM_WSIZE_16_BIT:
            bsp_sample_size_bits = 16U;
            break;
        case CY_PDM_PCM_WSIZE_18_BIT:
            bsp_sample_size_bits = 18U;
            break;
        case CY_PDM_PCM_WSIZE_20_BIT:
            bsp_sample_size_bits = 20U;
            break;
        case CY_PDM_PCM_WSIZE_24_BIT:
            bsp_sample_size_bits = 24U;
            break;
        case CY_PDM_PCM_WSIZE_32_BIT:
        default:
            bsp_sample_size_bits = 32U;
            break;
    }

    if (sample_size_bits >= bsp_sample_size_bits)
    {
        return 0U;
    }

    return (uint8_t)(bsp_sample_size_bits - sample_size_bits);
}

/****************************************************************************
* Function Name: pdm_pcm_init
******************************************************************************
* Summary:
*  Initializes PDM/PCM interface with BSP defaults for advanced settings.
*  By default it sets up the pdm_pcm_read() function to be non-blocking and
*  starts capture.
*
* Parameters:
*  channels: Number of channels. The BSP must be configured to support at 
*      least this number of channels.
*  sample_size_bits: Sample size in bits. It affects how the pdm_pcm_read()
*      function returns data and is independent of the sample size 
*      configured in the BSP. Maximum supported sample size is 24 bits.
*
* Return:
*  PDM/PCM status code.
*
*****************************************************************************/
pdm_pcm_status_t pdm_pcm_init(uint8_t channels, uint8_t sample_size_bits)
{
    return pdm_pcm_init_adv(channels, 
                            sample_size_bits, 0U, 0U, 
                            PDM_PCM_CFG_DEFAULT_CONFIG);
}

/****************************************************************************
* Function Name: pdm_pcm_init_adv
******************************************************************************
* Summary:
*  Initializes PDM/PCM interface with advanced runtime settings.
*
* Parameters:
*  channels: Number of channels.
*  sample_size_bits: Sample size in bits.
*  sample_rate_hz: Output sample rate; zero uses BSP default.
*  oversample_ratio: Oversample ratio; zero uses BSP default.
*  config_mask: Interface behavior mask.
*
* Return:
*  PDM/PCM status code.
*
*****************************************************************************/
pdm_pcm_status_t pdm_pcm_init_adv(uint8_t channels,
                                  uint8_t sample_size_bits,
                                  uint32_t sample_rate_hz,
                                  uint16_t oversample_ratio,
                                  uint32_t config_mask)
{
    pdm_pcm_status_t status = PDM_PCM_SUCCESS;
    cy_en_pdm_pcm_status_t pdl_status;
    uint8_t num_configured_channels = 1U;
    uint32_t block_frequency_hz;
    cy_stc_pdm_pcm_config_v2_t pdm_pcm_cfg;
    cy_stc_pdm_pcm_channel_config_t channel_a_cfg;
#ifdef PDM_PCM_CHANNEL_B_CONFIG    
    cy_stc_pdm_pcm_channel_config_t channel_b_cfg;
#endif
#ifdef PDM_PCM_CHANNEL_C_CONFIG    
    cy_stc_pdm_pcm_channel_config_t channel_c_cfg;
#endif
#ifdef PDM_PCM_CHANNEL_D_CONFIG    
    cy_stc_pdm_pcm_channel_config_t channel_d_cfg;
#endif
#ifdef PDM_PCM_CHANNEL_E_CONFIG
    cy_stc_pdm_pcm_channel_config_t channel_e_cfg;
#endif
#ifdef PDM_PCM_CHANNEL_F_CONFIG
    cy_stc_pdm_pcm_channel_config_t channel_f_cfg;
#endif

    if (pdm_pcm_context.initialized)
    {
        return PDM_PCM_BAD_STATE;
    }

    /* Check if the maximum frame size is a power of two */
    if (IS_POWER_OF_TWO(PDM_PCM_MAX_FRAME_SIZE_IN_BYTES) == false)
    {
        return PDM_PCM_WRONG_FRAME_SIZE;
    }

    memcpy(&pdm_pcm_cfg, &PDM_PCM_config, sizeof(cy_stc_pdm_pcm_config_v2_t));

    memcpy(&channel_a_cfg, &PDM_PCM_CHANNEL_A_CONFIG, sizeof(cy_stc_pdm_pcm_channel_config_t));
#ifdef PDM_PCM_CHANNEL_B_CONFIG
    memcpy(&channel_b_cfg, &PDM_PCM_CHANNEL_B_CONFIG, sizeof(cy_stc_pdm_pcm_channel_config_t));
    num_configured_channels++;
#endif
#ifdef PDM_PCM_CHANNEL_C_CONFIG
    memcpy(&channel_c_cfg, &PDM_PCM_CHANNEL_C_CONFIG, sizeof(cy_stc_pdm_pcm_channel_config_t));
    num_configured_channels++;
#endif
#ifdef PDM_PCM_CHANNEL_D_CONFIG
    memcpy(&channel_d_cfg, &PDM_PCM_CHANNEL_D_CONFIG, sizeof(cy_stc_pdm_pcm_channel_config_t));
    num_configured_channels++;
#endif
#ifdef PDM_PCM_CHANNEL_E_CONFIG
    memcpy(&channel_e_cfg, &PDM_PCM_CHANNEL_E_CONFIG, sizeof(cy_stc_pdm_pcm_channel_config_t));
    num_configured_channels++;
#endif
#ifdef PDM_PCM_CHANNEL_F_CONFIG
    memcpy(&channel_f_cfg, &PDM_PCM_CHANNEL_F_CONFIG, sizeof(cy_stc_pdm_pcm_channel_config_t));
    num_configured_channels++;
#endif

    /* Check if the number of configured channels is less than the requested channels */
    if (num_configured_channels < channels)
    {
        return PDM_PCM_BAD_PARAM;
    }

    /* Set the trigger level for the PDM/PCM FIFO */
    channel_a_cfg.rxFifoTriggerLevel = PDM_PCM_FIFO_TRIGGER_LEVEL;

    /* Check only channel A word size */
    pdm_pcm_context.bits_to_shift = pdm_pcm_get_shift_bits_for_sample_format(&channel_a_cfg, sample_size_bits);

    /* Check the sample format based on the number of bits in the sample */
    if (sample_size_bits == 8U)
    {
        pdm_pcm_context.sample_format = PDM_PCM_SAMPLE_FORMAT_8_BIT;
    }
    else if (sample_size_bits <= 16U)
    {
        pdm_pcm_context.sample_format = PDM_PCM_SAMPLE_FORMAT_16_BIT;
    }
    else if (sample_size_bits <= 24U)
    {
        pdm_pcm_context.sample_format = PDM_PCM_SAMPLE_FORMAT_32_BIT;
    }
    else
    {
        return PDM_PCM_BAD_PARAM;
    }

    /* Enforce the highest sample_size_bits */
    sample_size_bits += pdm_pcm_context.bits_to_shift;

    pdm_pcm_apply_word_size(&channel_a_cfg, sample_size_bits);
#ifdef PDM_PCM_CHANNEL_B_CONFIG
    pdm_pcm_apply_word_size(&channel_b_cfg, sample_size_bits);
#endif
#ifdef PDM_PCM_CHANNEL_C_CONFIG
    pdm_pcm_apply_word_size(&channel_c_cfg, sample_size_bits);
#endif
#ifdef PDM_PCM_CHANNEL_D_CONFIG
    pdm_pcm_apply_word_size(&channel_d_cfg, sample_size_bits);
#endif
#ifdef PDM_PCM_CHANNEL_E_CONFIG
    pdm_pcm_apply_word_size(&channel_e_cfg, sample_size_bits);
#endif
#ifdef PDM_PCM_CHANNEL_F_CONFIG
    pdm_pcm_apply_word_size(&channel_f_cfg, sample_size_bits);
#endif

    /* Get the PDM PCM block frequency */
    block_frequency_hz = Cy_SysClk_PeriPclkGetFrequency((en_clk_dst_t) PDM_PCM_GRP_NUM, 
                                                        PDM_PCM_DIV_TYPE, 
                                                        PDM_PCM_DIV_NUM);

    /* If sample rate and oversample_ratio is provided, try to apply them */
    if ((sample_rate_hz != 0U) && (oversample_ratio != 0U))
    {
        /* Check if the desired sample rate is achievable */
        if ((block_frequency_hz % (sample_rate_hz * oversample_ratio)) != 0)
        {
            return PDM_PCM_BAD_CLOCK;
        }

        /* Check if using one of the supported oversample_ratio */
        if ((oversample_ratio != PDM_PCM_OVERSAMPLE_RATIO_32) && 
            (oversample_ratio != PDM_PCM_OVERSAMPLE_RATIO_48) && 
            (oversample_ratio != PDM_PCM_OVERSAMPLE_RATIO_64) && 
            (oversample_ratio != PDM_PCM_OVERSAMPLE_RATIO_96))
        {
            return PDM_PCM_BAD_CLOCK;
        }

        /* Calculate the desired PDM/PCM clock divider */
        uint32_t divider_value = block_frequency_hz / sample_rate_hz / oversample_ratio;

        /* Check if a valid divider value */
        if (divider_value > 256)
        {
            return PDM_PCM_BAD_CLOCK;
        }

        /* Set the new clock divider */
        pdm_pcm_cfg.clkDiv = divider_value - 1;

        /* Apply FIR filters based on the sample rate and oversample ratio */
        status = pdm_pcm_apply_fir_coefficients(&pdm_pcm_cfg, sample_rate_hz, oversample_ratio);

        if (PDM_PCM_SUCCESS != status)
        {
            return status;
        }

        /* Set the new decimation rates based on the oversample ratio and sample rate*/
        /* Set the sample delay based on divider value */
        status |= pdm_pcm_apply_oversample_ratio(&channel_a_cfg, sample_rate_hz, oversample_ratio);
        channel_a_cfg.sampledelay = (divider_value / 4) - 1;
#ifdef PDM_PCM_CHANNEL_B_CONFIG
        status |= pdm_pcm_apply_oversample_ratio(&channel_b_cfg, sample_rate_hz, oversample_ratio);
        channel_b_cfg.sampledelay = ((3 * divider_value) / 4) - 1;
#endif
#ifdef PDM_PCM_CHANNEL_C_CONFIG
        status |= pdm_pcm_apply_oversample_ratio(&channel_c_cfg, sample_rate_hz, oversample_ratio);
        channel_c_cfg.sampledelay = (divider_value / 4) - 1;
#endif
#ifdef PDM_PCM_CHANNEL_D_CONFIG
        status |= pdm_pcm_apply_oversample_ratio(&channel_d_cfg, sample_rate_hz, oversample_ratio);
        channel_d_cfg.sampledelay = ((3 * divider_value) / 4) - 1;
#endif
#ifdef PDM_PCM_CHANNEL_E_CONFIG
        status |= pdm_pcm_apply_oversample_ratio(&channel_e_cfg, sample_rate_hz, oversample_ratio);
        channel_e_cfg.sampledelay = (divider_value / 4) - 1;
#endif
#ifdef PDM_PCM_CHANNEL_F_CONFIG
        status |= pdm_pcm_apply_oversample_ratio(&channel_f_cfg, sample_rate_hz, oversample_ratio);
        channel_f_cfg.sampledelay = ((3 * divider_value) / 4) - 1;
#endif
        if (PDM_PCM_SUCCESS != status)
        {
            return PDM_PCM_BAD_CLOCK;
        }

    }
    else
    {
        /* If sample rate and oversample ratio is not provided, rely on the 
           device-configurator settings and just read the effective sample rate */
    }

    pdl_status = Cy_PDM_PCM_Init(PDM_PCM_HW, &pdm_pcm_cfg);
    if (pdl_status != CY_PDM_PCM_SUCCESS)
    {
        return PDM_PCM_HW_ERROR;
    }
  
    Cy_PDM_PCM_Channel_Enable(PDM_PCM_HW, PDM_PCM_CHANNEL_A_INDEX);
    Cy_PDM_PCM_Channel_Init(PDM_PCM_HW, &channel_a_cfg, PDM_PCM_CHANNEL_A_INDEX);
    
#ifdef PDM_PCM_CHANNEL_B_CONFIG
    Cy_PDM_PCM_Channel_Enable(PDM_PCM_HW, PDM_PCM_CHANNEL_B_INDEX);
    Cy_PDM_PCM_Channel_Init(PDM_PCM_HW, &channel_b_cfg, PDM_PCM_CHANNEL_B_INDEX);
#endif
#ifdef PDM_PCM_CHANNEL_C_CONFIG
    Cy_PDM_PCM_Channel_Enable(PDM_PCM_HW, PDM_PCM_CHANNEL_C_INDEX);
    Cy_PDM_PCM_Channel_Init(PDM_PCM_HW, &channel_c_cfg, PDM_PCM_CHANNEL_C_INDEX);
#endif
#ifdef PDM_PCM_CHANNEL_D_CONFIG
    Cy_PDM_PCM_Channel_Enable(PDM_PCM_HW, PDM_PCM_CHANNEL_D_INDEX); 
    Cy_PDM_PCM_Channel_Init(PDM_PCM_HW, &channel_d_cfg, PDM_PCM_CHANNEL_D_INDEX);
#endif
#ifdef PDM_PCM_CHANNEL_E_CONFIG
    Cy_PDM_PCM_Channel_Enable(PDM_PCM_HW, PDM_PCM_CHANNEL_E_INDEX);
    Cy_PDM_PCM_Channel_Init(PDM_PCM_HW, &channel_e_cfg, PDM_PCM_CHANNEL_E_INDEX);
#endif
#ifdef PDM_PCM_CHANNEL_F_CONFIG
    Cy_PDM_PCM_Channel_Enable(PDM_PCM_HW, PDM_PCM_CHANNEL_F_INDEX);
    Cy_PDM_PCM_Channel_Init(PDM_PCM_HW, &channel_f_cfg, PDM_PCM_CHANNEL_F_INDEX);
#endif

    Cy_PDM_PCM_Channel_ClearInterrupt(PDM_PCM_HW, PDM_PCM_CHANNEL_A_INDEX, CY_PDM_PCM_INTR_MASK);
    Cy_PDM_PCM_Channel_SetInterruptMask(PDM_PCM_HW, PDM_PCM_CHANNEL_A_INDEX, PDM_PCM_INTR_MASK);

    uint32_t intr_status = Cy_SysInt_Init(&pdm_irq_cfg, pdm_pcm_irq_handler);
    if (intr_status != CY_SYSINT_SUCCESS)
    {
        return PDM_PCM_HW_ERROR;
    }

    NVIC_ClearPendingIRQ(pdm_irq_cfg.intrSrc);
    NVIC_EnableIRQ(pdm_irq_cfg.intrSrc);
    
    
    pdm_pcm_context.channels = channels;
    pdm_pcm_context.config_mask = config_mask;
    pdm_pcm_context.irq_overflow = false;
    pdm_pcm_context.overflow_count = 0U;
    pdm_pcm_context.trigger_level_per_channel = 0U;
    pdm_pcm_context.trigger_callback_armed = false;
    pdm_pcm_context.trigger_callback = NULL;
    pdm_pcm_context.trigger_callback_arg = NULL;
    pdm_pcm_context.soft_gain_db = PDM_PCM_SOFT_GAIN_INVALID;
    pdm_pcm_context.initialized = true;
    pdm_pcm_context.rd_idx = 0U;
    pdm_pcm_context.wr_idx = 0U;

    if ((config_mask & PDM_PCM_CFG_START_ON_INIT) != 0UL)
    {
        return pdm_pcm_start();
    }

    return PDM_PCM_SUCCESS;
}

/****************************************************************************
* Function Name: pdm_pcm_start
******************************************************************************
* Summary:
*  Starts PDM/PCM capture by activating configured channels and IRQ handling.
*
* Parameters:
*  None
*
* Return:
*  PDM/PCM status code.
*
*****************************************************************************/
pdm_pcm_status_t pdm_pcm_start(void)
{
    if (!pdm_pcm_context.initialized)
    {
        return PDM_PCM_BAD_STATE;
    }

    Cy_PDM_PCM_Activate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_A_INDEX);

#ifdef PDM_PCM_CHANNEL_B_CONFIG
    if (pdm_pcm_context.channels > 1U)
    {
        Cy_PDM_PCM_Activate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_B_INDEX);
    }
#endif
#ifdef PDM_PCM_CHANNEL_C_CONFIG
    if (pdm_pcm_context.channels > 2U)
    {
        Cy_PDM_PCM_Activate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_C_INDEX);
    }
#endif
#ifdef PDM_PCM_CHANNEL_D_CONFIG
    if (pdm_pcm_context.channels > 3U)
    {
        Cy_PDM_PCM_Activate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_D_INDEX);
    }
#endif
#ifdef PDM_PCM_CHANNEL_E_CONFIG
    if (pdm_pcm_context.channels > 4U)
    {
        Cy_PDM_PCM_Activate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_E_INDEX);
    }
#endif
#ifdef PDM_PCM_CHANNEL_F_CONFIG
    if (pdm_pcm_context.channels > 5U)
    {
        Cy_PDM_PCM_Activate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_F_INDEX);
    }
#endif

    return PDM_PCM_SUCCESS;
}

/****************************************************************************
* Function Name: pdm_pcm_stop
******************************************************************************
* Summary:
*  Stops PDM/PCM capture by deactivating channels and disabling IRQ handling.
*
* Parameters:
*  None
*
* Return:
*  PDM/PCM status code.
*
*****************************************************************************/
pdm_pcm_status_t pdm_pcm_stop(void)
{
    if (!pdm_pcm_context.initialized)
    {
        return PDM_PCM_BAD_STATE;
    }

    Cy_PDM_PCM_DeActivate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_A_INDEX);

#ifdef PDM_PCM_CHANNEL_B_CONFIG
    if (pdm_pcm_context.channels > 1U)
    {
        Cy_PDM_PCM_DeActivate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_B_INDEX);
    }
#endif

#ifdef PDM_PCM_CHANNEL_C_CONFIG
    if (pdm_pcm_context.channels > 2U)
    {
        Cy_PDM_PCM_DeActivate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_C_INDEX);
    }
#endif
#ifdef PDM_PCM_CHANNEL_D_CONFIG
    if (pdm_pcm_context.channels > 3U)
    {
        Cy_PDM_PCM_DeActivate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_D_INDEX);
    }
#endif
#ifdef PDM_PCM_CHANNEL_E_CONFIG
    if (pdm_pcm_context.channels > 4U)
    {
        Cy_PDM_PCM_DeActivate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_E_INDEX);
    }
#endif
#ifdef PDM_PCM_CHANNEL_F_CONFIG
    if (pdm_pcm_context.channels > 5U)
    {
        Cy_PDM_PCM_DeActivate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_F_INDEX);
    }
#endif
    return PDM_PCM_SUCCESS;
}

/****************************************************************************
* Function Name: pdm_pcm_deinit
******************************************************************************
* Summary:
*  De-initializes PDM/PCM interface and disables internal interrupt handling.
*
* Parameters:
*  None
*
* Return:
*  PDM/PCM status code.
*
*****************************************************************************/
pdm_pcm_status_t pdm_pcm_deinit(void)
{
    if (!pdm_pcm_context.initialized)
    {
        return PDM_PCM_BAD_STATE;
    }

    NVIC_DisableIRQ(pdm_irq_cfg.intrSrc);
    NVIC_ClearPendingIRQ(pdm_irq_cfg.intrSrc);

    Cy_PDM_PCM_Channel_SetInterruptMask(PDM_PCM_HW, PDM_PCM_CHANNEL_A_INDEX, 0U);
    Cy_PDM_PCM_Channel_ClearInterrupt(PDM_PCM_HW, PDM_PCM_CHANNEL_A_INDEX, CY_PDM_PCM_INTR_MASK);

    Cy_PDM_PCM_DeActivate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_A_INDEX);
#ifdef PDM_PCM_CHANNEL_B_CONFIG
    Cy_PDM_PCM_DeActivate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_B_INDEX);
#endif
#ifdef PDM_PCM_CHANNEL_C_CONFIG
    Cy_PDM_PCM_DeActivate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_C_INDEX);
#endif
#ifdef PDM_PCM_CHANNEL_D_CONFIG
    Cy_PDM_PCM_DeActivate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_D_INDEX);
#endif
#ifdef PDM_PCM_CHANNEL_E_CONFIG
    Cy_PDM_PCM_DeActivate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_E_INDEX);
#endif
#ifdef PDM_PCM_CHANNEL_F_CONFIG
    Cy_PDM_PCM_DeActivate_Channel(PDM_PCM_HW, PDM_PCM_CHANNEL_F_INDEX);
#endif

    (void)memset(&pdm_pcm_context, 0, sizeof(pdm_pcm_context));
    return PDM_PCM_SUCCESS;
}

/****************************************************************************
* Function Name: pdm_pcm_set_hw_gain
******************************************************************************
* Summary:
*  Updates hardware gain in dBs.
*
* Parameters:
*  gain_in_db: Gain value in dB.
*
* Return:
*  PDM/PCM status code.
*
*****************************************************************************/
pdm_pcm_status_t pdm_pcm_set_hw_gain(int8_t gain_in_db)
{
    uint8_t scale;

    if (gain_in_db < PDM_PCM_GAIN_MIN_DB)
    {
        scale = PDM_PCM_GAIN_MIN_SCALE;
    }
    else if (gain_in_db > PDM_PCM_GAIN_MAX_DB)
    {
        scale = PDM_PCM_GAIN_MAX_SCALE;
    }
    else 
    {
        /* Linear mapping between gain in dB and FIR scale. The mapping is based on the PDM/PCM FIR scale settings. */
        scale = (uint8_t) (((PDM_PCM_GAIN_MAX_SCALE - PDM_PCM_GAIN_MIN_SCALE) * ((int16_t) gain_in_db)) / 
                            (PDM_PCM_GAIN_MAX_DB - PDM_PCM_GAIN_MIN_DB) - 
                            (PDM_PCM_GAIN_MAX_SCALE - PDM_PCM_GAIN_MIN_SCALE) * (PDM_PCM_GAIN_MAX_DB) / 
                            (PDM_PCM_GAIN_MAX_DB - PDM_PCM_GAIN_MIN_DB));
        if (gain_in_db < 0)
        {
            scale++; /* Compensate for rounding towards zero in integer division */
        }
    }

    Cy_PDM_PCM_SetGain(PDM_PCM_HW, PDM_PCM_CHANNEL_A_INDEX, (cy_en_pdm_pcm_gain_sel_t) scale);
#ifdef PDM_PCM_CHANNEL_B_CONFIG
    Cy_PDM_PCM_SetGain(PDM_PCM_HW, PDM_PCM_CHANNEL_B_INDEX, (cy_en_pdm_pcm_gain_sel_t) scale);
#endif
#ifdef PDM_PCM_CHANNEL_C_CONFIG
    Cy_PDM_PCM_SetGain(PDM_PCM_HW, PDM_PCM_CHANNEL_C_INDEX, (cy_en_pdm_pcm_gain_sel_t) scale);
#endif
#ifdef PDM_PCM_CHANNEL_D_CONFIG
    Cy_PDM_PCM_SetGain(PDM_PCM_HW, PDM_PCM_CHANNEL_D_INDEX, (cy_en_pdm_pcm_gain_sel_t) scale);
#endif
#ifdef PDM_PCM_CHANNEL_E_CONFIG
    Cy_PDM_PCM_SetGain(PDM_PCM_HW, PDM_PCM_CHANNEL_E_INDEX, (cy_en_pdm_pcm_gain_sel_t) scale);
#endif
#ifdef PDM_PCM_CHANNEL_F_CONFIG
    Cy_PDM_PCM_SetGain(PDM_PCM_HW, PDM_PCM_CHANNEL_F_INDEX, (cy_en_pdm_pcm_gain_sel_t) scale);
#endif

    return PDM_PCM_SUCCESS;
}

/****************************************************************************
* Function Name: pdm_pcm_set_soft_gain
******************************************************************************
* Summary:
*  Updates software gain in dB for samples copied into the internal ring
*  buffer. 
*
* Parameters:
*  gain_in_db: Gain value in dB.
*
* Return:
*  PDM/PCM status code.
*
*****************************************************************************/
pdm_pcm_status_t pdm_pcm_set_soft_gain(int8_t gain_in_db)
{
    int intr;

    intr = Cy_SysLib_EnterCriticalSection();
    pdm_pcm_context.soft_gain_db = gain_in_db;
    Cy_SysLib_ExitCriticalSection(intr);

    return PDM_PCM_SUCCESS;
}

/****************************************************************************
* Function Name: pdm_pcm_get_buffered_sample_count
******************************************************************************
* Summary:
*  Returns the number of buffered samples per channel currently available in
*  the internal ring buffer.
*
* Parameters:
*  sample_count_per_channel: Pointer to receive the sample count.
*
* Return:
*  PDM/PCM status code.
*
*****************************************************************************/
pdm_pcm_status_t pdm_pcm_get_buffered_sample_count(uint32_t *sample_count_per_channel)
{
    int intr;
    int32_t available_samples_per_channel;

    if (!pdm_pcm_context.initialized)
    {
        return PDM_PCM_BAD_STATE;
    }

    if (sample_count_per_channel == NULL)
    {
        return PDM_PCM_BAD_PARAM;
    }

    intr = Cy_SysLib_EnterCriticalSection();
    available_samples_per_channel = pdm_pcm_get_available_samples_per_channel();
    Cy_SysLib_ExitCriticalSection(intr);

    if (available_samples_per_channel < 0)
    {
        /* Ring buffer overflow detected: producer wrote more data than buffer capacity.
           Recover by resetting indices to prevent further data corruption. */
        intr = Cy_SysLib_EnterCriticalSection();
        pdm_pcm_context.rd_idx = pdm_pcm_context.wr_idx;
        Cy_SysLib_ExitCriticalSection(intr);
        *sample_count_per_channel = 0U;
        return PDM_PCM_OVERFLOW;
    }

    *sample_count_per_channel = (uint32_t) available_samples_per_channel;
    return PDM_PCM_SUCCESS;
}

/****************************************************************************
* Function Name: pdm_pcm_set_trigger_callback
******************************************************************************
* Summary:
*  Registers or clears a callback that is invoked when the buffered sample
*  count reaches the requested level per channel.
*
* Parameters:
*  level_per_channel: Ring-buffer watermark per channel in samples.
*  callback: Function invoked when the watermark is reached. Pass NULL to
*      clear the current callback.
*  arg: User argument passed back to the callback.
*
* Return:
*  PDM/PCM status code.
*
*****************************************************************************/
pdm_pcm_status_t pdm_pcm_set_trigger_callback(uint32_t level_per_channel,
                                              void (*callback)(void *arg),
                                              void *arg)
{
    int intr;
    int32_t available_samples_per_channel;
    bool invoke_callback = false;

    if (!pdm_pcm_context.initialized)
    {
        return PDM_PCM_BAD_STATE;
    }

    if ((callback != NULL) && (level_per_channel == 0U))
    {
        return PDM_PCM_BAD_PARAM;
    }

    intr = Cy_SysLib_EnterCriticalSection();
    pdm_pcm_context.trigger_level_per_channel = level_per_channel;
    pdm_pcm_context.trigger_callback = callback;
    pdm_pcm_context.trigger_callback_arg = arg;

    if (callback == NULL)
    {
        pdm_pcm_context.trigger_callback_armed = false;
    }
    else
    {
        available_samples_per_channel = pdm_pcm_get_available_samples_per_channel();
        pdm_pcm_context.trigger_callback_armed =
            (available_samples_per_channel < (int32_t) level_per_channel);
        invoke_callback = !pdm_pcm_context.trigger_callback_armed;
    }
    Cy_SysLib_ExitCriticalSection(intr);

    if (invoke_callback)
    {
        callback(arg);
    }

    return PDM_PCM_SUCCESS;
}

/****************************************************************************
* Function Name: pdm_pcm_blocking_read
******************************************************************************
* Summary:
*  Blocks until requested samples are available or timeout expires.
*
* Parameters:
*  buffer: Destination buffer.
*  samples_per_channel: Number of samples per channel to read.
*  timeout_ms: Timeout in milliseconds.
*
* Return:
*  PDM/PCM status code.
*
*****************************************************************************/
pdm_pcm_status_t pdm_pcm_blocking_read(void *buffer,
                                       uint32_t samples_per_channel,
                                       uint32_t timeout_ms)
{
    pdm_pcm_status_t status;

    if ((!pdm_pcm_context.initialized) || (buffer == NULL) || (samples_per_channel == 0U))
    {
        return PDM_PCM_BAD_PARAM;
    }

    while (1) 
    {
        status = pdm_pcm_nonblocking_read(buffer,
                                          samples_per_channel,
                                          NULL);

        /* If successfully read, break and return success */
        if (status == PDM_PCM_SUCCESS)
        {
            break;
        }

        /* Check if any error */
        if (status != PDM_PCM_NOT_ENOUGH_DATA)
        {
            return status;
        }

        /* Check if timeout expired, if yes, return error */
        if (timeout_ms == 0)
        {
            return PDM_PCM_TIMEOUT;
        }

        timeout_ms--;

#if (defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE))
        (void)cy_rtos_delay_milliseconds(1U);
#else
        /* If not RTOS aware, do a busy wait for 1 millisecond before trying again */
        Cy_SysLib_Delay(1U);
#endif

    } 

    return PDM_PCM_SUCCESS;
}

/****************************************************************************
* Function Name: pdm_pcm_nonblocking_read
******************************************************************************
* Summary:
*  Reads available samples without blocking.
*
* Parameters:
*  buffer: Destination buffer.
*  samples_per_channel: Requested samples per channel.
*  samples_read_per_channel: Number of samples returned. If this parameter is 
*      NULL and the requested number of samples is not available, the function
*      returns PDM_PCM_NOT_ENOUGH_DATA. If this parameter is not NULL, the 
*      function returns the number of samples that were read, which can be less
*      than the requested number if not enough samples are available.
*
* Return:
*  PDM/PCM status code.
*
*****************************************************************************/
pdm_pcm_status_t pdm_pcm_nonblocking_read(void *buffer,
                                          uint32_t samples_per_channel,
                                          uint32_t *samples_read_per_channel)
{
    ring_read_func_t read_func;
    int32_t num_samples_in_ring;
    uint32_t samples_to_read;
    int intr;

    if ((!pdm_pcm_context.initialized) || (buffer == NULL) || (samples_per_channel == 0U))
    {
        return PDM_PCM_BAD_PARAM;
    }

    intr = Cy_SysLib_EnterCriticalSection();
    num_samples_in_ring = pdm_pcm_get_available_samples_per_channel();
    Cy_SysLib_ExitCriticalSection(intr);

    if (num_samples_in_ring == 0)
    {
        return PDM_PCM_NOT_ENOUGH_DATA;
    } 
    else if (num_samples_in_ring < 0)
    {
        /* Ring buffer overflow detected: producer wrote more data than buffer capacity.
           The ring buffer contains corrupted data. Recover by resetting indices and
           discarding buffered data. */
        intr = Cy_SysLib_EnterCriticalSection();
        pdm_pcm_context.rd_idx = pdm_pcm_context.wr_idx;
        Cy_SysLib_ExitCriticalSection(intr);
        return PDM_PCM_OVERFLOW;
    }

    /* If internal overflow, returns a hardware error */
    if (pdm_pcm_context.irq_overflow)
    {
        pdm_pcm_context.irq_overflow = false;
        if (samples_read_per_channel != NULL)
        {
            *samples_read_per_channel = 0U;
        }
        return PDM_PCM_HW_ERROR;
    }

    if (pdm_pcm_context.sample_format == PDM_PCM_SAMPLE_FORMAT_32_BIT)
    {
        read_func = pdm_pcm_read_ring_32bit_value;
    }
    else if (pdm_pcm_context.sample_format == PDM_PCM_SAMPLE_FORMAT_16_BIT)
    {
        read_func = pdm_pcm_read_ring_16bit_value;
    }
    else
    {
        read_func = pdm_pcm_read_ring_8bit_value;
    }

    /* Check how many samples to read */
    if (num_samples_in_ring < samples_per_channel)
    {
        /* If it can't return how many samples were read, then return not enough data */
        if (samples_read_per_channel == NULL)
        {
            return PDM_PCM_NOT_ENOUGH_DATA;
        }

        samples_to_read = num_samples_in_ring;
    }
    else 
    {
        samples_to_read = samples_per_channel;
    }

    /* Check if needs to return the number of samples read per channel */
    if (samples_read_per_channel != NULL)
    {
        *samples_read_per_channel = (uint32_t) samples_to_read;
    }

    /* Check if read data should be interleaved */
    if ((pdm_pcm_context.config_mask & PDM_PCM_CFG_INTERLEAVED) != 0UL)
    {
        /* Populate the buffer with the available samples */
        for (uint32_t i = 0; i < (samples_to_read*pdm_pcm_context.channels); i++)
        {
            buffer = read_func(buffer);
        }
    }
    else
    {
        uint8_t *channel_buffer;
        uint8_t buffer_offset = (pdm_pcm_context.sample_format == PDM_PCM_SAMPLE_FORMAT_32_BIT) ? 4U :
                                (pdm_pcm_context.sample_format == PDM_PCM_SAMPLE_FORMAT_16_BIT) ? 2U : 1U;

        /* Populate the buffer with the available samples, deinterleaving them by channel */
        for (uint32_t i = 0; i < samples_to_read; i++)
        {
            channel_buffer = (uint8_t *) (((uint8_t *) buffer) + (buffer_offset * i));
            for (uint8_t ch = 0; ch < pdm_pcm_context.channels; ch++)
            {
                read_func((void *) channel_buffer);
                channel_buffer += (buffer_offset * samples_to_read);
            }
        }
    }

    intr = Cy_SysLib_EnterCriticalSection();
    num_samples_in_ring = pdm_pcm_get_available_samples_per_channel();
    if ((pdm_pcm_context.trigger_callback != NULL) &&
        (num_samples_in_ring < (int32_t) pdm_pcm_context.trigger_level_per_channel))
    {
        pdm_pcm_context.trigger_callback_armed = true;
    }
    Cy_SysLib_ExitCriticalSection(intr);

    return PDM_PCM_SUCCESS;
}

/****************************************************************************
* Function Name: pdm_pcm_read
******************************************************************************
* Summary:
*  Dispatches read operation to blocking or non-blocking implementation, 
*  depending on the init_adv() configuration. If blocking read is selected, 
*  this function will block until the requested number of samples is available 
*  or a timeout occurs. If non-blocking read is selected, this function
*  will return immediately with the available samples, or an error code if the
*  requested number of samples is not available.
*
* Parameters:
*  buffer: Destination buffer.
*  samples_per_channel: Requested samples per channel.
*
* Return:
*  PDM/PCM status code.
*
*****************************************************************************/
pdm_pcm_status_t pdm_pcm_read(void *buffer, uint32_t samples_per_channel)
{
    if ((pdm_pcm_context.config_mask & PDM_PCM_CFG_NONBLOCKING) != 0UL)
    {
        return pdm_pcm_nonblocking_read(buffer, samples_per_channel, NULL);
    }

    return pdm_pcm_blocking_read(buffer, samples_per_channel, PDM_PCM_MAX_TIMEOUT_MS);
}

/* [] END OF FILE */

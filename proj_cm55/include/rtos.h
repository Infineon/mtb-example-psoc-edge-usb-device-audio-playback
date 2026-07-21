/******************************************************************************
* File Name   : rtos.h
*
* Description : This file contains the function prototypes and constants
*               related to the RTOS.
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
#ifndef RTOS_H
#define RTOS_H

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*****************************************************************************
* Headers
*****************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"


/******************************************************************************
* Macros
******************************************************************************/
#define AUDIO_APP_TASK_PRIORITY     (1u)
#define AUDIO_IN_TASK_PRIORITY      (3u)
#define AUDIO_OUT_TASK_PRIORITY     (3u)
#define AUDIO_TASK_STACK_DEPTH      (1024U) /* In bytes */
#define AUDIO_OUT_TASK_STACK_DEPTH  (2048U) /* In bytes */
#define AUDIO_IN_TASK_STACK_DEPTH   (2048U) /* In bytes */

/* Enabling or disabling a MCWDT requires a wait time of upto 2 CLK_LF cycles  
 * to come into effect. This wait time value will depend on the actual CLK_LF  
 * frequency set by the BSP.
 */
#define LPTIMER_1_WAIT_TIME_USEC            (62U)

/* Define the LPTimer interrupt priority number. '1' implies highest priority. 
 */
#define APP_LPTIMER_INTERRUPT_PRIORITY      (1U)


/******************************************************************************
* Global variables
******************************************************************************/
/* Task Handlers */
extern TaskHandle_t rtos_audio_app_task;


#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* RTOS_H */

/* [] END OF FILE */

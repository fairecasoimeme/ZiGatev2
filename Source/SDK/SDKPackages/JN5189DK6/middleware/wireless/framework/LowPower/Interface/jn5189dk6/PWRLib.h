/*! *********************************************************************************
* The Clear BSD License
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017, 2019 NXP
* All rights reserved.
*
* \file
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the disclaimer
* below) provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the names of its contributors
*   may be used to endorse or promote products derived from this software
*   without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************** */

#ifndef __PWR_LIB_H__
#define __PWR_LIB_H__

/*****************************************************************************
 *                               INCLUDED HEADERS                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the headers that this module needs to include.    *
 * Note that it is not a good practice to include header files into header   *
 * files, so use this section only if there is no other better solution.     *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
 #include "PWR_Interface.h"

/*****************************************************************************
 *                             PUBLIC MACROS                                *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*****************************************************************************
 *                             PRIVATE MACROS                                *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*****************************************************************************
 *                        PRIVATE TYPE DEFINITIONS                           *
 *---------------------------------------------------------------------------*
 * Add to this section all the data types definitions: stuctures, unions,    *
 * enumerations, typedefs ...                                                *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

typedef struct
{
	uint64_t wakeup_src;
	uint32_t wakeup_io;
	uint32_t sleep_cfg;
} pwrlib_pd_cfg_t;

/*****************************************************************************
 *                               PUBLIC VARIABLES(External)                  *
 *---------------------------------------------------------------------------*
 * Add to this section all the variables and constants that have global      *
 * (project) scope.                                                          *
 * These variables / constants can be accessed outside this module.          *
 * These variables / constants shall be preceded by the 'extern' keyword in  *
 * the interface header.                                                     *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
extern volatile PWR_WakeupReason_t PWRLib_MCU_WakeupReason;


/*****************************************************************************
 *                            PUBLIC FUNCTIONS                               *
 *---------------------------------------------------------------------------*
 * Add to this section all the global functions prototype preceded (as a     *
 * good practice) by the keyword 'extern'                                    *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*---------------------------------------------------------------------------
 * Name: PWRLib_Init
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_Init(void);

/*---------------------------------------------------------------------------
 * Name: PWRLib_MCUEnter_Sleep
 * Description: Puts the processor into Sleep .

                Mode of operation details:
                 - ARM core enters Sleep Mode
                 - ARM core is clock gated (HCLK = OFF)
                 - peripherals are functional
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_MCU_Enter_Sleep(void);

/*---------------------------------------------------------------------------
 * Name: PWRLib_SetLowPowerMode
 * Description:
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_SetDeepSleepMode(uint8_t lpMode);

/*---------------------------------------------------------------------------
 * Name: PWRLib_GetDeepSleepMode
 * Description:
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint8_t PWRLib_GetDeepSleepMode(void);

/*---------------------------------------------------------------------------
 * Name: PWRLib_ClearWakeupReason
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_ClearWakeupReason(void);

/*---------------------------------------------------------------------------
 * Name: PWRLib_EnterPowerDownMode
 * Description: - request to switch in power down mode
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_EnterPowerDownMode(pwrlib_pd_cfg_t *pd_cfg);

/*---------------------------------------------------------------------------
 * Name: PWRLib_EnterPowerDownModeRamOff
 * Description: - request to switch in power down mode
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_EnterPowerDownModeRamOff(pwrlib_pd_cfg_t *pd_cfg);

/*---------------------------------------------------------------------------
 * Name: PWRLib_EnterDeepDownMode
 * Description: - request to switch in power down mode
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_EnterDeepDownMode(pwrlib_pd_cfg_t *pd_cfg);

/*---------------------------------------------------------------------------
 * Name: PWR_UpdateWakeupReason
 * Description: - request to switch in power down mode
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_UpdateWakeupReason(void);

#endif /* __PWR_LIB_H__ */


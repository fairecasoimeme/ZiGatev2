/*! *********************************************************************************
* The Clear BSD License
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2019 NXP
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

#ifndef _PWR_CONFIGURATION_H_
#define _PWR_CONFIGURATION_H_

/*****************************************************************************
 *                               INCLUDED HEADERS                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the headers that this module needs to include.    *
 * Note that it is not a good practice to include header files into header   *
 * files, so use this section only if there is no other better solution.     *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
#include "TimersManager.h"

/************************************************************************************
 *************************************************************************************
 * Module configuration constants
 *************************************************************************************
 ************************************************************************************/

/*-----------------------------------------------------------------------------
 * To enable/disable all of the code in this PWR/PWRLib files.
 *   TRUE =  1: Use Lowpower functions (Normal)
 *   FALSE = 0: Don't use Lowpower - only WFI. Will cut variables and code out. But
 *              functions still exist. Useful for debugging and test purposes
 */
#ifndef cPWR_UsePowerDownMode
#define cPWR_UsePowerDownMode            FALSE
#endif

#ifndef cPWR_FullPowerDownMode
#define cPWR_FullPowerDownMode           FALSE
#endif

#ifndef cPWR_EnableDeepSleepMode_4
#define cPWR_EnableDeepSleepMode_4     	 FALSE
#endif

#ifndef gAppDeepSleepMode_c
#define gAppDeepSleepMode_c    1
#endif

#define cPWR_NoClockGating                     0
#define cPWR_ClockGating                       1
#define cPWR_DeepSleep_RamOffOsc32kOn          2      
#define cPWR_PowerDown_RamRet                  3
#define cPWR_DeepSleep_RamOffOsc32kOff         4


#if cPWR_UsePowerDownMode
  #if cPWR_FullPowerDownMode
  #define cPWR_DeepSleepMode                     cPWR_PowerDown_RamRet
  #else
  #define cPWR_DeepSleepMode                     cPWR_ClockGating
  #endif
#else
  #define cPWR_DeepSleepMode                     cPWR_NoClockGating
#endif

#ifndef gSupportBle
/* BLE features (checks BLE status, uses BLE timers) */
#define gSupportBle (1)
#endif

/* 32k FRO clock: if using 32k FRO (gClkUseFro32K=1) then default to recalibrating it
 * on every wake (gClkRecalFro32K=1). If 32k FRO should only be calibrated on first
 * start, user must explicitly set gClkRecalFro32K=0 */
#if (defined gClkUseFro32K) && (gClkUseFro32K != 0) && !(defined gClkRecalFro32K)
#define gClkRecalFro32K (1)
#endif

#endif /* _PWR_CONFIGURATION_H_ */


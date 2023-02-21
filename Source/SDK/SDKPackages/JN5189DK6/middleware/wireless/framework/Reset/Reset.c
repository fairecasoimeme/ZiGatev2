/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
* All rights reserved.
*
* \file
*
* MCU reset related functions implementation
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "fsl_device_registers.h"
#include "EmbeddedTypes.h"

#ifdef CPU_JN518X
#include "fsl_reset.h"
#endif

#if defined gLoggingActive_d && (gLoggingActive_d > 0)
#include "dbg_logging.h"
#ifndef DBG_RST
#define DBG_RST 0
#endif
#define RST_DBG_LOG(fmt, ...)  if (DBG_RST) do { DbgLogAdd(__FUNCTION__ , fmt, VA_NUM_ARGS(__VA_ARGS__), ##__VA_ARGS__); } while (0);
#else
#define RST_DBG_LOG(...)
#endif

#ifndef gResetSystemReset_d
#define gResetSystemReset_d 0
#endif
/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

/*
* SafeActions_on_RamOff is declared weak.
* Its actual implementation is expected in Applmain.c
*/
#if defined(__IAR_SYSTEMS_ICC__)
__weak void SafeActions_on_RamOff(void)
#elif defined(__GNUC__)
__attribute__((weak)) void SafeActions_on_RamOff(void)
#endif
{ }
/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
void ResetMCU(void)
{
    RST_DBG_LOG("");

    SafeActions_on_RamOff();

#ifndef CPU_JN518X
    NVIC_SystemReset();
#else
#if gResetSystemReset_d
    /* Full IC reset */
    RESET_SystemReset();
#else
    /* Let ResetMCU just reset the ARM core so that the ARM core executes a Cold boot
     *  but keeps its RAM on, allowing the exchange of information across reset
     * */
    RESET_ArmReset();
#endif
#endif
  /* never reached */
    while(1){}
}



/********************************** EOF ***************************************/

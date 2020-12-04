/*
* Copyright 2019 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "fsl_common.h"
#include "clock_config.h"
#include "system_JN5189.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*******************************************************************************
 * Code
 ******************************************************************************/
void BOARD_BootClockVLPR(void)
{
}

void BOARD_BootClockRUN(void)
{
    /* Already set to mainCLK by default */
    CLOCK_AttachClk(kMAIN_CLK_to_ASYNC_APB);

    /* 32MHz clock */
    CLOCK_EnableClock(kCLOCK_Xtal32M);    /* Normally started already */
    
    /* Make sure it is disabled */
    CLOCK_AttachClk(kNONE_to_USART_CLK);

    /* 32KHz clock */
    CLOCK_EnableClock(kCLOCK_Xtal32k); /* CLOCK_32k_source can be either Fro32k or Xtal32k */
    CLOCK_AttachClk(kXTAL32K_to_OSC32K_CLK); /* OSC CLK is using 32 k/m crystals */

    /* WWDT clock config (32k oscillator, no division) */
    CLOCK_AttachClk(kOSC32K_to_WDT_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivWdtClk, 1, true);

}

void BOARD_BootClockHSRUN(void)
{
}

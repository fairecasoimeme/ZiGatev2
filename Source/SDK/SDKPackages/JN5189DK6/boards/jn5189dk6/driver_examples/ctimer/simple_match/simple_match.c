/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_ctimer.h"

#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CTIMER CTIMER0
#define CTIMER_MAT_OUT kCTIMER_Match_3
#define CTIMER_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define LED_PORT 0
#define LED_PIN 0

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void delay(uint32_t d);


/*******************************************************************************
* Variables
******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void delay(uint32_t d)
{
    // 800000 gives about 200ms
    volatile uint32_t i = 0;
    for (i = 0; i < d; ++i)
    {
        __asm("NOP"); /* delay */
    }
}


/*!
 * @brief Main function
 */
int main(void)
{
    ctimer_config_t config;
    ctimer_match_config_t matchConfig;

    /* Init hardware*/
    /* Security code to allow debug access */
    SYSCON->CODESECURITYPROT = 0x87654320;

    CLOCK_EnableAPBBridge();
    /* Init the boards */
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitPins();

    delay(10000);
    LED_RED1_INIT(1);
    PRINTF("CTimer match example to toggle the output on a match\r\n");

    CTIMER_GetDefaultConfig(&config);

    CTIMER_Init(CTIMER, &config);

    matchConfig.enableCounterReset = true;
    matchConfig.enableCounterStop = false;
    matchConfig.matchValue = CTIMER_CLK_FREQ / 2;
    matchConfig.outControl = kCTIMER_Output_Toggle;
    matchConfig.outPinInitState = true;
    matchConfig.enableInterrupt = false;
    CTIMER_SetupMatch(CTIMER, CTIMER_MAT_OUT, &matchConfig);
    CTIMER_StartTimer(CTIMER);

    while (1)
    {
        /* No timer match output pin connected to a LED
        * toggle LED manually according to match status
        */
        uint32_t mask;
        mask = (1 << (CTIMER_EMR_EM0_SHIFT + CTIMER_MAT_OUT));
        if (CTIMER->EMR & mask)
        {
            LED_RED1_ON();
        }
        else
        {
            LED_RED1_OFF();
        }
    }
}

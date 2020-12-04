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
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CTIMER CTIMER0
#define CTIMER_MAT0_OUT kCTIMER_Match_0
#define CTIMER_MAT1_OUT kCTIMER_Match_3
#define CTIMER_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define LED_PORT_0 0
#define LED_PIN_0 0
#define LED_PORT_1 0
#define LED_PIN_1 3

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void ctimer_match0_callback(uint32_t flags);
void ctimer_match1_callback(uint32_t flags);

/* Array of function pointers for callback for each channel */
ctimer_callback_t ctimer_callback_table[] = {
    ctimer_match0_callback, NULL, NULL, ctimer_match1_callback, NULL, NULL, NULL, NULL};


/*******************************************************************************
* Variables
******************************************************************************/
/* Match Configuration for Channel 0 */
static ctimer_match_config_t matchConfig0;
/* Match Configuration for Channel 1 */
static ctimer_match_config_t matchConfig1;

/*******************************************************************************
 * Code
 ******************************************************************************/

void ctimer_match1_callback(uint32_t flags)
{
    static uint32_t count = 0;
    static uint32_t matchUpdateCount = 8;

    if (++count > matchUpdateCount)
    {
        count = 0;
        matchConfig1.matchValue >>= 1;
        matchUpdateCount <<= 1;
        if (matchUpdateCount == (1 << 8))
        {
            matchUpdateCount = 8;
            matchConfig1.matchValue = CTIMER_CLK_FREQ / 2;
        }
        CTIMER_SetupMatch(CTIMER, CTIMER_MAT1_OUT, &matchConfig1);
    }
}

void ctimer_match0_callback(uint32_t flags)
{
    static uint32_t count = 0;
    static uint32_t matchUpdateCount = 8;

    if (++count > matchUpdateCount)
    {
        count = 0;
        matchConfig0.matchValue >>= 1;
        matchUpdateCount <<= 1;
        if (matchUpdateCount == (1 << 8))
        {
            matchUpdateCount = 8;
            matchConfig0.matchValue = CTIMER_CLK_FREQ / 2;
        }
        CTIMER_SetupMatch(CTIMER, CTIMER_MAT0_OUT, &matchConfig0);
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    ctimer_config_t config;

    /* Init hardware*/
    /* Security code to allow debug access */
    SYSCON->CODESECURITYPROT = 0x87654320;

    CLOCK_EnableAPBBridge();
    /* attach clock for USART(debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* reset FLEXCOMM for USART */
    RESET_PeripheralReset(kFC0_RST_SHIFT_RSTn);

    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitPins();
    LED_RED1_INIT(1);
    LED_RED2_INIT(1);
    PRINTF("CTimer match example to toggle the output. \r\n");
    PRINTF("This example uses interrupt to change the match period. \r\n");

    CTIMER_GetDefaultConfig(&config);

    CTIMER_Init(CTIMER, &config);

    /* Configuration 0 */
    matchConfig0.enableCounterReset = true;
    matchConfig0.enableCounterStop = false;
    matchConfig0.matchValue = CTIMER_CLK_FREQ / 2;
    matchConfig0.outControl = kCTIMER_Output_Toggle;
    matchConfig0.outPinInitState = false;
    matchConfig0.enableInterrupt = true;

    /* Configuration 1 */
    matchConfig1.enableCounterReset = true;
    matchConfig1.enableCounterStop = false;
    matchConfig1.matchValue = CTIMER_CLK_FREQ / 2;
    matchConfig1.outControl = kCTIMER_Output_Toggle;
    matchConfig1.outPinInitState = true;
    matchConfig1.enableInterrupt = true;

    CTIMER_RegisterCallBack(CTIMER, &ctimer_callback_table[0], kCTIMER_MultipleCallback);
    CTIMER_SetupMatch(CTIMER, CTIMER_MAT0_OUT, &matchConfig0);
    CTIMER_SetupMatch(CTIMER, CTIMER_MAT1_OUT, &matchConfig1);
    CTIMER_StartTimer(CTIMER);

    while (1)
    {
        /* No timer match output pin connected to a LED
        * toggle LED manually according to match status
        */
        uint32_t mask0, mask1;
        mask0 = (1 << (CTIMER_EMR_EM0_SHIFT + CTIMER_MAT0_OUT));
        mask1 = (1 << (CTIMER_EMR_EM0_SHIFT + CTIMER_MAT1_OUT));
        if (CTIMER->EMR & mask0)
        {
            LED_RED1_ON();
        }
        else
        {
            LED_RED1_OFF();
        }

        if (CTIMER->EMR & mask1)
        {
            LED_RED2_ON();
        }
        else
        {
            LED_RED2_OFF();
        }
    }
}

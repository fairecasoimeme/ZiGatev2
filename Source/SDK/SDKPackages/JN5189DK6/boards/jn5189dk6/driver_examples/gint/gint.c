/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_gint.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_GINT0_PORT kGINT_Port0
#define DEMO_GINT0_POL_MASK ~(1U << 4)
#define DEMO_GINT0_ENA_MASK (1U << 4)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
* @brief Call back for GINT0 event
*/
void gint0_callback(void)
{
    PRINTF("\f\r\nGINT0 event detected\r\n");
}

/*!
* @brief Call back for GINT1 event
*/
void gint1_callback(void)
{
    PRINTF("\f\r\nGINT1 event detected\r\n");
}

/*!
* @brief Main function
*/
int main(void)
{
    /* Security code to allow debug access */
    SYSCON->CODESECURITYPROT = 0x87654320;

    /* attach clock for USART(debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* reset FLEXCOMM for USART */
    RESET_PeripheralReset(kFC0_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kGPIO0_RST_SHIFT_RSTn);

    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitPins();
    /* Clear screen*/
    PRINTF("%c[2J", 27);
    /* Set cursor location at [0,0] */
    PRINTF("%c[0;0H", 27);
    PRINTF("\f\r\nGroup GPIO input interrupt example\r\n");

    /* Initialize GINT0 & GINT1 */
    GINT_Init(GINT0);

#if defined(FSL_FEATURE_SOC_GINT_COUNT) && (FSL_FEATURE_SOC_GINT_COUNT > 1)
    GINT_Init(GINT1);
#endif

    /* Setup GINT0 for edge trigger, "OR" mode */
    GINT_SetCtrl(GINT0, kGINT_CombineOr, kGINT_TrigEdge, gint0_callback);

/* Setup GINT1 for edge trigger, "AND" mode */

#if defined(FSL_FEATURE_SOC_GINT_COUNT) && (FSL_FEATURE_SOC_GINT_COUNT > 1)
    GINT_SetCtrl(GINT1, kGINT_CombineAnd, kGINT_TrigEdge, gint1_callback);
#endif
    /* Select pins & polarity for GINT0 */
    GINT_ConfigPins(GINT0, DEMO_GINT0_PORT, DEMO_GINT0_POL_MASK, DEMO_GINT0_ENA_MASK);

/* Select pins & polarity for GINT1 */

#if defined(FSL_FEATURE_SOC_GINT_COUNT) && (FSL_FEATURE_SOC_GINT_COUNT > 1)
    GINT_ConfigPins(GINT1, DEMO_GINT1_PORT, DEMO_GINT1_POL_MASK, DEMO_GINT1_ENA_MASK);
#endif
    /* Enable callbacks for GINT0 & GINT1 */
    GINT_EnableCallback(GINT0);
    PRINTF("\r\nGINT0 event is configured\r\n");

#if defined(FSL_FEATURE_SOC_GINT_COUNT) && (FSL_FEATURE_SOC_GINT_COUNT > 1)
    GINT_EnableCallback(GINT1);
    PRINTF("\r\nGINT1 event is configured\r\n");
#endif

    PRINTF("\r\nPress corresponding switch to generate event\r\n");
    while (1)
    {
        __WFI();
    }
}

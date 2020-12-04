/*
* Copyright 2019 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "fsl_common.h"
#include "fsl_iocon.h"
#include "fsl_gpio.h"
#include "fsl_pint.h"
#include "fsl_inputmux.h"
#include "pin_mux.h"

#include "board.h"
#include "gpio_pins.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

#if gUartDebugConsole_d
const iocon_group_t dk6_dbgusart_io[] = {
     [0] = {
         .port = 0,
         .pin =  BOARD_DEBUG_UART_TX_PIN,
         .modefunc =  IOCON_PIO_FUNC(IOCON_DBG_UART_MODE_FUNC) | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN,
     },
     [1] = {
         .port = 0,
         .pin =  BOARD_DEBUG_UART_RX_PIN,
         .modefunc =  IOCON_PIO_FUNC(IOCON_DBG_UART_MODE_FUNC)  | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN,
     }
};
#endif

#if gUartAppConsole_d
 const iocon_group_t dk6_hostif_usart_io[] = {
     [0] = {
         .port = 0,
         .pin =  BOARD_APP_UART_TX_PIN,
         .modefunc =  IOCON_PIO_FUNC(IOCON_DBG_UART_MODE_FUNC)  | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN,
     },
     [1] = {
         .port = 0,
         .pin =  BOARD_APP_UART_RX_PIN,
         .modefunc =  IOCON_PIO_FUNC(IOCON_DBG_UART_MODE_FUNC)  | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN,
     }
};
#endif

 const iocon_group_t dk6_swd_io[] = {
      [0] = {
          .port = 0,
          .pin =  IOCON_SWCLK_PIN,
          .modefunc =  IOCON_PIO_FUNC(IOCON_SWD_MODE_FUNC)  | IOCON_MODE_INACT | IOCON_DIGITAL_EN,
      },
      [1] = {
          .port = 0,
          .pin =  IOCON_SWDIO_PIN,
          .modefunc =  IOCON_PIO_FUNC(IOCON_SWD_MODE_FUNC)  | IOCON_MODE_INACT | IOCON_DIGITAL_EN,
      }
 };


/*****************************************************************************
 * Local Prototypes
 ****************************************************************************/


 /*****************************************************************************
 * Private functions
 ****************************************************************************/
static void ConfigureConsolePort(void)
{
    /* UART0 RX/TX pins */
    IOCON_PinMuxSet(IOCON, 0, 8, IOCON_MODE_INACT | IOCON_FUNC2 | IOCON_DIGITAL_EN);
    IOCON_PinMuxSet(IOCON, 0, 9, IOCON_MODE_INACT | IOCON_FUNC2 | IOCON_DIGITAL_EN);
}

static void ConfigureDebugPort(void)
{
    /* SWD SWCLK/SWDIO pins */
    IOCON_PinMuxSet(IOCON, 0, 12, IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGITAL_EN);
    IOCON_PinMuxSet(IOCON, 0, 13, IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGITAL_EN);
#ifdef ENABLE_DEBUG_PORT_SWO
    /* SWD SWO pin (optional) */
    IOCON_PinMuxSet(IOCON, 0, 14, IOCON_FUNC5 | IOCON_MODE_INACT | IOCON_DIGITAL_EN);
    SYSCON->TRACECLKDIV = 0; /* Clear HALT bit */
#endif
}

/*******************************************************************************
 * Code
 ******************************************************************************/
void BOARD_InitPins(void)
{
    /* Enable IOCON clock */
    CLOCK_EnableClock(kCLOCK_Iocon);
    CLOCK_EnableClock(kCLOCK_InputMux);

    /* Console signals */
    ConfigureConsolePort();

    /* Debugger signals */
    ConfigureDebugPort();

    /* IOCON clock left on, this is needed if CLKIN is used. */
    /* Initialize GPIO */
    RESET_PeripheralReset(kGPIO0_RST_SHIFT_RSTn);
    CLOCK_EnableClock(kCLOCK_Gpio0);
}
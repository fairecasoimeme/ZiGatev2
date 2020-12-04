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

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitSPIFIPins:
- options: {callFromInitBoot: 'false', coreID: cm4, enableClock: 'true'}
- pin_list:
  - {pin_num: '22', peripheral: SPIFI, signal: CLK, pin_signal: PIO0_18/SPI1_MISO/ISO7816_IO/CT32B0_MAT1/PWM7/USART0_TXD/SPIFI_CLK/ADC0_4, mode: inactive, invert: disabled,
    open_drain: disabled, filter_off: disabled, slew_rate_0: fast, slew_rate_1: fast, ssel: disabled}
  - {pin_num: '19', peripheral: SPIFI, signal: CSN, pin_signal: PIO0_16/SPI1_SSELN0/ISO7816_RST/PWM5/I2C0_SDA/PDM1_CLK/SPIFI_CSN/ADC0_2, mode: pullUp, invert: disabled,
    open_drain: disabled, filter_off: disabled, slew_rate_0: fast, slew_rate_1: fast, ssel: disabled}
  - {pin_num: '24', peripheral: SPIFI, signal: 'SPIFI_IO, 2', pin_signal: PIO0_20/IR_BLASTER/USART1_TXD/PWM8/RFTX/SPIFI_IO2/ACP, mode: inactive, invert: disabled,
    open_drain: disabled, filter_off: disabled, slew_rate_0: fast, slew_rate_1: fast, ssel: disabled}
  - {pin_num: '21', peripheral: SPIFI, signal: 'SPIFI_IO, 3', pin_signal: PIO0_17/SPI1_MOSI/ISO7816_CLK/SWO/PWM6/CLK_OUT/SPIFI_IO3/ADC0_3, mode: inactive, invert: disabled,
    open_drain: disabled, filter_off: disabled, slew_rate_0: fast, slew_rate_1: fast, ssel: disabled}
  - {pin_num: '23', peripheral: SPIFI, signal: SPIFI_IO0_or_SPIFI_MOSI, pin_signal: PIO0_19/ADO/USART1_RXD/CLK_IN/PWM4/USART0_RXD/SPIFI_IO0/ADC0_5, mode: inactive,
    invert: disabled, open_drain: disabled, filter_off: disabled, slew_rate_0: fast, slew_rate_1: fast, ssel: disabled}
  - {pin_num: '25', peripheral: SPIFI, signal: SPIFI_IO1_or_SPIFI_MISO, pin_signal: PIO0_21/IR_BLASTER/USART1_SCK/PWM9/RFRX/SWO/SPIFI_IO1/ACM, mode: inactive, invert: disabled,
    open_drain: disabled, filter_off: disabled, slew_rate_0: fast, slew_rate_1: fast, ssel: disabled}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitSPIFIPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M4 */
void BOARD_InitSPIFIPins(void)
{
    /* Enables the clock for the I/O controller block. 0: Disable. 1: Enable.: 0x01u */
    CLOCK_EnableClock(kCLOCK_Iocon);

    const uint32_t port0_pin16_config = (/* Pin is configured as SPIFI_CSN */
                                         IOCON_PIO_FUNC7 |
                                         /* Selects pull-up function */
                                         IOCON_PIO_MODE_PULLUP |
                                         /* Fast mode, slew rate control is enabled */
                                         IOCON_PIO_SLEW0_FAST |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Input filter disabled */
                                         IOCON_PIO_INPFILT_OFF |
                                         /* Fast mode, slew rate control is enabled */
                                         IOCON_PIO_SLEW1_FAST |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI |
                                         /* SSEL is disabled */
                                         IOCON_PIO_SSEL_DI);
    /* PORT0 PIN16 (coords: 19) is configured as SPIFI_CSN */
    IOCON_PinMuxSet(IOCON, 0U, 16U, port0_pin16_config);

    const uint32_t port0_pin17_config = (/* Pin is configured as SPIFI_IO3 */
                                         IOCON_PIO_FUNC7 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Fast mode, slew rate control is enabled */
                                         IOCON_PIO_SLEW0_FAST |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Input filter disabled */
                                         IOCON_PIO_INPFILT_OFF |
                                         /* Fast mode, slew rate control is enabled */
                                         IOCON_PIO_SLEW1_FAST |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI |
                                         /* SSEL is disabled */
                                         IOCON_PIO_SSEL_DI);
    /* PORT0 PIN17 (coords: 21) is configured as SPIFI_IO3 */
    IOCON_PinMuxSet(IOCON, 0U, 17U, port0_pin17_config);

    const uint32_t port0_pin18_config = (/* Pin is configured as SPIFI_CLK */
                                         IOCON_PIO_FUNC7 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Fast mode, slew rate control is enabled */
                                         IOCON_PIO_SLEW0_FAST |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Input filter disabled */
                                         IOCON_PIO_INPFILT_OFF |
                                         /* Fast mode, slew rate control is enabled */
                                         IOCON_PIO_SLEW1_FAST |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI |
                                         /* SSEL is disabled */
                                         IOCON_PIO_SSEL_DI);
    /* PORT0 PIN18 (coords: 22) is configured as SPIFI_CLK */
    IOCON_PinMuxSet(IOCON, 0U, 18U, port0_pin18_config);

    const uint32_t port0_pin19_config = (/* Pin is configured as SPIFI_IO0 */
                                         IOCON_PIO_FUNC7 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Fast mode, slew rate control is enabled */
                                         IOCON_PIO_SLEW0_FAST |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Input filter disabled */
                                         IOCON_PIO_INPFILT_OFF |
                                         /* Fast mode, slew rate control is enabled */
                                         IOCON_PIO_SLEW1_FAST |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI |
                                         /* SSEL is disabled */
                                         IOCON_PIO_SSEL_DI);
    /* PORT0 PIN19 (coords: 23) is configured as SPIFI_IO0 */
    IOCON_PinMuxSet(IOCON, 0U, 19U, port0_pin19_config);

    const uint32_t port0_pin20_config = (/* Pin is configured as SPIFI_IO2 */
                                         IOCON_PIO_FUNC7 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Fast mode, slew rate control is enabled */
                                         IOCON_PIO_SLEW0_FAST |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Input filter disabled */
                                         IOCON_PIO_INPFILT_OFF |
                                         /* Fast mode, slew rate control is enabled */
                                         IOCON_PIO_SLEW1_FAST |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI |
                                         /* SSEL is disabled */
                                         IOCON_PIO_SSEL_DI);
    /* PORT0 PIN20 (coords: 24) is configured as SPIFI_IO2 */
    IOCON_PinMuxSet(IOCON, 0U, 20U, port0_pin20_config);

    const uint32_t port0_pin21_config = (/* Pin is configured as SPIFI_IO1 */
                                         IOCON_PIO_FUNC7 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Fast mode, slew rate control is enabled */
                                         IOCON_PIO_SLEW0_FAST |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Input filter disabled */
                                         IOCON_PIO_INPFILT_OFF |
                                         /* Fast mode, slew rate control is enabled */
                                         IOCON_PIO_SLEW1_FAST |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI |
                                         /* SSEL is disabled */
                                         IOCON_PIO_SSEL_DI);
    /* PORT0 PIN21 (coords: 25) is configured as SPIFI_IO1 */
    IOCON_PinMuxSet(IOCON, 0U, 21U, port0_pin21_config);
}
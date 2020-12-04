/*
* Copyright 2019 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

#include "board.h"
#include "fsl_common.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/
       /*!
        * @brief configure all pins for this demo/example
        *
        */

#define IOCON_PIO_DIGITAL_EN 0x80u    /*!<@brief Enables digital function */
#define IOCON_PIO_FUNC7 0x07u         /*!<@brief Selects pin function 7 */
#define IOCON_PIO_INPFILT_OFF 0x0100u /*!<@brief Input filter disabled */
#define IOCON_PIO_INV_DI 0x00u        /*!<@brief Input function is not inverted */
#define IOCON_PIO_MODE_INACT 0x10u    /*!<@brief No addition pin function */
#define IOCON_PIO_MODE_PULLUP 0x00u   /*!<@brief Selects pull-up function */
#define IOCON_PIO_OPENDRAIN_DI 0x00u  /*!<@brief Open drain is disabled */
#define IOCON_PIO_SLEW0_FAST 0x20u    /*!<@brief Fast mode, slew rate control is enabled */
#define IOCON_PIO_SLEW1_FAST 0x0200u  /*!<@brief Fast mode, slew rate control is enabled */
#define IOCON_PIO_SSEL_DI 0x00u       /*!<@brief SSEL is disabled */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitSPIFIPins(void); /* Function assigned for the Cortex-M4 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

#endif /* _PIN_MUX_H_  */

/*****************************************************************************
 *
 * MODULE:             JN-AN-1247
 *
 * COMPONENT:          app.h
 *
 * DESCRIPTION:        Hardware defines
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5169,
 * JN5179, JN5189].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright NXP B.V. 2017-2018. All rights reserved
 *
 ***************************************************************************/
#ifndef _APP_H_
#define _APP_H_
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "zQueue.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
/* default to uart 0 */
#ifndef UART
#define UART            (USART0)
#endif

/* default BAUD rate 115200 */
#ifndef UART_BAUD_RATE
#define UART_BAUD_RATE        115200U
#endif

#define APP_isrUart         FLEXCOMM0_IRQHandler
#define UART0_IRQ           USART0_IRQn
#define APP_BOARD_TEST_GPIO_PORT        0

#define APP_BOARD_TEST_LED0_PIN             4
#define APP_BOARD_TEST_LED3_PIN             10

#if (ZIGBEE_USE_FRAMEWORK != 0)
#define DMA_BUFFER_LENGTH                   256
#define USART_TX_DMA_CHANNEL	            1
#define USART_RX_DMA_CHANNEL	            0
#define UART_CLK_FREQ                       CLOCK_GetFreq(kCLOCK_Fro32M)
#endif

#define LED1_DIO_PIN APP_BOARD_TEST_LED0_PIN
#define LED2_DIO_PIN APP_BOARD_TEST_LED3_PIN
#define LED_DIO_PINS ( 1 << LED1_DIO_PIN | 1 << LED2_DIO_PIN )

extern PUBLIC tszQueue APP_msgAppEvents;

#ifdef CLD_GREENPOWER
extern PUBLIC uint8 u8GPTimerTick;
extern PUBLIC uint8 u8TimerPowerOn;
extern PUBLIC tszQueue APP_msgGPZCLTimerEvents;
extern uint8 au8GPZCLEvent[];
extern uint8 u8GPZCLTimerEvent;
#endif
/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);
void BOARD_SetClockForPowerMode(void);
/*${prototype:end}*/


#endif /* _APP_H_ */

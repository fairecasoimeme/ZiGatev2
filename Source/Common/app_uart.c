/*****************************************************************************
 *
 * MODULE:
 *
 * COMPONENT:
 *
 * AUTHOR:
 *
 * DESCRIPTION:
 *
 * $HeadURL: $
 *
 * $Revision: $
 *
 * $LastChangedBy: $
 *
 * $LastChangedDate: $
 *
 * $Id:  $
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5179].
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
 * Copyright NXP B.V. 2016. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include "board.h"
#include "app.h"
#include "fsl_usart.h"
#include "app_uart.h"
#include "dbg.h"
#include "portmacro.h"

#if (ZIGBEE_USE_FRAMEWORK == 0)
#include <stdarg.h>
#include "stdlib.h"
#include "app_common.h"
#include "ZQueue.h"
#else
#include "usart_dma_rxbuffer.h"
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/




/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/


/****************************************************************************
 *
 * NAME: vUART_Init
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PUBLIC void UART_vInit(void)
{
#if (ZIGBEE_USE_FRAMEWORK == 0)
    usart_config_t config;

    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = UART_BAUD_RATE;
    config.enableTx = true;
    config.enableRx = true;

    USART_Init(UART, &config, CLOCK_GetFreq(kCLOCK_Fro32M));

    /* Enable RX interrupt. */
    USART_EnableInterrupts(UART, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    EnableIRQ(UART0_IRQ);
#else
    USART_DMA_Init();
#endif

}

/****************************************************************************
 *
 * NAME: vUART_SetBuadRate
 *
 * DESCRIPTION:
 *
 * PARAMETERS: Name        RW  Usage
 *
 * RETURNS:
 *
 ****************************************************************************/

PUBLIC void UART_vSetBaudRate ( uint32    u32BaudRate )
{

    int result;

    /* setup baudrate */
    result = USART_SetBaudRate(UART, u32BaudRate, CLOCK_GetFreq(kCLOCK_Fro32M));
    if (kStatus_Success != result)
    {
        //DBG_vPrintf(TRACE_UART,"\r\nFailed to set UART speed ");
    }

}


#if (ZIGBEE_USE_FRAMEWORK == 0)
/****************************************************************************
 *
 * NAME: APP_isrUart
 *
 * DESCRIPTION: Handle interrupts from uart
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/


void APP_isrUart ( void )
{
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(UART)) {
            USART_ClearStatusFlags(UART, kUSART_RxError);
            uint8 u8Byte = USART_ReadByte(UART);
            ZQ_bQueueSend(&APP_msgSerialRx, &u8Byte);
    }
}

#endif
/****************************************************************************
 *
 * NAME: UART_vRtsStopFlow
 *
 * DESCRIPTION:
 * Set UART RS-232 RTS line high to stop any further data coming in
 *
 ****************************************************************************/

PUBLIC void UART_vRtsStopFlow ( void )
{
	//USART_EnableCTS(UART, TRUE);

}

/****************************************************************************
 *
 * NAME: UART_vRtsStartFlow
 *
 * DESCRIPTION:
 * Set UART RS-232 RTS line low to allow further data
 *
 ****************************************************************************/

PUBLIC void UART_vRtsStartFlow(void)
{
	//USART_EnableCTS(UART, FALSE);
}
/* [I SP001222_P1 283] end */

/****************************************************************************
 *
 * NAME: vUART_TxChar
 *
 * DESCRIPTION:
 * Set UART RS-232 RTS line low to allow further data
 *
 ****************************************************************************/
PUBLIC void UART_vTxChar(uint8 u8Char)
{

    USART_WriteByte(UART, u8Char);
    /* Wait to finish transfer */
    while (!(UART->FIFOSTAT & USART_FIFOSTAT_TXEMPTY_MASK))
    {
    }
}

/****************************************************************************
 *
 * NAME: vUART_TxReady
 *
 * DESCRIPTION:
 * Set UART RS-232 RTS line low to allow further data
 *
 ****************************************************************************/
PUBLIC bool_t UART_bTxReady()
{

    return ( (kUSART_TxFifoEmptyFlag|kUSART_TxFifoNotFullFlag) & USART_GetStatusFlags(UART) );

}

/****************************************************************************
 *
 * NAME: vUART_SetTxInterrupt
 *
 * DESCRIPTION:
 * Enable / disable the tx interrupt
 *
 ****************************************************************************/
PUBLIC void UART_vSetTxInterrupt(bool_t bState)
{

    USART_DisableInterrupts(UART, (kUSART_TxErrorInterruptEnable | kUSART_RxErrorInterruptEnable | kUSART_TxLevelInterruptEnable| kUSART_RxLevelInterruptEnable) );
    if (bState == FALSE)
    {
        USART_EnableInterrupts(UART, (kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable) );
    }
    else
    {
        USART_EnableInterrupts(UART, (kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable | kUSART_TxLevelInterruptEnable|kUSART_TxErrorInterruptEnable ));
    }

}


/****************************************************************************
 *
 * NAME: UART_vOverrideInterrupt
 *
 * DESCRIPTION:
 *
 *
 ****************************************************************************/
PUBLIC void UART_vOverrideInterrupt(bool_t bState)
{

    if (!bState)
    {
        USART_DisableInterrupts(UART, (kUSART_TxErrorInterruptEnable | kUSART_RxErrorInterruptEnable | kUSART_TxLevelInterruptEnable| kUSART_RxLevelInterruptEnable) );
    }
    else
    {
        USART_EnableInterrupts(UART, (kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable | kUSART_TxLevelInterruptEnable|kUSART_TxErrorInterruptEnable ));
    }

}
#if (ZIGBEE_USE_FRAMEWORK != 0)

/****************************************************************************
 *
 * NAME: UART_bBufferReceive
 *
 * DESCRIPTION:
 *
 * PARAMETERS: Name        RW  Usage
 *
 * RETURNS:
 *
 ****************************************************************************/
PUBLIC bool_t UART_bBufferReceive ( uint8* pu8Data )
{
      bool_t bReturn = FALSE;
      if (USART_DMA_ReadBytes(pu8Data,1) == 0)
      {
          bReturn = FALSE;
      }
      else
      {
          bReturn = TRUE;
      }
      return bReturn;
}
#endif
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

/*****************************************************************************
 *
 * MODULE:             Appliance Statistics Cluster
 *
 * COMPONENT:          ApplianceControlClientCommands.c
 *
 * DESCRIPTION:        Appliance Control Cluster definition
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5148, JN5142, JN5139]. 
 * You, and any third parties must reproduce the copyright and warranty notice 
 * and any other legend of ownership on each  copy or partial copy of the software.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  
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
 * Copyright NXP B.V. 2012. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>

#include "zcl.h"
#include "zcl_customcommand.h"

#include "ApplianceControl.h"
#include "ApplianceControl_internal.h"

#include "dbg.h"


#ifdef DEBUG_CLD_APPLIANCE_CONTROL
#define TRACE_APPLIANCE_CONTROL    TRUE
#else
#define TRACE_APPLIANCE_CONTROL    FALSE
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
/***        Public Functions                                              ***/
/****************************************************************************/

#ifdef APPLIANCE_CONTROL_SERVER
/****************************************************************************
 **
 ** NAME:       eCLD_ACSignalStateResponseORSignalStateNotificationSend
 **
 ** DESCRIPTION:
 ** Sends an Execution of command
 **
 ** PARAMETERS:                                                         Name                               Usage
 ** uint8                                                               u8SourceEndPointId                 Source EP Id
 ** uint8                                                               u8DestinationEndPointId            Destination EP Id
 ** tsZCL_Address                                                       *psDestinationAddress              Destination Address
 ** uint8                                                               *pu8TransactionSequenceNumber      Sequence number Pointer
 **    teCLD_ApplianceControl_ServerCommandId                           eCommandId                         Command ID
 **    bool_t                                                             bApplianceStatusTwoPresent         Is ApplianceStatusTwo is presnet?
 ** tsCLD_AC_SignalStateResponseORSignalStateNotificationPayload        *psPayload                         Payload
 **
 ** RETURN:
 ** teZCL_Status
 **
 ****************************************************************************/
PUBLIC  teZCL_Status eCLD_ACSignalStateResponseORSignalStateNotificationSend(
                    uint8                                                               u8SourceEndPointId,
                    uint8                                                               u8DestinationEndPointId,
                    tsZCL_Address                                                       *psDestinationAddress,
                    uint8                                                               *pu8TransactionSequenceNumber,
                    teCLD_ApplianceControl_ServerCommandId                              eCommandId,
                    bool_t                                                                bApplianceStatusTwoPresent,
                    tsCLD_AC_SignalStateResponseORSignalStateNotificationPayload        *psPayload)
{

    tsZCL_TxPayloadItem asPayloadDefinition[] = {
    {1,                                     E_ZCL_UINT8,                            &psPayload->eApplianceStatus},
    {1,                                     E_ZCL_UINT8,                            &psPayload->u8RemoteEnableFlagAndDeviceStatusTwo},
    {1,                                     E_ZCL_UINT24,                           &psPayload->u24ApplianceStatusTwo},
                                                };
    
    uint8    u8PayloadSize;
    if(bApplianceStatusTwoPresent){
        u8PayloadSize = sizeof(asPayloadDefinition) / sizeof(tsZCL_TxPayloadItem);
    }else{
        u8PayloadSize = 2; 
    }
    return eZCL_CustomCommandSend(u8SourceEndPointId,
                                  u8DestinationEndPointId,
                                  psDestinationAddress,
                                  APPLIANCE_MANAGEMENT_CLUSTER_ID_APPLIANCE_CONTROL,
                                  TRUE,
                                  eCommandId,
                                  pu8TransactionSequenceNumber,
                                  asPayloadDefinition,
                                  FALSE,
                                  0,
                                  u8PayloadSize);
}

/****************************************************************************
 **
 ** NAME:       eCLD_ACExecutionOfCommandReceive
 **
 ** DESCRIPTION:
 ** Receives Execution Of command
 **
 ** PARAMETERS:                                                     Name                               Usage
 ** ZPS_tsAfEvent                                                   *pZPSevent                         Event Received
 ** uint8                                                           *pu8TransactionSequenceNumber      Sequence number Pointer
 ** tsCLD_AC_ExecutionOfCommandPayload                              *psPayload                         Payload
 **
 ** RETURN:
 ** teZCL_Status
 **
 ****************************************************************************/
PUBLIC  teZCL_Status eCLD_ACExecutionOfCommandReceive(
                                                ZPS_tsAfEvent                                                       *pZPSevent,
                                                uint8                                                               *pu8TransactionSequenceNumber,
                                                tsCLD_AC_ExecutionOfCommandPayload                                  *psPayload)
{
    uint16 u16ActualQuantity;
    tsZCL_RxPayloadItem asPayloadDefinition[] = {
             {1,                                &u16ActualQuantity,         E_ZCL_UINT8,                           &(psPayload->eExecutionCommandId)}
                                                          };
    return eZCL_CustomCommandReceive(pZPSevent,
                                     pu8TransactionSequenceNumber,
                                     asPayloadDefinition,
                                     sizeof(asPayloadDefinition) / sizeof(tsZCL_RxPayloadItem),
                                     E_ZCL_ACCEPT_EXACT);

}

/****************************************************************************
 **
 ** NAME:       eCLD_ACSignalStateReceive
 **
 ** DESCRIPTION:
 ** Receives signal state command
 **
 ** PARAMETERS:                                                     Name                               Usage
 ** ZPS_tsAfEvent                                                   *pZPSevent                         Event Received
 ** uint8                                                           *pu8TransactionSequenceNumber      Sequence number Pointer
 **
 ** RETURN:
 ** teZCL_Status
 **
 ****************************************************************************/
PUBLIC  teZCL_Status eCLD_ACSignalStateReceive(
                                                ZPS_tsAfEvent                                                       *pZPSevent,
                                                uint8                                                               *pu8TransactionSequenceNumber)
{
    return eZCL_CustomCommandReceive(pZPSevent,
                                     pu8TransactionSequenceNumber,
                                     0,
                                     0,
                                     E_ZCL_ACCEPT_EXACT|E_ZCL_DISABLE_DEFAULT_RESPONSE);

}
#endif
/****************************************************************************/
/***        Private Functions                                             ***/
/****************************************************************************/
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

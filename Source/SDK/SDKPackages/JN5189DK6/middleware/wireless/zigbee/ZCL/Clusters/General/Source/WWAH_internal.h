/*****************************************************************************
 *
 * MODULE:             Door Lock Cluster
 *
 * COMPONENT:          WWAH_internal.h
 *
 * DESCRIPTION:        The internal API for the Door Lock Cluster
 *
 *****************************************************************************
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

#ifndef  WWAH_INTERNAL_H_INCLUDED
#define  WWAH_INTERNAL_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/

#include "jendefs.h"

#include "zcl.h"
#include "WWAH.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef struct
{
    zuint8                          u8NumberOfBeacons;
    zuint8                          *pu8BeaconSurvey;
}tsCLD_WWAH_SurveyBeaconsResponsePayloadInternal;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC teZCL_Status eCLD_WWAHCommandHandler(
        ZPS_tsAfEvent               *pZPSevent,
        tsZCL_EndPointDefinition    *psEndPointDefinition,
        tsZCL_ClusterInstance       *psClusterInstance);

#ifdef WWAH_CLIENT
PUBLIC  teZCL_Status eCLD_WWAHCommandAPSLinkKeyAuthorizationQueryResponseReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_APSLinkKeyAuthorizationQueryResponsePayload      *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandPoweringOnOffNotificationReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_PoweringOnOffNotificationPayload                 *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandShortAddressChangeReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_ShortAddressChangePayload                *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandAPSACKEnablementQueryResponseReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_APSACKEnablementQueryResponsePayload             *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandPowerDescriptorChangeReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_PowerDescriptorChangePayload             *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandDebugReportQueryResponseReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_DebugReportQueryResponsePayload                  *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHNewDebugReportNotificationReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAHNewDebugReportNotificationPayload                 *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandTrustCenterForClusterQueryResponseReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_TCForClusterServerQueryResponsePayload           *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandSurveyBeaconsResponseReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_SurveyBeaconsResponsePayloadInternal             *psPayload);

#endif /* WWAH_CLIENT */

#ifdef WWAH_SERVER
PUBLIC  teZCL_Status eCLD_WWAHCommandEnableOrDisableAPSLinkKeyAuthorizationRequestReceive(
        ZPS_tsAfEvent                                                     *pZPSevent,
        uint8                                                             *pu8TransactionSequenceNumber,
        tsCLD_WWAH_EnableOrDisableAPSLinkKeyAuthorizationRequestPayload   *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandAPSLinkKeyAuthorizationQueryRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_APSLinkKeyAuthorizationQueryRequestPayload       *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandRequestNewAPSLinkKeyRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandEnableWWAHAppEventRetryAlgorithmRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_EnableWWAHAppEventRetryAlgorithmRequestPayload   *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandDisableWWAHAppEventRetryAlgorithmRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandRequestTimeRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandEnableWWAHRejoinAlgorithmRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_EnableWWAHRejoinAlgorithmRequestPayload          *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandDisableWWAHRejoinAlgorithmRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandSetIASZoneEnrollmentMethodRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_SetIASZoneEnrollmentMethodRequestPayload         *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandClearBindingTableRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandEnablePeriodicRouterCheckInsRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_EnablePeriodicRouterCheckInsRequestPayload       *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandDisablePeriodicRouterCheckInsRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandSetMACPollCCAWaitTimeRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_SetMACPollCCAWaitTimeRequestPayload              *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandSetPendingNetworkUpdateRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_SetPendingNetworkUpdateRequestPayload            *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandRequireAPSACKsOnUnicastsRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_RequireAPSACKsOnUnicastsRequestPayload           *psPayload);        
PUBLIC  teZCL_Status eCLD_WWAHCommandRemoveAPSACKsOnUnicastsRequirementRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandAPSACKEnablementQueryRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandDebugReportQueryRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_DebugReportQueryRequestPayload                   *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandUseTrustCenterForClusterServerRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_UseTCForClusterServerPayload                     *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandDisableMGMTLeaveWithoutRejoinRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandEnableWWAHParentClassificationRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandEnableTCSecurityonNwkKeyRotationRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHDisableWWAHBadParentRecoveryRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHDisableConfigurationModeRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandSurveyBeaconsReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_SurveyBeaconsPayload                             *psPayload);  
PUBLIC  teZCL_Status eCLD_WWAHCommandDisableOTADowngradesRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandDisableTouchlinkInterpanMessageRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandDisableParentClassificationRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandEnableBadParentRecoveryRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandEnableConfigurationModeRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC teZCL_Status eCLD_WWAHCommandTrustCenterForClusterServerQueryRequestReceive(
        ZPS_tsAfEvent                                               *pZPSevent,
        uint8                                                       *pu8TransactionSequenceNumber);
PUBLIC  teZCL_Status eCLD_WWAHCommandAPSLinkKeyAuthorizationQueryResponseSend(
        uint8                                                       u8SourceEndPointId,
        uint8                                                       u8DestinationEndPointId,
        tsZCL_Address                                               *psDestinationAddress,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_APSLinkKeyAuthorizationQueryResponsePayload      *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandAPSACKEnablementQueryResponseSend(
        uint8                                                       u8SourceEndPointId,
        uint8                                                       u8DestinationEndPointId,
        tsZCL_Address                                               *psDestinationAddress,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_APSACKEnablementQueryResponsePayload             *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandDebugReportQueryResponseSend(
        uint8                                                       u8SourceEndPointId,
        uint8                                                       u8DestinationEndPointId,
        tsZCL_Address                                               *psDestinationAddress,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_DebugReportQueryResponsePayload                  *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandNewDebugReportNotificationSend(
        uint8                                                       u8SourceEndPointId,
        uint8                                                       u8DestinationEndPointId,
        tsZCL_Address                                               *psDestinationAddress,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAHNewDebugReportNotificationPayload                 *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandTrustCenterForClusterServerQueryResponseSend(
        uint8                                                       u8SourceEndPointId,
        uint8                                                       u8DestinationEndPointId,
        tsZCL_Address                                               *psDestinationAddress,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_TCForClusterServerQueryResponsePayload       *psPayload);
PUBLIC  teZCL_Status eCLD_WWAHCommandSurveyBeaconsResponseSend(
        uint8                                                       u8SourceEndPointId,
        uint8                                                       u8DestinationEndPointId,
        tsZCL_Address                                               *psDestinationAddress,
        uint8                                                       *pu8TransactionSequenceNumber,
        tsCLD_WWAH_SurveyBeaconsResponsePayloadInternal             *psPayload);        
#endif /* WWAH_SERVER */

#endif /* WWAH_INTERNAL_H_INCLUDED*/

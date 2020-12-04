/*
* Copyright 2020 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*!=================================================================================================
\file       app_latency_measure.c
\brief      This is a public source file for the latency measurement application
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "app_latency_measure.h"
#include "app_init.h"

#include "network_utils.h"
#include "FunctionLib.h"

#include <openthread/thread.h>
#include <openthread/instance.h>
#include <openthread/cli.h>
#include <openthread/ip6.h>
#include <openthread/coap.h>
/*==================================================================================================
Private macros
==================================================================================================*/
#define gTimeMeasure_TestUriPathString_c    "tst"    /*!< Latency Measure Test URI Path string */
#define gLatMeasure_TestCliString           "latency"

/*==================================================================================================
Private function prototypes
==================================================================================================*/
static uint8_t *LatencyMeasure_TestTlv(otInstance *pOtInstance, uint16_t noOfBytes,
                 uint8_t *inputBytes, uint32_t *pOutSize, bool_t isMcast);
static void LatencyMeasure_TestReqCb(void *aContext, otMessage *aMessage,
            const otMessageInfo *aMessageInfo);
static void LatencyMeasure_TestRspCb(void *aContext, otMessage *aMessage,
            const otMessageInfo *aMessageInfo, otError aResult);
static void LatencyMeasure_TestTlvPayload(otInstance *pOtInstance,uint8_t cmd, uint8_t *pData,
            uint8_t *pDataIn, uint32_t dataInLen, bool_t isMcast);
static void LatencyMeasure_ProcessTestRsp(uint8_t *pData, timeMeasure_RspData_t *rspData);
static void LatencyMeasure_CliCommand( int argc, char *argv[]);

/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private global variables declarations
==================================================================================================*/
static uint32_t mResultsReqTime;    /*!< Large network request latency saved on multicast */
static uint8_t mReceivedSeqNum;     /*!< Sequence number of the multicast packet */

timeMeasureEventCallback_t timeMeasureEventCb = NULL;

otCoapResource gAPP_TIME_MEASURE_URI_PATH = {.mUriPath = gTimeMeasure_TestUriPathString_c,
                                             .mHandler = LatencyMeasure_TestReqCb,
                                             .mContext = NULL, .mNext = NULL };
static uint16_t mMsgId = 0;
const otCliCommand latMeasureCliCmd = {.mName = gLatMeasure_TestCliString,
                                      .mCommand = LatencyMeasure_CliCommand };
static otInstance *mpOtInstance;
/*==================================================================================================
Public global variables declarations
==================================================================================================*/


/*==================================================================================================
Public functions
==================================================================================================*/

/*!*************************************************************************************************
\fn     void LatencyMeasure_Init(otInstance *pOtInstance)
\brief  This function is used to initialize network diagnostic module.

\param  [in]    pOtInstance    Open Thread instance pointer
***************************************************************************************************/
void LatencyMeasure_Init
(
    otInstance *pOtInstance
)
{
    mpOtInstance = pOtInstance;

    gAPP_TIME_MEASURE_URI_PATH.mContext = pOtInstance;
    otCoapAddResource(pOtInstance, &gAPP_TIME_MEASURE_URI_PATH);

    otCliSetUserCommands(&latMeasureCliCmd, sizeof(latMeasureCliCmd)/sizeof(otCliCommand));
}

/*!*************************************************************************************************
\fn     timeMeasureStatus_t LatencyMeasure_TestReq(otInstance *pOtInstance, timeMeasure_CoapMsg_t *pCoapMsg,
        uint16_t *pMsgId)
\brief  This function is used to send a Time Measure Test Request (Large network testing).

\param  [in]    instanceId    Open Thread instance pointer
        [in]    pCoapMsg      Pointer to CoAP message
        [out]   pMsgId        CoAP message Id: used for synchronization between Req and Rsp

\return         timeMeasureStatus_t   Status of the operation
***************************************************************************************************/
timeMeasureStatus_t LatencyMeasure_TestReq
(
    otInstance *pOtInstance,
    timeMeasure_CoapMsg_t *pCoapMsg,
    uint16_t *pMsgId
)
{
    timeMeasureStatus_t status = gMeasureStatus_InvalidParam_c;
    timeMeasure_ReqCoapMsg_t pMsg;
    uint8_t *coapMsg = NULL;
    otCoapType coapType;
    otCoapCode coapCode;
    bool sendMsg = FALSE;
    uint16_t bufferSize;
    otError error;

    if((NULL != pOtInstance) && (otThreadGetDeviceRole(pOtInstance)) > OT_DEVICE_ROLE_DETACHED)
    {
        status = gMeasureStatus_Success_c;
        otMessageInfo messageInfo;
        otMessage *pOtCoapMsg = otCoapNewMessage(pOtInstance, NULL);

        if(pOtCoapMsg)
        {
            /* Complete CoAP message */
            FLib_MemSet(&messageInfo, 0, sizeof(messageInfo));
            messageInfo.mPeerAddr = pCoapMsg->dstIpAddr;
            messageInfo.mPeerPort = OT_DEFAULT_COAP_PORT;

            if((pCoapMsg->payload[0] == gDiagnosticTlv_DiagTestData_c) ||
               (pCoapMsg->payload[0] == gDiagnosticTlv_DiagTestResults_c))
            {
                coapCode = OT_COAP_CODE_GET;

                /* Get size for requested times structure, received payload length and payload over FSCI */
                bufferSize = sizeof(timeMeasure_ReqCoapMsg_t) + pCoapMsg->payloadSize + 1;
                pMsg.cmdId = pCoapMsg->payload[0];

                if(pCoapMsg->payload[0] == gDiagnosticTlv_DiagTestData_c)
                {
                    uint8_t idx = 0;
                    /* Get tx req timestamp */
                    uint32_t txReqTimestamp = NWKU_GetTimestampMs();

                    coapMsg = MEM_BufferAlloc(bufferSize);

                    if(coapMsg != NULL)
                    {
                        FLib_MemSet(coapMsg, 0, bufferSize);
                        /* Clear send structure */
                        FLib_MemSet(&pMsg.reqLatency, 0, sizeof(uint32_t));
                        FLib_MemSet(&pMsg.rspLatency, 0, sizeof(uint32_t));
                        FLib_MemSet(&pMsg.offset, 0, sizeof(uint32_t));

                        /* Copy sequence number into structure */
                        pMsg.seqNum = pCoapMsg->payload[1];
                        /* Copy tx request timestamp for computing */
                        FLib_MemCpy(&pMsg.reqLatency, &txReqTimestamp, sizeof(uint32_t));

                        /* Copy structure into CoAP payload */
                        FLib_MemCpy(coapMsg + idx, &pMsg, sizeof(pMsg));
                        idx += sizeof(pMsg);

                        /* Copy payload length */
                        FLib_MemCpy(coapMsg + idx, &pCoapMsg->payloadSize, 1);
                        idx += 1;

                        /* Copy payload */
                        FLib_MemCpy(coapMsg + idx, &pCoapMsg->payload[2], pCoapMsg->payloadSize);
                    }
                    else
                    {
                        return gMeasureStatus_NoMem_c;
                    }
                }

                /* Add CoAP options */
                if (IP6_IsMulticastAddr(&pCoapMsg->dstIpAddr.mFields))
                {
                    coapType = OT_COAP_TYPE_NON_CONFIRMABLE;
                }
                else
                {
                    coapType = OT_COAP_TYPE_CONFIRMABLE;
                }

                if(pCoapMsg->payload[0] == gDiagnosticTlv_DiagTestData_c)
                {
                    /* Send the command and the sender timestamp */
                    sendMsg = TRUE;
                }
                else
                {
                    /* Send the command id for gathering the results */
                    bufferSize = 1;
                    coapMsg = pCoapMsg->payload;
                    sendMsg = TRUE;
                }
            }
            else if((pCoapMsg->payload[0] == gDiagnosticTlv_DiagTestWarmReset_c) ||
                    (pCoapMsg->payload[0] == gDiagnosticTlv_DiagTestColdReset_c))
            {

                /* Send the command id */
                coapCode = OT_COAP_CODE_POST;
                coapType = OT_COAP_TYPE_NON_CONFIRMABLE;
                bufferSize = 1;
                coapMsg = pCoapMsg->payload;
                sendMsg = TRUE;
            }

            if (sendMsg)
            {
                otCoapMessageInit(pOtCoapMsg, coapType, coapCode);
                otCoapMessageGenerateToken(pOtCoapMsg, 4);

                error = otCoapMessageAppendUriPathOptions(pOtCoapMsg, gTimeMeasure_TestUriPathString_c);
                error |= otCoapMessageSetPayloadMarker(pOtCoapMsg);
                error |= otMessageAppend(pOtCoapMsg, (const void *)coapMsg, bufferSize);

                if (error == OT_ERROR_NONE)
                {
                    error = otCoapSendRequest(pOtInstance, pOtCoapMsg, &messageInfo, LatencyMeasure_TestRspCb, NULL);
                    if ((error != OT_ERROR_NONE) && (pOtCoapMsg != NULL))
                    {
                        otMessageFree(pOtCoapMsg);
                    }
                }
                else
                {
                    otMessageFree(pOtCoapMsg);
                }

                MEM_BufferFree(coapMsg);
            }
            else
            {
                status = gMeasureStatus_InvalidParam_c;
                otMessageFree(pOtCoapMsg);
            }

            if(pMsgId)
            {
                *pMsgId = mMsgId++;
            }
        }
        else
        {
            status = gMeasureStatus_NoMem_c;
        }
    }
    return status;
}

/*==================================================================================================
Private functions
==================================================================================================*/
static void LatencyMeasure_CliCommand
(
    int argc,
    char *argv[]
)
{

    timeMeasure_CoapMsg_t pCoapMsg;
    otError error = OT_ERROR_INVALID_ARGS;
    timeMeasureStatus_t status = gMeasureStatus_InvalidParam_c;

    if (argc == 4)
    {
        error = otIp6AddressFromString(argv[0], &pCoapMsg.dstIpAddr);

        if (OT_ERROR_NONE == error)
        {
            pCoapMsg.payloadSize = NWKU_atoi(argv[1]);
            pCoapMsg.payload = MEM_BufferAlloc(pCoapMsg.payloadSize + 2);

            if (NULL != pCoapMsg.payload)
            {
                pCoapMsg.payload[0] = NWKU_AsciiToHex((uint8_t*)argv[2], 2);
                pCoapMsg.payload[1] = NWKU_AsciiToHex((uint8_t*)argv[3], 2);
                FLib_MemSet(pCoapMsg.payload + 2, 0xAA, pCoapMsg.payloadSize);

                status = LatencyMeasure_TestReq(mpOtInstance, &pCoapMsg, NULL);
                MEM_BufferFree(pCoapMsg.payload);
            }
            else
            {
                status = gMeasureStatus_NoMem_c;
            }
        }
    }

    if (error == OT_ERROR_INVALID_ARGS)
    {
        otCliOutputFormat("Invalid arguments\n\r");
    }
    else
    {
        if (gMeasureStatus_Success_c == status)
        {
            otCliOutputFormat("Started latency test\n\r");
        }
        else
        {
            otCliOutputFormat("Latency test start fail\n\r");
        }
    }
}

/*!*************************************************************************************************
\private
\fn     static uint8_t *MgmtDiagnostic_DiagTestTlv(instanceId_t instanceId, uint16_t noOfBytes,
                                                   uint8_t *inputBytes, uint32_t *pOutSize, bool_t isMcast)
\brief  Network diagnostic generic large network TLV function.

\param  [in]   instanceId      Thread instance id
\param  [in]   noOfBytes       Number of received bytes
\param  [in]   inputBytes      Pointer to received bytes
\param  [out]  pOutSize        Pointer to calculated length of the returned response
\param  [in]   isMcast         Flag for signaling received multicast packet

\return        uint8_t *       Pointer to returned response
 ***************************************************************************************************/
static uint8_t *LatencyMeasure_TestTlv
(
    otInstance *pOtInstance,
    uint16_t noOfBytes,
    uint8_t *inputBytes,
    uint32_t *pOutSize,
    bool_t isMcast
)
{
    uint8_t cmd = 0;
    uint32_t outLength = 0;
    uint8_t *pOutDiagTlv = NULL;
    uint32_t reqLatency;
    uint16_t currentIdx = 0;
    bool_t negative = FALSE;

    cmd = inputBytes[0];

    if(cmd != THR_ALL_FFs8)
    {
        if(cmd == gDiagnosticTlv_DiagTestData_c)
        {
            uint32_t rxReqTimestamp = 0, txReqTimestamp = 0;
            /* Get rx req timestamp */
            rxReqTimestamp = NWKU_GetTimestampMs();
            FLib_MemCpy(&txReqTimestamp, &inputBytes[1], sizeof(uint32_t));

            /* Length of the returned packet */
            if(!isMcast)
            {
                outLength = noOfBytes;
            }
            else
            {
                outLength = sizeof(timeMeasure_ReqCoapMsg_t);
            }

            /* Calculate the request latency */
            if(txReqTimestamp >= rxReqTimestamp)
            {
                reqLatency = txReqTimestamp - rxReqTimestamp;
            }
            else
            {
                reqLatency = rxReqTimestamp - txReqTimestamp;
                /* Mark the fact that the resulted latency is negative */
                negative = TRUE;
            }
        }
        else if(cmd == gDiagnosticTlv_DiagTestResults_c)
        {
            /* Length of the returned packet */
            outLength = sizeof(timeMeasure_ReqCoapMsg_t);
        }

        if(outLength)
        {
            pOutDiagTlv = MEM_BufferAlloc(outLength);
            if(pOutDiagTlv)
            {
                FLib_MemSet(pOutDiagTlv, 0, outLength);

                /* Return command id in response header */
                pOutDiagTlv[0] = cmd;
                currentIdx += 1;

                if(cmd == gDiagnosticTlv_DiagTestData_c)
                {
                    /* Copy the calculated request latency and mark the
                     * sign of the result into response packet */
                    FLib_MemCpy(&pOutDiagTlv[currentIdx], &reqLatency, sizeof(uint32_t));
                    /* Mark the sign of the result into the offset member if necessary */
                    if(negative)
                    {
                        uint32_t sign = 1;
                        currentIdx += (2 * sizeof(uint32_t));
                        FLib_MemCpy(&pOutDiagTlv[currentIdx], &sign, sizeof(uint32_t));
                        currentIdx -= sizeof(uint32_t);
                    }
                    else
                    {
                        currentIdx += sizeof(uint32_t);
                    }
                }

                /* Get payload */
                LatencyMeasure_TestTlvPayload(pOtInstance,cmd, &pOutDiagTlv[currentIdx],
                                              inputBytes, outLength, isMcast);
            }
        }
        if(isMcast)
        {
            if(cmd == gDiagnosticTlv_DiagTestData_c)
            {
                uint8_t idx = (sizeof(timeMeasure_ReqCoapMsg_t) - 1);

                /* Save the computed request latency */
                mResultsReqTime = reqLatency;
                /* Save the received sequence number */
                mReceivedSeqNum = inputBytes[idx];

                if(timeMeasureEventCb != NULL)
                {
                    timeMeasureEventCb(mResultsReqTime, mReceivedSeqNum);
                }
            }
            else if((cmd == gDiagnosticTlv_DiagTestColdReset_c) || (cmd == gDiagnosticTlv_DiagTestWarmReset_c))
            {
                LatencyMeasure_TestTlvPayload(pOtInstance,cmd, NULL, NULL, outLength, isMcast);
            }
        }
    }

    if(pOutSize)
    {
        *pOutSize = outLength;
    }

    return pOutDiagTlv;
}

/*!*************************************************************************************************
\private
\fn     static void MgmtDiagnostic_DiagTestReqCb(coapSessionStatus_t sessionStatus, uint8_t *pData,
                                                 coapSession_t *pSession, uint32_t dataLen)
\brief  Callback for Diagnostic_DiagTestReq command.

\param  [in]    sessionStatus    Session status
        [in]    pData            Pointer to data
        [in]    pSession         Pointer to CoAP session
        [in]    pDataLen         Data length
***************************************************************************************************/
static void LatencyMeasure_TestReqCb
(
    void *aContext,
    otMessage *aMessage,
    const otMessageInfo *aMessageInfo
)
{
    uint32_t rspLength = 0x00;
    uint8_t *pRspPayload = NULL;
    bool_t isMcast = FALSE;

    otCoapCode msgCode;
    otCoapType msgType = otCoapMessageGetType(aMessage);

    otError error = OT_ERROR_NONE;
    uint16_t dataLen = otMessageGetLength(aMessage) - otMessageGetOffset(aMessage);

    uint8_t *pData = MEM_BufferAlloc(dataLen);

    if (NULL != pData)
    {
        otMessageRead(aMessage, otMessageGetOffset(aMessage), (void *)pData, dataLen);

        if(msgType == OT_COAP_TYPE_NON_CONFIRMABLE)
        {
            isMcast = TRUE;
        }

        /* Add CoAP options */
        msgType = OT_COAP_TYPE_ACKNOWLEDGMENT;
        msgCode = OT_COAP_CODE_CONTENT;

        pRspPayload = LatencyMeasure_TestTlv(aContext, dataLen, pData, &rspLength, isMcast);

        if(!pRspPayload && !isMcast)
        {
            msgCode = OT_COAP_CODE_BAD_REQUEST;
        }
    }
    else
    {
        msgType = OT_COAP_TYPE_ACKNOWLEDGMENT;
        msgCode = OT_COAP_CODE_INTERNAL_ERROR;
    }

    /* Send CoAP message only on unicast requests */
    if(!isMcast)
    {
        otMessage *responseMessage = NULL;
        responseMessage = otCoapNewMessage(aContext, NULL);

        if(responseMessage != NULL)
        {
            error = otCoapMessageInitResponse(responseMessage, aMessage, msgType, msgCode);

            if (NULL != pRspPayload)
            {
                error |= otCoapMessageSetPayloadMarker(responseMessage);
                error |= otMessageAppend(responseMessage, (const void *)pRspPayload, rspLength);
            }
            if (error == OT_ERROR_NONE)
            {
                error = otCoapSendResponse(aContext, responseMessage, aMessageInfo);
                if((error != OT_ERROR_NONE) && (responseMessage != NULL))
                {
                    otMessageFree(responseMessage);
                }
            }
            else
            {
                otMessageFree(responseMessage);
            }
        }
    }
    else
    {
        /* Print the test result directly */
        uint32_t reqLat, rspLat, offset;
        timeMeasure_ReqCoapMsg_t *pRspMsg = (timeMeasure_ReqCoapMsg_t *)pRspPayload;
        FLib_MemCpy(&reqLat, pRspMsg->reqLatency, sizeof(uint32_t));
        FLib_MemCpy(&rspLat, pRspMsg->rspLatency, sizeof(uint32_t));
        FLib_MemCpy(&offset, pRspMsg->offset, sizeof(uint32_t));

        otCliOutputFormat("Success Size=%d CmdId=%02x SeqNb=%d ReqLat=%d RspLat=%d Offset=%d\n\r",
                  rspLength, pRspPayload[0], pRspPayload[13], reqLat, rspLat, offset);

        otCliOutputFormat("Done\n\r");
    }

    MEM_BufferFree(pRspPayload);
    MEM_BufferFree(pData);

}

/*!*************************************************************************************************
\private
\fn     static void MgmtDiagnostic_DiagTestRspCb(coapSessionStatus_t sessionStatus, uint8_t *pData,
                                                 coapSession_t *pSession, uint32_t dataLen)
\brief  Callback for Diagnostic_DiagTestRsp command.

\param  [in]    sessionStatus    Session status
        [in]    pData            Pointer to data
        [in]    pSession         Pointer to CoAP session
        [in]    pDataLen         Data length
***************************************************************************************************/
static void LatencyMeasure_TestRspCb
(
     void *aContext,
     otMessage *aMessage,
     const otMessageInfo *aMessageInfo,
     otError aResult
)
{
    timeMeasure_RspData_t rspData;

    rspData.status = (aResult == OT_ERROR_NONE) ? TRUE : FALSE;
    if (TRUE == rspData.status)
    {
        rspData.payloadLen = otMessageGetLength(aMessage) - otMessageGetOffset(aMessage);
        rspData.msgId = otCoapMessageGetMessageId(aMessage);

        uint8_t *pPayload = MEM_BufferAlloc(rspData.payloadLen);
        if (NULL != pPayload)
        {
            otMessageRead(aMessage, otMessageGetOffset(aMessage), (void *)pPayload, rspData.payloadLen);
            LatencyMeasure_ProcessTestRsp(pPayload, &rspData);

            otCliOutputFormat("Success MsgID=%d Size=%d CmdId=%02x SeqNb=%d ReqLat=%d RspLat=%d Offset=%d\n\r",
                              rspData.msgId, rspData.payloadLen, rspData.cmdId, rspData.seqNum, rspData.reqLatency,
                              rspData.rspLatency, rspData.offset);

            otCliOutputFormat("Done\n\r");

            MEM_BufferFree(pPayload);
        }
        else
        {
            otCliOutputFormat("Failed, no memory\n\r");
        }
    }
}

/*!*************************************************************************************************
\private
\fn     static void MgmtDiagnostic_DiagTestTlvPayload(uint8_t tlvIdx,  uint8_t *pData, uint32_t *txTime,
                                                      uint32_t dataInLen, bool_t isMcast)
\brief  Get diagnostics test payload (large network usage).

\param  [in]   tlvIdx            Network diagnostic TLV Index
\param  [out]  pData             Pointer to data out
\param  [in]   pDataIn           Pointer to data in
\param  [in]   dataInLen         Length of input data
\param  [in]   isMcast           Flag for signaling received multicast packet
 ***************************************************************************************************/
static void LatencyMeasure_TestTlvPayload
(
    otInstance *pOtInstance,
    uint8_t cmd,
    uint8_t *pData,
    uint8_t *pDataIn,
    uint32_t dataInLen,
    bool_t isMcast
)
{
    uint32_t tlvPayloadLength = 0;
    switch(cmd)
    {
        case gDiagnosticTlv_DiagTestColdReset_c:
        {
            otInstanceFactoryReset(pOtInstance);
            break;
        }
        case gDiagnosticTlv_DiagTestWarmReset_c:
        {
            APP_ResetMcuOnTimeout(300, FALSE);
            break;
        }
        case gDiagnosticTlv_DiagTestData_c:
        {
            //if(!isMcast)
            {
                uint8_t diagTestSize = sizeof(timeMeasure_ReqCoapMsg_t);
                uint32_t txRspTimestamp = NWKU_GetTimestampMs();

                /* Copy tx response timestamp into response packet */
                FLib_MemCpy(pData, &txRspTimestamp, sizeof(uint32_t));

                /* Go to the last element in mgmtDiagTest_CoapMsg_t structure */
                tlvPayloadLength += (2 * sizeof(uint32_t));

                /* Copy back the received sequence number */
                FLib_MemCpy(pData + tlvPayloadLength, pDataIn + (diagTestSize - 1), sizeof(uint8_t));

                /* Increment length after sequence number copy */
                tlvPayloadLength += 1;

                if(!isMcast)
                {
                    /* Copy back the received payload length and payload
                     * to test latency for variable CoAP packets */
                    FLib_MemCpy(pData + diagTestSize, pDataIn + diagTestSize, dataInLen - diagTestSize);
                }
            }
            break;
        }
        case gDiagnosticTlv_DiagTestResults_c:
        {
            if(!isMcast)
            {
                /* Copy request latency calculated on multicast into response packet */
                FLib_MemCpy(pData + tlvPayloadLength, &mResultsReqTime, sizeof(uint32_t));

                /* Go to the sequence number position */
                tlvPayloadLength += (3 * sizeof(uint32_t));

                /* Copy sequence number into packet */
                FLib_MemCpy(pData + tlvPayloadLength, &mReceivedSeqNum, sizeof(uint8_t));
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

/*!*************************************************************************************************
\private
\fn     static void MgmtDiagnostic_ProcessDiagTestRsp(void *pData)
\brief  Process DiagTest Data response packet.

\param  [in]    pData            Pointer to data

***************************************************************************************************/
static void LatencyMeasure_ProcessTestRsp
(
    uint8_t *pData,
    timeMeasure_RspData_t *rspData
)
{


    uint32_t reqLatency;
    uint32_t txRspTimestamp;
    int32_t offset = 0, rspLatency;
    uint32_t uval;


    rspData->cmdId = pData[0];

    /* Set on the request latency */
    uint8_t idx = 1;

    /* Get receive response timestamp, use uval for optimization */
    uval = NWKU_GetTimestampMs();

    /* Copy the request latency */
    FLib_MemCpy(&reqLatency, &pData[idx], sizeof(uint32_t));
    FLib_MemCpy(&rspData->reqLatency, &pData[idx], sizeof(uint32_t));

    /* Set on the transmission response timestamp */
    idx += sizeof(uint32_t);
    FLib_MemCpy(&txRspTimestamp, &pData[idx], sizeof(uint32_t));

    /* Calculate response latency */
    rspLatency = txRspTimestamp - uval;

    if(rspLatency < 0)
    {
        uval = (uint32_t)(THR_ALL_FFs32 - rspLatency);
    }
    else
    {
        uval = (uint32_t)rspLatency;
    }

    /* Copy response latency into packet */
    //FLib_MemCpy(&pData[idx], &uval, sizeof(uint32_t));
    FLib_MemCpy(&rspData->rspLatency, &uval, sizeof(uint32_t));

    idx += sizeof(uint32_t);
    /* Verify the sign of the request latency */
    FLib_MemCpy(&uval, &pData[idx], sizeof(uint32_t));

    if(uval == 1)
    {
        offset -= reqLatency;
    }
    else
    {
        offset += reqLatency;
    }
    offset = (offset - rspLatency)/2;

    if(offset < 0)
    {
        uval = (uint32_t)(THR_ALL_FFs32 - offset);
    }
    else
    {
        uval = (uint32_t)offset;
    }

    /* Copy offset into packet */
    FLib_MemCpy(&rspData->offset, &uval, sizeof(uint32_t));
    //FLib_MemCpy(&pData[idx], &uval, sizeof(uint32_t));

    idx += sizeof(uint32_t);
    rspData->seqNum = pData[idx];
}

/*==================================================================================================
Private debug functions
==================================================================================================*/

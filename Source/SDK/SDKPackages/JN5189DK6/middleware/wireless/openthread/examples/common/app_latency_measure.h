/*
* Copyright 2020 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _APP_LATENCY_MEASURE_H
#define _APP_LATENCY_MEASURE_H

/*!=================================================================================================
\file       app_latency_measure.h
\brief      This is the header file for the latency measurement application
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"

#include <openthread/ip6.h>
/*==================================================================================================
Public macros
==================================================================================================*/

/* Large Network diagnostics */
#define gDiagnosticTlv_DiagTestColdReset_c                0xB0    /*!< Cold/Factory reset */
#define gDiagnosticTlv_DiagTestWarmReset_c                0xB1    /*!< CPU reset */
#define gDiagnosticTlv_DiagTestData_c                     0xB2    /*!< Large network data */
#define gDiagnosticTlv_DiagTestResults_c                  0xB3    /*!< Large network results */

/*==================================================================================================
Public type definitions
==================================================================================================*/

/*!
\brief    Callback function for diagnostic event

\param  [in]    reqLatency           Request latency
\param  [in]    seqNum               Sequence number
*/
typedef void (*timeMeasureEventCallback_t) (uint32_t reqLatency, uint8_t seqNum);

/*! Ota status */
typedef enum timeMeasureStatus_tag
{
    gMeasureStatus_Success_c          = 0x00,
    gMeasureStatus_Failed_c           = 0x01,
    gMeasureStatus_InvalidInstance_c  = 0x02,
    gMeasureStatus_InvalidParam_c     = 0x03,
    gMeasureStatus_NoMem_c            = 0x06,
} timeMeasureStatus_t;

/*! Network diagnostic CoAP message format structure */
typedef struct timeMeasure_CoapMsg_tag
{
      otIp6Address dstIpAddr;       /*!< CoAP destination IP address */
      uint8_t payloadSize;          /*!< CoAP payload size */
      uint8_t *payload;             /*!< CoAP payload - variable data size */
}timeMeasure_CoapMsg_t;

/*! Large network diagnostic CoAP message format structure */
typedef struct timeMeasure_ReqCoapMsg_tag
{
      uint8_t cmdId;             /*!< Requested command */
      uint8_t reqLatency[4];     /*!< Latency on request */
      uint8_t rspLatency[4] ;    /*!< Latency on response */
      uint8_t offset[4];         /*!< Calculated offset */
      uint8_t seqNum;            /*!< Sequence number (useful in multicast) */
}timeMeasure_ReqCoapMsg_t;

/*! Network diagnostic response data format structure */
typedef struct timeMeasure_RspData_tag
{
  uint16_t status;           /*!< Status: TRUE for success, FALSE otherwise */
  uint16_t msgId;            /*!< CoAP message Id: used for synchronization between Req and Rsp */
  uint32_t payloadLen;       /*!< Payload length */
  uint32_t reqLatency;       /*!< Latency on request */
  uint32_t rspLatency;       /*!< Latency on response */
  uint32_t offset;           /*!< Calculated offset */
  uint8_t seqNum;            /*!< Sequence number (useful in multicast) */
  uint8_t cmdId;             /*!< Requested command */
}timeMeasure_RspData_t;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/


/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn     void LatencyMeasure_Init(otInstance *pOtInstance)
\brief  This function is used to initialize network diagnostic module.

\param  [in]    pOtInstance    Open Thread instance pointer
***************************************************************************************************/
void LatencyMeasure_Init(otInstance *pOtInstance);

/*!*************************************************************************************************
\fn     timeMeasureStatus_t LatencyMeasure_TestReq(otInstance *pOtInstance, timeMeasure_CoapMsg_t *pCoapMsg,
        uint16_t *pMsgId)
\brief  This function is used to send a Time Measure Test Request (Large network testing).

\param  [in]    instanceId    Open Thread instance pointer
        [in]    pCoapMsg      Pointer to CoAP message
        [out]   pMsgId        CoAP message Id: used for synchronization between Req and Rsp

\return         timeMeasureStatus_t   Status of the operation
***************************************************************************************************/
timeMeasureStatus_t LatencyMeasure_TestReq(otInstance *pOtInstance, timeMeasure_CoapMsg_t *pCoapMsg,
                                                   uint16_t *pMsgId);

#ifdef __cplusplus
}
#endif

/*================================================================================================*/
#endif  /* _APP_LATENCY_MEASURE_H */

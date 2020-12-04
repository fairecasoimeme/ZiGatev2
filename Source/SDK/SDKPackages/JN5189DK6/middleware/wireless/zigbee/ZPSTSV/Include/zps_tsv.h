/*****************************************************************************
 *
 * MODULE:      ZPSTSV
 *
 * COMPONENT:   zps_tsv.h
 *
 * DESCRIPTION:
 *
 *****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on each
 * copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd. 2008 All rights reserved
 *
 ****************************************************************************/

#ifndef ZPS_TSV_H_
#define ZPS_TSV_H_

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/

#include "jendefs.h"
#include "tsv_pub.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define ZPS_TSV_TIME_SEC(v) ((uint32)(v) * 62500UL)
#define ZPS_TSV_TIME_MSEC(v) ((uint32)(v) * 62UL)

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

typedef TSV_ResultCode_e ZPS_teTsvResultCode;
typedef void (*ZPS_prTsvExpireCb)(void *, void *);

typedef struct
{
    ZPS_prTsvExpireCb prCallback;
    ZPS_prTsvExpireCb pvContext;
    ZPS_prTsvExpireCb pvParam;
} ZPS_tsTsvTimerInfo;

typedef struct
{
    TSV_Timer_s sTsvTimer;
    ZPS_tsTsvTimerInfo sTimerInfo;
} ZPS_tsTsvTimer;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC uint32 zps_u32GetTsvVersion(void);
void ZPS_vTsvInit(void (*prExpireFn)(ZPS_tsTsvTimerInfo *));

PUBLIC ZPS_teTsvResultCode ZPS_eTsvOpen(
    ZPS_tsTsvTimer *psTimer,
    ZPS_prTsvExpireCb prExpireCb,
    void *pvContext,
    void *pvParam);

PUBLIC ZPS_teTsvResultCode ZPS_eTsvClose(
    ZPS_tsTsvTimer* psTimer,
    bool_t bInvokeCBIfRunning);

PUBLIC ZPS_teTsvResultCode ZPS_eTsvStart(
    ZPS_tsTsvTimer* psTimer,
    uint32 u32Value);

PUBLIC ZPS_teTsvResultCode ZPS_eTsvStop(ZPS_tsTsvTimer* psTimer);

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#ifdef __cplusplus
};
#endif

#endif /* ZPS_TSV_H_ */

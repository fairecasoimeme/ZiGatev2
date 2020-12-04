/*
* Copyright 2019 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "zigbee_config.h"
#include "ZQueue.h"
#include "bdb_api.h"
#include "ZTimer.h"
#include "zps_apl_af.h"
#include "pdum_gen.h"
#include "app_zcl_task.h"

uint8_t u8TimerZCL;

tszQueue APP_msgBdbEvents;


/****************************************************************************
*
* NAME: APP_vStopZigbeeTimers
*
* DESCRIPTION:
*
*
* RETURNS:
* Never
*
****************************************************************************/
void APP_vStopZigbeeTimers(void)
{
    ZTIMER_eStop(u8TimerZCL);
}


/****************************************************************************
 *
 * NAME: APP_vRunZigbee
 *
 * DESCRIPTION:
 * Main  execution loop
 *
 * RETURNS:
 * Never
 *
 ****************************************************************************/
void APP_vRunZigbee(void)
{
		zps_taskZPS();
        bdb_taskBDB();        
}

/****************************************************************************
 *
 * NAME: APP_vInitZigbeeResources
 *
 * DESCRIPTION:
 * Main  execution loop
 *
 * RETURNS:
 * Never
 *
 ****************************************************************************/
void APP_vInitZigbeeResources(void)
{
    ZTIMER_eOpen(&u8TimerZCL,           APP_cbTimerZclTick ,    NULL, ZTIMER_FLAG_PREVENT_SLEEP);
    ZQ_vQueueCreate(&APP_msgBdbEvents,        BDB_QUEUE_SIZE,          sizeof(BDB_tsZpsAfEvent),    NULL);    
    ZQ_vQueueCreate(&zps_msgMlmeDcfmInd,      MLME_QUEQUE_SIZE,        sizeof(MAC_tsMlmeVsDcfmInd), NULL);
    ZQ_vQueueCreate(&zps_msgMcpsDcfmInd,      MCPS_QUEUE_SIZE,         sizeof(MAC_tsMcpsVsDcfmInd), NULL);
    ZQ_vQueueCreate(&zps_msgMcpsDcfm,         MCPS_DCFM_QUEUE_SIZE,    sizeof(MAC_tsMcpsVsCfmData), NULL);
    ZQ_vQueueCreate(&zps_TimeEvents,          TIMER_QUEUE_SIZE,        sizeof(zps_tsTimeEvent),     NULL);
    PDUM_vInit();
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

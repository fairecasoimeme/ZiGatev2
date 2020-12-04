/*
* Copyright 2019 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef APP_COMMON_H_
#define APP_COMMON_H_

#include "EmbeddedTypes.h"
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define NETWORK_RESTART_TIME    ZTIMER_TIME_MSEC(1000)
#define POLL_TIME               ZTIMER_TIME_MSEC(5000)
#define POLL_TIME_FAST          ZTIMER_TIME_MSEC(250)
#define GP_ZCL_TICK_TIME        ZTIMER_TIME_MSEC(1)


/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef enum
{
	APP_E_EVENT_NONE,
	APP_E_EVENT_BUTTON_DOWN,
	APP_E_EVENT_BUTTON_UP,
	APP_E_EVENT_SERIAL_TOGGLE,
	APP_E_EVENT_SERIAL_NWK_STEER,
	APP_E_EVENT_SERIAL_FIND_BIND_START,
	APP_E_EVENT_SERIAL_FORM_NETWORK
} teAppEvents;

typedef enum
{
    E_STARTUP,
    E_RUNNING
} teNodeState;

typedef struct
{
    teNodeState     eNodeState;
}tsDeviceDesc;

typedef struct
{
    uint8_t u8Button;
    uint32_t u32DIOState;
} APP_tsEventButton;

typedef struct
{
	teAppEvents eType;
    union
    {
        APP_tsEventButton            sButton;
    }uEvent;
} APP_tsEvent;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/
uint8_t APP_u8GetDeviceEndpoint(void);
teNodeState APP_eGetCurrentApplicationState (void);
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#endif /*APP_COMMON_H_*/

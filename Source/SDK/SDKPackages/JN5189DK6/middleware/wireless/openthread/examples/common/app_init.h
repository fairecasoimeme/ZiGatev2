/*
* Copyright 2019 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _APP_INIT_H
#define _APP_INIT_H

/*!=================================================================================================
\file       app_init.h
\brief      This is the header file for the initial system startup module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "LED.h"
#include "board.h"
#include <openthread/thread.h>

/*==================================================================================================
Public macros
==================================================================================================*/

/* LowPower is desable  gLpmIncluded_d = 0*/
#ifndef gLpmIncluded_d
  #define gLpmIncluded_d 0
#endif


/* app device state: */
typedef enum appDeviceState_tag{
    /* configuration states */
    gDeviceState_FactoryDefault_c,
    gDeviceState_JoiningOrAttaching_c,
    gDeviceState_NwkOperationPending_c,
    gDeviceState_Leader_c,
    gDeviceState_NwkConnected_c,
    gDeviceState_NwkFailure_c,
    gDeviceState_ActiveRouter_c,
    /* application states */
    gDeviceState_AppLedOn_c,
    gDeviceState_AppLedOff_c,
    gDeviceState_AppLedFlash_c,
    gDeviceState_AppLedToggle_c,
    gDeviceState_AppLedRgb_c,
    gDeviceState_AppLedColorWheel_c,
}appDeviceState_t;

/* app device mode: */
typedef enum appDeviceMode_tag{
    gDeviceMode_Configuration_c,
    gDeviceMode_Application_c
}appDeviceMode_t;

#define APP_GetState()           gAppDeviceState
#define APP_SetState(state)      gAppDeviceState = (state);
#define APP_GetMode()            gAppDeviceMode
#define APP_SetMode(state)       gAppDeviceMode = (state);

#define LED_TX_ACTIVITY                     LED2

#define AppTxLedActivityOn()                LED_Operate(LED_TX_ACTIVITY, gLedOn_c)
#define AppTxLedActivityOff()               LED_Operate(LED_TX_ACTIVITY, gLedOff_c)
/*==================================================================================================
Public type definitions
==================================================================================================*/

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
extern void (* pfAppKeyboardHandler)(uint8_t*);

extern appDeviceState_t  gAppDeviceState;
extern appDeviceMode_t  gAppDeviceMode;

extern otPanId gPanId;
extern uint8_t gChannel;
extern uint32_t gScanMask;
extern bool_t isOutOfBand;
extern bool_t isCommissioned;
extern char gPskd[] ;

extern void ResetMCU(void);

/*==================================================================================================
Public function prototypes
==================================================================================================*/
/*!*************************************************************************************************
 \fn     APP_GetResetMcuTimeout
 \brief  Return the interval time until a MCU reset occurs
 \return  the time interval; 0 means that no Mcu reset was programmed
 ***************************************************************************************************/
uint32_t APP_GetResetMcuTimeout(void);

/*!*************************************************************************************************
 \fn     APP_ResetMcuOnTimeout
 \brief  Reset the MCU on timeout
 \param  [in]    timeoutMs  timeout in milliseconds
 \param  [in]    resetToFactory
 \return         None
 ***************************************************************************************************/
void APP_ResetMcuOnTimeout(uint32_t timeoutMs, bool_t resetToFactory);

/*!*************************************************************************************************
\fn     void APP_OtFactoryReset(void)
\brief  This function is used to reset device to factory default settings.

\return      NONE
***************************************************************************************************/
void APP_OtFactoryReset(void);

/*!*************************************************************************************************
\fn     void APP_OtSetMulticastAddresses(otInstance *pOtInstance)
\brief  This function is used to set the multicast addresses from the stack for application usage.

\param  [in]    pOtInstance      Pointer to OpenThread instance

\return         NONE
***************************************************************************************************/
void APP_OtSetMulticastAddresses(otInstance *pOtInstance);

/*================================================================================================*/
#endif  /* _USER_APP_H */

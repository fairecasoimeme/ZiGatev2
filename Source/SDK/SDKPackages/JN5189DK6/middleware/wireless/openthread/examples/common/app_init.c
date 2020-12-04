/*
* Copyright 2019 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*!=================================================================================================
 \file       app_init.c
 \brief      This is a public source file for the initial system startup module. It contains
 the implementation of the interface functions.
 ==================================================================================================*/

/*==================================================================================================
 Include Files
 ==================================================================================================*/
#include "fsl_device_registers.h"
#include "fsl_os_abstraction.h"
#if WDOG_ENABLE
#include "fsl_wdog.h"
#endif

/* FSL Framework */
#include "RNG_Interface.h"
#include "LED.h"
#include "MemManager.h"

#include "TimersManager.h"
#include "Keyboard.h"
#if gLpmIncluded_d
#include "PWR_Interface.h"
#endif

#include "fsl_wtimer.h"
#include "Panic.h"
#include "SecLib.h"
#include "app_init.h"
#include "board.h"
#include "network_utils.h"
#include "FunctionLib.h"

#include <openthread-system.h>
#include <openthread/instance.h>
#if OT_USE_CLI
#include <openthread/cli.h>
#elif OT_USE_SPINEL
#include <openthread/ncp.h>
#endif
#include <openthread/tasklet.h>
#include <openthread/platform/radio.h>

extern void APP_Init(void);
extern void APP_Handler(void);

extern void vMMAC_IntHandlerBbc();
extern void vMMAC_IntHandlerPhy();

#if gLpmIncluded_d
extern void App_SedWakeUpFromKeyBoard(void);
#endif

extern void App_SetCustomEui64(uint8_t *aIeeeEui64);

#if gHybridApp_d
/* Bluetooth Low Energy */
#include "ble_init.h"
extern void BleApp_Init(void);
extern void BleApp_Thread (uint32_t param);
#endif
/*==================================================================================================
 Private macros
 ==================================================================================================*/

/*! The default reset MCU timeout in milliseconds */
#ifndef THR_RESET_MCU_TIMEOUT_MS
    #define THR_RESET_MCU_TIMEOUT_MS                   500 /* milliseconds */
#endif

/*==================================================================================================
 Private type definitions
 ==================================================================================================*/

/*==================================================================================================
 Private prototypes
 ==================================================================================================*/
static void OT_Init(void);
static void KBD_Callback(uint8_t events);
static void APP_TimeoutResetMcu(uint32_t timeoutMs, bool_t resetToFactory);
static void APP_HandleMcuResetOnIdle(void);
#if gLpmIncluded_d
static void APP_HandleLowPowerOnIdle(void);
static void APP_EnterLpCb(void);
static void APP_ExitLpCb(void);
#endif
#if WDOG_ENABLE
static void APP_WDOG_Init(void);
static void APP_WDOG_Refresh(void);
#endif

/*==================================================================================================
 Private global variables declarations
 ==================================================================================================*/

void (*pfAppKeyboardHandler)(uint8_t*) = NULL;

/*!< reset MCU timestamp <microseconds> */
static uint64_t gSwResetTimestamp = 0;
/*!< boolean -  nvm format */
static bool_t gResetToFactory = FALSE;

static char initString[] = "app";

bool_t isOutOfBand = OT_DEV_IS_OUT_OF_BAND_CONFIGURED;
otPanId gPanId = OT_PAN_ID;
uint8_t gChannel = OT_CHANNEL;
uint32_t gScanMask = OT_SCANCHANNEL_MASK;
uint32_t eventMask = 0;
bool_t isCommissioned;
char gPskd[] = OT_PSK_D;
bool_t isUniqueExtAddr = OT_UNIQUE_EUI64_ADDR_ENABLED;
uint8_t gOtExtAddr[] = OT_EUI64_ADDR;
otMasterKey gOtMasterKey = OT_MASTER_KEY;

#if gLpmIncluded_d
extern bool_t bSleepAllowed;
#endif
/*==================================================================================================
 Public global variables declarations
 ==================================================================================================*/
otInstance *gpOpenThreadInstance = NULL;
/*==================================================================================================
 Public functions
 ==================================================================================================*/
extern void Stack_to_APP_Handler(uint32_t flags, void *param);
void main_task(uint32_t param)
{
    static uint8_t mainInitialized = FALSE;

    if (!mainInitialized)
    {
        mainInitialized = TRUE;

#if WDOG_ENABLE
        /* Init watchdog module */
        APP_WDOG_Init();
#endif
        /* Init memory blocks manager */
        MEM_Init();

        SecLib_Init();
        /* Init  timers module */
        TMR_Init();
        TMR_TimeStampInit();

        /* Install uMac interrupt handlers */
        OSA_InstallIntHandler(ZIGBEE_MAC_IRQn, vMMAC_IntHandlerBbc);
        OSA_InstallIntHandler(ZIGBEE_MODEM_IRQn, vMMAC_IntHandlerPhy);

        /* Initialize OpenThread Module */
        OT_Init();

#if gLEDSupported_d
        /* Init Led module */
        LED_Init();
#endif

        if(gpOpenThreadInstance != NULL)
        {
#if !gLpmIncluded_d
#if OT_USE_CLI
            otCliUartInit(gpOpenThreadInstance);
#elif OT_USE_SPINEL
            otNcpInit(gpOpenThreadInstance);
#endif
#endif
            otSetStateChangedCallback(gpOpenThreadInstance, Stack_to_APP_Handler, NULL);
        }

#if gLpmIncluded_d
        PWR_Init();
        PWR_DisallowDeviceToSleep();
        PWR_ChangeDeepSleepMode(3);
#endif
        /* Initialize Keyboard (Switches) Module */
        KBD_Init(KBD_Callback);

        /* Init demo application */
        APP_Init();
    }

    /* Main Application Loop (idle state) */
    while (1)
    {
#if WDOG_ENABLE
        /* Restart the watchdog so it doesn't reset */
        APP_WDOG_Refresh();
#endif

        /* Application handler */
        APP_Handler();

        /* Treat low power */
        if (gpOpenThreadInstance != NULL)
        {
            otSysProcessDrivers(gpOpenThreadInstance);

            while (otTaskletsArePending(gpOpenThreadInstance))
            {
                otTaskletsProcess(gpOpenThreadInstance);
                /* Make sure we process events comming from interrupts in the meantime */
                otSysProcessDrivers(gpOpenThreadInstance);
            }
#if gLpmIncluded_d
            APP_HandleLowPowerOnIdle();
#endif
        }

        /* Reset MCU */
        APP_HandleMcuResetOnIdle();
        /* For BareMetal break the while(1) after 1 run */
        if (gUseRtos_c == 0)
        {
            break;
        }
    }
}

/*!*************************************************************************************************
 \fn     APP_ResetMcuOnTimeout
 \brief  Reset the MCU on timeout
 \param  [in]    timeoutMs  timeout in milliseconds
 \param  [in]    resetToFactory
 \return         None
 ***************************************************************************************************/
void APP_ResetMcuOnTimeout(uint32_t timeoutMs, bool_t resetToFactory)
{
    gResetToFactory = resetToFactory;
    gSwResetTimestamp = TMR_GetTimestamp();
    gSwResetTimestamp += (timeoutMs * 1000); /* microseconds*/
}

/*!*************************************************************************************************
 \fn     APP_GetResetMcuTimeout
 \brief  Return the interval time until a MCU reset occurs
 \return  the time interval; 0 means that no Mcu reset was programmed
 ***************************************************************************************************/
uint32_t APP_GetResetMcuTimeout(void)
{
    uint32_t timeInterval = 0;

    if (gSwResetTimestamp > TMR_GetTimestamp())
    {
        timeInterval = (uint32_t) ((gSwResetTimestamp - TMR_GetTimestamp())
                / 1000);
    }

    return timeInterval;
}
/*!*************************************************************************************************
\fn     void APP_CriticalExitCb(uint32_t location, uint32_t param)
\brief  If the stack is in a deadlock situation, it calls APP_CriticalExitCb.

\param  [in]  location  Address where the Panic occurred
\param  [in]  param     Parameter with extra debug information
***************************************************************************************************/
void APP_CriticalExitCb
(
    uint32_t location,
    uint32_t param
)
{
   panic(0, location, param, 0);
   ResetMCU();
}

/*==================================================================================================
 Private functions
 ==================================================================================================*/
/*!*************************************************************************************************
\fn     void OT_Init(void)
\brief  Initialize OpenThread module.

\return      NONE
***************************************************************************************************/
static void OT_Init
(
    void
)
{
    char *argv[1] = {0};
    argv[0] = &initString[0];
    otLinkModeConfig linkModeConfig = {0};

    /* Initialize OpenThread timer, RNG and Radio */
    otSysInit(1, argv);

    gpOpenThreadInstance = otInstanceInitSingle();

    /* OT stack will be configured with values from config.h */
    if(gpOpenThreadInstance != NULL)
    {

#if gLpmIncluded_d
        PWR_RegisterLowPowerEnterCallback(APP_EnterLpCb);
        PWR_RegisterLowPowerExitCallback(APP_ExitLpCb);
#endif

        if (FALSE == isUniqueExtAddr)
        {
            App_SetCustomEui64(gOtExtAddr);
        }

        if(!FLib_MemCmp((const void *)&gOtMasterKey, (const void *)otThreadGetMasterKey(gpOpenThreadInstance), sizeof(otMasterKey)))
        {
            otThreadSetMasterKey(gpOpenThreadInstance, &gOtMasterKey);
        }
        
        if (FALSE == otDatasetIsCommissioned(gpOpenThreadInstance))
        {
            isCommissioned = FALSE;
            if (0xFFFF != gPanId)
            {
                otLinkSetPanId(gpOpenThreadInstance, gPanId);
            }
            otLinkSetChannel(gpOpenThreadInstance, gChannel);

            linkModeConfig.mRxOnWhenIdle       = RX_ON_IDLE;
            linkModeConfig.mSecureDataRequests = SECURE_DATA_REQUESTS;
            linkModeConfig.mDeviceType         = DEVICE_TYPE;
            linkModeConfig.mNetworkData        = NETWORK_DATA;

            otThreadSetLinkMode(gpOpenThreadInstance, linkModeConfig);
        }
        else
        {
            isCommissioned = TRUE;
        }
    }
}
#if gLpmIncluded_d
/*!*************************************************************************************************
\private
\fn     void APP_EnterLpCb(void)
\brief
***************************************************************************************************/
static void APP_EnterLpCb
(
    void
)
{
    BOARD_SetPinsForPowerMode();

    /* Need to disable radio before going to sleep */
    otPlatRadioDisable(gpOpenThreadInstance);
}

/*!*************************************************************************************************
\private
\fn     void APP_ExitLpCb(void)
\brief
***************************************************************************************************/
static void APP_ExitLpCb
(
    void
)
{
    TMR_ReInit();

    KBD_PrepareExitLowPower();

#if gLEDSupported_d
    LED_PrepareExitLowPower();
#endif

    /* Install uMac interrupt handlers */
    OSA_InstallIntHandler(ZIGBEE_MAC_IRQn, vMMAC_IntHandlerBbc);
    OSA_InstallIntHandler(ZIGBEE_MODEM_IRQn, vMMAC_IntHandlerPhy);

    WTIMER_EnableInterrupts(WTIMER_TIMER0_ID);
    WTIMER_EnableInterrupts(WTIMER_TIMER1_ID);

    /* Radio must be re-enabled after waking up from sleep. The module is completely disabled in
       power down mode */
    otPlatRadioEnable(gpOpenThreadInstance);
}
#endif
/*!*************************************************************************************************
 \fn  static void KBD_Callback(uint8_t events)
 \brief  This is a callback function called from the KBD module.

 \param  [in]    events  value of the events

 \return         void
 ***************************************************************************************************/
static void KBD_Callback(uint8_t events)
{
    /* memory optimisation - app keyboard handler handles the pointer as an events mask*/
    eventMask = eventMask | (uint32_t) (1 << events);
}

/*!*************************************************************************************************
\fn     void APP_OtFactoryReset(void)
\brief  This function is used to reset device to factory default settings.

\return      NONE
***************************************************************************************************/
void APP_OtFactoryReset
(
    void
)
{
     APP_TimeoutResetMcu(THR_RESET_MCU_TIMEOUT_MS, TRUE);
}

/*!*************************************************************************************************
\fn     void APP_OtSetMulticastAddresses(otInstance *pOtInstance)
\brief  This function is used to set the multicast addresses from the stack for application usage.

\param  [in]    pOtInstance      Pointer to OpenThread instance

\return         NONE
***************************************************************************************************/
void APP_OtSetMulticastAddresses
(
    otInstance *pOtInstance
)
{
    otIp6Address *pAddr = NULL;

    pAddr = NWKU_GetSpecificMcastAddr(pOtInstance, gMcastLLAddrAllThrNodes_c);

    if(pAddr != NULL)
    {
        FLib_MemCpy(&in6addr_linklocal_allthreadnodes, pAddr, sizeof(otIp6Address));
    }

    pAddr = NWKU_GetSpecificMcastAddr(pOtInstance, gMcastMLAddrAllThrNodes_c);

    if(pAddr != NULL)
    {
        FLib_MemCpy(&in6addr_realmlocal_allthreadnodes, pAddr, sizeof(otIp6Address));
    }
}

/*!*************************************************************************************************
\fn     void APP_TimeoutResetMcu(uint32_t timeoutMs, bool_t resetToFactory)
\brief  This function is used to reset the device after a specific timeout.

\param [in]  timeoutMs       Time expressed in milliseconds units.
       [in]  resetToFactory  If TRUE, the device will be reseted to factory

\return      NONE
***************************************************************************************************/
static void APP_TimeoutResetMcu(uint32_t timeoutMs, bool_t resetToFactory)
{
    APP_ResetMcuOnTimeout(timeoutMs, resetToFactory);
}

/*!*************************************************************************************************
 \fn     APP_HandleMcuResetOnIdle
 \brief  Reset the MCU on idle
 \param  [in]
 \return         None
 ***************************************************************************************************/
static void APP_HandleMcuResetOnIdle(void)
{
    if ((gSwResetTimestamp) && (gSwResetTimestamp < TMR_GetTimestamp()))
    {
        gSwResetTimestamp = 0;

        if (gResetToFactory)
        {
            /* OpenThread handles NVM erase, modules reinitialization and board reset */
            otInstanceFactoryReset(gpOpenThreadInstance);
        }
        else
        {
            ResetMCU();
        }
    }
}

/*!*************************************************************************************************
 \fn     APP_HandleLowPowerOnIdle
 \brief  Handle low power on idle
 \param  [in]
 \return         None
 ***************************************************************************************************/
#if gLpmIncluded_d
static void APP_HandleLowPowerOnIdle(void)
{
    if( PWR_CheckIfDeviceCanGoToSleep() && bSleepAllowed)
    {
        PWR_WakeupReason_t wakeupReason;
        wakeupReason = PWR_EnterLowPower();

        if(wakeupReason.Bits.FromKeyBoard)
        {
            PWR_DisallowDeviceToSleep();
        }
    }
}
#endif

/*!*************************************************************************************************
 \fn     static void APP_WDOG_Init(void)
 \brief  Init watch dog if enabled
 ***************************************************************************************************/
#if WDOG_ENABLE
static void APP_WDOG_Init(void)
{

    uint32_t i=0;

    WDOG_Init(wdog_base, &wdogConfig);
    /* Accessing register by bus clock */
    for (i = 0; i < 256; i++)
    {
        (void)WDOG->RSTCNT;
    }
}

/*!*************************************************************************************************
 \fn     static void APP_WDOG_Refresh(void)
 \brief  Refresh watch dog if enabled
 ***************************************************************************************************/

static void APP_WDOG_Refresh(void)
{
    uint32_t wdogTimer = (uint32_t)((((uint32_t)wdog_base->TMROUTH) << 16U) | (wdog_base->TMROUTL));
    /* Restart the watchdog so it doesn't reset */
    if(wdogTimer > (wdogConfig.timeoutValue >> 3U))
    {
        WDOG_Refresh(wdog_base);
    }
}
#endif
/*==================================================================================================
 Private debug functions
 ==================================================================================================*/

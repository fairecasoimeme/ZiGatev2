/*****************************************************************************
 *
 * MODULE: ControlBridge
 *
 * COMPONENT: app_start.c
 *
 * $AUTHOR: Faisal Bhaiyat $
 *
 * DESCRIPTION:
 *
 * $HeadURL:  $
 *
 * $Revision: 54887 $
 *
 * $LastChangedBy: nxp29741 $
 *
 * $LastChangedDate:  $
 *
 * $Id: app_start.c  $
 *
 *****************************************************************************
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
 * Copyright NXP B.V. 2016-2019. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include "fsl_power.h"

#ifdef APP_LOW_POWER_API
#include "PWR_interface.h"
#else
#include "pwrm.h"
#endif

#include "pdum_apl.h"
#include "pdum_nwk.h"
#include "pdum_gen.h"
#include "PDM.h"
#include "dbg.h"
#include "zps_gen.h"
#include "zps_apl_af.h"
#include "AppApi.h"
#include "zps_nwk_pub.h"
#include "zps_mac.h"
#include "rnd_pub.h"
#include <string.h>
#include "SerialLink.h"
#include "app_Znc_cmds.h"
#include "app_uart.h"
#include "mac_pib.h"
#include "PDM_IDs.h"
#include "app_common.h"
#include "Log.h"
#include "app_events.h"
#include "zcl_common.h"
#include "temp_sensor_drv.h"
#include "radio.h"
#include "PDM.h"
#include "PDM_IDs.h"
#include "fsl_os_abstraction.h"
#if (ZIGBEE_USE_FRAMEWORK != 0)
#include "SecLib.h"
#include "RNG_Interface.h"
#include "MemManager.h"
#include "TimersManager.h"
#endif

#ifdef STACK_MEASURE
#include "StackMeasure.h"
#endif

#ifdef CLD_OTA
#include "app_ota_server.h"
#endif
#include "app.h"
#include "fsl_wwdt.h"

#ifdef CLD_GREENPOWER
#include "app_green_power.h"
#include "app_power_on_counter.h"
#endif

#ifdef APP_NCI_ICODE
#include "nci_nwk.h"
#include "app_nci_icode.h"
#endif

#include "Flash_Adapter.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef DEBUG_WDR
#define DEBUG_WDR                                                 TRUE
#endif

#ifndef UART_DEBUGGING
#define UART_DEBUGGING                                            FALSE
#endif

#ifndef TRACE_APPSTART
#define TRACE_APPSTART                                            FALSE
#endif

#ifndef TRACE_EXC
#define TRACE_EXC                                                 TRUE
#endif

#define APP_NUM_STD_TMRS                5

PUBLIC uint8 u8TimerPowerOn;

#ifdef CLD_GREENPOWER
    PUBLIC uint8 u8GPTimerTick;
    #define APP_NUM_GP_TMRS             1
    #define GP_TIMER_QUEUE_SIZE         2
#else
    #define APP_NUM_GP_TMRS             0
#endif

#if (defined APP_NCI_ICODE)
#define APP_NUM_NCI_TMRS           1
#else
#define APP_NUM_NCI_TMRS           0
#endif

#ifdef CLD_GREENPOWER
PUBLIC tszQueue APP_msgGPZCLTimerEvents;
#if (ZIGBEE_USE_FRAMEWORK == 0)
uint8 au8GPZCLEvent[ GP_TIMER_QUEUE_SIZE];
#endif
uint8 u8GPZCLTimerEvent;
#endif

#define TIMER_QUEUE_SIZE                                           8
#define MLME_QUEQUE_SIZE                                           8
#define MCPS_QUEUE_SIZE                                            27
#define MCPS_DCFM_QUEUE_SIZE                                       8
#define ZPS_QUEUE_SIZE                                             2
#define APP_QUEUE_SIZE                                             8

#if (ZIGBEE_USE_FRAMEWORK == 0)
#define RX_QUEUE_SIZE                                              150
#endif

#define BDB_QUEUE_SIZE                                             2

#define APP_ZTIMER_STORAGE                                         (APP_NUM_STD_TMRS + APP_NUM_GP_TMRS + APP_NUM_NCI_TMRS)

#define WDT_CLK_FREQ CLOCK_GetFreq(kCLOCK_WdtOsc)

#define APP_WATCHDOG_STACK_DUMP FALSE

/* Interrupt priorities */
#define APP_BASE_INTERRUPT_PRIORITY (5)
#define APP_WATCHDOG_PRIOIRTY       (1)

/* Temperature ADC defines */
#define RADIO_TEMP_UPDATE_MS               300000
#define APP_ADC_BASE                       ADC0
#define APP_ADC_TEMPERATURE_SENSOR_CHANNEL 7U
#define APP_ADC_TEMPERATURE_DELAY_US       300
#define APP_ADC_TEMPERATURE_SAMPLES        8
#define TRACE_MAIN_RADIO                   FALSE
#define RADIO_TEMP_UPDATE                  TRUE

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PUBLIC void app_vFormatAndSendUpdateLists ( void );
PUBLIC void APP_vInitResources ( void );
PUBLIC void APP_vSetUpHardware ( void );
void vfExtendedStatusCallBack ( ZPS_teExtendedStatus    eExtendedStatus );
PRIVATE void vInitialiseApp ( void );
PRIVATE void APP_cbTimerZclTick (void*    pvParam);
extern void vDebugExceptionHandlersInitialise(void);
PUBLIC void APP_vRadioTempUpdate(bool_t bLoadCalibration);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
bool_t                    bLedActivate =  TRUE;
uint8_t					  bPowerCEFCC  ;//=  APP_API_MODULE_HPM05;
bool_t					  bCtrlFlow    =  FALSE;
PUBLIC tsLedState         s_sLedState =  { LED2_DIO_PIN,  ZTIMER_TIME_MSEC(1),  FALSE };

tsZllState sZllState = {
        .eState             = FACTORY_NEW,
        .eNodeState         = E_STARTUP,
        .u8DeviceType       = 0,
        .u8MyChannel        = 11,
        .u16MyAddr          = TL_MIN_ADDR,
#ifdef FULL_FUNC_DEVICE
        .u16FreeAddrLow     = TL_MIN_ADDR,
        .u16FreeAddrHigh    = TL_MAX_ADDR,
        .u16FreeGroupLow    = TL_MIN_GROUP,
        .u16FreeGroupHigh   = TL_MAX_GROUP,
#endif
#ifdef CLD_OTA
        .bValid                 = FALSE,
        .u64IeeeAddrOfServer    = 0,
        .u16NwkAddrOfServer     = 0,
        .u8OTAserverEP          = 0
#endif
};


PUBLIC tszQueue           APP_msgBdbEvents;
PUBLIC tszQueue           APP_msgAppEvents;

ZTIMER_tsTimer            asTimers[APP_ZTIMER_STORAGE + BDB_ZTIMER_STORAGE];
#if (ZIGBEE_USE_FRAMEWORK == 0)
PUBLIC tszQueue           APP_msgSerialRx;
zps_tsTimeEvent           asTimeEvent [ TIMER_QUEUE_SIZE ];
MAC_tsMcpsVsDcfmInd       asMacMcpsInd [ MCPS_QUEUE_SIZE ];
MAC_tsMlmeVsDcfmInd       asMacMlmeVsDcfmInd [ MLME_QUEQUE_SIZE ];
BDB_tsZpsAfEvent          asBdbEvent [ BDB_QUEUE_SIZE ];
APP_tsEvent               asAppMsg [ APP_QUEUE_SIZE ];
MAC_tsMcpsVsCfmData       asMacMcpsDcfm [ MCPS_DCFM_QUEUE_SIZE ];
uint8                     au8AtRxBuffer [ RX_QUEUE_SIZE ];
#endif
uint8                     u8IdTimer;
uint8                     u8TmrToggleLED;
uint8                     u8HaModeTimer;
uint8                     u8TickTimer;
#if (defined APP_NCI_ICODE)
uint8                     u8TimerNci;
#endif
uint8                     u8JoinedDevice =  0;
uint8                     au8LinkRxBuffer[270];

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
#ifdef FULL_FUNC_DEVICE
extern PUBLIC void APP_cbTimerCommission ( void*    pvParam );
#endif

extern void OSA_TimeInit(void);
extern const uint8_t gUseRtos_c;

static volatile uint8_t wdt_update_count = 0;

gpio_pin_config_t led_config = {
    kGPIO_DigitalOutput, 1,
};

#if APP_WATCHDOG_STACK_DUMP
extern void ExceptionUnwindStack(uint32_t * pt);
#endif
void System_IRQHandler(void)
{
#ifdef WATCHDOG_ALLOWED
    uint32_t wdtStatus = WWDT_GetStatusFlags(WWDT);
    /* The chip should reset before this happens. For this interrupt to occur,
     * it means that the WD timeout flag has been set but the reset has not occurred  */
    if (wdtStatus & kWWDT_TimeoutFlag)
    {
        /* A watchdog feed didn't occur prior to window timeout */
        /* Stop WDT */
        WWDT_Disable(WWDT);
        WWDT_ClearStatusFlags(WWDT, kWWDT_TimeoutFlag);
        DBG_vPrintf(TRUE, "Watchdog timeout flag\r\n");
#if APP_WATCHDOG_STACK_DUMP
        /* 0x4015f28  0x32fc2    (Placed on stack by stack dump function)
         * 0x4015f2c  0xfffffff9 (Placed on stack by stack dump function)
         * 0x4015f30  0x0        R0
         * 0x4015f34  0x114ed    R1
         * 0x4015f38  0x0        R2
         * 0x4015f3c  0x40082100 R3
         * 0x4015f40  0x0        R12
         * 0x4015f44  0xe4d      Link Register (R14) (return address at time of exception)
         * 0x4015f48  0xe4c      Return address (PC at time of exception)
         * 0x4015f4c  0x41000000 PSR */
        ExceptionUnwindStack((uint32_t *) __get_MSP());
#endif
    }

    /* Handle warning interrupt */
    if (wdtStatus & kWWDT_WarningFlag)
    {
        /* A watchdog feed didn't occur prior to warning timeout */
        WWDT_ClearStatusFlags(WWDT, kWWDT_WarningFlag);
#if APP_WATCHDOG_STACK_DUMP
        if (wdt_update_count < 7)
#else
        if (wdt_update_count < 8)
#endif
        {
            /* Feed only for the first 8 warnings then allow for a WDT reset to occur */
            wdt_update_count++;
            WWDT_Refresh(WWDT);
            DBG_vPrintf(TRUE,"Watchdog warning flag %d\r\n", wdt_update_count);
        }
#if APP_WATCHDOG_STACK_DUMP
        else
        {
             DBG_vPrintf(TRUE,"Watchdog last warning %d\r\n", wdt_update_count);
             /* 0x4015f28  0x32fc2    (Placed on stack by stack dump function)
              * 0x4015f2c  0xfffffff9 (Placed on stack by stack dump function)
              * 0x4015f30  0x0        R0
              * 0x4015f34  0x114ed    R1
              * 0x4015f38  0x0        R2
              * 0x4015f3c  0x40082100 R3
              * 0x4015f40  0x0        R12
              * 0x4015f44  0xe4d      Link Register (R14) (return address at time of exception)
              * 0x4015f48  0xe4c      Return address (PC at time of exception)
              * 0x4015f4c  0x41000000 PSR */
             ExceptionUnwindStack((uint32_t *) __get_MSP());
        }
#endif
    }
#endif
}

/****************************************************************************
 *
 * NAME: vAppMain
 *
 * DESCRIPTION:
 * Entry point for application from a cold start.
 *
 * RETURNS:
 * Never returns.
 *
 ****************************************************************************/
PUBLIC void vAppMain(void)
{
    uint8          u8FormJoin;
    uint8          au8LinkTxBuffer[50];
    wwdt_config_t  config;
#ifdef STACK_MEASURE
    vInitStackMeasure ( );
#endif
    /* Initialise debugging */

    /* Send debug output through SerialLink to host */
    vSL_LogInit ();
    vDebugExceptionHandlersInitialise();

    DBG_vPrintf ( TRACE_APPSTART, "APP: Entering APP_vInitResources()\n" );
    APP_vInitResources ( );

    DBG_vPrintf ( TRACE_APPSTART, "APP: Entering APP_vSetUpHardware()\n" );
    UART_vInit ( );
    UART_vRtsStartFlow ( );
    vLog_Printf ( TRACE_APPSTART,LOG_DEBUG, "\n\nInitialising \n" );
    vLog_Printf ( TRACE_EXC, LOG_INFO, "\n** Control Bridge Reset** " );
    WWDT_GetDefaultConfig(&config);

    if (((PMC->RESETCAUSE) & PMC_RESETCAUSE_WDTRESET_MASK) == PMC_RESETCAUSE_WDTRESET_MASK)
    {
        extern uint32 _pvHeapStart;
        vLog_Printf ( TRACE_EXC, LOG_CRIT, "\n\n\n%s WATCHDOG RESET @ %08x ", "WDR", _pvHeapStart );
        vSL_LogFlush ( );
        /* Clear flag */
        PMC->RESETCAUSE = PMC_RESETCAUSE_WDTRESET_MASK;
    }

    vInitialiseApp ();
    //app_vFormatAndSendUpdateLists ( );

    if (sZllState.eNodeState == E_RUNNING)
    {
        /* Declared within if statement. If it is declared at the top
         * the function, the while loop will cause the data to be on
         * the stack forever.
         */
        if( sZllState.u8DeviceType >=  1 )
        {
            u8FormJoin = 0;
        }
        else
        {
            u8FormJoin = 1;
        }
        APP_vSendJoinedFormEventToHost ( u8FormJoin, au8LinkTxBuffer );
        vSL_WriteMessage ( E_SL_MSG_NODE_NON_FACTORY_NEW_RESTART,
                           1,
                           ( uint8* ) &sZllState.eNodeState,
						   0 ) ;
        BDB_vStart();
    }
    else
    {
        vSL_WriteMessage( E_SL_MSG_NODE_FACTORY_NEW_RESTART,
                          1,
                          ( uint8* ) &sZllState.eNodeState,
						  0 );
    }
    ZTIMER_eStart ( u8TickTimer, ZCL_TICK_TIME );
    SYSCON -> MAINCLKSEL       = 3;  /* 32 M FRO */
    SystemCoreClockUpdate();
    OSA_TimeInit();

#ifdef APP_NCI_ICODE
    DBG_vPrintf(TRACE_APPSTART, "\r\nAPP: Entering APP_vNciStart()");
    APP_vNciStart(CONTROLBRIDGE_ZLO_ENDPOINT);
#else
    DBG_vPrintf(TRACE_APPSTART, "\r\nAPP: NOT entering APP_vNciStart()");
#endif
}

void vAppRegisterPWRCallbacks(void)
{
/* Non sleeping device so no callbacks registered */
}

void APP_cbToggleLED ( void* pvParam )
{
    tsLedState*    psLedState =  ( tsLedState* ) pvParam;

	if (bLedActivate)
    {
		//vAHI_DioSetOutput ( LED2_DIO_PIN, 0 );


	    if( ZPS_vNwkGetPermitJoiningStatus ( ZPS_pvAplZdoGetNwkHandle ( ) ) )
	    {
	        //GPIO_PinWrite(GPIO, APP_BOARD_TEST_GPIO_PORT, APP_BOARD_TEST_LED0_PIN, (psLedState->u32LedState & (1 << LED1_DIO_PIN)));
	        GPIO_PinWrite(GPIO, APP_BOARD_TEST_GPIO_PORT, APP_BOARD_TEST_LED3_PIN, ((psLedState->u32LedState & (1 << LED2_DIO_PIN) ) >> LED2_DIO_PIN));
	    }
	    else
	    {
	        //GPIO_PinWrite(GPIO, APP_BOARD_TEST_GPIO_PORT, APP_BOARD_TEST_LED0_PIN, (psLedState->u32LedState & (LED1_DIO_PIN)));
	        //GPIO_PinWrite(GPIO, APP_BOARD_TEST_GPIO_PORT, APP_BOARD_TEST_LED3_PIN,  (( (~psLedState->u32LedState) & (LED2_DIO_PIN) ) >> LED2_DIO_PIN));
	    	GPIO_PinWrite(GPIO, APP_BOARD_TEST_GPIO_PORT, APP_BOARD_TEST_LED3_PIN, 1);

	    }
	    psLedState->u32LedState =  ( ~psLedState->u32LedState ) & LED_DIO_PINS;

	    if( u8JoinedDevice == 10 )
	    {
	        if(  !ZPS_vNwkGetPermitJoiningStatus ( ZPS_pvAplZdoGetNwkHandle ( ) ) )
	        {
	            psLedState->u32LedToggleTime =  ZTIMER_TIME_MSEC ( 1 );
	        }

	        if( ZPS_vNwkGetPermitJoiningStatus ( ZPS_pvAplZdoGetNwkHandle ( ) ) )
	        {
	            psLedState->u32LedToggleTime =  ZTIMER_TIME_MSEC ( 500 );
	        }
	        u8JoinedDevice =  0;
	    }
	    u8JoinedDevice++;

		ZTIMER_eStart( u8TmrToggleLED, psLedState->u32LedToggleTime );
    }else{
    	 //vAHI_DioSetOutput(0, LED2_DIO_PIN);
    	//GPIO_PinWrite(GPIO, APP_BOARD_TEST_GPIO_PORT, APP_BOARD_TEST_LED0_PIN, 0);
    	GPIO_PinWrite(GPIO, APP_BOARD_TEST_GPIO_PORT, APP_BOARD_TEST_LED3_PIN, 0);

    }

}

#ifdef FULL_FUNC_DEVICE
/****************************************************************************
 *
 * NAME: APP_vInitialiseNode
 *
 * DESCRIPTION:
 * Initialises the application related functions
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC tsZllEndpointInfoTable * psGetEndpointRecordTable(void)
{
    return &sEndpointTable;
}

/****************************************************************************
 *
 * NAME: psGetGroupRecordTable
 *
 * DESCRIPTION:
 * return the address of the group record table
 *
 * RETURNS:
 * pointer to the group record table
 *
 ****************************************************************************/
PUBLIC tsZllGroupInfoTable * psGetGroupRecordTable(void)
{
    return &sGroupTable;
}
#endif

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vInitialiseApp
 *
 * DESCRIPTION:
 * Initialises Zigbee stack, hardware and application.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vInitialiseApp ( void )
{
    uint16            u16DataBytesRead;
    BDB_tsInitArgs    sArgs;
    uint8             u8DeviceType;

    /* Initialise Power Manager even on non-sleeping nodes as it allows the
     * device to doze when in the idle task */
#ifdef APP_LOW_POWER_API
    (void) PWR_ChangeDeepSleepMode(PWR_E_SLEEP_OSCON_RAMON);
    PWR_Init();
    PWR_vForceRadioRetention(TRUE);
#else
    PWRM_vInit(E_AHI_SLEEP_OSCON_RAMON);
    PWRM_vForceRadioRetention(TRUE);
#endif

    PDM_eInitialise(1200, 63, NULL);
    PDUM_vInit();


    /* Update radio temperature (loading calibration) */
    APP_vRadioTempUpdate(TRUE);

    #ifdef CLD_GREENPOWER
    vManagePowerOnCountLoadRecord();
    vAPP_GP_LoadPDMData();
    #endif

    sZllState.eNodeState =  E_STARTUP;
    /* Restore any application data previously saved to flash */
    PDM_eReadDataFromRecord ( PDM_ID_APP_ZLL_CMSSION,
                              &sZllState,
                              sizeof ( tsZllState ),
                              &u16DataBytesRead );
#ifdef FULL_FUNC_DEVICE
    PDM_eReadDataFromRecord ( PDM_ID_APP_END_P_TABLE,
                              &sEndpointTable,
                              sizeof ( tsZllEndpointInfoTable ),
                              &u16DataBytesRead );
    PDM_eReadDataFromRecord ( PDM_ID_APP_GROUP_TABLE,
                              &sGroupTable,
                              sizeof ( tsZllGroupInfoTable ),
                              &u16DataBytesRead );
#endif
    ZPS_u32MacSetTxBuffers ( 5 );

    if ( sZllState.eNodeState == E_RUNNING )
    {
        u8DeviceType = ( sZllState.u8DeviceType >=  2 ) ? 1 : sZllState.u8DeviceType;
        APP_vConfigureDevice ( u8DeviceType );
        ZPS_eAplAfInit ( );
    }
    else
    {

        ZPS_eAplAfInit ( );
        sZllState.u8DeviceType =  0;
        ZPS_vNwkNibSetChannel ( ZPS_pvAplZdoGetNwkHandle(), DEFAULT_CHANNEL);
        ZPS_vNwkNibSetPanId (ZPS_pvAplZdoGetNwkHandle(), (uint16) RND_u32GetRand ( 1, 0xfff0 ) );
    }
    //Envoie message Start after PDM loaded
    uint8_t au8values[1];
    uint8_t u8Length=0;
    ZNC_BUF_U8_UPD  ( &au8values[ 0 ], 0,      u8Length );
    vSL_WriteMessage ( E_SL_MSG_PDM_LOADED,
                                       1,
                                       au8values,
                                       0 );


    //DBG_vPrintf(TRACE_APPSTART, "\r\nAPP: NV_STORAGE_START_ADDRESS @ %08x ",NV_STORAGE_START_ADDRESS);
    //DBG_vPrintf(TRACE_APPSTART, "\r\nAPP: NV_STORAGE_END_ADDRESS @ %08x ",NV_STORAGE_END_ADDRESS);
    /* If the device state has been restored from flash, re-start the stack
     * and set the application running again.
     */
    sBDB.sAttrib.bbdbNodeIsOnANetwork      =  ( ( sZllState.eNodeState >= E_RUNNING ) ? ( TRUE ) : ( FALSE ) );
    sBDB.sAttrib.bTLStealNotAllowed        =  !sBDB.sAttrib.bbdbNodeIsOnANetwork;
    sArgs.hBdbEventsMsgQ                   =  &APP_msgBdbEvents;
    BDB_vInit ( &sArgs );

    ZPS_vExtendedStatusSetCallback(vfExtendedStatusCallBack);

    APP_ZCL_vInitialise();
    /* Needs to be after we initialise the ZCL and only if we are already
     * running. If we are not running we will send the notify after we
     * have a network formed notification.
     */
    if (sZllState.eNodeState == E_RUNNING)
    {
#ifdef CLD_OTA
        vAppInitOTA();
#endif
    }
#ifdef CLD_GREENPOWER
    vManagePowerOnCountInit();
#endif
}

/****************************************************************************
 *
 * NAME: main_task
 *
 * DESCRIPTION:
 * Main application task.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void main_task (uint32_t parameter)
{
    /* e.g. osaEventFlags_t ev; */
    static uint8_t initialized = FALSE;

    if(!initialized)
    {
        /* place initialization code here... */
        /*myEventId = OSA_EventCreate(TRUE);*/
        initialized = TRUE;
#if (ZIGBEE_USE_FRAMEWORK != 0)
        RNG_Init();
        SecLib_Init();
        MEM_Init();
        TMR_Init();
#endif
        vAppMain();
        bSetPermitJoinForever = TRUE;

    }
    while(1)
    {
         /* place event handler code here... */
        zps_taskZPS ( );
        bdb_taskBDB ( );
        APP_vHandleAppEvents ( );
        APP_vProcessRxData ( );
        ZTIMER_vTask ( );

#ifdef DBG_ENABLE
        vSL_LogFlush ( ); /* flush buffers */
#endif
#ifdef WATCHDOG_ALLOWED
        /* Re-load the watch-dog timer. Execution must return through the idle
         * task before the CPU is suspended by the power manager. This ensures
         * that at least one task / ISR has executed within the watchdog period
         * otherwise the system will be reset.
         */
        /* Kick the watchdog */
        WWDT_Refresh(WWDT);
        wdt_update_count = 0;
#endif
        /*
         * suspends CPU operation when the system is idle or puts the device to
         * sleep if there are no activities in progress
         */
#ifdef APP_LOW_POWER_API
        (void) PWR_EnterLowPower();
#else
        PWRM_vManagePower();
#endif

        if(!gUseRtos_c)
        {
            break;
        }
    }
}

/****************************************************************************
 *
 * NAME: APP_vSetUpHardware
 *
 * DESCRIPTION:
 * Set up interrupts
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vSetUpHardware ( void )
{
    wwdt_config_t config;
    uint32_t wdtFreq;
    /* Enable DMA access to RAM (assuming default configuration and MAC
     * buffers not in first block of RAM) */
    *(volatile uint32 *)0x40001000 = 0xE000F733;
    *(volatile uint32 *)0x40001004 = 0x109;
    /* Board pin, clock, debug console init */
    BOARD_InitHardware();
    /* Initialise output LED GPIOs */
    GPIO_PinInit(GPIO, APP_BOARD_TEST_GPIO_PORT, APP_BOARD_TEST_LED0_PIN, &led_config);
    GPIO_PinInit(GPIO, APP_BOARD_TEST_GPIO_PORT, APP_BOARD_TEST_LED3_PIN, &led_config);

#ifdef WATCHDOG_ALLOWED
    WWDT_GetDefaultConfig(&config);
    /* Replace default config values where required */
        /* The WDT divides the input frequency into it by 4 */
    wdtFreq = WDT_CLK_FREQ/4 ;
    NVIC_EnableIRQ(WDT_BOD_IRQn);
    /*
     * Set watchdog feed time constant to approximately 1s - 8 warnings
     * Set watchdog warning time to 512 ticks after feed time constant
     * Set watchdog window time to 1s
     */
    config.timeoutValue = wdtFreq * 1;
    config.warningValue = 512;
    config.windowValue = wdtFreq * 1;
    /* Configure WWDT to reset on timeout */
    config.enableWatchdogReset = true;
    config.clockFreq_Hz = WDT_CLK_FREQ;

    WWDT_Init(WWDT, &config);
    /* First feed starts the watchdog */
    WWDT_Refresh(WWDT);
#endif

    int   iAppInterrupt;
    for (iAppInterrupt = NotAvail_IRQn; iAppInterrupt < BLE_WAKE_UP_TIMER_IRQn; iAppInterrupt ++)
    {
        NVIC_SetPriority(iAppInterrupt, APP_BASE_INTERRUPT_PRIORITY);
    }
    NVIC_SetPriority(WDT_BOD_IRQn, APP_WATCHDOG_PRIOIRTY);

    SYSCON -> MAINCLKSEL       = 3;  /* 32 M FRO */
    SystemCoreClockUpdate();
    OSA_TimeInit();
}

/****************************************************************************
 *
 * NAME: APP_cbTimerGPZclTick
 *
 * DESCRIPTION:
 * Task kicked by the tick timer
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
#ifdef CLD_GREENPOWER
PUBLIC void APP_cbTimerGPZclTick(void *pvParam)
{
    ZTIMER_eStart(u8GPTimerTick, GP_ZCL_TICK_TIME);
    u8GPZCLTimerEvent = E_ZCL_TIMER_CLICK_MS;
    ZQ_bQueueSend(&APP_msgGPZCLTimerEvents, &u8GPZCLTimerEvent);
}
#endif


/****************************************************************************
 *
 * NAME: APP_vInitResources
 *
 * DESCRIPTION:
 * Initialise resources (timers, queue's etc)
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vInitResources ( void )
{

    vLog_Printf ( TRACE_APPSTART,LOG_DEBUG, "APP: Initialising resources...\n");
    vLog_Printf ( TRACE_APPSTART,LOG_DEBUG, "APP: ZPS_tsAfEvent = %d bytes\n",    sizeof ( ZPS_tsAfEvent ) );
    vLog_Printf ( TRACE_APPSTART,LOG_DEBUG, "APP: zps_tsTimeEvent = %d bytes\n",  sizeof ( zps_tsTimeEvent ) );

    /* Initialise the Z timer module */
    ZTIMER_eInit ( asTimers, sizeof(asTimers) / sizeof(ZTIMER_tsTimer));

    /* Create Z timers */
    ZTIMER_eOpen ( &u8TickTimer,       APP_cbTimerZclTick,          NULL,                      ZTIMER_FLAG_PREVENT_SLEEP );
    ZTIMER_eOpen ( &u8IdTimer,         APP_vIdentifyEffectEnd,      NULL,                      ZTIMER_FLAG_PREVENT_SLEEP );
    ZTIMER_eOpen ( &u8TmrToggleLED,    APP_cbToggleLED,             &s_sLedState,              ZTIMER_FLAG_PREVENT_SLEEP );
    ZTIMER_eOpen ( &u8HaModeTimer,     App_TransportKeyCallback,    &u64CallbackMacAddress,    ZTIMER_FLAG_PREVENT_SLEEP );

    #if (defined APP_NCI_ICODE)
    ZTIMER_eOpen(&u8TimerNci,           APP_cbNciTimer,         NULL, ZTIMER_FLAG_PREVENT_SLEEP);
    #endif

    #ifdef CLD_GREENPOWER
    ZTIMER_eOpen(&u8TimerPowerOn,  APP_cbTimerPowerCount,      NULL, ZTIMER_FLAG_PREVENT_SLEEP);
    ZTIMER_eOpen(&u8GPTimerTick,       APP_cbTimerGPZclTick,        NULL,                      ZTIMER_FLAG_PREVENT_SLEEP );
    #endif

    /* Create all the queues */
    #if (ZIGBEE_USE_FRAMEWORK == 0)
        ZQ_vQueueCreate ( &APP_msgBdbEvents,      BDB_QUEUE_SIZE,         sizeof ( BDB_tsZpsAfEvent ),       (uint8*)asBdbEvent );
        ZQ_vQueueCreate ( &zps_msgMlmeDcfmInd,    MLME_QUEQUE_SIZE,       sizeof ( MAC_tsMlmeVsDcfmInd ),    (uint8*)asMacMlmeVsDcfmInd );
        ZQ_vQueueCreate ( &zps_msgMcpsDcfmInd,    MCPS_QUEUE_SIZE,        sizeof ( MAC_tsMcpsVsDcfmInd ),    (uint8*)asMacMcpsInd );
        ZQ_vQueueCreate ( &zps_msgMcpsDcfm,       MCPS_DCFM_QUEUE_SIZE,   sizeof ( MAC_tsMcpsVsCfmData ),     (uint8*)asMacMcpsDcfm);
        ZQ_vQueueCreate ( &zps_TimeEvents,        TIMER_QUEUE_SIZE,       sizeof ( zps_tsTimeEvent ),        (uint8*)asTimeEvent );
        ZQ_vQueueCreate ( &APP_msgAppEvents,      APP_QUEUE_SIZE,         sizeof ( APP_tsEvent ),            (uint8*)asAppMsg );
        ZQ_vQueueCreate ( &APP_msgSerialRx,       RX_QUEUE_SIZE,          sizeof ( uint8 ),                  (uint8*)au8AtRxBuffer );
        #ifdef CLD_GREENPOWER
        ZQ_vQueueCreate(&APP_msgGPZCLTimerEvents, GP_TIMER_QUEUE_SIZE,    sizeof(uint8),                     (uint8*)au8GPZCLEvent);
        #endif
    #else
        ZQ_vQueueCreate ( &APP_msgBdbEvents,      BDB_QUEUE_SIZE,         sizeof ( BDB_tsZpsAfEvent ),       NULL);
        ZQ_vQueueCreate ( &zps_msgMlmeDcfmInd,    MLME_QUEQUE_SIZE,       sizeof ( MAC_tsMlmeVsDcfmInd ),    NULL);
        ZQ_vQueueCreate ( &zps_msgMcpsDcfmInd,    MCPS_QUEUE_SIZE,        sizeof ( MAC_tsMcpsVsDcfmInd ),    NULL);
        ZQ_vQueueCreate ( &zps_msgMcpsDcfm,       MCPS_DCFM_QUEUE_SIZE,   sizeof ( MAC_tsMcpsVsCfmData ),    NULL);
        ZQ_vQueueCreate ( &zps_TimeEvents,        TIMER_QUEUE_SIZE,       sizeof ( zps_tsTimeEvent ),        NULL);
        ZQ_vQueueCreate ( &APP_msgAppEvents,      APP_QUEUE_SIZE,         sizeof ( APP_tsEvent ),            NULL);
        #ifdef CLD_GREENPOWER
        ZQ_vQueueCreate(&APP_msgGPZCLTimerEvents, GP_TIMER_QUEUE_SIZE,    sizeof(uint8),                     NULL);
        #endif
    #endif

    vZCL_RegisterHandleGeneralCmdCallBack (APP_vProfileWideCommandSupportedForCluster );
    vZCL_RegisterCheckForManufCodeCallBack(APP_bZCL_IsManufacturerCodeSupported);
    DBG_vPrintf(TRACE_APPSTART, "APP: Initialising resources complete\n");
}

/****************************************************************************
 *
 * NAME: APP_vRadioTempUpdate
 *
 * DESCRIPTION:
 * Callback function from radio driver requesting write of calibration
 * settings to PDM
 *
 * RETURNS:
 * TRUE  - successful
 * FALSE - failed
 *
 ****************************************************************************/
PUBLIC void APP_vRadioTempUpdate(bool_t bLoadCalibration)
{
#if RADIO_TEMP_UPDATE
    bool_t bStatus = TRUE;

    /* Debug */
    DBG_vPrintf(TRACE_MAIN_RADIO, "\r\n%d: APP_vRadioTempUpdate(%d)",OSA_TimeGetMsec(), bLoadCalibration);
    /* Need to load calibration data ? */
    if (bLoadCalibration)
    {
        /* Read temperature and adc calibration parameter once. Store ADC calibration values into ADC register */
        bStatus = load_calibration_param_from_flash(APP_ADC_BASE);
        /* Debug */
        DBG_vPrintf(TRACE_MAIN_RADIO, "\r\n%d: load_calibration_param_from_flash() = %d", OSA_TimeGetMsec(), bStatus);
    }
    /* OK to read temperature */
    if (bStatus)
    {
        /* Temp in 128th degree */
        int32 i32Temp128th;
        /* Read temperature (in 128th degree) */
        bStatus = get_temperature(APP_ADC_BASE, APP_ADC_TEMPERATURE_SENSOR_CHANNEL, APP_ADC_TEMPERATURE_DELAY_US, APP_ADC_TEMPERATURE_SAMPLES, &i32Temp128th);
        /* Debug */
        DBG_vPrintf(TRACE_MAIN_RADIO, "\r\n%d: get_temperature() = %d", OSA_TimeGetMsec(), bStatus);
        /* Success ? */
        if (bStatus)
        {
            /* Temp in half degree */
            int16 i16Temp2th = (int16)(i32Temp128th / 64);
            /* Debug */
            DBG_vPrintf(TRACE_MAIN_RADIO, ", i32Temp128th = %d, i16Temp2th = %d", i32Temp128th, i16Temp2th);
            /* Pass to radio driver */
            vRadio_Temp_Update(i16Temp2th);
        }
    }
#endif
}

#if RADIO_TEMP_UPDATE
/****************************************************************************
 *
 * NAME: bRadioCB_WriteNVM
 *
 * DESCRIPTION:
 * Callback function from radio driver requesting write of calibration
 * settings to PDM
 *
 * RETURNS:
 * TRUE  - successful
 * FALSE - failed
 *
 ****************************************************************************/
PUBLIC bool_t bRadioCB_WriteNVM(uint8 *pu8DataBlock, uint16 u16DataLength)
{
    bool_t bReturn = FALSE;
    PDM_teStatus eStatus;

    eStatus = PDM_eSaveRecordData(PDM_ID_RADIO_SETTINGS, (void *)pu8DataBlock, u16DataLength);

    if (PDM_E_STATUS_OK == eStatus)
    {
        bReturn = TRUE;
    }

    /* Debug */
    DBG_vPrintf(TRACE_MAIN_RADIO, "\r\n%d: bRadioCB_WriteNVM(%d) = %d, eStatus = %d", OSA_TimeGetMsec(), u16DataLength, bReturn, eStatus);

    return bReturn;
}
#endif

#if RADIO_TEMP_UPDATE
/****************************************************************************
 *
 * NAME: u16RadioCB_ReadNVM
 *
 * DESCRIPTION:
 * Callback function from radio driver requesting read of calibration
 * settings from PDM
 *
 * RETURNS:
 * Number of bytes read
 *
 ****************************************************************************/
PUBLIC uint16 u16RadioCB_ReadNVM(uint8 *pu8DataBlock, uint16 u16DataLength)
{
    PDM_teStatus eStatus;
    uint16 u16BytesRead = 0;

    eStatus = PDM_eReadDataFromRecord(PDM_ID_RADIO_SETTINGS, (void *)pu8DataBlock, u16DataLength, &u16BytesRead);

    /* Debug */
    DBG_vPrintf(TRACE_MAIN_RADIO, "\r\n%d: u16RadioCB_ReadNVM(%d) = %d, eStatus = %d",OSA_TimeGetMsec(), u16DataLength, u16BytesRead, eStatus);

    return u16BytesRead;
}
#endif

/****************************************************************************
 *
 * NAME: app_vFormatAndSendUpdateLists
 *
 * DESCRIPTION:
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void app_vFormatAndSendUpdateLists ( void )
{
    typedef struct
    {
        uint16     u16Clusterid;
        uint16*    au16Attibutes;
        uint32     u32Asize;
        uint8*     au8command;
        uint32     u32Csize;
    }    tsAttribCommand;

    uint16                 u16Length =  0;
    uint8                  u8Count = 0 ;
    uint8                  u8BufferLoop =  0;
    uint8                  au8LinkTxBuffer[256];

    /*List of clusters per endpoint */
    uint16    u16ClusterListHA [ ]  =  { 0x0000, 0x0001, 0x0003, 0x0004, 0x0005, 0x0006,
                                         0x0008, 0x0019, 0x0101, 0x1000, 0x0300, 0x0201, 0x0204 };
    uint16    u16ClusterListHA2 [ ] =  { 0x0405, 0x0500, 0x0400, 0x0402, 0x0403, 0x0405, 0x0406,
                                         0x0702, 0x0b03, 0x0b04 , 0x1000 };

    /*list of attributes per cluster */

    uint16    u16AttribBasic [ ] =  { 0x0000, 0x0001, 0x0002, 0x0003, 0x0004,
                                      0x0005, 0x0006, 0x0007, 0x4000 };
    uint16    u16AttribIdentify [ ] =  { 0x000 };
    uint16    u16AttribGroups [ ] =  { 0x000 };
    uint16    u16AttribScenes [ ] =  { 0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005 };
    uint16    u16AttribOnOff [ ] =  { 0x0000, 0x4000, 0x4001, 0x4002 };
    uint16    u16AttribLevel [ ] =  { 0x0000, 0x0001, 0x0010, 0x0011 };
    uint16    u16AttribColour [ ] =  { 0x0000, 0x0001, 0x0002, 0x0007, 0x0008,
                                       0x0010, 0x0011, 0x0012, 0x0013, 0x0015,
                                       0x0016, 0x0017, 0x0019, 0x001A, 0x0020,
                                       0x0021, 0x0022, 0x0024, 0x0025, 0x0026,
                                       0x0028, 0x0029, 0x002A, 0x4000, 0x4001,
                                       0x4002, 0x4003, 0x4004, 0x4006, 0x400A,
                                       0x400B, 0x400C };
    uint16    u16AttribThermostat [ ] =  { 0x0000, 0x0003, 0x0004, 0x0011, 0x0012,
                                           0x001B, 0x001C };
    uint16    u16AttribHum [ ] =  { 0x0000, 0x0001, 0x0002, 0x0003 };
    uint16    u16AttribPower [ ] =  { 0x0020, 0x0034 };
    uint16    u16AttribIllumM [ ] =  { 0x000, 0x0001, 0x0002, 0x0003, 0x0004 };
    uint16    u16AttribIllumT [ ] =  { 0x000, 0x0001, 0x0002 };
    uint16    u16AttribSM [ ] =  { 0x0000, 0x0300, 0x0301, 0x0302, 0x0306, 0x0400 };
    /*list of commands per cluster */
    uint8     u8CommandBasic [ ] =  { 0 };
    uint8     u8CommandIdentify [ ] =  { 0, 1, 0x40 };
    uint8     u8CommandGroups [ ] =  { 0, 1, 2, 3, 4, 5 };
    uint8     u8CommandScenes [ ] =  { 0, 1, 2, 3, 4, 5, 6,
                                    0x40, 0x41, 0x42 };
    uint8     u8CommandsOnOff [ ] =  { 0, 1, 2, 0x40, 0x41, 0x42 };
    uint8     u8CommandsLevel [ ] =  { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
    uint8     u8CommandsColour [ ] =  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                                     0xa, 0x40, 0x41, 0x42, 0x43, 0x44, 0x47,
                                     0x4b, 0x4c, 0xfe, 0xff };
    uint8     u8CommandThermostat [ ] = {0};
    uint8     u8CommandHum [ ] =  { 0 };
    uint8     u8CommandPower [ ] =  { 0 };
    uint8     u8CommandIllumM [ ] =  { 0 };
    uint8     u8CommandIllumT [ ] =  { 0 };
    uint8     u8CommandSM [ ] =  { 0 };

    tsAttribCommand    asAttribCommand[13] =  {  { 0x0000, u16AttribBasic, ( sizeof( u16AttribBasic ) / sizeof ( u16AttribBasic [ 0 ] ) ),
                                                  u8CommandBasic, ( sizeof ( u8CommandBasic ) / sizeof ( u8CommandBasic [ 0 ] )  )},
                                                 { 0x0003, u16AttribIdentify, ( sizeof ( u16AttribIdentify ) / sizeof ( u16AttribIdentify [ 0 ]  ) ),
                                                   u8CommandIdentify, ( sizeof ( u8CommandIdentify ) / sizeof ( u8CommandIdentify [ 0 ]  ) ) },
                                                 { 0x0004, u16AttribGroups, ( sizeof ( u16AttribGroups ) / sizeof ( u16AttribGroups [ 0 ]  ) ),
                                                   u8CommandGroups, ( sizeof ( u8CommandGroups ) / sizeof ( u8CommandGroups [ 0 ]  ) ) },
                                                 { 0x0005, u16AttribScenes, ( sizeof ( u16AttribScenes ) / sizeof ( u16AttribScenes [ 0 ]  ) ),
                                                   u8CommandScenes, ( sizeof( u8CommandScenes )  / sizeof ( u8CommandScenes [ 0 ]  ) ) },
                                                 { 0x0006, u16AttribOnOff,  ( sizeof ( u16AttribOnOff ) / sizeof ( u16AttribOnOff [ 0 ]  ) ),
                                                   u8CommandsOnOff, ( sizeof ( u8CommandsOnOff )/ sizeof ( u8CommandsOnOff [ 0 ]  ) ) },
                                                 { 0x0008, u16AttribLevel,  ( sizeof ( u16AttribLevel ) / sizeof ( u16AttribLevel [ 0 ]  ) ),
                                                   u8CommandsLevel, ( sizeof ( u8CommandsLevel ) / sizeof ( u8CommandsLevel [ 0 ]  ) ) },
                                                 { 0x0300, u16AttribColour, ( sizeof ( u16AttribColour ) / sizeof ( u16AttribColour [ 0 ]  ) ),
                                                   u8CommandsColour, ( sizeof ( u8CommandsColour ) / sizeof ( u8CommandsColour [ 0 ]  ) ) },
                                                 { 0x0201, u16AttribThermostat, ( sizeof ( u16AttribThermostat ) / sizeof ( u16AttribThermostat [ 0 ]  ) ),
                                                   u8CommandThermostat, ( sizeof ( u8CommandThermostat ) / sizeof ( u8CommandThermostat [ 0 ]  ) ) },
                                                 { 0x0405, u16AttribHum, ( sizeof ( u16AttribHum ) / sizeof ( u16AttribHum [ 0 ]  ) ),
                                                   u8CommandHum, ( sizeof ( u8CommandHum ) / sizeof ( u8CommandHum [ 0 ]  ) ) },
                                                 { 0x0001, u16AttribPower, ( sizeof ( u16AttribPower ) /sizeof( u16AttribPower [ 0 ]  ) ),
                                                   u8CommandPower, ( sizeof ( u8CommandPower ) /sizeof( u8CommandPower [ 0 ]  ) ) },
                                                 { 0x0400, u16AttribIllumM, ( sizeof ( u16AttribIllumM ) /sizeof( u16AttribIllumM [ 0 ]  ) ),
                                                   u8CommandIllumM, ( sizeof ( u8CommandIllumM ) /sizeof( u8CommandIllumM [ 0 ]  ) ) },
                                                 { 0x0402, u16AttribIllumT, ( sizeof ( u16AttribIllumT ) /sizeof( u16AttribIllumT [ 0 ]  ) ),
                                                   u8CommandIllumT, ( sizeof ( u8CommandIllumT ) /sizeof( u8CommandIllumT [ 0 ]  ) ) },
                                                 { 0x0702, u16AttribSM, ( sizeof ( u16AttribSM ) / sizeof ( u16AttribSM [ 0 ]  ) ),
                                                   u8CommandSM, ( sizeof ( u8CommandSM ) / sizeof ( u8CommandSM [ 0 ]  ) )} };



    /* Cluster list endpoint HA */
    ZNC_BUF_U8_UPD  ( &au8LinkTxBuffer [ 0 ],            CONTROLBRIDGE_ZLO_ENDPOINT,    u16Length );
    ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    0x0104,                                  u16Length );
    while ( u8BufferLoop < ( sizeof ( u16ClusterListHA ) /  sizeof( u16ClusterListHA [ 0 ] ) ) )
    {
        ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    u16ClusterListHA [ u8BufferLoop ],    u16Length );
        u8BufferLoop++;
    }
    vSL_WriteMessage ( E_SL_MSG_NODE_CLUSTER_LIST,
                       u16Length,
                       au8LinkTxBuffer,
					   0);

        /* Cluster list endpoint HA */
    u16Length = u8BufferLoop =  0;
    ZNC_BUF_U8_UPD  ( &au8LinkTxBuffer [ 0 ],         CONTROLBRIDGE_ZLO_ENDPOINT,    u16Length );
    ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ], 0x0104,                                  u16Length );
    while ( u8BufferLoop <  ( sizeof ( u16ClusterListHA2 ) /  sizeof ( u16ClusterListHA2 [ 0 ] ) )  )
    {

        ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ], u16ClusterListHA2 [ u8BufferLoop ],    u16Length );
        u8BufferLoop++;
    }
    vSL_WriteMessage ( E_SL_MSG_NODE_CLUSTER_LIST,
                       u16Length,
                       au8LinkTxBuffer,
					   0 );

        /* Attribute list basic cluster HA EP*/
    for ( u8Count =  0; u8Count < 13; u8Count++ )
    {
        u16Length =  0;
        u8BufferLoop =  0;
        ZNC_BUF_U8_UPD  ( &au8LinkTxBuffer [ 0 ],            CONTROLBRIDGE_ZLO_ENDPOINT,     u16Length );
        ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    0x0104,                                   u16Length );
        ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    asAttribCommand[u8Count].u16Clusterid,    u16Length );
        while ( u8BufferLoop <   asAttribCommand [ u8Count ].u32Asize   )
        {
            ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    asAttribCommand[u8Count].au16Attibutes [ u8BufferLoop ],    u16Length );
            u8BufferLoop++;
        }
        vSL_WriteMessage ( E_SL_MSG_NODE_ATTRIBUTE_LIST,
                           u16Length,
                           au8LinkTxBuffer,
						   0);
        u16Length =  0;
        u8BufferLoop =  0;
        ZNC_BUF_U8_UPD  ( &au8LinkTxBuffer [ 0 ],            CONTROLBRIDGE_ZLO_ENDPOINT,     u16Length );
        ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    0x0104,                                   u16Length );
        ZNC_BUF_U16_UPD ( &au8LinkTxBuffer [ u16Length ],    asAttribCommand[u8Count].u16Clusterid,    u16Length );
        while ( u8BufferLoop <   asAttribCommand [ u8Count ].u32Csize  )
        {
            ZNC_BUF_U8_UPD ( &au8LinkTxBuffer [ u16Length ],    asAttribCommand[u8Count].au8command [ u8BufferLoop ],    u16Length );
            u8BufferLoop++;
        }
        vSL_WriteMessage ( E_SL_MSG_NODE_COMMAND_ID_LIST,
                           u16Length,
                           au8LinkTxBuffer,
						   0);
    }

}

void vfExtendedStatusCallBack ( ZPS_teExtendedStatus    eExtendedStatus )
{
   // vLog_Printf ( TRACE_EXC,LOG_DEBUG, "ERROR: Extended status %x\n", eExtendedStatus );
    uint16                 u16Length =  0;
	uint8                  au8LinkTxBuffer[256];
	vLog_Printf ( TRACE_EXC,LOG_DEBUG, "ERROR: Extended status %x\n", eExtendedStatus );

	/*switch (eExtendedStatus)
	{
		case ZPS_XS_E_NO_FREE_EXTENDED_ADDR:

			PDM_vDeleteDataRecord(PDM_ID_INTERNAL_APS_KEYS);
			break;
	}*/
	u16Length =  0;
	ZNC_BUF_U8_UPD  ( &au8LinkTxBuffer [ 0 ],  eExtendedStatus,     u16Length );
	vSL_WriteMessage ( 0x9999,
							   u16Length,
							   au8LinkTxBuffer,
							   0);
	switch (eExtendedStatus)
	{
		case ZPS_XS_E_NO_FREE_EXTENDED_ADDR:

			//PDM_vDeleteDataRecord(PDM_ID_INTERNAL_APS_KEYS);
			//PDM_vDeleteDataRecord(PDM_ID_INTERNAL_APS_KEYS);
			//RESET_SystemReset();
			break;

		default:
			break;
	}
}

#if (defined PDM_EEPROM && DBG_ENABLE)
PRIVATE void vPdmEventHandlerCallback ( uint32                  u32EventNumber,
                                        PDM_eSystemEventCode    eSystemEventCode )
{
	 //RAJOUT FRED v3.1b
	uint16_t u16Length =  0;
	uint8    au8LinkTxBuffer[50];
	ZNC_BUF_U8_UPD  ( &au8LinkTxBuffer [u16Length] ,  eSystemEventCode,   u16Length );
	ZNC_BUF_U32_UPD  ( &au8LinkTxBuffer [u16Length] ,  u32EventNumber,   u16Length );
	vSL_WriteMessage ( E_SL_MSG_EVENT_PDM,
			   u16Length,
			   au8LinkTxBuffer,
			   0);


}
#endif

#if (defined PDM_EEPROM)
#if TRACE_APPSTART
/****************************************************************************
 *
 * NAME: vPdmEventHandlerCallback
 *
 * DESCRIPTION:
 * Handles PDM callback, information the application of PDM conditions
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
/*PRIVATE void vPdmEventHandlerCallback ( uint32                  u32EventNumber,
                                        PDM_eSystemEventCode    eSystemEventCode )
{
	DBG_vPrintf(TRACE_APPSTART, "Erreur Memoire : %d\n", eSystemEventCode);

    switch ( eSystemEventCode )
    {
        //
        // The next three events will require the application to take some action
        //
        case E_PDM_SYSTEM_EVENT_WEAR_COUNT_TRIGGER_VALUE_REACHED:
                vLog_Printf ( TRACE_EXC,LOG_DEBUG, "PDM: Segment %d reached trigger wear level\n", u32EventNumber);
            break;
        case E_PDM_SYSTEM_EVENT_DESCRIPTOR_SAVE_FAILED:
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Record Id %d failed to save\n", u32EventNumber);
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Capacity %d\n", u8PDM_CalculateFileSystemCapacity() );
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Occupancy %d\n", u8PDM_GetFileSystemOccupancy() );
            break;
        case E_PDM_SYSTEM_EVENT_PDM_NOT_ENOUGH_SPACE:
        		DBG_vPrintf(TRACE_APPSTART,  "PDM: Record %d not enough space\n", u32EventNumber);
        		DBG_vPrintf(TRACE_APPSTART, "PDM: Capacity %d\n", u8PDM_CalculateFileSystemCapacity() );
        		DBG_vPrintf(TRACE_APPSTART,  "PDM: Occupancy %d\n", u8PDM_GetFileSystemOccupancy() );
            break;
        case E_PDM_SYSTEM_EVENT_LARGEST_RECORD_FULL_SAVE_NO_LONGER_POSSIBLE:
				DBG_vPrintf(TRACE_APPSTART,  "PDM: Record %d not enough space\n", u32EventNumber);
				DBG_vPrintf(TRACE_APPSTART, "PDM: Capacity %d\n", u8PDM_CalculateFileSystemCapacity() );
				DBG_vPrintf(TRACE_APPSTART,  "PDM: Occupancy %d\n", u8PDM_GetFileSystemOccupancy() );
			break;

        //
        // The following events are really for information only
        //
        case E_PDM_SYSTEM_EVENT_EEPROM_SEGMENT_HEADER_REPAIRED:
                vLog_Printf ( TRACE_EXC,LOG_DEBUG, "PDM: Segment %d header repaired\n", u32EventNumber);
            break;
        case E_PDM_SYSTEM_EVENT_SYSTEM_INTERNAL_BUFFER_WEAR_COUNT_SWAP:
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Segment %d buffer wear count swap\n", u32EventNumber);
            break;
        case E_PDM_SYSTEM_EVENT_SYSTEM_DUPLICATE_FILE_SEGMENT_DETECTED:
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Segement %d duplicate selected\n", u32EventNumber);
            break;
        default:
                vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "PDM: Unexpected call back Code %d Number %d\n", eSystemEventCode, u32EventNumber);
            break;
    }
}*/
#endif
#endif

PRIVATE void APP_cbTimerZclTick (void*    pvParam)
{
    static uint8 u8Tick100Ms = 9;
    static uint8 u8Tick1S = 0;
    static uint8 u8Tick1S2 = 30;
    static uint16 countDevices=0;
    static uint32 u32RadioTempUpdateMs = 0;
    ZPS_tsNwkNib * thisNib;
	uint8  u8SeqNum = 0;
	void * thisNet = ZPS_pvAplZdoGetNwkHandle();
	thisNib = ZPS_psNwkNibGetHandle(thisNet);

    tsZCL_CallBackEvent sCallBackEvent;

    if(ZTIMER_eStart(u8TickTimer, ZTIMER_TIME_MSEC(100)) != E_ZTIMER_OK)
    {
        vLog_Printf ( TRACE_EXC,LOG_DEBUG,  "APP: Failed to Start Tick Timer\n" );
    }

    /* Provide 100ms tick to cluster */
    eZCL_Update100mS();

    /* Provide 1sec tick to cluster - Wrap 1 second  */
    u8Tick100Ms++;
    if(u8Tick100Ms > 9)
    {
    	u8Tick1S++;
    	u8Tick1S2++;
    	sControlBridge.sTimeServerCluster.utctTime++;
    	if (u8Tick1S >=60)
    	{
    		u8Tick1S=0;
    		if ((sZllState.u8HeartBeat == 1) || (sZllState.u8RawMode == RAW_MODE_ON))
    		{
				uint32  u32Data = sControlBridge.sTimeServerCluster.utctTime;
				uint8   au8Datas[4];
				uint8   u8L=0;

				ZNC_BUF_U32_UPD ( &au8Datas[ u8L ], u32Data, u8L);


				vSL_WriteMessage ( E_SL_MSG_HEARTBEAT,
								   sizeof ( uint32 ),
								   au8Datas,
								   0);
			}

			if (thisNib->sTbl.pu16AddrMapNwk[countDevices] < 0xfffe)
			{
				APP_eZdpMgmtLqiRequest ( thisNib->sTbl.pu16AddrMapNwk[countDevices],
																				  0,
																				  &u8SeqNum );

			}

    	}

    	if (u8Tick1S2 >=70)
		{
			u8Tick1S2=0;

			if (thisNib->sTbl.pu16AddrMapNwk[countDevices] < 0xfffe)
			{

				APP_eZdpMgmtRtgRequest ( thisNib->sTbl.pu16AddrMapNwk[countDevices],
															  0,
															  &u8SeqNum );

			}
			countDevices++;
			if (countDevices>thisNib->sTblSize.u16AddrMap)
			{
				countDevices=0;
			}
		}
#ifdef CLD_BAS_ATTR_APPLICATION_LEGRAND
    	sControlBridge.sBasicServerCluster.u32PrivateLegrand++;
#endif
        u8Tick100Ms = 0;
        sCallBackEvent.pZPSevent = NULL;
        sCallBackEvent.eEventType = E_ZCL_CBET_TIMER;
        vZCL_EventHandler(&sCallBackEvent);
    }

    u32RadioTempUpdateMs += 100; /* 100ms for normal timer use */
    /* Need to update radio temp ? */
    if(u32RadioTempUpdateMs >= RADIO_TEMP_UPDATE_MS)
    {
        /* Reset timer counter */
        u32RadioTempUpdateMs = 0;
        /* Update radio temperature (not loading calibration) */
        APP_vRadioTempUpdate(FALSE);
    }
}

bool_t APP_vProfileWideCommandSupportedForCluster ( uint16 u16Clusterid )
{
    if ( u16Clusterid == MEASUREMENT_AND_SENSING_CLUSTER_ID_OCCUPANCY_SENSING)
    {
        return TRUE;
    }
    return FALSE;
}

void hardware_init(void)
{
    APP_vSetUpHardware();
}

bool_t APP_bZCL_IsManufacturerCodeSupported(uint16 u16ManufacturerCode)
{
	return TRUE;
}

/*PRIVATE void vPdmEventHandlerCallback ( uint32                  u32EventNumber,
                                        PDM_eSystemEventCode    eSystemEventCode )
{
	 //RAJOUT FRED v3.1b
	uint16_t u16Length =  0;
	uint8    au8LinkTxBuffer[50];
	ZNC_BUF_U8_UPD  ( &au8LinkTxBuffer [u16Length] ,  eSystemEventCode,   u16Length );
	ZNC_BUF_U32_UPD  ( &au8LinkTxBuffer [u16Length] ,  u32EventNumber,   u16Length );
	vSL_WriteMessage ( E_SL_MSG_EVENT_PDM,
			   u16Length,
			   au8LinkTxBuffer,
			   0);


}*/


/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

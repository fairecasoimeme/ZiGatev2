/*
* Copyright 2008-2010 Jennic Ltd
* Copyright 2010-2022 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef MICRO_MAC_H
#define MICRO_MAC_H

#if defined __cplusplus
extern "C" {
#endif


/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include "jendefs.h"

/****************************************************************************/
/***        Macro/Type Definitions                                        ***/
/****************************************************************************/

/* Place buffers in a specific section - memory preserved during sleep mode */
#define MAC_BUFFER_SECTION __attribute__ ((section (".mac_buffer")))

/* Address obtained from linker */
extern uint32 __mac_buffer_base;

#if (defined BUILD_JN518x_ES2)
    #if (defined ROM_BUILD_FOR_ZB)
        /* Offset into base of RAM bank 0 */
        #define MAC_BUFFER_BASE    (0x04000000)
    #else
        /* Address obtained from linker: will be aligned on 128kB boundary */
        #define MAC_BUFFER_BASE ((uint32)&__mac_buffer_base)
    #endif
    /* MAC buffer address at end of range: not valid */
    #define MAC_BUFFER_INVALID (MAC_BUFFER_BASE + 0x1fffc)
#else
    /* Address obtained from linker: must be aligned on 32kB boundary */
    #define MAC_BUFFER_BASE ((uint32)&__mac_buffer_base)
    /* MAC buffer address at end of range: not valid. Value is set at run-time
       in vMiniMac_Init */
    #define MAC_BUFFER_INVALID (u32MacBufferInvalidAddr)
#endif

typedef struct
{
    uint32 u32L;  /**< Low word */
    uint32 u32H;  /**< High word */
} tsExtAddr;

typedef union
{
    uint16    u16Short;
    tsExtAddr sExt;
} tuAddr;

/* Structure for building a MAC frame, where the MAC header alignment is
   handled by the hardware */
typedef struct
{
    uint8           u8PayloadLength;
    uint8           u8SequenceNum;
    uint16          u16FCF;
    uint16          u16DestPAN;
    uint16          u16SrcPAN;
    tuAddr          uDestAddr;
    tuAddr          uSrcAddr;
    uint16          u16FCS;
    uint16          u16Unused;
    union
    {
        uint8     au8Byte[127]; /* Payload as both bytes and words */
        uint32    au32Word[32];
    } uPayload;
} tsMacFrame;

/* Structure for building a PHY frame, where the MAC header format is
   undefined */
typedef struct
{
    uint8           u8PayloadLength;
    uint8           au8Padding[3];
    union
    {
        uint8     au8Byte[127]; /* Payload as both bytes and words */
        uint32    au32Word[32];
    } uPayload;
} tsPhyFrame;

typedef struct
{
    uint8  u8SecurityLevel;
    uint8  u8KeyIdMode;
    uint8  u8KeyIndex;
    bool_t bPassedSecurity;
} tsSecurity;

typedef struct
{
    tsMacFrame     sFrameBody;
    tsSecurity     sSecurityData;
    uint32         u32Timestamp;
    uint8          u8LinkQuality;
    uint8          u8Msq;
} tsRxFrameFormat;

/* Options for reception, to pass to vMMAC_StartReceive. User should select
   one from each pair of options, and logical OR the options together */
typedef enum
{
    /* Receive start time: now or delayed */
    E_MMAC_RX_START_NOW        = 0x0002,
    E_MMAC_RX_DELAY_START      = 0x0003,

    /* Timing alignment for auto ack transmission: normal or aligned to
       backoff clock (used in CAP period in beacon networks) */
    E_MMAC_RX_ALIGN_NORMAL     = 0x0000,
    E_MMAC_RX_ALIGNED          = 0x0004,

    /* Wait for auto ack and retry: don't use or use */
    E_MMAC_RX_NO_AUTO_ACK      = 0x0000,
    E_MMAC_RX_USE_AUTO_ACK     = 0x0008,

    /* Poll Response Mode: turns on Rx after 12 symbol after
       Start Rx instead of observing inter-frame spacing rules*/
    E_MMAC_RX_USE_PRSP         = 0x0010,

    /* Malformed packets: reject or accept */
    E_MMAC_RX_NO_MALFORMED     = 0x0000,
    E_MMAC_RX_ALLOW_MALFORMED  = 0x0400,

    /* Frame Check Sequence errors: reject or accept */
    E_MMAC_RX_NO_FCS_ERROR     = 0x0000,
    E_MMAC_RX_ALLOW_FCS_ERROR  = 0x0200,

    /* Address matching: enable or disable */
    E_MMAC_RX_NO_ADDRESS_MATCH = 0x0000,
    E_MMAC_RX_ADDRESS_MATCH    = 0x0100
} teRxOption;

/* Options for transmission, to pass to vMMAC_StartMacTransmit or
   vMMAC_StartPhyTransmit. User should select one from each set of options,
   and logical OR the options together */
typedef enum
{
    /* Transmit start time: now or delayed */
    E_MMAC_TX_START_NOW       = 0x02,
    E_MMAC_TX_DELAY_START     = 0x03,

    /* Wait for auto ack and retry: don't use or use */
    E_MMAC_TX_NO_AUTO_ACK     = 0x00,
    E_MMAC_TX_USE_AUTO_ACK    = 0x08,

    /* Clear channel assessment: don't use or use, plus option to align to
       backoff clock */
    E_MMAC_TX_NO_CCA          = 0x00,
    E_MMAC_TX_USE_CCA         = 0x10,
    E_MMAC_TX_USE_CCA_ALIGNED = 0x20,

    /* v2MAC only */
    E_MMAC_TX_ENC             = 0x40,   /* encrypt frame before transmission */
    E_MMAC_TX_HDR_UPD         = 0x80,    /* frame header can be updated */

    /* Enable RX in CCA */
    E_MMAC_TX_RX_CCA          = 0x200,

    E_MMAC_TX_INVALID         = 0x80000000 /* Invalid option to force txOptions on 4 bytes */
} teTxOption;

#define TXCTL_MASK 0x3f     /* TXCTL has 6 valid bits - mask for Tx options */
#define CSMA_CTX_POS 12     /* offset of CSMA context in the Tx options */

#define MAX_TX_TRIES 15     /* TXTRIES has 4 valid bits */

#define NB(x) (((x) >> 4) & 0x7)        /* TXMBEBT bits 11:8 */
#define MAXBE(x) (((x) >> 8) & 0xf)     /* TXMBEBT bits 6:4 */

/* Flags for receive status, as returned by u32MMAC_GetRxErrors */
typedef enum
{
    E_MMAC_RXSTAT_ERROR     = 0x01, /* Frame check sequence error */
    E_MMAC_RXSTAT_ABORTED   = 0x02, /* Reception aborted by user */
    E_MMAC_RXSTAT_MALFORMED = 0x20  /* Frame was malformed */
} teRxStatus;

/* Flags for transmit status, as returned by u32MMAC_GetTxErrors */
typedef enum
{
    E_MMAC_TXSTAT_CCA_BUSY = 0x01, /* Channel wasn't free */
    E_MMAC_TXSTAT_NO_ACK   = 0x02, /* Ack requested but not seen */
    E_MMAC_TXSTAT_ABORTED  = 0x04, /* Transmission aborted by user */
    E_MMAC_TXSTAT_RX_ABORT = 0x08, /* Transmission aborted by Rx in CCA */
    E_MMAC_TXSTAT_TXTO     = 0x20, /* Radio transmission timeout */
    E_MMAC_TXSTAT_TXPCTO   = 0x40  /* Modem transmission timeout */
} teTxStatus;

/* Flags for interrupt status, as returned to handler registered with
   vMMAC_EnableInterrupts and as used in the mask passed to
   vMMAC_ConfigureInterruptSources, u32MMAC_PollInterruptSource,
   u32MMAC_PollInterruptSourceUntilFired */
typedef enum
{
    E_MMAC_INT_TX_COMPLETE  = 0x01, /* Transmission attempt has finished */
    E_MMAC_INT_RX_HEADER    = 0x02, /* MAC header has been received */
    E_MMAC_INT_RX_COMPLETE  = 0x04  /* Complete frame has been received */
} teIntStatus;

/* CCA mode to use when transmitting. Use with vMMAC_SetCcaMode(). Default is
   E_MMAC_CCAMODE_ENERGY */
typedef enum
{
    E_MMAC_CCAMODE_ENERGY            = 0x01, /* Energy above threshold */
    E_MMAC_CCAMODE_CARRIER           = 0x02, /* Carrier sense */
    E_MMAC_CCAMODE_ENERGY_OR_CARRIER = 0x03  /* Either energy or carrier */
} teCcaMode;

/* 
 * Invalid rssi value, this should be greater than
 * maximum returned value from radio
 */
#define MMAC_INVALID_RSSI (127)

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
/* Initialisation */
/* Protocol ID tagged functions to be used in multi-context MMAC */
PUBLIC void __vMMAC_Enable(uint8 u8ProtoTag);
PUBLIC void __vMMAC_Disable(uint8 u8ProtoTag);
PUBLIC void __vMMAC_ConfigureRadio(uint8 u8ProtoTag);
PUBLIC void __vMMAC_SetChannel(uint8 u8ProtoTag, uint8 u8Channel);
PUBLIC void __vMMAC_SetChannelAndPower(uint8 u8ProtoTag, uint8 u8Channel, int i8TxPower);
PUBLIC int8 __i8MMAC_GetTxPowerLevel(uint8 u8ProtoTag);
PUBLIC void __vMMAC_EnableAntennaDiversity(uint8 u8ProtoTag);
PUBLIC void __vMMAC_DisableAntennaDiversity(uint8 u8ProtoTag);

#ifdef MAC_PROTO_TAG
#define vMMAC_Enable() __vMMAC_Enable((MAC_PROTO_TAG))
#define vMMAC_Disable() __vMMAC_Disable((MAC_PROTO_TAG))
#define vMMAC_ConfigureRadio() __vMMAC_ConfigureRadio((MAC_PROTO_TAG))
#define vMMAC_SetChannel(u8Channel) __vMMAC_SetChannel((MAC_PROTO_TAG), (u8Channel))
#define vMMAC_SetChannelAndPower(u8Channel, i8TxPower) __vMMAC_SetChannelAndPower((MAC_PROTO_TAG), (u8Channel), (i8TxPower))
#define i8MMAC_GetTxPowerLevel() __i8MMAC_GetTxPowerLevel((MAC_PROTO_TAG))
#define vMMAC_EnableAntennaDiversity() __vMMAC_EnableAntennaDiversity((MAC_PROTO_TAG))
#define vMMAC_DisableAntennaDiversity() __vMMAC_DisableAntennaDiversity((MAC_PROTO_TAG))
#else
PUBLIC void vMMAC_Enable(void);
PUBLIC void vMMAC_Disable(void);
PUBLIC void vMMAC_ConfigureRadio(void);
PUBLIC void vMMAC_SetChannel(uint8 u8Channel);
PUBLIC void vMMAC_SetChannelAndPower(uint8 u8Channel, int i8TxPower);
PUBLIC int8 i8MMAC_GetTxPowerLevel(void);
PUBLIC void vMMAC_EnableAntennaDiversity(void);
PUBLIC void vMMAC_DisableAntennaDiversity(void);
#endif

/* Interrupt control */
/* Protocol ID tagged functions to be used in multi-context MMAC */
PUBLIC void __vMMAC_EnableInterrupts(uint8 u8ProtoTag, void (*prHandler)(uint32 u32Mask));
PUBLIC void __vMMAC_RegisterPhyIntHandler(uint8 u8ProtoTag, void (*prHandler)(uint32 u32Mask));                 /* Doesn't do anything */
PUBLIC void __vMMAC_ConfigureInterruptSources(uint8 u8ProtoTag, uint32 u32Mask);
PUBLIC uint32 __u32MMAC_GetInterrupts(uint8 u8ProtoTag);
PUBLIC void __vMMAC_ClearInterrupts(uint8 u8ProtoTag, uint32 u32Mask);
PUBLIC uint32 __u32MMAC_GetInterrupts(uint8 u8ProtoTag);
PUBLIC uint32 __u32MMAC_PollInterruptSource(uint8 u8ProtoTag, uint32 u32Mask);
PUBLIC uint32 __u32MMAC_PollInterruptSourceUntilFired(uint8 u8ProtoTag, uint32 u32Mask);

#ifdef MAC_PROTO_TAG
#define vMMAC_EnableInterrupts(prHandler) __vMMAC_EnableInterrupts((MAC_PROTO_TAG), (prHandler))
#define vMMAC_RegisterPhyIntHandler(prHandler) __vMMAC_RegisterPhyIntHandler((MAC_PROTO_TAG), (prHandler))      /* Not supported */
#define vMMAC_ConfigureInterruptSources(u32Mask) __vMMAC_ConfigureInterruptSources((MAC_PROTO_TAG), (u32Mask))
#define u32MMAC_GetInterrupts() __u32MMAC_GetInterrupts((MAC_PROTO_TAG))
#define vMMAC_ClearInterrupts(u32Mask) __vMMAC_ClearInterrupts((MAC_PROTO_TAG), u32Mask);
#define u32MMAC_PollInterruptSource(u32Mask) __u32MMAC_PollInterruptSource((MAC_PROTO_TAG), (u32Mask))
#define u32MMAC_PollInterruptSourceUntilFired(u32Mask) __u32MMAC_PollInterruptSourceUntilFired((MAC_PROTO_TAG), (u32Mask))
#else
PUBLIC void vMMAC_EnableInterrupts(void (*prHandler)(uint32 u32Mask));
PUBLIC void vMMAC_RegisterPhyIntHandler(void (*prHandler)(uint32 u32Mask));
PUBLIC void vMMAC_ConfigureInterruptSources(uint32 u32Mask);
PUBLIC uint32 u32MMAC_GetInterrupts(void);
PUBLIC void vMMAC_ClearInterrupts(uint32 u32Mask);
PUBLIC uint32 u32MMAC_PollInterruptSource(uint32 u32Mask);
PUBLIC uint32 u32MMAC_PollInterruptSourceUntilFired(uint32 u32Mask);
#endif

/* Miscellaneous */
/* Protocol ID tagged functions to be used in multi-context MMAC */
PUBLIC uint32 __u32MMAC_GetTime(uint8 u8ProtoTag);
PUBLIC void __vMMAC_RadioOff(uint8 u8ProtoTag);
PUBLIC uint32 __u32MMAC_GetSSMState(uint8 u8ProtoTag);
PUBLIC void __vMMAC_RadioWait_unsafe(uint8 u8ProtoTag, uint16 u16WaitDelay);
PUBLIC void __vMMAC_RadioToOffAndWait(uint8 u8ProtoTag);
PUBLIC void __vMMAC_SetCutOffTimer(uint8 u8ProtoTag, uint32 u32CutOffTime, bool_t bEnable);
PUBLIC void __vMMAC_SynchroniseBackoffClock(uint8 u8ProtoTag, bool_t bEnable);
PUBLIC void __vMMAC_GetMacAddress(uint8 u8ProtoTag, tsExtAddr *psMacAddr);
PUBLIC uint8 __u8MMAC_EnergyDetect(uint8 u8ProtoTag, uint32 u32DurationSymbols);
PUBLIC int16 __i16MMAC_GetRSSI(uint8 u8ProtoTag);
PUBLIC uint32 __u32MMAC_GetPhyState(uint8 u8ProtoTag);
PUBLIC void __vMMAC_RxCtlUpdate(uint8 u8ProtoTag, uint32 u32NewValue);
PUBLIC void __vMMAC_AbortRadio(uint8 u8ProtoTag);
PUBLIC void __vMMAC_PromiscuousMode(uint8 u8ProtoTag, bool_t bPromiscuous);
PUBLIC void __vMMAC_WriteCcaThreshold(uint8 u8ProtoTag, uint8 u8CcaThreshold);
PUBLIC uint8 __u8MMAC_ReadCcaThreshold(uint8 u8ProtoTag);
PUBLIC void __vMMAC_SetPRBSS(uint8 u8ProtoTag, uint32 u32Seed);

#ifdef MAC_PROTO_TAG
#define u32MMAC_GetTime() __u32MMAC_GetTime((MAC_PROTO_TAG))
#define vMMAC_RadioOff() __vMMAC_RadioOff((MAC_PROTO_TAG))
#define u32MMAC_GetSSMState() __u32MMAC_GetSSMState((MAC_PROTO_TAG))
#define vMMAC_RadioToOffAndWait() __vMMAC_RadioToOffAndWait((MAC_PROTO_TAG))
#define vMMAC_RadioWait_unsafe(u16WaitDelay) __vMMAC_RadioWait_unsafe((MAC_PROTO_TAG), u16WaitDelay)
#define vMMAC_SetCutOffTimer(u32CutOffTime, bEnable) __vMMAC_SetCutOffTimer((MAC_PROTO_TAG), (u32CutOffTime), (bEnable))
#define vMMAC_SynchroniseBackoffClock(bEnable) __vMMAC_SynchroniseBackoffClock((MAC_PROTO_TAG), (bEnable))
#define vMMAC_GetMacAddress(psMacAddr) __vMMAC_GetMacAddress((MAC_PROTO_TAG), (psMacAddr))
#define u8MMAC_EnergyDetect(u32DurationSymbols) __u8MMAC_EnergyDetect((MAC_PROTO_TAG), (u32DurationSymbols))
#define i16MMAC_GetRSSI() __i16MMAC_GetRSSI((MAC_PROTO_TAG))
#define u32MMAC_GetPhyState() __u32MMAC_GetPhyState((MAC_PROTO_TAG))
#define vMMAC_RxCtlUpdate(u32NewValue) __vMMAC_RxCtlUpdate((MAC_PROTO_TAG), (u32NewValue))
#define vMMAC_AbortRadio() __vMMAC_AbortRadio((MAC_PROTO_TAG))
#define vMMAC_PromiscuousMode(bPromiscuous) __vMMAC_PromiscuousMode((MAC_PROTO_TAG), (bPromiscuous))
#define vMMAC_WriteCcaThreshold(u8CcaThreshold) __vMMAC_WriteCcaThreshold((MAC_PROTO_TAG), (u8CcaThreshold))
#define u8MMAC_ReadCcaThreshold() __u8MMAC_ReadCcaThreshold((MAC_PROTO_TAG))
#define vMMAC_SetPRBSS(u32Seed) __vMMAC_SetPRBSS((MAC_PROTO_TAG), (u32Seed))
#else
PUBLIC uint32 u32MMAC_GetTime(void);
PUBLIC void vMMAC_RadioOff(void);
PUBLIC uint32 u32MMAC_GetSSMState(void);
PUBLIC void vMMAC_RadioWait_unsafe(uint16 u16WaitDelay);
PUBLIC void vMMAC_RadioToOffAndWait(void);
PUBLIC void vMMAC_SetCutOffTimer(uint32 u32CutOffTime, bool_t bEnable);
PUBLIC void vMMAC_SynchroniseBackoffClock(bool_t bEnable);
PUBLIC void vMMAC_GetMacAddress(tsExtAddr *psMacAddr);
PUBLIC uint8 u8MMAC_EnergyDetect(uint32 u32DurationSymbols);
PUBLIC int16 i16MMAC_GetRSSI(void);
PUBLIC uint32 u32MMAC_GetPhyState(void);
PUBLIC void vMMAC_RxCtlUpdate(uint32 u32NewValue);
PUBLIC void vMMAC_AbortRadio(void);
PUBLIC void vMMAC_PromiscuousMode(bool_t bPromiscuous);
PUBLIC void vMMAC_WriteCcaThreshold(uint8 u8CcaThreshold);
PUBLIC uint8 u8MMAC_ReadCcaThreshold(void);
PUBLIC void vMMAC_SetPRBSS(uint32 u32Seed);
#endif

/* Receive */
/* Protocol ID tagged functions to be used in multi-context MMAC */
PUBLIC void __vMMAC_SetRxAddress(uint8 u8ProtoTag, uint32 u32PanId, uint16 u16Short,
                               tsExtAddr *psMacAddr);
PUBLIC void __vMMAC_SetRxPanId(uint8 u8ProtoTag, uint32 u32PanId);
PUBLIC void __vMMAC_SetRxShortAddr(uint8 u8ProtoTag, uint16 u16Short);
PUBLIC void __vMMAC_SetRxExtendedAddr(uint8 u8ProtoTag, tsExtAddr *psMacAddr);
PUBLIC void __vMMAC_SetRxStartTime(uint8 u8ProtoTag, uint32 u32Time);
PUBLIC void __vMMAC_StartMacReceive(uint8 u8ProtoTag, tsMacFrame *psFrame, teRxOption eOptions);
PUBLIC void __vMMAC_StartPhyReceive(uint8 u8ProtoTag, tsPhyFrame *psFrame, teRxOption eOptions);
PUBLIC void __vMMAC_SetRxFrame(uint8 u8ProtoTag, tsRxFrameFormat *pRxFrame);
PUBLIC void __vMMAC_SetRxProm(uint8 u8ProtoTag, uint32_t u32Prom);
PUBLIC bool_t __bMMAC_RxDetected(uint8 u8ProtoTag);
PUBLIC uint32 __u32MMAC_GetRxErrors(uint8 u8ProtoTag);
PUBLIC uint32 __u32MMAC_GetRxTime(uint8 u8ProtoTag);
PUBLIC uint8 __u8MMAC_GetRxLqi(uint8 u8ProtoTag, uint8 *pu8Msq);
PUBLIC uint32 __u32MMAC_GetRxFrame(uint8 u8ProtoTag);

#ifdef MAC_PROTO_TAG
#define vMMAC_SetRxAddress(u32PanId, u16Short, psMacAddr) __vMMAC_SetRxAddress((MAC_PROTO_TAG), (u32PanId), (u16Short), (psMacAddr))
#define vMMAC_SetRxPanId(u32PanId) __vMMAC_SetRxPanId((MAC_PROTO_TAG), (u32PanId))
#define vMMAC_SetRxShortAddr(u16Short) __vMMAC_SetRxShortAddr((MAC_PROTO_TAG), (u16Short))
#define vMMAC_SetRxExtendedAddr(psMacAddr) __vMMAC_SetRxExtendedAddr((MAC_PROTO_TAG), (psMacAddr))
#define vMMAC_SetRxStartTime(u32Time) __vMMAC_SetRxStartTime((MAC_PROTO_TAG), (u32Time))
#define vMMAC_StartMacReceive(psFrame, eOptions) __vMMAC_StartMacReceive((MAC_PROTO_TAG), (psFrame), (eOptions))
#define vMMAC_StartPhyReceive(psFrame, eOptions) __vMMAC_StartPhyReceive((MAC_PROTO_TAG), (psFrame), (eOptions))
#define vMMAC_SetRxFrame(pRxFrame) __vMMAC_SetRxFrame((MAC_PROTO_TAG), (pRxFrame))
#define u32MMAC_GetRxFrame() __u32MMAC_GetRxFrame((MAC_PROTO_TAG))
#define vMMAC_SetRxProm(u32Prom) __vMMAC_SetRxProm((MAC_PROTO_TAG), (u32Prom))
#define bMMAC_RxDetected() __bMMAC_RxDetected((MAC_PROTO_TAG))
#define u32MMAC_GetRxErrors() __u32MMAC_GetRxErrors((MAC_PROTO_TAG))
#define u32MMAC_GetRxTime() __u32MMAC_GetRxTime((MAC_PROTO_TAG))
#define u8MMAC_GetRxLqi(pu8Msq) __u8MMAC_GetRxLqi((MAC_PROTO_TAG), (pu8Msq))
#else
PUBLIC void vMMAC_SetRxAddress(uint32 u32PanId, uint16 u16Short,
                               tsExtAddr *psMacAddr);
PUBLIC void vMMAC_SetRxPanId(uint32 u32PanId);
PUBLIC void vMMAC_SetRxShortAddr(uint16 u16Short);
PUBLIC void vMMAC_SetRxExtendedAddr(tsExtAddr *psMacAddr);
PUBLIC void vMMAC_SetRxStartTime(uint32 u32Time);
PUBLIC void vMMAC_StartMacReceive(tsMacFrame *psFrame, teRxOption eOptions);
PUBLIC void vMMAC_StartPhyReceive(tsPhyFrame *psFrame, teRxOption eOptions);
PUBLIC void vMMAC_SetRxFrame(tsRxFrameFormat *pRxFrame);
PUBLIC uint32 u32MMAC_GetRxFrame(void);
PUBLIC void vMMAC_SetRxProm(uint32_t u32Prom);
PUBLIC bool_t bMMAC_RxDetected(void);
PUBLIC uint32 u32MMAC_GetRxErrors(void);
PUBLIC uint32 u32MMAC_GetRxTime(void);
PUBLIC uint8 u8MMAC_GetRxLqi(uint8 *pu8Msq);
#endif

/* Transmit */
/* Protocol ID tagged functions to be used in multi-context MMAC */
PUBLIC void __vMMAC_SetTxParameters(uint8 u8ProtoTag, uint8 u8Attempts, uint8 u8MinBE,
                                  uint8 u8MaxBE, uint8 u8MaxBackoffs);
PUBLIC void __vMMAC_SetTxMinBE(uint8 u8ProtoTag, uint8 u8MinBE);
PUBLIC void __vMMAC_SetTxMaxBE(uint8 u8ProtoTag, uint8 u8MaxBE);
PUBLIC void __vMMAC_SetTxMaxBackoffs(uint8 u8ProtoTag, uint8 u8MaxBackoffs);
PUBLIC void __vMMAC_SetTxStartTime(uint8 u8ProtoTag, uint32 u32Time);
PUBLIC void __vMMAC_SetCcaMode(uint8 u8ProtoTag, teCcaMode eCcaMode);
PUBLIC void __vMMAC_StartMacTransmit(uint8 u8ProtoTag, tsMacFrame *psFrame, teTxOption eOptions);
PUBLIC void __vMMAC_StartPhyTransmit(uint8 u8ProtoTag, tsPhyFrame *psFrame, teTxOption eOptions);
PUBLIC void __vMMAC_SetTxPend(uint8 u8ProtoTag, bool_t bTxPend);
PUBLIC uint32 __u32MMAC_GetTxErrors(uint8 u8ProtoTag);
PUBLIC bool_t __bMMAC_PowerStatus(uint8 u8ProtoTag);
PUBLIC uint32 __u32MMAC_GetCsmaContext(uint8 u8ProtoTag);
PUBLIC uint32 __u32MMAC_GetTxRetries(uint8 u8ProtoTag);
PUBLIC void __vMMAC_SetTxRetries(uint8 u8ProtoTag, uint32 u32TxRetries);
PUBLIC void __vMMAC_SetTxFrame(uint8 u8ProtoTag, tsMacFrame *pTxFrame);
PUBLIC uint32 __u32MMAC_GetTxFrame(uint8 u8ProtoTag);

#ifdef MAC_PROTO_TAG
#define vMMAC_SetTxParameters(u8Attempts, u8MinBE, u8MaxBE, u8MaxBackoffs) __vMMAC_SetTxParameters((MAC_PROTO_TAG), (u8Attempts), (u8MinBE), (u8MaxBE), (u8MaxBackoffs))
#define vMMAC_SetTxMinBE(u8MinBE) __vMMAC_SetTxMinBE((MAC_PROTO_TAG), (u8MinBE))
#define vMMAC_SetTxMaxBE(u8MaxBE) __vMMAC_SetTxMaxBE((MAC_PROTO_TAG), (u8MaxBE))
#define vMMAC_SetTxMaxBackoffs(u8MaxBackoffs) __vMMAC_SetTxMaxBackoffs((MAC_PROTO_TAG), (u8MaxBackoffs))
#define vMMAC_SetTxStartTime(u32Time) __vMMAC_SetTxStartTime((MAC_PROTO_TAG), (u32Time))
#define vMMAC_SetCcaMode(eCcaMode) __vMMAC_SetCcaMode((MAC_PROTO_TAG), (eCcaMode))
#define u32MMAC_GetCsmaContext() __u32MMAC_GetCsmaContext((MAC_PROTO_TAG))
#define u32MMAC_GetTxRetries() __u32MMAC_GetTxRetries((MAC_PROTO_TAG))
#define vMMAC_SetTxRetries(u32TxRetries) __vMMAC_SetTxRetries((MAC_PROTO_TAG), (u32TxRetries))
#define vMMAC_StartMacTransmit(psFrame, eOptions) __vMMAC_StartMacTransmit((MAC_PROTO_TAG), (psFrame), (eOptions))
#define vMMAC_StartPhyTransmit(psFrame, eOptions) __vMMAC_StartPhyTransmit((MAC_PROTO_TAG), (psFrame), (eOptions))
#define vMMAC_SetTxPend(bTxPend) __vMMAC_SetTxPend((MAC_PROTO_TAG), (bTxPend))
#define u32MMAC_GetTxErrors() __u32MMAC_GetTxErrors((MAC_PROTO_TAG))
#define bMMAC_PowerStatus() __bMMAC_PowerStatus((MAC_PROTO_TAG))
#define u32MMAC_GetTxFrame() __u32MMAC_GetTxFrame((MAC_PROTO_TAG))
#define vMMAC_SetTxFrame(pTxFrame) __vMMAC_SetTxFrame((MAC_PROTO_TAG), (pTxFrame))
#else
PUBLIC void vMMAC_SetTxParameters(uint8 u8Attempts, uint8 u8MinBE,
                                  uint8 u8MaxBE, uint8 u8MaxBackoffs);
PUBLIC void vMMAC_SetTxMinBE(uint8 u8MinBE);
PUBLIC void vMMAC_SetTxMaxBE(uint8 u8MaxBE);
PUBLIC void vMMAC_SetTxMaxBackoffs(uint8 u8MaxBackoffs);
PUBLIC void vMMAC_SetTxStartTime(uint32 u32Time);
PUBLIC void vMMAC_SetCcaMode(teCcaMode eCcaMode);
PUBLIC uint32 u32MMAC_GetCsmaContext(void);
PUBLIC uint32 u32MMAC_GetTxRetries(void);
PUBLIC void vMMAC_SetTxRetries(uint32 u32TxRetries);
PUBLIC void vMMAC_StartMacTransmit(tsMacFrame *psFrame, teTxOption eOptions);
PUBLIC void vMMAC_StartPhyTransmit(tsPhyFrame *psFrame, teTxOption eOptions);
PUBLIC void vMMAC_SetTxPend(bool_t bTxPend);
PUBLIC uint32 u32MMAC_GetTxErrors(void);
PUBLIC bool_t bMMAC_PowerStatus(void);
PUBLIC uint32 u32MMAC_GetTxFrame(void);
PUBLIC void vMMAC_SetTxFrame(tsMacFrame *pTxFrame);
#endif


#if defined (JENNIC_CHIP_FAMILY_JN518x)
/* Must be called before vMMAC_Enable() */
PUBLIC void vMMAC_SetTxPowerMode(bool_t mode);
PUBLIC void vMMAC_GetMacAddressN(tsExtAddr *psMacAddr, uint8 u8Idx);
#undef CHIP_IS_HITXPOWER_CAPABLE
#define CHIP_IS_HITXPOWER_CAPABLE() ((CHIP_K32W041AM == Chip_GetType()) || (CHIP_K32W041A == Chip_GetType()))

/* TODO - Add support for v2MAC in MacSchedMMAC.c */
/* IEEE 802.15.4-2015 support - frame version 2 */
void __V2MMAC_Enable(uint8 u8ProtoTag);

#ifdef MAC_PROTO_TAG
#define V2MMAC_Enable() __V2MMAC_Enable((MAC_PROTO_TAG))
#else
void V2MMAC_Enable(void);
#endif

void V2MMAC_RegisterIntHandler(void (*f)(uint32_t));

void __vMMAC_StartV2MacReceive(uint8 u8ProtoTag, tsPhyFrame *psFrame, teRxOption eOptions);
void __vMMAC_StartV2MacTransmit(uint8 u8ProtoTag, tsPhyFrame *psFrame, teTxOption eOptions, uint32_t txTime);
bool_t __v2MAC_is_rx_ongoing(uint8_t u8ProtoTag);
bool_t __v2MAC_is_tx_ongoing(uint8_t u8ProtoTag);

#ifdef MAC_PROTO_TAG
#define vMMAC_StartV2MacReceive(psFrame, eOptions) __vMMAC_StartV2MacReceive((MAC_PROTO_TAG), (psFrame), (eOptions))
#define vMMAC_StartV2MacTransmit(psFrame, eOptions, txTime) __vMMAC_StartV2MacTransmit((MAC_PROTO_TAG), (psFrame), (eOptions), (txTime))
#define v2MAC_is_rx_ongoing() __v2MAC_is_rx_ongoing((MAC_PROTO_TAG))
#define v2MAC_is_tx_ongoing() __v2MAC_is_tx_ongoing((MAC_PROTO_TAG))
#else
void vMMAC_StartV2MacReceive(tsPhyFrame *psFrame, teRxOption eOptions);
void vMMAC_StartV2MacTransmit(tsPhyFrame *psFrame, teTxOption eOptions, uint32_t txTime);
bool_t v2MAC_is_rx_ongoing();
bool_t v2MAC_is_tx_ongoing();
#endif

uint32_t u32V2MAC_GetRxErrors();
uint32_t u32V2MAC_GetTxErrors();

uint32_t u32V2MAC_GetRxTimestamp();

void V2MMAC_EnableCsl(uint32_t cslPeriod);
void V2MMAC_SetCslSampleTime(uint32_t cslSampleTime);

void V2MMAC_SetMacKey(uint8_t keyId, const void *prevKey, const void *currKey, const void *nextKey);
void V2MMAC_SetMacFrameCounter(uint32_t macFrameCounter);

void V2MMAC_RegisterEncFn(void (*f)(void *, const void *));

void V2MMAC_RegisterEnhAckVsIeFn(uint8_t (*l)(void *), void (*g)(void *, uint8_t *));
#endif

#if defined BUILD_OPT_DYNAMIC_MMAC
PUBLIC void vMMAC_HandleAbortInt(void);
PUBLIC void vMMAC_DynResume(void);
#endif
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif /* #ifndef MICRO_MAC_H */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

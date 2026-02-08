/*****************************************************************************
 *
 * MODULE: ControlBridge
 *
 * COMPONENT: app_network_recovery.h
 *
 * DESCRIPTION: Network Recovery - Backup/Restore network state for ZHA compatibility
 *
 *****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved.
 *
 * Copyright NXP B.V. 2016-2024. All rights reserved
 *
 ****************************************************************************/

#ifndef APP_NETWORK_RECOVERY_H_
#define APP_NETWORK_RECOVERY_H_

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include "jendefs.h"
#include "zps_apl.h"
#include "zps_nwk_nib.h"
#include "zps_apl_aib.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define NWK_RECOVERY_VERSION        0x02    /* Version 2: includes device table */
#define NWK_RECOVERY_KEY_LENGTH     16
#define NWK_RECOVERY_MAX_DEVICES    64      /* Maximum devices in backup */

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/**
 * Network Recovery Data Structure
 * Contains all essential network information needed for backup/restore
 * Total size should be kept minimal for serial transfer efficiency
 */
typedef struct
{
    /* Header */
    uint8   u8Version;                              /* Structure version for compatibility */
    uint8   u8Reserved[3];                          /* Reserved for alignment */

    /* Network Identification */
    uint64  u64ExtPanId;                            /* Extended PAN ID */
    uint64  u64IeeeAddress;                         /* Coordinator IEEE address */
    uint16  u16PanId;                               /* Short PAN ID */
    uint16  u16NwkAddress;                          /* Network address (0x0000 for coordinator) */

    /* Network Parameters */
    uint8   u8Channel;                              /* Current channel */
    uint8   u8NwkUpdateId;                          /* Network update ID */
    uint8   u8Depth;                                /* Network depth */
    uint8   u8CapabilityInfo;                       /* Capability information */

    /* Security */
    uint8   au8NwkKey[NWK_RECOVERY_KEY_LENGTH];     /* Network key */
    uint8   u8ActiveKeySeqNum;                      /* Active key sequence number */
    uint8   u8SecurityLevel;                        /* Security level */
    uint8   u8Reserved2[2];                         /* Reserved for alignment */

    /* Frame Counters */
    uint32  u32OutgoingFrameCounter;                /* Outgoing NWK frame counter */
    uint32  u32ApsFrameCounter;                     /* APS frame counter */

    /* Trust Center */
    uint64  u64TrustCenterAddress;                  /* Trust Center IEEE address */

} tsNwkRecovery;

/**
 * Device Entry for Network Recovery
 * Contains IEEE to NWK address mapping for each device
 */
typedef struct
{
    uint64  u64IeeeAddress;                         /* Device IEEE address */
    uint16  u16NwkAddress;                          /* Device network address */
    uint8   u8Flags;                                /* Device flags (router/end device, etc.) */
    uint8   u8Reserved;                             /* Reserved for alignment */
} tsNwkRecoveryDevice;

/**
 * Extended Network Recovery Data Structure (Version 2)
 * Includes device table for complete network restore
 */
typedef struct
{
    tsNwkRecovery       sNetwork;                   /* Base network recovery data */
    uint8               u8DeviceCount;              /* Number of devices in table */
    uint8               u8Reserved[3];              /* Reserved for alignment */
    tsNwkRecoveryDevice asDevices[NWK_RECOVERY_MAX_DEVICES]; /* Device table */
} tsNwkRecoveryExt;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/**
 * Obtain network recovery data from current network state
 * @param psNwkRecovery Pointer to structure to fill with recovery data
 */
PUBLIC void vNetworkRecoveryObtainRecoverData(tsNwkRecovery *psNwkRecovery);

/**
 * Insert/restore network recovery data to reinitialize network
 * @param psNwkRecovery Pointer to structure containing recovery data
 */
PUBLIC void vNetworkRecoveryInsertRecoverData(tsNwkRecovery *psNwkRecovery);

/**
 * Obtain extended network recovery data including device table
 * @param psNwkRecoveryExt Pointer to extended structure to fill
 */
PUBLIC void vNetworkRecoveryObtainRecoverDataExt(tsNwkRecoveryExt *psNwkRecoveryExt);

/**
 * Insert/restore extended network recovery data including device table
 * @param psNwkRecoveryExt Pointer to extended structure containing recovery data
 */
PUBLIC void vNetworkRecoveryInsertRecoverDataExt(tsNwkRecoveryExt *psNwkRecoveryExt);

/**
 * Get the number of devices currently in the network
 * @return Number of devices in address map
 */
PUBLIC uint8 u8NetworkRecoveryGetDeviceCount(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#endif /* APP_NETWORK_RECOVERY_H_ */

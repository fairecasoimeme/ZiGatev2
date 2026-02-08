/*****************************************************************************
 *
 * MODULE: ControlBridge
 *
 * COMPONENT: app_network_recovery.c
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

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <string.h>
#include "jendefs.h"
#include "dbg.h"
#include "PDM.h"
#include "zps_apl.h"
#include "zps_apl_af.h"
#include "zps_apl_zdo.h"
#include "zps_apl_aib.h"
#include "zps_nwk_nib.h"
#include "zps_nwk_pub.h"
#include "zps_nwk_sec.h"
#include "app_network_recovery.h"
#include "app_common.h"
#include "bdb_api.h"
#include "Log.h"
#include "PDM_IDs.h"

#ifndef TRACE_APP
#define TRACE_APP TRUE
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vNetworkRecoveryObtainRecoverData
 *
 * DESCRIPTION:
 * Extracts current network state into a recovery structure for backup
 *
 * PARAMETERS:
 * psNwkRecovery - Pointer to structure to fill with current network state
 *
 ****************************************************************************/
PUBLIC void vNetworkRecoveryObtainRecoverData(tsNwkRecovery *psNwkRecovery)
{
    void *pvNwk;
    ZPS_tsNwkNib *psNib;
    ZPS_tsAplAib *psAib;

    /* Clear structure first */
    memset(psNwkRecovery, 0, sizeof(tsNwkRecovery));

    /* Get handles */
    pvNwk = ZPS_pvAplZdoGetNwkHandle();
    psNib = ZPS_psAplZdoGetNib();
    psAib = ZPS_psAplAibGetAib();

    if (pvNwk == NULL || psNib == NULL || psAib == NULL)
    {
        vLog_Printf(TRACE_APP, LOG_ERR, "\nNwkRecovery: Failed to get handles");
        return;
    }

    /* Header */
    psNwkRecovery->u8Version = NWK_RECOVERY_VERSION;

    /* Network Identification */
    psNwkRecovery->u64ExtPanId = ZPS_u64NwkNibGetEpid(pvNwk);
    psNwkRecovery->u64IeeeAddress = ZPS_u64NwkNibGetExtAddr(pvNwk);
    psNwkRecovery->u16PanId = ZPS_u16NwkNibGetMacPanId(pvNwk);
    psNwkRecovery->u16NwkAddress = ZPS_u16NwkNibGetNwkAddr(pvNwk);

    /* Network Parameters from persisted data */
    psNwkRecovery->u8Channel = psNib->sPersist.u8VsChannel;
    psNwkRecovery->u8NwkUpdateId = psNib->sPersist.u8UpdateId;
    psNwkRecovery->u8Depth = psNib->sPersist.u8VsDepth;
    psNwkRecovery->u8CapabilityInfo = psNib->sPersist.u8CapabilityInformation;

    /* Security - Get the active network key */
    if (psNib->sTbl.psSecMatSet != NULL)
    {
        uint8 u8ActiveKeySeq = psNib->sPersist.u8ActiveKeySeqNumber;

        /* Find the key with matching sequence number */
        /* Typically there are 2 key slots (0 and 1) */
        memcpy(psNwkRecovery->au8NwkKey,
               psNib->sTbl.psSecMatSet[0].au8Key,
               NWK_RECOVERY_KEY_LENGTH);

        psNwkRecovery->u8ActiveKeySeqNum = u8ActiveKeySeq;
    }

    psNwkRecovery->u8SecurityLevel = psNib->u8SecurityLevel;

    /* Frame Counters */
#if (JENNIC_CHIP_FAMILY != JN516x) && (JENNIC_CHIP_FAMILY != JN517x)
    psNwkRecovery->u32OutgoingFrameCounter = psNib->sPersist.u32OutFC;
#else
    psNwkRecovery->u32OutgoingFrameCounter = psNib->sTbl.u32OutFC;
#endif

    /* APS Frame Counter - not directly accessible, set to 0 */
    psNwkRecovery->u32ApsFrameCounter = 0;

    /* Trust Center */
    psNwkRecovery->u64TrustCenterAddress = ZPS_eAplAibGetApsTrustCenterAddress();

    vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery Extract: EPID=%016llx PAN=%04x Ch=%d",
                psNwkRecovery->u64ExtPanId,
                psNwkRecovery->u16PanId,
                psNwkRecovery->u8Channel);
}

/****************************************************************************
 *
 * NAME: vNetworkRecoveryInsertRecoverData
 *
 * DESCRIPTION:
 * Restores network state from a recovery structure and persists to PDM.
 * After calling this function, a device reset is recommended to ensure
 * the stack reinitializes with the restored network parameters.
 *
 * PARAMETERS:
 * psNwkRecovery - Pointer to structure containing recovery data
 *
 ****************************************************************************/
PUBLIC void vNetworkRecoveryInsertRecoverData(tsNwkRecovery *psNwkRecovery)
{
    void *pvNwk;
    ZPS_tsNwkNib *psNib;
    ZPS_tsAplAib *psAib;

    /* Validate version - accept version 1 or 2 */
    if (psNwkRecovery->u8Version != 0x01 && psNwkRecovery->u8Version != 0x02)
    {
        vLog_Printf(TRACE_APP, LOG_ERR, "\nNwkRecovery: Unsupported version %d",
                    psNwkRecovery->u8Version);
        return;
    }

    /* Get handles */
    pvNwk = ZPS_pvAplZdoGetNwkHandle();
    psNib = ZPS_psAplZdoGetNib();
    psAib = ZPS_psAplAibGetAib();

    if (pvNwk == NULL || psNib == NULL)
    {
        vLog_Printf(TRACE_APP, LOG_ERR, "\nNwkRecovery: Failed to get handles");
        return;
    }

    vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery Restore: EPID=%016llx PAN=%04x Ch=%d",
                psNwkRecovery->u64ExtPanId,
                psNwkRecovery->u16PanId,
                psNwkRecovery->u8Channel);

    /* Restore Network Identification */
    ZPS_vNwkNibSetExtPanId(pvNwk, psNwkRecovery->u64ExtPanId);
    ZPS_vNwkNibSetPanId(pvNwk, psNwkRecovery->u16PanId);
    ZPS_vNwkNibSetNwkAddr(pvNwk, psNwkRecovery->u16NwkAddress);

    /* Restore Network Parameters - both runtime and persist copies */
    ZPS_vNwkNibSetChannel(pvNwk, psNwkRecovery->u8Channel);
    psNib->sPersist.u8VsChannel = psNwkRecovery->u8Channel;
    psNib->sPersist.u8UpdateId = psNwkRecovery->u8NwkUpdateId;
    psNib->sPersist.u8VsDepth = psNwkRecovery->u8Depth;
    psNib->sPersist.u8CapabilityInformation = psNwkRecovery->u8CapabilityInfo;
    psNib->sPersist.u16VsPanId = psNwkRecovery->u16PanId;
    psNib->sPersist.u16NwkAddr = psNwkRecovery->u16NwkAddress;
    psNib->sPersist.u64ExtPanId = psNwkRecovery->u64ExtPanId;

    /* Restore Security */
    if (psNib->sTbl.psSecMatSet != NULL)
    {
        memcpy(psNib->sTbl.psSecMatSet[0].au8Key,
               psNwkRecovery->au8NwkKey,
               NWK_RECOVERY_KEY_LENGTH);

        psNib->sTbl.psSecMatSet[0].u8KeySeqNum = psNwkRecovery->u8ActiveKeySeqNum;
        psNib->sPersist.u8ActiveKeySeqNumber = psNwkRecovery->u8ActiveKeySeqNum;

        /* Mark key as valid */
        psNib->sTbl.psSecMatSet[0].u8KeyType = ZPS_NWK_SEC_NETWORK_KEY;
    }

    /* Restore Frame Counter - add safety margin to avoid replay attacks */
#if (JENNIC_CHIP_FAMILY != JN516x) && (JENNIC_CHIP_FAMILY != JN517x)
    psNib->sPersist.u32OutFC = psNwkRecovery->u32OutgoingFrameCounter + 1000;
#else
    psNib->sTbl.u32OutFC = psNwkRecovery->u32OutgoingFrameCounter + 1000;
#endif

    /*
     * CRITICAL: Reset incoming frame counters to 0
     * This allows devices to communicate with us after restore.
     * Without this, their messages would be rejected as replay attacks.
     */
    if (psNib->sTbl.pu32InFCSet != NULL)
    {
        memset(psNib->sTbl.pu32InFCSet, 0, sizeof(uint32) * psNib->sTblSize.u16NtActv);
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Reset %d incoming frame counters",
                    psNib->sTblSize.u16NtActv);
    }

    /* Restore Trust Center Address */
    ZPS_eAplAibSetApsTrustCenterAddress(psNwkRecovery->u64TrustCenterAddress);

    /* Set AIB fields for coordinator */
    if (psAib != NULL)
    {
        psAib->u64ApsTrustCenterAddress = psNwkRecovery->u64TrustCenterAddress;
        psAib->u64ApsUseExtendedPanid = psNwkRecovery->u64ExtPanId;

        /* Critical for coordinator: set designated coordinator flag */
        if (psNwkRecovery->u16NwkAddress == 0x0000)
        {
            psAib->bApsDesignatedCoordinator = TRUE;
        }

        /* Set insecure join for permit join to work */
        psAib->bApsUseInsecureJoin = TRUE;
    }

    /* Set AIB via API functions for proper internal state */
    ZPS_eAplAibSetApsUseExtendedPanId(psNwkRecovery->u64ExtPanId);
    if (psNwkRecovery->u16NwkAddress == 0x0000)
    {
        ZPS_eAplAibSetApsDesignatedCoordinator(TRUE);
    }
    ZPS_eAplAibSetApsUseInsecureJoin(TRUE);

    vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Saving to PDM...");

    /*
     * Persist all ZPS records to PDM
     * This saves: NIB, AIB, security material, address maps, etc.
     */
    ZPS_vSaveAllZpsRecords();

    /* Also save the security material key separately to ensure it's persisted */
    if (psNib->sTbl.psSecMatSet != NULL)
    {
        PDM_eSaveRecordData(PDM_ID_INTERNAL_SEC_MATERIAL_KEY,
                            psNib->sTbl.psSecMatSet,
                            sizeof(ZPS_tsNwkSecMaterialSet) * 2);
    }

    /*
     * CRITICAL: Restore sZllState - this is essential for the stack to function
     * After factory reset, sZllState.eState = FACTORY_NEW and eNodeState = E_STARTUP
     * We need to set them to NOT_FACTORY_NEW and E_RUNNING for the stack to operate
     */
    vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Restoring sZllState...");

    /* Set state to indicate we are on a network */
    sZllState.eState = NOT_FACTORY_NEW;
    sZllState.eNodeState = E_RUNNING;

    /* Set device type based on NWK address (0x0000 = coordinator) */
    if (psNwkRecovery->u16NwkAddress == 0x0000)
    {
        sZllState.u8DeviceType = ZPS_ZDO_DEVICE_COORD;
    }
    else
    {
        sZllState.u8DeviceType = ZPS_ZDO_DEVICE_ROUTER;
    }

    /* Set channel */
    sZllState.u8MyChannel = psNwkRecovery->u8Channel;
    sZllState.u16MyAddr = psNwkRecovery->u16NwkAddress;

#ifdef FULL_FUNC_DEVICE
    /* Reset address allocation ranges if not already set */
    if (sZllState.u16FreeAddrLow == 0)
    {
        sZllState.u16FreeAddrLow = 0x0001;
        sZllState.u16FreeAddrHigh = 0xFFF7;
        sZllState.u16FreeGroupLow = 0x0001;
        sZllState.u16FreeGroupHigh = 0xFFF7;
    }
#endif

    /* Save sZllState to PDM */
    PDM_eSaveRecordData(PDM_ID_APP_ZLL_CMSSION, &sZllState, sizeof(tsZllState));

    /* Also update BDB attribute to indicate we are on a network */
    sBDB.sAttrib.bbdbNodeIsOnANetwork = TRUE;

    /*
     * CRITICAL: Activate the network stack in RAM
     * Without this, the stack remains in an inactive state and commands fail with 0x89
     */
    vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Activating network stack...");

    /* Set device type in ZDO layer */
    if (psNwkRecovery->u16NwkAddress == 0x0000)
    {
        ZPS_vNwkSetDeviceType(pvNwk, ZPS_NWK_DT_COORDINATOR);
        ZPS_vSetZdoDeviceType(ZPS_ZDO_DEVICE_COORD);
    }
    else
    {
        ZPS_vNwkSetDeviceType(pvNwk, ZPS_NWK_DT_ROUTER);
        ZPS_vSetZdoDeviceType(ZPS_ZDO_DEVICE_ROUTER);
    }

    /* Set network depth (0 for coordinator) */
    ZPS_vNwkNibSetDepth(pvNwk, psNwkRecovery->u8Depth);

    /* Activate the network state - this is CRITICAL */
    ZPS_vSetNwkStateActive(pvNwk);

    /* Start the router/coordinator - FALSE means no device announce */
    ZPS_eAplZdoStartRouter(FALSE);

    vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Restore complete - network active");
}

/****************************************************************************
 *
 * NAME: u8NetworkRecoveryGetDeviceCount
 *
 * DESCRIPTION:
 * Returns the number of devices in the address map table
 *
 * RETURNS:
 * Number of valid entries in the address map
 *
 ****************************************************************************/
PUBLIC uint8 u8NetworkRecoveryGetDeviceCount(void)
{
    ZPS_tsNwkNib *psNib = ZPS_psAplZdoGetNib();
    uint8 u8Count = 0;
    uint16 u16Index;

    if (psNib == NULL || psNib->sTbl.pu64AddrExtAddrMap == NULL)
    {
        return 0;
    }

    /* Count valid entries in address map */
    for (u16Index = 0; u16Index < psNib->sTblSize.u16AddrMap; u16Index++)
    {
        /* Check if entry is valid (IEEE address != 0) */
        if (psNib->sTbl.pu64AddrExtAddrMap[u16Index] != 0)
        {
            u8Count++;
        }
    }

    return u8Count;
}

/****************************************************************************
 *
 * NAME: vNetworkRecoveryObtainRecoverDataExt
 *
 * DESCRIPTION:
 * Extracts current network state including device table for complete backup.
 * Uses the same table access method as E_SL_MSG_GET_DISPLAY_ADDRESS_MAP_TABLE
 * to ensure consistency with the Zigbee stack's address mapping.
 *
 * PARAMETERS:
 * psNwkRecoveryExt - Pointer to extended structure to fill
 *
 ****************************************************************************/
PUBLIC void vNetworkRecoveryObtainRecoverDataExt(tsNwkRecoveryExt *psNwkRecoveryExt)
{
    ZPS_tsNwkNib *psNib;
    void *pvNwk;
    uint16 u16Index;
    uint8 u8DevIndex = 0;

    /* Clear structure first */
    memset(psNwkRecoveryExt, 0, sizeof(tsNwkRecoveryExt));

    /* Get base network data */
    vNetworkRecoveryObtainRecoverData(&psNwkRecoveryExt->sNetwork);

    /* Update version to indicate extended format */
    psNwkRecoveryExt->sNetwork.u8Version = 0x02;

    /* Get NIB for device table access */
    pvNwk = ZPS_pvAplZdoGetNwkHandle();
    psNib = ZPS_psAplZdoGetNib();

    if (psNib == NULL || pvNwk == NULL)
    {
        vLog_Printf(TRACE_APP, LOG_WARNING, "\nNwkRecovery: NIB not available");
        psNwkRecoveryExt->u8DeviceCount = 0;
        return;
    }

    if (psNib->sTbl.pu16AddrMapNwk == NULL || psNib->sTbl.pu16AddrLookup == NULL)
    {
        vLog_Printf(TRACE_APP, LOG_WARNING, "\nNwkRecovery: Address map tables not available");
        psNwkRecoveryExt->u8DeviceCount = 0;
        return;
    }

    vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: AddrMap size=%d", psNib->sTblSize.u16AddrMap);

    /* Extract device table entries using the same method as GET_DISPLAY_ADDRESS_MAP_TABLE */
    for (u16Index = 0; u16Index < psNib->sTblSize.u16AddrMap &&
                       u8DevIndex < NWK_RECOVERY_MAX_DEVICES; u16Index++)
    {
        uint16 u16NwkAddr = psNib->sTbl.pu16AddrMapNwk[u16Index];

        /* Skip invalid entries (0xFFFE = unknown, 0xFFFF = broadcast) */
        if (u16NwkAddr < 0xFFFE)
        {
            /* Use the lookup table to get the IEEE address, same as 0x0015 command */
            uint64 u64IeeeAddr = ZPS_u64NwkNibGetMappedIeeeAddr(pvNwk, psNib->sTbl.pu16AddrLookup[u16Index]);

            /* Skip coordinator (NWK = 0x0000) and invalid IEEE addresses */
            if (u16NwkAddr != 0x0000 && u64IeeeAddr != 0)
            {
                psNwkRecoveryExt->asDevices[u8DevIndex].u64IeeeAddress = u64IeeeAddr;
                psNwkRecoveryExt->asDevices[u8DevIndex].u16NwkAddress = u16NwkAddr;
                psNwkRecoveryExt->asDevices[u8DevIndex].u8Flags = 0;  /* Reserved for future use */
                psNwkRecoveryExt->asDevices[u8DevIndex].u8Reserved = 0;

                vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery Device[%d]: IEEE=%016llx NWK=%04x",
                            u8DevIndex, u64IeeeAddr, u16NwkAddr);

                u8DevIndex++;
            }
        }
    }

    /* Also check the Neighbor Table Active for devices that might not be in address map */
    if (psNib->sTbl.psNtActv != NULL && u8DevIndex < NWK_RECOVERY_MAX_DEVICES)
    {
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: NtActv size=%d", psNib->sTblSize.u16NtActv);

        for (u16Index = 0; u16Index < psNib->sTblSize.u16NtActv &&
                           u8DevIndex < NWK_RECOVERY_MAX_DEVICES; u16Index++)
        {
            uint16 u16NwkAddr = psNib->sTbl.psNtActv[u16Index].u16NwkAddr;

            /* Skip invalid and coordinator entries */
            if (u16NwkAddr < 0xFFFE && u16NwkAddr != 0x0000)
            {
                uint64 u64IeeeAddr = ZPS_u64NwkNibGetMappedIeeeAddr(pvNwk, psNib->sTbl.psNtActv[u16Index].u16Lookup);

                if (u64IeeeAddr != 0)
                {
                    /* Check if this device is already in our list */
                    bool_t bAlreadyAdded = FALSE;
                    uint8 u8CheckIdx;
                    for (u8CheckIdx = 0; u8CheckIdx < u8DevIndex; u8CheckIdx++)
                    {
                        if (psNwkRecoveryExt->asDevices[u8CheckIdx].u64IeeeAddress == u64IeeeAddr)
                        {
                            bAlreadyAdded = TRUE;
                            break;
                        }
                    }

                    if (!bAlreadyAdded)
                    {
                        psNwkRecoveryExt->asDevices[u8DevIndex].u64IeeeAddress = u64IeeeAddr;
                        psNwkRecoveryExt->asDevices[u8DevIndex].u16NwkAddress = u16NwkAddr;
                        psNwkRecoveryExt->asDevices[u8DevIndex].u8Flags =
                            psNib->sTbl.psNtActv[u16Index].uAncAttrs.bfBitfields.u1PowerSource ? 0x01 : 0x00;
                        psNwkRecoveryExt->asDevices[u8DevIndex].u8Reserved = 0;

                        vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery NtActv[%d]: IEEE=%016llx NWK=%04x",
                                    u8DevIndex, u64IeeeAddr, u16NwkAddr);

                        u8DevIndex++;
                    }
                }
            }
        }
    }

    psNwkRecoveryExt->u8DeviceCount = u8DevIndex;

    vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Extracted total %d devices", u8DevIndex);
}

/****************************************************************************
 *
 * NAME: vNetworkRecoveryInsertRecoverDataExt
 *
 * DESCRIPTION:
 * Restores network state including device table from extended backup
 *
 * PARAMETERS:
 * psNwkRecoveryExt - Pointer to extended structure containing recovery data
 *
 ****************************************************************************/
PUBLIC void vNetworkRecoveryInsertRecoverDataExt(tsNwkRecoveryExt *psNwkRecoveryExt)
{
    ZPS_tsNwkNib *psNib;
    uint8 u8Index;

    /* First restore base network data */
    vNetworkRecoveryInsertRecoverData(&psNwkRecoveryExt->sNetwork);

    /* Get NIB for device table restoration */
    psNib = ZPS_psAplZdoGetNib();
    if (psNib == NULL || psNib->sTbl.pu64AddrExtAddrMap == NULL ||
        psNib->sTbl.pu16AddrMapNwk == NULL)
    {
        vLog_Printf(TRACE_APP, LOG_ERR, "\nNwkRecovery: Cannot restore devices - no address map");
        return;
    }

    if (psNwkRecoveryExt->u8DeviceCount == 0)
    {
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: No devices to restore");
        return;
    }

    vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Restoring %d devices...",
                psNwkRecoveryExt->u8DeviceCount);

    /* Restore each device entry to the address map */
    for (u8Index = 0; u8Index < psNwkRecoveryExt->u8DeviceCount &&
                      u8Index < NWK_RECOVERY_MAX_DEVICES; u8Index++)
    {
        uint64 u64IeeeAddr = psNwkRecoveryExt->asDevices[u8Index].u64IeeeAddress;
        uint16 u16NwkAddr = psNwkRecoveryExt->asDevices[u8Index].u16NwkAddress;

        /* Skip invalid entries */
        if (u64IeeeAddr == 0 || u16NwkAddr == 0xFFFF)
        {
            continue;
        }

        /* Find a free slot in the address map or update existing */
        uint16 u16Slot;
        bool_t bFound = FALSE;

        /* First check if this IEEE already exists */
        for (u16Slot = 0; u16Slot < psNib->sTblSize.u16AddrMap; u16Slot++)
        {
            if (psNib->sTbl.pu64AddrExtAddrMap[u16Slot] == u64IeeeAddr)
            {
                /* Update existing entry */
                psNib->sTbl.pu16AddrMapNwk[u16Slot] = u16NwkAddr;
                bFound = TRUE;
                vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Updated device slot %d: IEEE=%016llx NWK=%04x",
                            u16Slot, u64IeeeAddr, u16NwkAddr);
                break;
            }
        }

        /* If not found, find a free slot */
        if (!bFound)
        {
            for (u16Slot = 0; u16Slot < psNib->sTblSize.u16AddrMap; u16Slot++)
            {
                if (psNib->sTbl.pu64AddrExtAddrMap[u16Slot] == 0)
                {
                    /* Insert new entry */
                    psNib->sTbl.pu64AddrExtAddrMap[u16Slot] = u64IeeeAddr;
                    psNib->sTbl.pu16AddrMapNwk[u16Slot] = u16NwkAddr;

                    /* Update lookup table if available */
                    if (psNib->sTbl.pu16AddrLookup != NULL)
                    {
                        psNib->sTbl.pu16AddrLookup[u16Slot] = u16Slot;
                    }

                    bFound = TRUE;
                    vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Added device slot %d: IEEE=%016llx NWK=%04x",
                                u16Slot, u64IeeeAddr, u16NwkAddr);
                    break;
                }
            }
        }

        if (!bFound)
        {
            vLog_Printf(TRACE_APP, LOG_WARNING, "\nNwkRecovery: No slot for device IEEE=%016llx",
                        u64IeeeAddr);
        }
    }

    /*
     * CRITICAL: Restore Neighbor Table (psNtActv) entries
     * Without this, the coordinator cannot route messages to restored devices.
     * The neighbor table contains routing information that allows the stack
     * to send messages directly to neighboring devices.
     */
    if (psNib->sTbl.psNtActv != NULL)
    {
        vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Restoring Neighbor Table entries...");

        for (u8Index = 0; u8Index < psNwkRecoveryExt->u8DeviceCount &&
                          u8Index < NWK_RECOVERY_MAX_DEVICES; u8Index++)
        {
            uint64 u64IeeeAddr = psNwkRecoveryExt->asDevices[u8Index].u64IeeeAddress;
            uint16 u16NwkAddr = psNwkRecoveryExt->asDevices[u8Index].u16NwkAddress;
            uint8 u8Flags = psNwkRecoveryExt->asDevices[u8Index].u8Flags;

            /* Skip invalid entries */
            if (u64IeeeAddr == 0 || u16NwkAddr == 0xFFFF || u16NwkAddr == 0x0000)
            {
                continue;
            }

            /* Find a free slot in the Neighbor Table */
            uint16 u16NtSlot;
            bool_t bNtFound = FALSE;

            /* First check if this device already exists in the neighbor table */
            for (u16NtSlot = 0; u16NtSlot < psNib->sTblSize.u16NtActv; u16NtSlot++)
            {
                if (psNib->sTbl.psNtActv[u16NtSlot].u16NwkAddr == u16NwkAddr)
                {
                    /* Entry already exists - update it */
                    bNtFound = TRUE;
                    vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: NtActv[%d] already has NWK=%04x",
                                u16NtSlot, u16NwkAddr);
                    break;
                }
            }

            /* If not found, look for a free slot */
            if (!bNtFound)
            {
                for (u16NtSlot = 0; u16NtSlot < psNib->sTblSize.u16NtActv; u16NtSlot++)
                {
                    /* Check if slot is free (NWK address 0xFFFF or 0xFFFE indicates unused) */
                    if (psNib->sTbl.psNtActv[u16NtSlot].u16NwkAddr >= 0xFFFE)
                    {
                        /* Find the lookup index for this IEEE address */
                        uint16 u16Lookup = 0xFFFF;
                        uint16 u16MapIdx;

                        for (u16MapIdx = 0; u16MapIdx < psNib->sTblSize.u16AddrMap; u16MapIdx++)
                        {
                            if (psNib->sTbl.pu64AddrExtAddrMap[u16MapIdx] == u64IeeeAddr)
                            {
                                u16Lookup = u16MapIdx;
                                break;
                            }
                        }

                        if (u16Lookup == 0xFFFF)
                        {
                            vLog_Printf(TRACE_APP, LOG_WARNING, "\nNwkRecovery: No lookup for IEEE=%016llx",
                                        u64IeeeAddr);
                            break;
                        }

                        /* Initialize the neighbor table entry */
                        memset(&psNib->sTbl.psNtActv[u16NtSlot], 0, sizeof(ZPS_tsNwkActvNtEntry));

                        psNib->sTbl.psNtActv[u16NtSlot].u16NwkAddr = u16NwkAddr;
                        psNib->sTbl.psNtActv[u16NtSlot].u16Lookup = u16Lookup;
                        psNib->sTbl.psNtActv[u16NtSlot].u8TxFailed = 0;
                        psNib->sTbl.psNtActv[u16NtSlot].u8LinkQuality = 255;  /* Best LQI initially */
                        psNib->sTbl.psNtActv[u16NtSlot].u8Age = 0;

                        /* Set bitfield attributes for a router device */
                        /* u8Flags bit 0 = PowerSource (1=mains, 0=battery) */
                        psNib->sTbl.psNtActv[u16NtSlot].uAncAttrs.bfBitfields.u1PowerSource = (u8Flags & 0x01) ? 1 : 0;
                        psNib->sTbl.psNtActv[u16NtSlot].uAncAttrs.bfBitfields.u1RxOnWhenIdle = 1;  /* Router */
                        psNib->sTbl.psNtActv[u16NtSlot].uAncAttrs.bfBitfields.u1DeviceType = 1;   /* Router (ZR/ZC) */
                        psNib->sTbl.psNtActv[u16NtSlot].uAncAttrs.bfBitfields.u2Relationship = 1; /* Child */
                        psNib->sTbl.psNtActv[u16NtSlot].uAncAttrs.bfBitfields.u1Authenticated = 1;
                        psNib->sTbl.psNtActv[u16NtSlot].uAncAttrs.bfBitfields.u1Used = 1;

                        bNtFound = TRUE;
                        vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Added NtActv[%d]: NWK=%04x Lookup=%d",
                                    u16NtSlot, u16NwkAddr, u16Lookup);
                        break;
                    }
                }
            }

            if (!bNtFound)
            {
                vLog_Printf(TRACE_APP, LOG_WARNING, "\nNwkRecovery: No free NtActv slot for NWK=%04x",
                            u16NwkAddr);
            }
        }
    }
    else
    {
        vLog_Printf(TRACE_APP, LOG_WARNING, "\nNwkRecovery: Neighbor Table not available");
    }

    /*
     * Rebuild the sorted neighbor table linked list
     * This is critical because the neighbor table uses a linked list structure
     * for efficient traversal. After manually adding entries, we need to rebuild it.
     */
    {
        void *pvNwk = ZPS_pvAplZdoGetNwkHandle();
        if (pvNwk != NULL)
        {
            vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Rebuilding NtActv sorted list...");
            zps_vBuildSortedActvNTList(pvNwk);
        }
    }

    /* Persist all records to PDM */
    vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Saving all ZPS records to PDM...");

    /* Save the entire NIB including address tables */
    ZPS_vSaveAllZpsRecords();

    /* Also save individual components explicitly to ensure complete persistence */
    {
        void *pvNwk = ZPS_pvAplZdoGetNwkHandle();
        if (pvNwk != NULL)
        {
            /* Save Neighbor Table */
            ZPS_vNwkSaveNt(pvNwk);
            vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Saved Neighbor Table");

            /* Save NIB (Network Information Base) */
            ZPS_vNwkSaveNib(pvNwk);
            vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Saved NIB");

            /* Save Security Material */
            ZPS_vNwkSaveSecMat(pvNwk);
            vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Saved Security Material");

            /* Save all NWK records */
            ZPS_vNwkSaveAllRecords(pvNwk);
            vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Saved all NWK records");
        }
    }

    vLog_Printf(TRACE_APP, LOG_DEBUG, "\nNwkRecovery: Device restore complete");
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

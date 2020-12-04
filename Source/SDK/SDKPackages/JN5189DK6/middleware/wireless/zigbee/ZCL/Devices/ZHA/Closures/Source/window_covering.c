/*****************************************************************************
 *
 * MODULE:             Window Covering Device
 *
 * COMPONENT:          window_covering.c
 *
 * DESCRIPTION:        ZigBee Window Covering profile functions
 *
 *****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5164,
 * JN5161, JN5148, JN5142, JN5139].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each  copy or partial copy of the software.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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
 * Copyright NXP B.V. 2019. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <string.h>

#include "zps_apl.h"
#include "zcl_heap.h"
#include "zcl.h"
#include "zcl_options.h"
#include "window_covering.h"

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
 * NAME: eZHA_RegisterWindowCoveringEndPoint
 *
 * DESCRIPTION:
 * Registers an window covering device with the ZCL layer
 *
 * PARAMETERS:  Name                            Usage
 *              u8EndPointIdentifier            Endpoint being registered
 *              cbCallBack                      Pointer to endpoint callback
 *              psDeviceInfo                    Pointer to struct containing
 *                                              data for endpoint
 *
 * RETURNS:
 * teZCL_Status
 *
 ****************************************************************************/
PUBLIC teZCL_Status eZHA_RegisterWindowCoveringEndPoint(
    uint8 u8EndPointIdentifier,
    tfpZCL_ZCLCallBackFunction cbCallBack,
    tsZHA_WindowCovering *psDeviceInfo)
{

    /* Fill in end point details */
    psDeviceInfo->sEndPoint.u8EndPointNumber = u8EndPointIdentifier;
    psDeviceInfo->sEndPoint.u16ManufacturerCode = ZCL_MANUFACTURER_CODE;
    psDeviceInfo->sEndPoint.u16ProfileEnum = HA_PROFILE_ID;
    psDeviceInfo->sEndPoint.bIsManufacturerSpecificProfile = FALSE;
    psDeviceInfo->sEndPoint.u16NumberOfClusters = sizeof(tsZHA_WindowCoveringClusterInstances) / sizeof(tsZCL_ClusterInstance);
    psDeviceInfo->sEndPoint.psClusterInstance = (tsZCL_ClusterInstance*)&psDeviceInfo->sClusterInstance;
    psDeviceInfo->sEndPoint.bDisableDefaultResponse = ZCL_DISABLE_DEFAULT_RESPONSES;
    psDeviceInfo->sEndPoint.pCallBackFunctions = cbCallBack;

    /* Mandatory server clusters */
#if (defined CLD_BASIC) && (defined BASIC_SERVER)
    /* Create an instance of a Basic cluster as a server */
    if(eCLD_BasicCreateBasic(&psDeviceInfo->sClusterInstance.sBasicServer,
                          TRUE,
                          &sCLD_Basic,
                          &psDeviceInfo->sBasicServerCluster,
                          &au8BasicClusterAttributeControlBits[0]) != E_ZCL_SUCCESS)
    {
        return E_ZCL_FAIL;
    }
#endif

/* Mandatory client clusters */
#if (defined CLD_BASIC) && (defined BASIC_CLIENT)
    /* Create an instance of a Basic cluster as a client */
    if(eCLD_BasicCreateBasic(&psDeviceInfo->sClusterInstance.sBasicClient,
                          FALSE,
                          &sCLD_Basic,
                          &psDeviceInfo->sBasicClientCluster,
                          &au8BasicClusterAttributeControlBits[0]) != E_ZCL_SUCCESS)
    {
        return E_ZCL_FAIL;
    }
#endif

#if (defined CLD_IDENTIFY) && (defined IDENTIFY_SERVER)
    /* Create an instance of an Identify cluster as a server */
    if(eCLD_IdentifyCreateIdentify(&psDeviceInfo->sClusterInstance.sIdentifyServer,
                          TRUE,
                          &sCLD_Identify,
                          &psDeviceInfo->sIdentifyServerCluster,
                          &au8IdentifyAttributeControlBits[0],
                          &psDeviceInfo->sIdentifyServerCustomDataStructure) != E_ZCL_SUCCESS)
    {
        return E_ZCL_FAIL;
    }
#endif

#if (defined CLD_IDENTIFY) && (defined IDENTIFY_CLIENT)
    /* Create an instance of an Identify cluster as a client */
    if(eCLD_IdentifyCreateIdentify(&psDeviceInfo->sClusterInstance.sIdentifyClient,
                          FALSE,
                          &sCLD_Identify,
                          &psDeviceInfo->sIdentifyClientCluster,
                          &au8IdentifyAttributeControlBits[0],
                          &psDeviceInfo->sIdentifyClientCustomDataStructure) != E_ZCL_SUCCESS)
    {
        return E_ZCL_FAIL;
    }
#endif

#if (defined CLD_GROUPS) && (defined GROUPS_SERVER)
    /* Create an instance of a Groups cluster as a server */
    if(eCLD_GroupsCreateGroups(&psDeviceInfo->sClusterInstance.sGroupsServer,
                          TRUE,
                          &sCLD_Groups,
                          &psDeviceInfo->sGroupsServerCluster,
                          &au8GroupsAttributeControlBits[0],
                          &psDeviceInfo->sGroupsServerCustomDataStructure,
                          &psDeviceInfo->sEndPoint) != E_ZCL_SUCCESS)
    {
        return E_ZCL_FAIL;
    }
#endif

#if (defined CLD_GROUPS) && (defined GROUPS_CLIENT)
    /* Create an instance of a Groups cluster as a client */
   if(eCLD_GroupsCreateGroups(&psDeviceInfo->sClusterInstance.sGroupsClient,
                          FALSE,
                          &sCLD_Groups,
                          &psDeviceInfo->sGroupsClientCluster,
                          &au8GroupsAttributeControlBits[0],
                          &psDeviceInfo->sGroupsClientCustomDataStructure ,
                          &psDeviceInfo->sEndPoint) != E_ZCL_SUCCESS)
    {
        return E_ZCL_FAIL;
    }
#endif

#if (defined CLD_SCENES) && (defined SCENES_SERVER)
    /* Create an instance of a Scenes cluster as a server */
    if(eCLD_ScenesCreateScenes(&psDeviceInfo->sClusterInstance.sScenesServer,
                          TRUE,
                          &sCLD_Scenes,
                          &psDeviceInfo->sScenesServerCluster,
                          &au8ScenesAttributeControlBits[0],
                          &psDeviceInfo->sScenesServerCustomDataStructure,
                          &psDeviceInfo->sEndPoint) != E_ZCL_SUCCESS)
    {
        // Need to convert from cluster specific to ZCL return type so we lose the extra information of the return code
        return E_ZCL_FAIL;
    }
#endif

#if (defined CLD_SCENES) && (defined SCENES_CLIENT)
    /* Create an instance of a Scenes cluster as a client */
    if(eCLD_ScenesCreateScenes(&psDeviceInfo->sClusterInstance.sScenesClient,
                          FALSE,
                          &sCLD_Scenes,
                          &psDeviceInfo->sScenesClientCluster,
                          &au8ScenesAttributeControlBits[0],
                          &psDeviceInfo->sScenesClientCustomDataStructure,
                          &psDeviceInfo->sEndPoint) != E_ZCL_SUCCESS)
    {
        // Need to convert from cluster specific to ZCL return type so we lose the extra information of the return code
        return E_ZCL_FAIL;
    }
#endif

#if (defined CLD_WINDOW_COVERING) && (defined WINDOW_COVERING_SERVER)
    /* Create an instance of a Window Covering cluster as a server */
    if(eCLD_WindowCoveringCreateWindowCovering(&psDeviceInfo->sClusterInstance.sWindowCoveringServer,
                          TRUE,
                          &sCLD_WindowCovering,
                          &psDeviceInfo->sWindowCoveringServerCluster,
                          &au8WindowCoveringAttributeControlBits[0],
                          &psDeviceInfo->sWindowCoveringServerCustomDataStructure /*,
                          &psDeviceInfo->sEndPoint*/) != E_ZCL_SUCCESS)
    {
        // Need to convert from cluster specific to ZCL return type so we lose the extra information of the return code
        return E_ZCL_FAIL;
    }
#endif

#if (defined CLD_WINDOW_COVERING) && (defined WINDOW_COVERING_CLIENT)
    /* Create an instance of a Scenes cluster as a client */
    if(eCLD_WindowCoveringCreateWindowCovering(&psDeviceInfo->sClusterInstance.sWindowCoveringClient,
                          FALSE,
                          &sCLD_WindowCoveringClient,
                          &psDeviceInfo->sWindowCoveringClientCluster,
                          &au8WindowCoveringClientAttributeControlBits[0],
                          &psDeviceInfo->sWindowCoveringClientCustomDataStructure /*,
                          &psDeviceInfo->sEndPoint*/) != E_ZCL_SUCCESS)
    {
        // Need to convert from cluster specific to ZCL return type so we lose the extra information of the return code
        return E_ZCL_FAIL;
    }
#endif

#if (defined CLD_OTA) && (defined OTA_CLIENT)
    /* Create an instance of an OTA cluster as a client */
    if(eOTA_Create(&psDeviceInfo->sClusterInstance.sOTAClient,
                      FALSE,  /* client */
                      &sCLD_OTA,
                      &psDeviceInfo->sCLD_OTA,  /* cluster definition */
                      u8EndPointIdentifier,
                      NULL,
                      &psDeviceInfo->sCLD_OTA_CustomDataStruct)!= E_ZCL_SUCCESS)
    {
        // Need to convert from cluster specific to ZCL return type so we lose the extra information of the return code
        return E_ZCL_FAIL;
    }
#endif

    return eZCL_Register(&psDeviceInfo->sEndPoint);
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/


/*****************************************************************************
 *
 * MODULE:             Window Covering Device
 *
 * COMPONENT:          window_covering.h
 *
 * DESCRIPTION:        Header for Window Covering Device
 *
 *****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products  [NXP Microcontrollers such as JN5168, JN5164,
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
 * Copyright NXP B.V. 2018. All rights reserved
 *
 ****************************************************************************/

#ifndef WINDOW_COVERING_H_
#define WINDOW_COVERING_H_

#if defined __cplusplus
extern "C" {
#endif

#include <jendefs.h>
#include "zcl.h"
#include "zcl_options.h"
#include "Basic.h"
#include "Identify.h"
#include "Groups.h"
#include "OnOff.h"
#ifdef CLD_OTA
#include "OTA.h"
#endif
#include "Scenes.h"
#include "WindowCovering.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define WINDOW_COVERING_DEVICE_ID 0x0202

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/* Holds cluster instances */
typedef struct
{
    /* All ) */
#if (defined CLD_BASIC) && (defined BASIC_SERVER)
    tsZCL_ClusterInstance sBasicServer;
#endif

#if (defined CLD_BASIC) && (defined BASIC_CLIENT)
    tsZCL_ClusterInstance sBasicClient;
#endif

#if (defined CLD_IDENTIFY) && (defined IDENTIFY_SERVER)
    tsZCL_ClusterInstance sIdentifyServer;
#endif

#if (defined CLD_IDENTIFY) && (defined IDENTIFY_CLIENT)
    tsZCL_ClusterInstance sIdentifyClient;
#endif

#if (defined CLD_GROUPS) && (defined GROUPS_SERVER)
    tsZCL_ClusterInstance sGroupsServer;
#endif

#if (defined CLD_GROUPS) && (defined GROUPS_CLIENT)
    tsZCL_ClusterInstance sGroupsClient;
#endif

#if (defined CLD_SCENES) && (defined SCENES_SERVER)
    tsZCL_ClusterInstance sScenesServer;
#endif

#if (defined CLD_SCENES) && (defined SCENES_CLIENT)
    tsZCL_ClusterInstance sScenesClient;
#endif

#if (defined CLD_WINDOW_COVERING) && (defined WINDOW_COVERING_SERVER)
    tsZCL_ClusterInstance sWindowCoveringServer;
#endif

#if (defined CLD_WINDOW_COVERING) && (defined WINDOW_COVERING_CLIENT)
    tsZCL_ClusterInstance sWindowCoveringClient;
#endif

/* OTA Cluster Instance */
#if (defined CLD_OTA) && (defined OTA_CLIENT)
    tsZCL_ClusterInstance sOTAClient;
#endif
} tsZHA_WindowCoveringClusterInstances __attribute__ ((aligned(4)));


/* Holds everything required to create an instance of base device */
typedef struct
{
    tsZCL_EndPointDefinition sEndPoint;

    /* Cluster instances */
    tsZHA_WindowCoveringClusterInstances sClusterInstance;

    /* Mandatory server clusters */
#if (defined CLD_BASIC) && (defined BASIC_SERVER)
    /* Basic Cluster - Server */
    tsCLD_Basic sBasicServerCluster;
#endif

#if (defined CLD_BASIC) && (defined BASIC_CLIENT)
    /* Basic Cluster - Client */
    tsCLD_Basic sBasicClientCluster;
#endif

#if (defined CLD_IDENTIFY) && (defined IDENTIFY_SERVER)
    /* Identify Cluster - Server */
    tsCLD_Identify sIdentifyServerCluster;
    tsCLD_IdentifyCustomDataStructure sIdentifyServerCustomDataStructure;
#endif

#if (defined CLD_IDENTIFY) && (defined IDENTIFY_CLIENT)
    /* Identify Cluster - Client */
    tsCLD_Identify sIdentifyClientCluster;
    tsCLD_IdentifyCustomDataStructure sIdentifyClientCustomDataStructure;
#endif

#if (defined CLD_GROUPS) && (defined GROUPS_SERVER)
    /* Groups Cluster - Server */
    tsCLD_Groups sGroupsServerCluster;
    tsCLD_GroupsCustomDataStructure sGroupsServerCustomDataStructure;
#endif

#if (defined CLD_GROUPS) && (defined GROUPS_CLIENT)
    /* Groups Cluster - Client */
    tsCLD_Groups sGroupsClientCluster;
    tsCLD_GroupsCustomDataStructure sGroupsClientCustomDataStructure;
#endif

#if (defined CLD_SCENES) && (defined SCENES_SERVER)
    /* Scenes Cluster - Server */
    tsCLD_Scenes sScenesServerCluster;
    tsCLD_ScenesCustomDataStructure sScenesServerCustomDataStructure;
#endif

#if (defined CLD_SCENES) && (defined SCENES_CLIENT)
    /* Scenes Cluster - Client */
    tsCLD_Scenes sScenesClientCluster;
    tsCLD_ScenesCustomDataStructure sScenesClientCustomDataStructure;
#endif

#if (defined CLD_WINDOW_COVERING) && (defined WINDOW_COVERING_SERVER)
    /* Window Covering Cluster - Server */
    tsCLD_WindowCovering sWindowCoveringServerCluster;
    tsCLD_WindowCoveringCustomDataStructure sWindowCoveringServerCustomDataStructure;
#endif

#if (defined CLD_WINDOW_COVERING) && (defined WINDOW_COVERING_CLIENT)
    /* Window Covering Cluster - Client */
    tsCLD_WindowCoveringClient sWindowCoveringClientCluster;
    tsCLD_WindowCoveringCustomDataStructure sWindowCoveringClientCustomDataStructure;
#endif

#if (defined CLD_OTA) && (defined OTA_CLIENT)
     /* OTA cluster - Client */
     tsCLD_AS_Ota sCLD_OTA;
     tsOTA_Common sCLD_OTA_CustomDataStruct;
#endif
} tsZHA_WindowCovering;


/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC teZCL_Status eZHA_RegisterWindowCoveringEndPoint(
    uint8 u8EndPointIdentifier,
    tfpZCL_ZCLCallBackFunction cbCallBack,
    tsZHA_WindowCovering *psDeviceInfo);

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

#endif /* WINDOW_COVERING_H_ */

/*****************************************************************************
 *
 * MODULE:             Window Covering Cluster
 *
 * COMPONENT:          cluster_window_covering.h
 *
 * DESCRIPTION:        Header for Window Covering Cluster
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

#ifndef CLUSTER_WINDOW_COVERING_H_
#define CLUSTER_WINDOW_COVERING_H_

#include <jendefs.h>
#include "zcl.h"
#include "zcl_options.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* Cluster ID's */
#define CLUSTER_ID_WINDOW_COVERING 0x0102

/****************************************************************************/
/*             Window Covering Cluster - Attributes                         */
/*                                                                          */
/* Add the following #define's to your zcl_options.h file to add optional   */
/* attributes to the window covering cluster.                               */
/****************************************************************************/
//#define CLD_WC_ATTR_PHYSICAL_CLOSED_LIMIT_LIFT
//#define CLD_WC_ATTR_PHYSICAL_CLOSED_LIMIT_TILT
//#define CLD_WC_ATTR_CURRENT_POSITION_LIFT
//#define CLD_WC_ATTR_CURRENT_POSITION_TILT
//#define CLD_WC_ATTR_NUMBER_OF_ACTUATIONS_LIFT
//#define CLD_WC_ATTR_NUMBER_OF_ACTUATIONS_TILT
//#define CLD_WC_ATTR_CURRENT_POSITION_LIFT_PERCENTAGE /* Mandatory for closed loop */
//#define CLD_WC_ATTR_CURRENT_POSITION_TILT_PERCENTAGE /* Mandatory for closed loop */
//#define CLD_WC_ATTR_INSTALLED_OPEN_LIMIT_LIFT
//#define CLD_WC_ATTR_INSTALLED_CLOSED_LIMIT_LIFT
//#define CLD_WC_ATTR_INSTALLED_OPEN_LIMIT_TILT
//#define CLD_WC_ATTR_INSTALLED_CLOSED_LIMIT_TILT
//#define CLD_WC_ATTR_VELOCITY_LIFT
//#define CLD_WC_ATTR_ACCELERATION_TIME_LIFT
//#define CLD_WC_ATTR_DECELERATION_TIME_LIFT
//#define CLD_WC_ATTR_INTERMEDIATE_SETPOINTS_LIFT
//#define CLD_WC_ATTR_INTERMEDIATE_SETPOINTS_TILT

/****************************************************************************/
/*             Window Covering Cluster - Commands                           */
/*                                                                          */
/* Add the following #define's to your zcl_options.h file to add optional   */
/* commands to the window covering cluster.                                 */
/****************************************************************************/
#define CLD_WC_CMD_GO_TO_LIFT_VALUE
#define CLD_WC_CMD_GO_TO_LIFT_PERCENTAGE
#define CLD_WC_CMD_GO_TO_TILT_VALUE
#define CLD_WC_CMD_GO_TO_TILT_PERCENTAGE

/* Bit definitions in Config/Status attribute */
#define CLD_WC_CONFIG_BIT_OPERATIONAL             (1 << 0)
#define CLD_WC_CONFIG_BIT_ONLINE                  (1 << 1)
#define CLD_WC_CONFIG_BIT_REVERSED                (1 << 2)
#define CLD_WC_CONFIG_BIT_LIFT_CLOSED_LOOP        (1 << 3)
#define CLD_WC_CONFIG_BIT_TILT_CLOSED_LOOP        (1 << 4)
#define CLD_WC_CONFIG_BIT_LIFT_ENCODER_CONTROLLED (1 << 5)
#define CLD_WC_CONFIG_BIT_TILT_ENCODER_CONTROLLED (1 << 6)

/* Bit definitions for Mode attribute */
#define CLD_WC_MODE_BIT_REVERSED                  (1 << 0)
#define CLD_WC_MODE_BIT_CALIBRATION               (1 << 1)
#define CLD_WC_MODE_BIT_MAINTENANCE               (1 << 2)
#define CLD_WC_MODE_BIT_LEDS_ON                   (1 << 3)

/* Bit definitions for Feature Map attribute */
#define CLD_WC_FEATURE_MAP_LF                     (1 << 0)
#define CLD_WC_FEATURE_MAP_TL                     (1 << 1)
#define CLD_WC_FEATURE_MAP_CL                     (1 << 2)

#ifndef CLD_WC_FEATURE_MAP
	#define CLD_WC_FEATURE_MAP CLD_WC_FEATURE_MAP_LF
#endif

#ifndef CLD_WC_CLUSTER_REVISION
    #define CLD_WC_CLUSTER_REVISION 3
#endif

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/* Window Covering Command IDs */
typedef enum
{

    E_CLD_WC_CMD_UP_OPEN                = 0x00, /* Mandatory */
    E_CLD_WC_CMD_DOWN_CLOSE,                    /* Mandatory */
    E_CLD_WC_CMD_STOP,                          /* Mandatory */
    E_CLD_WC_CMD_GO_TO_LIFT_VALUE       = 0x04,
    E_CLD_WC_CMD_GO_TO_LIFT_PERCENTAGE,
    E_CLD_WC_CMD_GO_TO_TILT_VALUE       = 0x07,
    E_CLD_WC_CMD_GO_TO_TILT_PERCENTAGE
} teCLD_WindowCovering_Command;

/* Window Covering Attribute IDs */
typedef enum
{
    /* Window Covering Information Attribute Set */
    E_CLD_WC_ATTR_ID_WINDOW_COVERING_TYPE      = 0x0000, /* Mandatory */
    E_CLD_WC_ATTR_ID_PHYSICAL_CLOSED_LIMIT_LIFT,         /* Optional */
    E_CLD_WC_ATTR_ID_PHYSICAL_CLOSED_LIMIT_TILT,         /* Optional */
    E_CLD_WC_ATTR_ID_CURRENT_POSITION_LIFT,              /* Optional */
    E_CLD_WC_ATTR_ID_CURRENT_POSITION_TILT,              /* Optional */
    E_CLD_WC_ATTR_ID_NUMBER_OF_ACTUATIONS_LIFT,          /* Optional */
    E_CLD_WC_ATTR_ID_NUMBER_OF_ACTUATIONS_TILT,          /* Optional */
    E_CLD_WC_ATTR_ID_CONFIG_STATUS,                      /* Mandatory */
    E_CLD_WC_ATTR_ID_CURRENT_POSITION_LIFT_PERCENTAGE,   /* Mandatory for closed loop */
    E_CLD_WC_ATTR_ID_CURRENT_POSITION_TILT_PERCENTAGE,   /* Mandatory for closed loop */
    /* Window Covering Settings Attribute Set */
    E_CLD_WC_ATTR_ID_INSTALLED_OPEN_LIMIT_LIFT = 0x0010, /* Mandatory for closed loop */
    E_CLD_WC_ATTR_ID_INSTALLED_CLOSED_LIMIT_LIFT,        /* Mandatory for closed loop */
    E_CLD_WC_ATTR_ID_INSTALLED_OPEN_LIMIT_TILT,          /* Mandatory for closed loop */
    E_CLD_WC_ATTR_ID_INSTALLED_CLOSED_LIMIT_TILT,        /* Mandatory for closed loop */
    E_CLD_WC_ATTR_ID_VELOCITY_LIFT,                      /* Optional */
    E_CLD_WC_ATTR_ID_ACCELERATION_TIME_LIFT,             /* Optional */
    E_CLD_WC_ATTR_ID_DECELERATION_TIME_LIFT,             /* Optional */
    E_CLD_WC_ATTR_ID_MODE,                               /* Mandatory */
    E_CLD_WC_ATTR_ID_INTERMEDIATE_SETPOINTS_LIFT,        /* Optional */
    E_CLD_WC_ATTR_ID_INTERMEDIATE_SETPOINTS_TILT         /* Optional */
} teCLD_WindowCovering_ClusterID;

/* Window Covering Type Attribute Values */
typedef enum
{
    E_CLD_WC_TYPE_ROLLERSHADE = 0x00,
    E_CLD_WC_TYPE_ROLLERSHADE_2_MOTOR,
    E_CLD_WC_TYPE_ROLLERSHADE_EXTERIOR,
    E_CLD_WC_TYPE_ROLLERSHADE_EXTERIOR_2_MOTOR,
    E_CLD_WC_TYPE_DRAPERY,
    E_CLD_WC_TYPE_AWNING,
    E_CLD_WC_TYPE_SHUTTER,
    E_CLD_WC_TYPE_TILT_BLIND_TILT_ONLY,
    E_CLD_WC_TYPE_TILT_BLIND_LIFT_AND_TILT,
    E_CLD_WC_TYPE_PROJECTOR_SCREEN
} teCLD_WindowCovering_Type;


/* Window Covering Cluster */
typedef struct
{
    zenum8            eWindowCoveringType;
#ifdef CLD_WC_ATTR_PHYSICAL_CLOSED_LIMIT_LIFT
    zuint16           u16PhysicalClosedLimitLift;
#endif
#ifdef CLD_WC_ATTR_PHYSICAL_CLOSED_LIMIT_TILT
    zuint16           u16PhysicalClosedLimitTilt;
#endif
#ifdef CLD_WC_ATTR_CURRENT_POSITION_LIFT
    zuint16           u16CurrentPositionLift;
#endif
#ifdef CLD_WC_ATTR_CURRENT_POSITION_TILT
    zuint16           u16CurrentPositionTilt;
#endif
#ifdef CLD_WC_ATTR_NUMBER_OF_ACTUATIONS_LIFT
    zuint16           u16NumberOfActuationsLift;
#endif
#ifdef CLD_WC_ATTR_NUMBER_OF_ACTUATIONS_TILT
    zuint16           u16NumberOfActuationsTilt;
#endif
    zbmap8            u8ConfigStatus;
#ifdef CLD_WC_ATTR_CURRENT_POSITION_LIFT_PERCENTAGE
    zuint8            u8CurrentPositionLiftPercentage;
#endif
#ifdef CLD_WC_ATTR_CURRENT_POSITION_TILT_PERCENTAGE
    zuint8            u8CurrentPositionTiltPercentage;
#endif
#ifdef CLD_WC_ATTR_INSTALLED_OPEN_LIMIT_LIFT
    zuint16           u16InstalledOpenLimitLift;
#endif
#ifdef CLD_WC_ATTR_INSTALLED_CLOSED_LIMIT_LIFT
    zuint16           u16InstalledClosedLimitLift;
#endif
#ifdef CLD_WC_ATTR_INSTALLED_OPEN_LIMIT_TILT
    zuint16           u16InstalledOpenLimitTilt;
#endif
#ifdef CLD_WC_ATTR_INSTALLED_CLOSED_LIMIT_TILT
    zuint16           u16InstalledClosedLimitTilt;
#endif
#ifdef CLD_WC_ATTR_VELOCITY_LIFT
    zuint16           u16VelocityLift;
#endif
#ifdef CLD_WC_ATTR_ACCELERATION_TIME_LIFT
    zuint16           u16AccelerationTimeLift;
#endif
#ifdef CLD_WC_ATTR_DECELERATION_TIME_LIFT
    zuint16           u16DecelerationTimeLift;
#endif
    zbmap8            u8Mode;
#ifdef CLD_WC_ATTR_INTERMEDIATE_SETPOINTS_LIFT
    tsZCL_OctetString sIntermediateSetpointsLift;
#endif
#ifdef CLD_WC_ATTR_INTERMEDIATE_SETPOINTS_TILT
    tsZCL_OctetString sIntermediateSetpointsTilt;
#endif
    zbmap32           u32FeatureMap;
    zuint16           u16ClusterRevision;
} tsCLD_WindowCovering;


#ifdef WINDOW_COVERING_CLIENT
/* Window Covering Cluster */
typedef struct
{
    zbmap32           u32FeatureMap;
    zuint16           u16ClusterRevision;
} tsCLD_WindowCoveringClient;
#endif

#ifdef CLD_WC_CMD_GO_TO_LIFT_VALUE
/* Go to lift value command payload */
typedef struct
{
    zuint16 u16LiftValue;
} tsCLD_WindowCovering_GoToLiftValuePayload;
#endif

#ifdef CLD_WC_CMD_GO_TO_LIFT_PERCENTAGE
/* Go to lift percentage command payload */
typedef struct
{
    zuint8  u8LiftPercentage;
} tsCLD_WindowCovering_GoToLiftPercentagePayload;
#endif

#ifdef CLD_WC_CMD_GO_TO_TILT_VALUE
/* Go to tilt value command payload */
typedef struct
{
    zuint16 u16TiltValue;
} tsCLD_WindowCovering_GoToTiltValuePayload;
#endif

#ifdef CLD_WC_CMD_GO_TO_TILT_PERCENTAGE
/* Go to tilt percentage command payload */
typedef struct
{
    zuint8  u8TiltPercentage;
} tsCLD_WindowCovering_GoToTiltPercentagePayload;
#endif

/* Custom data structure */
typedef struct
{
    uint8                   u8Dummy;
} tsCLD_WindowCoveringCustomDataStructure;

/* Window Covering Command Callback Event Structure */
typedef struct
{
    uint8                           u8CommandId;
    union
    {
    #ifdef CLD_WC_CMD_GO_TO_LIFT_VALUE
        tsCLD_WindowCovering_GoToLiftValuePayload      *psGoToLiftValuePayload;
    #endif
    #ifdef CLD_WC_CMD_GO_TO_LIFT_PERCENTAGE
        tsCLD_WindowCovering_GoToLiftPercentagePayload *psGoToLiftPercentagePayload;
    #endif
    #ifdef CLD_WC_CMD_GO_TO_TILT_VALUE
        tsCLD_WindowCovering_GoToTiltValuePayload      *psGoToTiltValuePayload;
    #endif
    #ifdef CLD_WC_CMD_GO_TO_TILT_PERCENTAGE
        tsCLD_WindowCovering_GoToTiltPercentagePayload *psGoToTiltPercentagePayload;
    #endif
    } uMessage;
} tsCLD_WindowCoveringCallBackMessage;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC teZCL_Status eCLD_WindowCoveringCreateWindowCovering(
    tsZCL_ClusterInstance                   *psClusterInstance,
    bool_t                                   bIsServer,
    tsZCL_ClusterDefinition                 *psClusterDefinition,
    void                                    *pvEndPointSharedStructPtr,
    uint8                                   *pu8AttributeControlBits,
    tsCLD_WindowCoveringCustomDataStructure *psCustomDataStructure);

#ifdef WINDOW_COVERING_CLIENT
PUBLIC teZCL_Status eCLD_WindowCoveringCommandSend(
    uint8                         u8SourceEndPointId,
    uint8                         u8DestinationEndPointId,
    tsZCL_Address                *psDestinationAddress,
    uint8                        *pu8TransactionSequenceNumber,
    teCLD_WindowCovering_Command  eCommand);

#ifdef CLD_WC_CMD_GO_TO_LIFT_VALUE
PUBLIC teZCL_Status eCLD_WindowCoveringCommandGoToLiftValueSend(
    uint8          u8SourceEndPointId,
    uint8          u8DestinationEndPointId,
    tsZCL_Address *psDestinationAddress,
    uint8         *pu8TransactionSequenceNumber,
    tsCLD_WindowCovering_GoToLiftValuePayload *psPayload);
#endif

#ifdef CLD_WC_CMD_GO_TO_LIFT_PERCENTAGE
PUBLIC teZCL_Status eCLD_WindowCoveringCommandGoToLiftPercentageSend(
    uint8          u8SourceEndPointId,
    uint8          u8DestinationEndPointId,
    tsZCL_Address *psDestinationAddress,
    uint8         *pu8TransactionSequenceNumber,
    tsCLD_WindowCovering_GoToLiftPercentagePayload *psPayload);
#endif

#ifdef CLD_WC_CMD_GO_TO_TILT_VALUE
PUBLIC teZCL_Status eCLD_WindowCoveringCommandGoToTiltValueSend(
    uint8          u8SourceEndPointId,
    uint8          u8DestinationEndPointId,
    tsZCL_Address *psDestinationAddress,
    uint8         *pu8TransactionSequenceNumber,
    tsCLD_WindowCovering_GoToTiltValuePayload *psPayload);
#endif

#ifdef CLD_WC_CMD_GO_TO_TILT_PERCENTAGE
PUBLIC teZCL_Status eCLD_WindowCoveringCommandGoToTiltPercentageSend(
    uint8          u8SourceEndPointId,
    uint8          u8DestinationEndPointId,
    tsZCL_Address *psDestinationAddress,
    uint8         *pu8TransactionSequenceNumber,
    tsCLD_WindowCovering_GoToTiltPercentagePayload *psPayload);
#endif
#endif /* WINDOW_COVERING_CLIENT */

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/
#ifdef WINDOW_COVERING_SERVER
extern tsZCL_ClusterDefinition sCLD_WindowCovering;
extern const tsZCL_AttributeDefinition asCLD_WindowCoveringClusterAttributeDefinitions[];
extern uint8 au8WindowCoveringAttributeControlBits[];
#endif

#ifdef WINDOW_COVERING_CLIENT
extern tsZCL_ClusterDefinition sCLD_WindowCoveringClient;
extern const tsZCL_AttributeDefinition asCLD_WindowCoveringClientClusterAttributeDefinitions[];
extern uint8 au8WindowCoveringClientAttributeControlBits[];
#endif

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

#endif /* CLUSTER_WINDOW_COVERING_H_ */

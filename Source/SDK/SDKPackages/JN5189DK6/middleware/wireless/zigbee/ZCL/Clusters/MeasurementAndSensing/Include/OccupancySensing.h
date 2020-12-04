/*****************************************************************************
 *
 * MODULE:             Occupancy Sensing Cluster
 *
 * COMPONENT:          OccupancySensing.h
 *
 * DESCRIPTION:        Header for Occupancy Sensing Cluster
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
 * Copyright NXP B.V. 2012. All rights reserved
 *
 ****************************************************************************/

#ifndef OCCUPANCY_SENSING_H
#define OCCUPANCY_SENSING_H

#include <jendefs.h>
#include "zcl.h"
#include "zcl_options.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef CLD_OS_CLUSTER_REVISION
    #define CLD_OS_CLUSTER_REVISION        2
#endif

#ifndef CLD_OS_FEATURE_MAP
    #define CLD_OS_FEATURE_MAP             0
#endif
/****************************************************************************/
/*       Occupancy Sensing Cluster - Optional Attributes                    */
/*                                                                          */
/* Add the following #define's to your zcl_options.h file to add optional   */
/* attributes to the Occupancy Sensing cluster.                             */
/****************************************************************************/

/* Enable the optional PIR Occupied to Unoccupied Delay attribute */
//#define CLD_OS_ATTR_PIR_OCCUPIED_TO_UNOCCUPIED_DELAY

/* Enable the optional PIR Unoccupied to Occupied Delay attribute */
//#define CLD_OS_ATTR_PIR_UNOCCUPIED_TO_OCCUPIED_DELAY

/* Enable the optional PIR Unoccupied to Occupied threshold attribute */
//#define CLD_OS_ATTR_PIR_UNOCCUPIED_TO_OCCUPIED_THRESHOLD

/* Enable the optional Ultrasonic Occupied to Unoccupied Delay attribute */
//#define CLD_OS_ATTR_ULTRASONIC_OCCUPIED_TO_UNOCCUPIED_DELAY

/* Enable the optional Ultrasonic Unoccupied to Occupied Delay attribute */
//#define CLD_OS_ATTR_ULTRASONIC_UNOCCUPIED_TO_OCCUPIED_DELAY

/* Enable the optional Ultrasonic Unoccupied to Occupied threshold attribute */
//#define CLD_OS_ATTR_ULTRASONIC_UNOCCUPIED_TO_OCCUPIED_THRESHOLD

/* Enable the optional Attribute Reporting Status attribute */
//#define CLD_OS_ATTR_ATTRIBUTE_REPORTING_STATUS

/* End of optional attributes */


/* Cluster ID's */
#define MEASUREMENT_AND_SENSING_CLUSTER_ID_OCCUPANCY_SENSING  0x0406


/* Occupancy */
#define E_CLD_OS_OCCUPIED                       (1 << 0)

/*OccupancySensorTypeBitmap*/

#define OS_OCCUPANCY_SENSOR_TYPE_BITMAP_PIR                              0
#define OS_OCCUPANCY_SENSOR_TYPE_BITMAP_ULTRASONIC                       1
#define OS_OCCUPANCY_SENSOR_TYPE_BITMAP_PHYSICAL_CONTACT                 2

#define OS_OCCUPANCY_SENSOR_TYPE_BITMAP_PIR_MASK                             (1 << OS_OCCUPANCY_SENSOR_TYPE_BITMAP_PIR)
#define OS_OCCUPANCY_SENSOR_TYPE_BITMAP_ULTRASONIC_MASK                      (1 << OS_OCCUPANCY_SENSOR_TYPE_BITMAP_ULTRASONIC)
#define OS_OCCUPANCY_SENSOR_TYPE_BITMAP_PHYSICAL_CONTACT_MASK                (1 << OS_OCCUPANCY_SENSOR_TYPE_BITMAP_PHYSICAL_CONTACT)


/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

typedef enum 
{
    /* Occupancy Sensor Information attribute set attribute ID's (4.8.2.2.1) */
    E_CLD_OS_ATTR_ID_OCCUPANCY                                  = 0x0000,   /* Mandatory */
    E_CLD_OS_ATTR_ID_OCCUPANCY_SENSOR_TYPE,                                 /* Mandatory */
    E_CLD_OS_ATTR_ID_OCCUPANCY_SENSOR_TYPE_BITMAP,

    /* PIR configuration set attribute ID's (4.8.2.2.2) */
    E_CLD_OS_ATTR_ID_PIR_OCCUPIED_TO_UNOCCUPIED_DELAY           = 0x0010,
    E_CLD_OS_ATTR_ID_PIR_UNOCCUPIED_TO_OCCUPIED_DELAY,
    E_CLD_OS_ATTR_ID_PIR_UNOCCUPIED_TO_OCCUPIED_THRESHOLD,
    /* Ultrasonic configuration set attribute ID's (4.8.2.2.3) */
    E_CLD_OS_ATTR_ID_ULTRASONIC_OCCUPIED_TO_UNOCCUPIED_DELAY    = 0x0020,
    E_CLD_OS_ATTR_ID_ULTRASONIC_UNOCCUPIED_TO_OCCUPIED_DELAY,
    E_CLD_OS_ATTR_ID_ULTRASONIC_UNOCCUPIED_TO_OCCUPIED_THRESHOLD,
    /* Physical Contact configuration set attribute ID's (4.8.2.2.4) */
    E_CLD_OS_ATTR_ID_PHYSICAL_CONTACT_OCCUPIED_TO_UNOCCUPIED_DELAY    = 0x0030,
    E_CLD_OS_ATTR_ID_PHYSICAL_CONTACT_UNOCCUPIED_TO_OCCUPIED_DELAY,
    E_CLD_OS_ATTR_ID_PHYSICAL_CONTACT_UNOCCUPIED_TO_OCCUPIED_THRESHOLD
    
} teCLD_OS_ClusterID;


/* Occupancy Sensor Types */
typedef enum 
{
    E_CLD_OS_SENSORT_TYPE_PIR,
    E_CLD_OS_SENSORT_TYPE_ULTRASONIC,
    E_CLD_OS_SENSORT_TYPE_PIR_AND_ULTRASONIC,
} teCLS_OS_SensorTypes;


/* Occupancy Sensing Cluster */
typedef struct
{
#ifdef OCCUPANCY_SENSING_SERVER    
    zbmap8                  u8Occupancy;

    zenum8                  eOccupancySensorType;

    zbmap8                  u8OccupancySensorTypeBitmap;

#ifdef CLD_OS_ATTR_PIR_OCCUPIED_TO_UNOCCUPIED_DELAY
    zuint16                  u16PIROccupiedToUnoccupiedDelay;
#endif

#ifdef CLD_OS_ATTR_PIR_UNOCCUPIED_TO_OCCUPIED_DELAY
    zuint16                  u16PIRUnoccupiedToOccupiedDelay;
#endif

#ifdef CLD_OS_ATTR_PIR_UNOCCUPIED_TO_OCCUPIED_THRESHOLD
    zuint8                  u8PIRUnoccupiedToOccupiedThreshold;
#endif

#ifdef CLD_OS_ATTR_ULTRASONIC_OCCUPIED_TO_UNOCCUPIED_DELAY
    zuint16                  u16UltrasonicOccupiedToUnoccupiedDelay;
#endif

#ifdef CLD_OS_ATTR_ULTRASONIC_UNOCCUPIED_TO_OCCUPIED_DELAY
    zuint16                  u16UltrasonicUnoccupiedToOccupiedDelay;
#endif

#ifdef CLD_OS_ATTR_ULTRASONIC_UNOCCUPIED_TO_OCCUPIED_THRESHOLD
    zuint8                  u8UltrasonicUnoccupiedToOccupiedThreshold;
#endif

#ifdef CLD_OS_ATTR_PHYSICAL_CONTACT_OCCUPIED_TO_UNOCCUPIED_DELAY
    zuint16                  u16PhysicalContactOccupiedToUnoccupiedDelay;
#endif

#ifdef CLD_OS_ATTR_PHYSICAL_CONTACT_UNOCCUPIED_TO_OCCUPIED_DELAY
    zuint16                   u16PhysicalContactUnoccupiedToOccupiedDelay;
#endif

#ifdef CLD_OS_ATTR_PHYSICAL_CONTACT_UNOCCUPIED_TO_OCCUPIED_THRESHOLD
    zuint8                  u8PhysicalContactUnoccupiedToOccupiedThreshold;
#endif
    
#ifdef CLD_OS_ATTR_ATTRIBUTE_REPORTING_STATUS
    zenum8                  u8AttributeReportingStatus;
#endif
#endif
    zbmap32                 u32FeatureMap;

    zuint16                 u16ClusterRevision;
} tsCLD_OccupancySensing;


/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC teZCL_Status eCLD_OccupancySensingCreateOccupancySensing(
                tsZCL_ClusterInstance              *psClusterInstance,
                bool_t                              bIsServer,
                tsZCL_ClusterDefinition            *psClusterDefinition,
                void                               *pvEndPointSharedStructPtr,
                uint8              *pu8AttributeControlBits);

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/

extern tsZCL_ClusterDefinition sCLD_OccupancySensing;
extern const tsZCL_AttributeDefinition asCLD_OccupancySensingClusterAttributeDefinitions[];
extern uint8 au8OccupancySensingAttributeControlBits[];

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

#endif /* OCCUPANCY_SENSING_H */

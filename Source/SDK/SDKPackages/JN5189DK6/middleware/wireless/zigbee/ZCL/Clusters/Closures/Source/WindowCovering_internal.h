/*****************************************************************************
 *
 * MODULE:             Window Covering Cluster
 *
 * COMPONENT:          cluster_window_covering_internal.h
 *
 * DESCRIPTION:        The internal API for the Window Covering Cluster
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
#ifndef  CLUSTER_WINDOW_COVERING_INTERNAL_H_
#define  CLUSTER_WINDOW_COVERING_INTERNAL_H_

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include "jendefs.h"
#include "zcl.h"
#include "WindowCovering.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC teZCL_Status eCLD_WindowCoveringCommandHandler(
                    ZPS_tsAfEvent               *pZPSevent,
                    tsZCL_EndPointDefinition    *psEndPointDefinition,
                    tsZCL_ClusterInstance       *psClusterInstance);

#ifdef WINDOW_COVERING_SERVER
PUBLIC teZCL_Status eCLD_WindowCoveringCommandReceive(
                    ZPS_tsAfEvent               *pZPSevent,
                    uint8                       *pu8TransactionSequenceNumber);

#ifdef CLD_WC_CMD_GO_TO_LIFT_VALUE
PUBLIC teZCL_Status eCLD_WindowCoveringCommandGoToLiftValueReceive(
    ZPS_tsAfEvent                             *pZPSevent,
    uint8                                     *pu8TransactionSequenceNumber,
    tsCLD_WindowCovering_GoToLiftValuePayload *psPayload);
#endif

#ifdef CLD_WC_CMD_GO_TO_LIFT_PERCENTAGE
PUBLIC teZCL_Status eCLD_WindowCoveringCommandGoToLiftPercentageReceive(
    ZPS_tsAfEvent                                  *pZPSevent,
    uint8                                          *pu8TransactionSequenceNumber,
    tsCLD_WindowCovering_GoToLiftPercentagePayload *psPayload);
#endif

#ifdef CLD_WC_CMD_GO_TO_TILT_VALUE
PUBLIC teZCL_Status eCLD_WindowCoveringCommandGoToTiltValueReceive(
    ZPS_tsAfEvent                             *pZPSevent,
    uint8                                     *pu8TransactionSequenceNumber,
    tsCLD_WindowCovering_GoToTiltValuePayload *psPayload);
#endif

#ifdef CLD_WC_CMD_GO_TO_TILT_PERCENTAGE
PUBLIC teZCL_Status eCLD_WindowCoveringCommandGoToTiltPercentageReceive(
    ZPS_tsAfEvent                                  *pZPSevent,
    uint8                                          *pu8TransactionSequenceNumber,
    tsCLD_WindowCovering_GoToTiltPercentagePayload *psPayload);
#endif
#endif /* WINDOW_COVERING_SERVER */

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* CLUSTER_WINDOW_COVERING_INTERNAL_H_ */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

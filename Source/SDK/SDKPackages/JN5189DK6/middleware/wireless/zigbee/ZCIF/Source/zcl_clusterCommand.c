/*****************************************************************************
 *
 * MODULE:             Smart Energy
 *
 * COMPONENT:          zcl_attribute.c
 *
 * DESCRIPTION:       Attribute read and write functions
 *
 ****************************************************************************
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
 * Copyright NXP B.V. 2016. All rights reserved
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <string.h>

#include "zcl.h"
#include "zcl_customcommand.h"
#include "zcl_internal.h"

#include "pdum_apl.h"
#include "zps_apl.h"
#include "zps_apl_af.h"


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
/***        Public Functions                                              ***/
/****************************************************************************/

/****************************************************************************
 **
 ** NAME:       bZCL_CheckManufacturerSpecificCommandFlagMatch
 **
 ** DESCRIPTION:
 **
 **
 ** PARAMETERS:                 Name                    Usage
 ** tsZCL_CommandDefinition  *psCommandDefinition   Attribute data type struct
 ** bool_t                      bManufacturerSpecificAttributeFlag
 **
 ** RETURN:
 ** teZCL_Status
 **
 ****************************************************************************/

PUBLIC bool_t bZCL_CheckManufacturerSpecificCommandFlagMatch(
                    tsZCL_CommandDefinition  *psCommandDefinition,
                    bool_t                    bManufacturerSpecificCommandFlag)
{

    bool_t bLocalManufacturerSpecificCommandFlag = FALSE;

    if(psCommandDefinition==NULL)
    {
        return(FALSE);
    }
    // get attribute flag and set bool flag
    if(psCommandDefinition->u8CommandFlags & E_ZCL_CF_MS)
    {
        bLocalManufacturerSpecificCommandFlag = TRUE;
    }
    // compare
    if(bLocalManufacturerSpecificCommandFlag == bManufacturerSpecificCommandFlag)
    {
        return(TRUE);
    }
    return(FALSE);
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/




/*****************************************************************************
 *
 * MODULE:
 *
 * COMPONENT:
 *
 * DESCRIPTION:
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
 * Copyright NXP B.V. 2016, 2022. All rights reserved
 *
 ****************************************************************************/

/* Standard includes. */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

//#define DBG_PDM_PORT

#include "PDM.h"
#include "FunctionLib.h"
#include "Flash_Adapter.h"
#include "SecLib.h"

#include "fsl_os_abstraction.h"
#if defined FSL_RTOS_FREE_RTOS && (FSL_RTOS_FREE_RTOS != 0)
#include "FreeRTOSConfig.h"
#include "projdefs.h"
#include "portable.h"
#else
#include "MemManager.h"
#endif

#include "fsl_debug_console.h"

#ifdef DBG_PDM_PORT
#include "dbg_logging.h"
#endif

typedef struct
{
    uint8_t u8Level;
} tsMicroIntStorage;


#if gUsePdm_d

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/


/* In this module pvHeap_Alloc, vHeap_Free, vHeap_ResetHeap,
 * vMicroIntEnableOnly, vMicroIntRestoreState
 * are default weak implementations that can be overridden
 * for USE_RTOS or DUAL_MODE_APP builds */

/****************************************************************************/
/***        Type Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
#ifndef DEBUG
extern uint32_t NV_STORAGE_END_ADDRESS[];
extern uint32_t NV_STORAGE_START_ADDRESS[];
#endif
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

#if defined FSL_RTOS_FREE_RTOS && (FSL_RTOS_FREE_RTOS != 0)
static osaMutexId_t osa_mutex;
#endif

static const PDM_portConfig_t       *pdm_PortContext_p = NULL;

/****************************************************************************/
/***        Imported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

typedef enum
{
    E_HEAP_ALLOC = 0,
    E_HEAP_RESET,
    E_FUNCTION_MAX
} eFunctionId;

/* Nested interrupt control */
#ifndef MICRO_SET_PRIMASK_LEVEL
#define MICRO_SET_PRIMASK_LEVEL(A) __set_PRIMASK(A)
#endif
// read back PRIMASK status into u32Store variable then disable the interrupts
#ifndef MICRO_DISABLE_AND_SAVE_INTERRUPTS
#define MICRO_DISABLE_AND_SAVE_INTERRUPTS(u32Store) \
    u32Store = __get_PRIMASK(); __disable_irq();
#endif
#ifndef MICRO_GET_ACTIVE_INT_LEVEL
#define MICRO_GET_ACTIVE_INT_LEVEL  __get_BASEPRI
#endif
#ifndef MICRO_SET_ACTIVE_INT_LEVEL_MAX
#define MICRO_SET_ACTIVE_INT_LEVEL_MAX(A) __set_BASEPRI_MAX(A)
#endif
#ifndef MICRO_SET_ACTIVE_INT_LEVEL
#define MICRO_SET_ACTIVE_INT_LEVEL(A) __set_BASEPRI(A)
#endif

static const PDM_EncryptDecryptCBs_t PDM_EncryptDecryptCBs = {
    .PDM_EncryptFunc  = &PDM_EncryptionCallback,
};

/****************************************************************************
 *
 * NAME:        pvHeap_Alloc
 *
 * DESCRIPTION:
 * Allocates a block of memory from the heap.
 *
 * RETURNS:
 * Pointer to block, or NULL if the heap didn't have enough space. If block
 * was already assigned, returns original value and doesn't take anything
 * from heap.
 *
 * NOTES:
 * If buffer has already been allocated, it is not cleared.
 * If it is a fresh allocation, it is cleared on request.
 *
 ****************************************************************************/
__PDM_WEAK_FUNC void *pvHeap_Alloc(void *pvPointer, uint32_t u32BytesNeeded, bool_t bClear)
{
    do {
        if (pvPointer != NULL) break;
#if defined FSL_RTOS_FREE_RTOS && (FSL_RTOS_FREE_RTOS != 0)
        pvPointer = pvPortMalloc(u32BytesNeeded);
#else
#if !defined gMemManagerLight || (gMemManagerLight == 0)
        pvPointer = MEM_BufferAllocWithId(u32BytesNeeded, gPdmMemPoolId_c, (void*)__get_LR());
#else
        pvPointer = MEM_BufferAlloc(u32BytesNeeded);
#endif
#endif
        if (pvPointer == NULL) break;
        if (bClear)
        {
            FLib_MemSet(pvPointer, 0, u32BytesNeeded);
        }
    } while (0);
    return pvPointer;
}

/****************************************************************************
 *
 * NAME:        vHeap_Free
 *
 * DESCRIPTION:
 * Release a block of memory from the heap.
 *
 * RETURNS:
 * none
 * NOTES:
 *
 ****************************************************************************/
__PDM_WEAK_FUNC void vHeap_Free(void *pvPointer)
{
#if defined FSL_RTOS_FREE_RTOS && (FSL_RTOS_FREE_RTOS != 0)
    vPortFree(pvPointer);
#else
    memStatus_t status;
    status = MEM_BufferFree(pvPointer);
    if (status != MEM_SUCCESS_c)
    {
#ifdef DEBUG
        PRINTF("MemMngt error freeing %08lx code=%d\r\n", (uint32_t)pvPointer, status);
#endif
    }
#endif
}

__PDM_WEAK_FUNC void vHeap_ResetHeap(void)
{
}
///////////////////////////////////////////////////////////////////////////////////////////////////////

__PDM_WEAK_FUNC void vMicroIntEnableOnly(tsMicroIntStorage *int_storage, uint32_t u32EnableMask)
{
    uint32_t primask_lvl;
    MICRO_DISABLE_AND_SAVE_INTERRUPTS(primask_lvl);
    int_storage->u8Level = (uint8_t)MICRO_GET_ACTIVE_INT_LEVEL();
    MICRO_SET_ACTIVE_INT_LEVEL_MAX( 96 );
    MICRO_SET_PRIMASK_LEVEL(primask_lvl);
}

__PDM_WEAK_FUNC void vMicroIntRestoreState(tsMicroIntStorage * int_storage)
{
    MICRO_SET_ACTIVE_INT_LEVEL(int_storage->u8Level);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////

__PDM_WEAK_FUNC void PDM_GetCounter(uint32_t val, uint32_t pCounter_l[4])
{
    /* Very basic example code for counter creation */
    for (int i= 0; i<4; i++)
    {
        pCounter_l[i] = val+i;
    }
}


/* if output buffer is NULL,  this is a decrypt in place after write, or after a PDM read,
    if content pointer by output buffer is NULL, this is a write and on the function return, outbuf buffer shall provide the address to the buffer
        where encrypted data are located, this could be either the staging buffer or the inut buffer (if no staging buffer)
   The function does not support *output buffer not NULL,  the caller can no decide where to encrypt the data
  Function shall be called with Mutex take so staging buffer is safe
*/
void PDM_EncryptionCallback(uint8_t *input_buffer, uint8_t **output_buffer, uint32_t NoOfBytes, uint32_t val)
{
    if ((pdm_PortContext_p!=NULL))
    {
        uint8_t             *encrypted_buffer_address = NULL;
        static int           interrupt_masked = 0;

        if ( output_buffer == NULL )
        {
            /* interrupt can not be masked twice by this call back
                if mutex is correctly taken before calling this function*/
            assert( interrupt_masked < 2 );
            if ( interrupt_masked == 1 )
            {
                /* interrupt have been masked for a write, so re enable the interrut now */
                interrupt_masked = 0;

                if (pdm_PortContext_p->config_flags & PDM_CNF_ENC_TMP_BUFF)
                {
                    /* ignore this since there is no need to decrypt back */
                    return;
                }
                else
                {
                    OSA_InterruptEnable();
                }
            }
            else
            {
                /* interrupt have not been masked, this is a simple read to decrypt */
            }
        }
        else if ( *output_buffer==NULL ) // write
        {
            /* interrupts shall not have been masked if we come here because this is the write */
            assert( interrupt_masked == 0 );

            if ( (pdm_PortContext_p->pStaging_buf!=NULL) && (pdm_PortContext_p->staging_buf_size>=NoOfBytes))
            {
                /* Use staging buffer to protect the RAM buffer input_buffer from modification during the encryption and the write */
                OSA_InterruptDisable();
                FLib_MemCpy(pdm_PortContext_p->pStaging_buf, input_buffer, NoOfBytes);
                OSA_InterruptEnable();

                /* now the copy has been done, the encryption will take place for the staging buffer */
                input_buffer = pdm_PortContext_p->pStaging_buf;
            }
            else
            {
                if (!(pdm_PortContext_p->config_flags & PDM_CNF_ENC_TMP_BUFF))
                {
                    /* encryption and write will occur in place, need to keep interrupt masked until write is completed */
                    OSA_InterruptDisable();
                }
                interrupt_masked++;
            }

            /* output buffer will be the encrypted buffer */
            *output_buffer = input_buffer;
        }
        else
        {
            /* encryption or copy in a given buffer is not supported */
            assert(0);
        }

        /* decrypt in place after write, or after read ,
            input buffer has been modified to staging buffer above eventually*/
        encrypted_buffer_address = input_buffer;

        /* We will encrypt input_buffer if encryption key not NULL
            todo : support efuse key, or check with an additional field */
        if (pdm_PortContext_p->config_flags & PDM_CNF_ENC_ENABLED)
        {
            uint32_t             pCounter_l[4];
            uint8_t             *pCounter_l_8;

            /* Generate 16 bytes counter from val, counter will be incremented so keep it local in pCounter_l*/
            PDM_GetCounter(val, pCounter_l);

            pCounter_l_8 = (uint8_t*) pCounter_l;

#ifdef DBG_PDM_PORT
            PRINTF("input_buffer=%x NoOfBytes=%d val=%d count=%x\r\n", (uint32_t)input_buffer, NoOfBytes, val);
            dump_octet_string("input_buffer", input_buffer, 16);
            //dump_octet_string("pCounter_l_8", pCounter_l_8, 16);
#endif

            AES_128_CTR((const uint8_t*)input_buffer, NoOfBytes, pCounter_l_8, (const uint8_t*)pdm_PortContext_p->pEncryptionKey, encrypted_buffer_address);

#ifdef DBG_PDM_PORT
            PRINTF("encrypted_buffer_address=%x NoOfBytes=%d val=%d count=%x\r\n", (uint32_t)encrypted_buffer_address, NoOfBytes);
            dump_octet_string("encrypted_buffer_address", encrypted_buffer_address, 16);

            if (output_buffer != NULL)
            {
                PRINTF("output_buffer not NULL, *output_buffer=%x\r\n", *output_buffer);
            }
            else
            {
                PRINTF("output_buffer is NULL\r\n");
            }
#endif
        }

    }
    else
    {
        /* no encryption request for input_buffer*/
        if (output_buffer!=NULL)
        {
            /* No change on buffer if encryption disabled */
            *output_buffer = input_buffer;
        }
    }
}
#endif

/****************************************************************************
 *
 * NAME:        PDM_Init
 *
 * DESCRIPTION:
 * Wrapper to call PDM Initialization with the right Flash configuration parameters
 *
 * Note: FLASH_Init must have been called beforehand in order to initialize gFlashConfig.
 *
 * RETURNS:
 * 0: if no error, negative value otherwise
 * NOTES:
 *
 ****************************************************************************/
int PDM_Init(void)
{
    int status = -1;

#if gUsePdm_d
    static bool pdm_init_done = false;

#ifdef DEBUG
    flash_config_t *pFlashConfig = &gFlashConfig;
#ifdef PDM_EXT_FLASH
    flash_config_t gExtFlashConfif = {0x0, 0x00100000, 4096};
    pFlashConfig = &gExtFlashConfif;
#endif
#endif

    do {
        if (pdm_init_done)
        {
            status = 0;
            break;
        }

        PDM_teStatus st = PDM_E_STATUS_INTERNAL_ERROR;
        uint8_t *base = (uint8_t*)NV_STORAGE_END_ADDRESS;
        size_t len = (size_t)((uint32_t)NV_STORAGE_START_ADDRESS + 1 - (uint32_t)NV_STORAGE_END_ADDRESS);
        PDM_config_t  pConfig;

#ifdef DEBUG
        NV_Init(); /* used to setup the gFlashConfig.PFlashTotalSize value */
        uint8_t * flash_top = (uint8_t*)(pFlashConfig->PFlashTotalSize + pFlashConfig->PFlashBlockBase);
        uint8_t * flash_base = (uint8_t*)pFlashConfig->PFlashBlockBase;
        assert(base >= flash_base);
        assert(base + len <= flash_top);
        assert(len > FLASH_PAGE_SIZE*2);
#endif

#if !defined gPdmNbSegments
#error "gPdmNbSegments must be defined in app_preinclude.h"
#endif

        /* Need to volatile read in NV_STORAGE_MAX_SECTORS for the compiler to generate the code. otherwise
         * it is skipped
         */
        volatile uint32_t nv_storage_sz = (uint32_t)NV_STORAGE_MAX_SECTORS;
        if (gPdmNbSegments != nv_storage_sz)
        {
            /* The number of PDM segments must be sufficient for NV_STORAGE_MAX_SECTORS PDM sectors
             * gPdmNbSegments is necessary for Pdm pool dimensioning but NV_STORAGE_MAX_SECTORS is only determined at link time
             */
#if defined DEBUG
            PRINTF("PDM/NVM NV storage sectors mismatch nv_storage_sz=%d\r\n", nv_storage_sz);
#endif
            break;
        }

        len /= gPdmNbSegments; /* calculate segment size */
        st = PDM_eInitialise((uint32_t)base / len, gPdmNbSegments, NULL);

        if (st != PDM_E_STATUS_OK)
        {
#if defined DEBUG
            PRINTF("PDM/NVM misconfiguration\r\n");
#endif
            assert(st != PDM_E_STATUS_OK);
            status = -st;
            break;
        }

#if defined FSL_RTOS_FREE_RTOS && (FSL_RTOS_FREE_RTOS != 0)
        /* create Mutex for PDM to cope with concurrent access */
        osa_mutex = OSA_MutexCreate();
        assert(osa_mutex != 0);
        pConfig.mutex =  osa_mutex;
#else
        /* Set to 1 so the callbacks are called,  callbacks will not be called with NULL */
        pConfig.mutex =  (void*)1;
#endif
        pConfig.PDM_EncryptionDecryptionCallbacks = &PDM_EncryptDecryptCBs;

        st = PDM_eSetConfig(&pConfig);
        if (st != PDM_E_STATUS_OK)
        {
#if defined DEBUG
            PRINTF("PDM/NVM misconfiguration\r\n");
#endif
            assert(st != PDM_E_STATUS_OK);
            status = -st;
            break;
        }

        pdm_init_done = true;
        status = 0;
    } while (0);
#endif

    assert(status == 0);

    return status;
}


PDM_teStatus PDM_SetEncryption(const PDM_portConfig_t *pdm_PortContext)
{
#if gUsePdm_d
    pdm_PortContext_p = pdm_PortContext;
    return PDM_E_STATUS_OK;
#else
    (void)pdm_PortContext;
    return PDM_E_STATUS_INTERNAL_ERROR;
#endif
}

#if gUsePdm_d
void PDM_vEnterCriticalSection(void *mutex)
{
    /* Mutex needs priority awareness to be freed properly: if low-priority
         task (e.g. idle/application in bare metal) gets mutex then
         high-priority task (e.g. interrupt) tries to grab it before it has
         been freed, the high-priority task will get stuck: effectively
         priority inversion. Fix is to elevate low-priority task: in bare
         metal this means we must disable interrupts, but that makes the mutex
         itself pointless */
#if defined FSL_RTOS_FREE_RTOS && (FSL_RTOS_FREE_RTOS != 0)
    osaStatus_t status;
    status = OSA_MutexLock((osaMutexId_t)mutex, osaWaitForever_c);
    (void)status;
    assert(status == osaStatus_Success);
#else
    OSA_InterruptDisable();
#endif
}

void PDM_vExitCriticalSection(void *mutex)
{
#if defined FSL_RTOS_FREE_RTOS && (FSL_RTOS_FREE_RTOS != 0)
    osaStatus_t status;
    status = OSA_MutexUnlock((osaMutexId_t)mutex);
    (void)status;
    assert(status == osaStatus_Success);
#else
    OSA_InterruptEnable();
#endif
}
#endif

/*-----------------------------------------------------------*/

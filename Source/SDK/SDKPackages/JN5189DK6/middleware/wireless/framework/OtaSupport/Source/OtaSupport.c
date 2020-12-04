/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2019 NXP
* All rights reserved.
*
* \file
*
* This source file contains the code that enables the OTA Programming protocol
* to load an image received over the air into an external memory, using
* the format that the Bootloader will understand
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#include "EmbeddedTypes.h"
#include "OtaSupport.h"
#include "Eeprom.h"
#include "MemManager.h"
#include "FunctionLib.h"
#include "Panic.h"

#include "Flash_Adapter.h"
#include "fsl_os_abstraction.h"
#include "fsl_device_registers.h"

#if gEnableOTAServer_d || gUpgradeImageOnCurrentDevice_d
#include "FsciInterface.h"
#include "FsciCommands.h"
#endif

#include "fsl_debug_console.h"
#ifdef gBootData_None_c
#include "rom_api.h"
#endif

#ifdef CPU_JN518X
#include "flash_header.h"
#include "rom_psector.h"
#include "rom_secure.h"
#include "rom_aes.h"

#if gOtaEepromPostedOperations_d
#include "Messaging.h"
#endif
#include "psector_api.h"

#define UPPER_TEXT_LIMIT 0x94000
#define LOWER_TEXT_LIMIT 0x04000

#define INT_FLASH_PAGES_NB(x)         (((x) + ((1<<FLASH_PAGE_SIZE_LOG)-1)) >>  FLASH_PAGE_SIZE_LOG)

#endif

#define KB(x) ((x)<< 10)
#define RAISE_ERROR(x, val)    { x = (val); break; }

#ifndef BIT
#define BIT(x) (1<<(x))
#endif

#if 0
#define OTA_MaskInt         OSA_DisableIRQGlobal()
#define OTA_UnMaskInt       OSA_EnableIRQGlobal()
#else
#define OTA_MaskInt
#define OTA_UnMaskInt
#endif

/******************************************************************************
*******************************************************************************
* Private Macros
*******************************************************************************
******************************************************************************/
#ifndef CPU_JN518X
/* There are 2 flags stored in the internal Flash of the MCU that tells
  1. whether there is a bootable image present in the external EEPROM
  2. whether the load of a bootable image from external EEPROM to internal
Flash has been completed. This second flag is useful in case the MCU is reset
while the loading of image from external EEPROM to internal Flash is in progress
  These 2 flags are always located at a fixed address in the internal FLASH */

#if defined(__CC_ARM)

  extern uint32_t Image$$BOOT_FLAGS$$Base[];
  #define gBootImageFlagsAddress_c       ((uint32_t)Image$$BOOT_FLAGS$$Base)

#else /* defined(__CC_ARM) */

  extern uint32_t __BootFlags_Start__[];
  #define gBootImageFlagsAddress_c       ((uint32_t)__BootFlags_Start__)

#endif /* defined(__CC_ARM) */

  /* BootFlagsSectorBitNo(x) */
#if defined(FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE)
#define  BootFlagsSectorBitNo(x)  ((x)/(uint32_t)FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE)
#elif defined(FSL_FEATURE_FLASH_PAGE_SIZE_BYTES)
#define  BootFlagsSectorBitNo(x)  ((x)/(uint32_t)FSL_FEATURE_FLASH_PAGE_SIZE_BYTES)
#endif
#else /* ifdef CPU_JN518X */
/* Offset value of bootBlockOffset in image */
#define BOOT_BLOCK_OFFSET_VALUE (uint32_t)(&((IMG_HEADER_T *)0)->bootBlockOffset)

/* Offset value of stated_size in boot block */
#define APP_STATED_SIZE_OFFSET  (uint32_t)(&((BOOT_BLOCK_T *)0)->stated_size)
#define TARGET_ADDRESS_OFFSET  (uint32_t)(&((BOOT_BLOCK_T *)0)->target_address)

#define SELECTED_IMG_ADDRESS     (uint32_t*)(0x9e820)

#define THUMB_ENTRY(x)                 (void*)((x) | 1)
typedef struct {
    IMAGE_CERT_T certificate;
    uint8_t signature[SIGNATURE_LEN];
} ImageCertificate_t;

typedef struct {
    uint8_t signature[SIGNATURE_LEN];
} ImageSignature_t;

#define SIGNATURE_WRD_LEN (SIGNATURE_LEN / 4)
#define CRC_FINALIZE(x) ((x) ^ ~0UL)

#define ROM_API_crc_update             THUMB_ENTRY(0x0300229c)
#define ROM_API_boot_CheckVectorSum    THUMB_ENTRY(0x03000648)



#if gExternalFlashIsCiphered_d /* CPU_JN8X feature */
#define ROM_API_efuse_AESKeyPresent      THUMB_ENTRY(0x030016ec)
#define ROM_API_aesLoadKeyFromSW         THUMB_ENTRY(0x030012a8)
#define ROM_API_aesProcess               THUMB_ENTRY(0x030013d0)
#define ROM_API_aesMode                  THUMB_ENTRY(0x03001210)
#define ROM_API_efuse_LoadUniqueKey      THUMB_ENTRY(0x030016f4)
#define ROM_API_aesLoadKeyFromOTP        THUMB_ENTRY(0x0300146c)

typedef bool (*efuse_AESKeyPresent_t)(void);
typedef int (*efuse_LoadUniqueKey_t)(void);
typedef uint32_t (*aesLoadKeyFromOTP_t)(AES_KEY_SIZE_T keySize);

const static efuse_LoadUniqueKey_t efuse_LoadUniqueKey   = (efuse_LoadUniqueKey_t)ROM_API_efuse_LoadUniqueKey;
const static aesLoadKeyFromOTP_t aesLoadKeyFromOTP       = (aesLoadKeyFromOTP_t)ROM_API_aesLoadKeyFromOTP;
const static efuse_AESKeyPresent_t efuse_AESKeyPresent   = (efuse_AESKeyPresent_t)ROM_API_efuse_AESKeyPresent;


#if gUsePasswordCiphering_d
const static  uint32_t (*aesLoadKeyFromSW)(AES_KEY_SIZE_T keySize,
                                           uint32_t *key)                         = ROM_API_aesLoadKeyFromSW;
const static  uint32_t (*aesMode)(AES_MODE_T modeVal,
                                  uint32_t flags)                                 = ROM_API_aesMode;
const static  uint32_t (*aesProcess)(uint32_t *pBlockIn,
                                     uint32_t *pBlockOut,
                                     uint32_t numBlocks)                          = ROM_API_aesProcess;
#endif  /* gUsePasswordCiphering_d */

#endif
#endif

#define gOtaVerifyWriteBufferSize_d (16) /* [bytes] */


/******************************************************************************
*******************************************************************************
* Private type definitions
*******************************************************************************
******************************************************************************/
#if gOtaEepromPostedOperations_d
typedef enum {
    FLASH_OP_WRITE,
    FLASH_OP_ERASE_THEN_WRITE,
    FLASH_OP_READ,
    FLASH_OP_ERASE_SECTOR,
    FLASH_OP_ERASE_BLOCK,
    FLASH_OP_ERASE_AREA,
    FLASH_OP_ERASE_NEXT_BLOCK,
    FLASH_OP_ERASE_NEXT_BLOCK_COMPLETE,
} FLASH_op_type;

typedef struct {
    FLASH_op_type op_type;
    uint32_t flash_addr;
    size_t sz;
    uint8_t buf[PROGRAM_PAGE_SZ];
} FLASH_TransactionOp_t;


#endif /* gOtaEepromPostedOperations_d */

typedef enum {
    CIPHER_NONE,
    CIPHER_EFUSE_KEY,
    CIPHER_SW_KEY
} ExtFlashCipherMode_t;

typedef struct {
#if (gEepromType_d != gEepromDevice_None_c) && (!gEnableOTAServer_d || (gEnableOTAServer_d && gUpgradeImageOnCurrentDevice_d))
    bool_t  LoadOtaImageInEepromInProgress; /*! Flag storing we are already in the process of writing an image received
                                             *  OTA in the EEPROM or not */

    uint32_t  OtaImageTotalLength;          /*! Total length of the OTA image that is currently being written in EEPROM */

    uint32_t  OtaImageCurrentLength;        /*! The length of the OTA image that has being written in EEPROM so far */

    uint32_t  CurrentEepromAddress;          /*! Current write address in the EEPROM */

    bool_t  NewImageReady;                   /*! When a new image is ready the flash flags will be write in idle task */
#endif
#if gOtaEepromPostedOperations_d
    anchor_t op_queue;                       /*! Queue of flash operations */
    FLASH_TransactionOp_t *cur_transaction;  /*! Element in which accumulation of PROGRAM_PAGE_SZ bytes is performed  */
    uint32_t EepromAddressWritten;           /*! Address of actual EEPROM address, must remain less than CurrentEepromAddress */
    uint32_t  OtaImageLengthWritten;         /*! Size actually written into EEPROM must be less than OtaImageCurrentLength */
    int cnt_idle_op;
    int max_cnt_idle;
    int q_sz;
    int q_max;
#endif
    ExtFlashCipherMode_t ciphered_mode;
#if gExternalFlashIsCiphered_d
    uint8_t aes_key[16];
#endif
#if gEnableOTAServer_d
     uint8_t   OtaFsciInterface;             /*! The FSCI interface used to download an image */
     otaServer_AppCB_t *pOTA_AppCB;          /*! Contains Application Callbacks for packets received over the serial interface */
#endif

#if gEnableOTAServer_d || gUpgradeImageOnCurrentDevice_d
    uint32_t  TotalUpdateSize;               /*! The size of the image to be downloaded */
#endif

#if gUpgradeImageOnCurrentDevice_d
    uint8_t NextPushChunkSeq;
#endif
#if ( gEepromParams_bufferedWrite_c == 1)
    bool_t gOtaInvalidateHeader;
#endif
} OtaFlashTaskContext_t;

/******************************************************************************
*******************************************************************************
* Private Prototypes
*******************************************************************************
******************************************************************************/
#if gEnableOTAServer_d || gUpgradeImageOnCurrentDevice_d
static bool_t OtaSupportCallback(clientPacket_t* pData);
#endif

static uint32_t OTA_GetMaxAllowedArchSize(void);
static void OTA_ProgressDisplay(uint32_t current_length);

#if (gOtaVerifyWrite_d > 0)
static otaResult_t OTA_CheckVerifyFlash(uint8_t * pData, uint32_t flash_addr, uint16_t length);
#endif

#ifdef CPU_JN518X
#if gEepromType_d != gEepromDevice_InternalFlash_c || defined(SOTA_ENABLED)
static bool_t OTA_SetNewPsectorOTAEntry(uint32_t imgAddr, uint8_t flag);
#endif

#if gBootData_None_c && (gEepromType_d == gEepromDevice_InternalFlash_c)
static uint32_t OTA_GetInternalStorageAddress(void);
#endif

#endif
static otaResult_t OTA_WriteToFlash(uint16_t NoOfBytes, uint32_t Addr, uint8_t *outbuf);

#if gOtaEepromPostedOperations_d
#if (gOtaErasePolicy_c == gOtaEraseAtImageStart_c)
static int OTA_EraseStorageArea(uint32_t Addr, int32_t size);
#endif
int OTA_TransactionResume(void);
static void OTA_WritePendingData(void);
static int OTA_TransactionQueuePurge(void);
static void OTA_MsgQueue(FLASH_TransactionOp_t * pMsg);
static void OTA_MsgDequeue(void);
static bool OTA_IsTransactionPending(void);
#endif /* gOtaEepromPostedOperations_d */

#if gExternalFlashIsCiphered_d
static otaResult_t OTA_CipherWrite(uint16_t NoOfBytes, uint32_t Addr, uint8_t *outbuf);
static otaResult_t OTA_ReadDecipher(uint16_t NoOfBytes, uint32_t Addr, uint8_t *inbuf);
#endif

/******************************************************************************
*******************************************************************************
* Private Memory Declarations
*******************************************************************************
******************************************************************************/



static OtaFlashTaskContext_t mHandle = {
        .LoadOtaImageInEepromInProgress = FALSE,
        .OtaImageTotalLength = 0,
        .OtaImageCurrentLength = 0,
        .CurrentEepromAddress = 0,
        .NewImageReady = FALSE,
#if gEnableOTAServer_d
        .OtaFsciInterface = 0,
        .pOTA_AppCB = NULL,
#endif
#if gEnableOTAServer_d || gUpgradeImageOnCurrentDevice_d
        .TotalUpdateSize = 0,
#endif
#if gUpgradeImageOnCurrentDevice_d
        .NextPushChunkSeq = 0,
#endif
#if gOtaEepromPostedOperations_d
        .q_sz = 0,
        .q_max = 0,
        .EepromAddressWritten = 0,
        .OtaImageLengthWritten = 0,
#endif
#if ( gEepromParams_bufferedWrite_c == 1)
        .gOtaInvalidateHeader = FALSE,
#endif
};


#ifndef CPU_JN518X

#if !gEnableOTAServer_d || (gEnableOTAServer_d && gUpgradeImageOnCurrentDevice_d)
/*! Variables used by the Bootloader */
#if defined(__IAR_SYSTEMS_ICC__)
#pragma location = "BootloaderFlags"
const bootInfo_t gBootFlags =
#elif defined(__GNUC__)
const bootInfo_t gBootFlags __attribute__ ((section(".BootloaderFlags"))) =
#elif defined(__CC_ARM)
volatile const bootInfo_t gBootFlags __attribute__ ((section(".BootloaderFlags"))) =
#else
    #error "Compiler unknown!"
#endif
{
    {gBootFlagUnprogrammed_c},
    {gBootValueForTRUE_c},
    {0x00, 0x02},
    {gBootFlagUnprogrammed_c},
};
#endif

#endif /* !gEnableOTAServer_d || (gEnableOTAServer_d && gUpgradeImageOnCurrentDevice_d) */



/******************************************************************************
*******************************************************************************
* Public Memory
*******************************************************************************
******************************************************************************/
#ifndef CPU_JN518X
otaMode_t gUpgradeMode = gUpgradeImageOnCurrentDevice_c;

#if !gEnableOTAServer_d && !gUpgradeImageOnCurrentDevice_d
uint16_t gBootFlagsSectorBitNo;
#endif
#endif

/******************************************************************************
*******************************************************************************
* Public Functions
*******************************************************************************
******************************************************************************/
#if gEnableOTAServer_d || gUpgradeImageOnCurrentDevice_d
/*! *********************************************************************************
* \brief  Registers the OTA component to FSCI
*
* \param[in] fsciInterface  The Id of the FSCI interface used by the Otap Server
* \param[in] pCB            Pointer to a table of callback function.
*
* \return  error code.
*
********************************************************************************** */
otaResult_t OTA_RegisterToFsci( uint32_t fsciInterface, otaServer_AppCB_t *pCB)
{
    pfFSCI_OtaSupportCallback = OtaSupportCallback;
#if gEnableOTAServer_d
    mHandle.OtaFsciInterface = (uint8_t)fsciInterface;
    mHandle.pOTA_AppCB = pCB;
#endif
    return gOtaSuccess_c;
}
#endif



/*****************************************************************************
*  OTA_StartImage
*
* \param[in] length: the length of the image to be written in the EEPROM
*
* \return
*  - gOtaInvalidParam_c: the intended length is bigger than the FLASH capacity
*  - gOtaInvalidOperation_c: the process is already started (can be cancelled)
*  - gOtaEepromError_c: can not detect external EEPROM
*
********************************************************************************** */
otaResult_t OTA_StartImage(uint32_t length)
{
    return OTA_StartImageWithMaxSize(length, OTA_GetMaxAllowedArchSize());
}

/*****************************************************************************
*  OTA_StartImageWithMaxSize
*
*  This function is called in order to start a session of writing a OTA image.
*
*
*****************************************************************************/
otaResult_t OTA_StartImageWithMaxSize(uint32_t length, uint32_t maxAllowedArchSize)
{
    otaResult_t status = gOtaSuccess_c;
    psector_page_data_t * mPage0Hdl = NULL;
    OTA_DEBUG_TRACE("%s - %d bytes gEepromParams_StartOffset_c=%x  total=0x%x\r\n", __FUNCTION__, length , gEepromParams_StartOffset_c, gEepromParams_TotalSize_c);
    OTA_DBG_LOG("bytes=%d StartOffset=0x%x total=0x%x", length , gEepromParams_StartOffset_c, gEepromParams_TotalSize_c);
    do {
#if (gEepromType_d != gEepromDevice_None_c) && (!gEnableOTAServer_d || (gEnableOTAServer_d && gUpgradeImageOnCurrentDevice_d))
        /* Check if we already have an operation of writing an OTA image in the EEPROM
           in progress and if yes, deny the current request */
        if(mHandle.LoadOtaImageInEepromInProgress)
            RAISE_ERROR(status, gOtaInvalidOperation_c);
        /* Check if the internal FLASH and the EEPROM have enough room to store
           the image */
        if((length > gFlashParams_MaxImageLength_c) ||
            (length > (gEepromParams_TotalSize_c - gBootData_Image_Offset_c)) ||
            (length > maxAllowedArchSize))
            RAISE_ERROR(status, gOtaImageTooLarge_c);

        /* Save the total length of the OTA image */
        mHandle.OtaImageTotalLength = length;
        /* Init the length of the OTA image currently written */
        mHandle.OtaImageCurrentLength = 0;
        /* Init the current EEPROM write address */
        mHandle.CurrentEepromAddress = gBootData_Image_Offset_c;
#if gOtaEepromPostedOperations_d
        mHandle.OtaImageLengthWritten = 0;
        mHandle.EepromAddressWritten = gBootData_Image_Offset_c;
#endif
        /* Mark that we have started loading an OTA image in EEPROM */
        mHandle.LoadOtaImageInEepromInProgress = TRUE;

        OTA_ProgressDisplay(mHandle.OtaImageCurrentLength);

#ifndef CPU_JN518X
    #if !gEnableOTAServer_d && !gUpgradeImageOnCurrentDevice_d
        gBootFlagsSectorBitNo = BootFlagsSectorBitNo(gBootImageFlagsAddress_c);
    #endif
#else
        /* Erase the whole space that will be necessary for the image */
    #if (gOtaErasePolicy_c == gOtaEraseAtImageStart_c)
        OTA_EraseStorageArea(mHandle.CurrentEepromAddress, mHandle.OtaImageTotalLength);
    #endif
        mPage0Hdl = psector_GetPage0Handle();
        if (mPage0Hdl == NULL) RAISE_ERROR(status, gOtaError_c);
    #if ( gEepromParams_bufferedWrite_c == 1)
        OTA_SetInvalidateFlag(TRUE);
    #endif
#endif /* CPU_JN518X */
#endif
    } while (0);
    return status;
}



#if gOtaEepromPostedOperations_d

#define MAX_CONSECUTIVE_TRANSACTIONS 3


int OTA_TransactionResume(void)
{
    int nb_treated = 0;
    while ( OTA_IsTransactionPending()                      /* There are queued flash operations pending in queue */
            && (nb_treated < MAX_CONSECUTIVE_TRANSACTIONS)) /* ... but do not schedule too many in a same pass */
    {
        OTA_MaskInt;
        if (EEPROM_isBusy())
        {
            OTA_UnMaskInt;
            /* There were transactions pending but we consumed none */
            mHandle.cnt_idle_op ++;
            if (mHandle.cnt_idle_op > mHandle.max_cnt_idle)
            {
                mHandle.max_cnt_idle = mHandle.cnt_idle_op;
                OTA_DEBUG_TRACE("Max Idle cnt %d\r\n", mHandle.max_cnt_idle);
            }
            break;
        }
        OTA_UnMaskInt;
        nb_treated ++;
        /* Use MSG_GetHead so as to leave Msg in queue so that op_type or sz can be transformed when operation completes
         * (in particular for block erasure) */
        FLASH_TransactionOp_t * pMsg = MSG_GetHead(&mHandle.op_queue);

        switch (pMsg->op_type) {
        case FLASH_OP_WRITE:
        {
            if (pMsg->sz < PROGRAM_PAGE_SZ) /* Should only happen at last chunk */
                FLib_MemSet(&pMsg->buf[pMsg->sz], 0, PROGRAM_PAGE_SZ-pMsg->sz);
#if gExternalFlashIsCiphered_d
            if (OTA_CipherWrite(pMsg->sz, pMsg->flash_addr, &pMsg->buf[0]) == gOtaSuccess_c)
#else
            if (OTA_WriteToFlash(pMsg->sz, pMsg->flash_addr, &pMsg->buf[0]) == gOtaSuccess_c)
#endif
            {
                mHandle.OtaImageLengthWritten += pMsg->sz;
                assert(mHandle.EepromAddressWritten == pMsg->flash_addr);
                mHandle.EepromAddressWritten += pMsg->sz;
            }
            else
            {
                OTA_DBG_LOG("OTA_WriteToFlash - FAILURE");
            }
            OTA_MsgDequeue();
            MSG_Free(pMsg);
        }
        break;

        case FLASH_OP_ERASE_AREA:
        {
            int remain_sz = (int)pMsg->sz;
            uint32_t erase_addr = pMsg->flash_addr;
            OTA_MaskInt;
            EEPROM_EraseArea(&erase_addr, (int32_t *) &remain_sz, true);
            OTA_UnMaskInt;

            pMsg->flash_addr = erase_addr;
            pMsg->sz = remain_sz;
            if (remain_sz <= 0)
            {
                OTA_MsgDequeue();
                MSG_Free(pMsg);
            }
            else
            {
                /* Leave head request in queue */
            }
            break;
        }
        case FLASH_OP_ERASE_NEXT_BLOCK:
            OTA_MaskInt;
            EEPROM_EraseNextBlock(pMsg->flash_addr, pMsg->sz);
            OTA_UnMaskInt;
            pMsg->op_type = FLASH_OP_ERASE_NEXT_BLOCK_COMPLETE;
            break;
        case FLASH_OP_ERASE_NEXT_BLOCK_COMPLETE:
        {
            ota_op_completion_cb_t cb = (ota_op_completion_cb_t)*(uint32_t*)(&pMsg->buf[0]);
            uint32_t param = *(uint32_t*)(&pMsg->buf[4]);
            if (cb != NULL)
                cb(param);
            OTA_MsgDequeue();
            MSG_Free(pMsg);
            break;
        }

        case FLASH_OP_ERASE_BLOCK:
        case FLASH_OP_ERASE_SECTOR:
            OTA_MaskInt;
            if (EEPROM_EraseBlock(pMsg->flash_addr, pMsg->sz) == ee_ok)
            {
                OTA_UnMaskInt;
                OTA_MsgDequeue();
                MSG_Free(pMsg);
            }
            break;
        default:
            assert(0);
        };
    }
    /* There were transactions pending but we consumed some */
    mHandle.cnt_idle_op = 0;

    return nb_treated;
}
/*****************************************************************************
*  OTA_WritePendingData
*
*  Writes pending data buffer into EEPROM
*
*****************************************************************************/
void OTA_WritePendingData(void)
{
    FLASH_TransactionOp_t * pMsg = mHandle.cur_transaction;

    do {
        if (pMsg == NULL) break;
        if (pMsg->sz == 0) break;
        mHandle.cur_transaction = NULL;
        /* Submit transaction */
        OTA_MsgQueue(pMsg);
        OTA_DEBUG_TRACE("%s - Addr=%x NoOfBytes=%d\r\n", __FUNCTION__,  pMsg->flash_addr , pMsg->sz);

        OTA_DBG_LOG("Submitted page Addr=%x size=%d", pMsg->flash_addr, pMsg->sz);

        /* Always take head of queue : we just queued something so we know it is not empty */
        OTA_TransactionResume();
        OTA_MaskInt;
        while (EEPROM_isBusy());
        OTA_UnMaskInt;

    } while (0);
}
/*****************************************************************************
*  OTA_TransactionQueuePurge
*
*  Purge queue and abandon current posted operations
*
*****************************************************************************/
int OTA_TransactionQueuePurge(void)
{
    int nb_purged = 0;
    while ( OTA_IsTransactionPending())
    {
        FLASH_TransactionOp_t * pMsg = MSG_GetHead(&mHandle.op_queue);
        if (pMsg == NULL) break;
        OTA_DEBUG_TRACE("%s - trashing Addr=%x NoOfBytes=%d\r\n", __FUNCTION__,  pMsg->flash_addr , pMsg->sz);

        OTA_MsgDequeue();
        MSG_Free(pMsg);
        nb_purged ++;
    }
    mHandle.cur_transaction = NULL;

    return nb_purged;
}
#endif /* gOtaEepromPostedOperations_d */


/*****************************************************************************
*  OTA_PullImageChunk
*
*  Read image chunk from external memory
*****************************************************************************/
otaResult_t OTA_PullImageChunk(uint8_t* pData, uint16_t length, uint32_t *pImageOffset)
{
    otaResult_t status = gOtaSuccess_c;
    OTA_DBG_LOG("OTA_PullImageChunk called\n");

#if (gEepromType_d != gEepromDevice_None_c)

    uint32_t mAbsoluteOffset;

    /* Validate parameters */
    if((length == 0) || (pData == NULL) || (pImageOffset == NULL))
    {
        status = gOtaInvalidParam_c;
    }
    else
    {
        mAbsoluteOffset = gBootData_Image_Offset_c + *pImageOffset;
#if gOtaEepromPostedOperations_d
        if (mAbsoluteOffset > mHandle.EepromAddressWritten
            && mAbsoluteOffset <= (mHandle.EepromAddressWritten + PROGRAM_PAGE_SZ)
            && (mAbsoluteOffset + length) <= (mHandle.EepromAddressWritten + PROGRAM_PAGE_SZ+1))
        {
            /* The asked buffer is still in RAM */
            FLib_MemCpy(pData, mHandle.cur_transaction->buf+(mAbsoluteOffset-PROGRAM_PAGE_SZ), length);
        }
        else if (mAbsoluteOffset <= mHandle.EepromAddressWritten
            && (mAbsoluteOffset + length) > mHandle.OtaImageLengthWritten
            && (mAbsoluteOffset + length) <= (mHandle.EepromAddressWritten + PROGRAM_PAGE_SZ+1))
        {
            uint16_t lenInFlash = (length-mHandle.OtaImageLengthWritten);
            uint16_t lenInRam = (length - lenInFlash);
            /* The asked buffer is in Flash and in RAM */
#if gExternalFlashIsCiphered_d
            OTA_ReadDecipher(lenInFlash, mAbsoluteOffset, pData);
#else
            EEPROM_ReadData(lenInFlash, mAbsoluteOffset, pData);
#endif
            pData += lenInFlash;
            FLib_MemCpy(pData, mHandle.cur_transaction->buf, lenInRam);
        }
        else
        /* The asked buffer is in Flash */
#endif
        {
#if gExternalFlashIsCiphered_d
            OTA_ReadDecipher(length, mAbsoluteOffset, pData);
#else
            EEPROM_ReadData(length, mAbsoluteOffset, pData);
#endif
        }
#if ( gEepromParams_bufferedWrite_c == 1)
        if ( mHandle.gOtaInvalidateHeader )
        {
            uint32_t i = 0;
            uint8_t *pBuffer =    pData;
            if( gInvalidateHeaderLength > *pImageOffset )
            {
                PRINTF("read gInvalidateHeaderLength %d offset %x \n", gInvalidateHeaderLength, *pImageOffset);
                while( i < length )
                {
                    pBuffer[i] ^= 0xFB;
                    i++;
                }
            }
        }
#endif
    }
#endif

    return status;
}

/*****************************************************************************
*  OTA_PushImageChunkBlocking
*
*  Blocking call to OTA_PushImageChunk. This function will first call OTA_MakeHeadRoomForNextBlock
*  to erase a block before writting pData in flash
*****************************************************************************/
otaResult_t OTA_PushImageChunkBlocking(uint8_t* pData, uint16_t length, uint32_t* pImageLength, uint32_t *pImageOffset)
{
    otaResult_t status = gOtaError_c;

    do
    {
#if gOtaEepromPostedOperations_d
        OTA_DBG_LOG("==> Blocking Addr=%x NoOfBytes=%d pImageOffset = 0x%x\r\n", mHandle.CurrentEepromAddress , length, *pImageOffset);
        if (OTA_MakeHeadRoomForNextBlock(length, NULL, 0) != gOtaSuccess_c)
        {
            OTA_DBG_LOG("OTA_MakeHeadRoomForNextBlock Failure\n");
            break;
        }
        /* Wait for the end of the erase */
        while (EEPROM_isBusy());
        OTA_TransactionResume();
        OTA_DBG_LOG("==> Unlocking Addr=%x \r\n", mHandle.CurrentEepromAddress);
        status = OTA_PushImageChunk(pData, length, pImageLength, 0);
#else
        status = OTA_PushImageChunk(pData, length, pImageLength, pImageOffset);
#endif
    } while (0);

    return status;
}

/*! *********************************************************************************
* \brief  Places the next image chunk into the external FLASH. The CRC will not be computed.
*
* \param[in] pData          pointer to the data chunk
* \param[in] length         the length of the data chunk
* \param[in] pImageLength   if it is not null and the function call is successful,
*                           it will be filled with the current length of the image
* \param[in] pImageOffset   if it is not null contains the current offset of the image
*
* \return
*  - gOtaInvalidParam_c: pData is NULL or the resulting image would be bigger than the
*       final image length specified with OTA_StartImage()
*  - gOtaInvalidOperation_c: the process is not started
*
********************************************************************************** */
otaResult_t OTA_PushImageChunk(uint8_t* pData, uint16_t length, uint32_t* pImageLength, uint32_t *pImageOffset)
{
    otaResult_t status = gOtaSuccess_c;

    OTA_DEBUG_TRACE("%s - %d bytes, Offset=0x%x\r\n", __FUNCTION__, length, mHandle.CurrentEepromAddress);
    OTA_DBG_LOG("pData=%x length=%x ", pData, length);

    do {
#if (gEepromType_d != gEepromDevice_None_c) && (!gEnableOTAServer_d || (gEnableOTAServer_d && gUpgradeImageOnCurrentDevice_d))
        /* Cannot add a chunk without a prior call to OTA_StartImage() */
        if( !mHandle.LoadOtaImageInEepromInProgress )
            RAISE_ERROR(status, gOtaInvalidOperation_c);

        /* Validate parameters */
        if((length == 0) || (pData == NULL))
            RAISE_ERROR(status, gOtaInvalidParam_c);

        /* Check if the chunk does not extend over the boundaries of the image */
        if(mHandle.OtaImageCurrentLength + length > mHandle.OtaImageTotalLength)
            RAISE_ERROR(status, gOtaInvalidParam_c);

        OTA_ProgressDisplay(mHandle.OtaImageCurrentLength + length);
        /* Received a chunk with offset */
        if(NULL != pImageOffset)
        {
            mHandle.CurrentEepromAddress = gBootData_Image_Offset_c + *pImageOffset;
#if ( gEepromParams_bufferedWrite_c == 1)
            /* Helps when you have a reset and the length gets misaligned ,
            The current offset should realign the length when this happens*/
            if(pImageLength != NULL)
            {
                mHandle.OtaImageCurrentLength = mHandle.CurrentEepromAddress;
            }

            if ( mHandle.gOtaInvalidateHeader )
            {
                uint32_t i = 0;
                uint8_t *pBuffer =    pData;
                if( gInvalidateHeaderLength > *pImageOffset )
                {
                    while( ( ( i + *pImageOffset ) < gInvalidateHeaderLength )  &&
                           ( i < length ) )
                    {
                        pBuffer[i] ^= 0xFB;
                        i++;
                    }
               }
            }
#endif

        }

#if gOtaEepromPostedOperations_d
        FLASH_TransactionOp_t * pMsg;
        uint32_t NoOfBytes;
        uint32_t Addr;
        uint8_t *Outbuf;
        NoOfBytes = length;
        Addr = mHandle.CurrentEepromAddress;
        Outbuf = pData;

        if(mHandle.OtaImageLengthWritten > mHandle.OtaImageCurrentLength)
            RAISE_ERROR(status, gOtaInvalidParam_c);

        while (NoOfBytes > 0)
        {
            uint8_t * p; /* write pointer to buffer */
            size_t remaining_space;
            size_t nb_bytes_copy;

            if (mHandle.cur_transaction != NULL)
            {
                pMsg = mHandle.cur_transaction;
                /* Current transaction was ongoing : continue filling it */
                remaining_space = PROGRAM_PAGE_SZ - pMsg->sz;
                OTA_DBG_LOG("continue NbOfBytes=%d Addr=%x, prev_sz=%d, remaining = %d", NoOfBytes, Addr, pMsg->sz, remaining_space);
                Addr += remaining_space;
            }
            else
            {
                OTA_DBG_LOG("NbOfBytes=%d Addr=%x", NoOfBytes, Addr);
                pMsg = MEM_BufferAllocWithId(sizeof(FLASH_TransactionOp_t),
                                             gOtaMemPoolId_c,
                                             (void*)__get_LR());
                if (pMsg == NULL)
                {
                    panic(ID_PANIC(1,1), (uint32_t) OTA_PushImageChunk, 0, 0);
                    RAISE_ERROR(status, gOtaError_c);
                }
                pMsg->flash_addr = Addr;
                pMsg->op_type = FLASH_OP_WRITE;
                pMsg->sz = 0;
                remaining_space = PROGRAM_PAGE_SZ;
            }
            p = &pMsg->buf[pMsg->sz];
            nb_bytes_copy = MIN(remaining_space, NoOfBytes);
            FLib_MemCpy(p, Outbuf, nb_bytes_copy);
            Outbuf += nb_bytes_copy;
            pMsg->sz += nb_bytes_copy;
            if (pMsg->sz == PROGRAM_PAGE_SZ)
            {
                OTA_DBG_LOG("==> OTA_PushImageChunk queued chunk add=0x%x, size = %d\n", pMsg->flash_addr, pMsg->sz);
                assert((pMsg->flash_addr%PROGRAM_PAGE_SZ) == 0);
                /* Submit transaction */
                OTA_MsgQueue(pMsg);

                OTA_DBG_LOG("Submitted page Addr=%x", pMsg->flash_addr);
                mHandle.cur_transaction = NULL;
            }
            else
            {
                mHandle.cur_transaction = pMsg;
                OTA_DBG_LOG("Holding Addr=%x", pMsg->flash_addr, pMsg->sz);
            }
            NoOfBytes -= nb_bytes_copy;
        }
#if 1
        if ( OTA_IsTransactionPending() )
        {
            /* Always take head of queue */
            OTA_TransactionResume();
        }
#endif
#else /* gOtaEepromPostedOperations_d */
        /* Try to write the data chunk into the external EEPROM */
        status = OTA_WriteToFlash(length, mHandle.CurrentEepromAddress, pData);
        if (status != gOtaSuccess_c)
            break;
#endif  /* gOtaEepromPostedOperations_d */
        /* Data chunk successfully written into EEPROM
        Update operation parameters */
        mHandle.CurrentEepromAddress  += length;
        mHandle.OtaImageCurrentLength += length;

        /* Return the currently written length of the OTA image to the caller */
        if(pImageLength != NULL)
        {
            *pImageLength = mHandle.OtaImageCurrentLength;
        }
#endif  /* (gEepromType_d != gEepromDevice_None_c) && (!gEnableOTAServer_d || (gEnableOTAServer_d && gUpgradeImageOnCurrentDevice_d)) */
    } while (0);
    return status;
}


/*! *********************************************************************************
* \brief  Finishes the writing of a new image to the permanent storage.
*         It will write the image header (signature and length) and footer (sector copy bitmap).
*
* \param[in] bitmap   pointer to a  byte array indicating the sector erase pattern for the
*                     internal FLASH before the image update.
*
* \return
*  - gOtaInvalidOperation_c: the process is not started,
*  - gOtaEepromError_c: error while trying to write the EEPROM
*
********************************************************************************** */
otaResult_t OTA_CommitImage(uint8_t* pBitmap)
{
#ifdef gBootData_None_c
    NOT_USED(pBitmap);
#endif
    otaResult_t status = gOtaSuccess_c;

    OTA_DEBUG_TRACE("%s OtaImageCurrentLength=%d\r\n", __FUNCTION__, mHandle.OtaImageCurrentLength);
    OTA_DBG_LOG("OtaImageCurrentLength=%x ", mHandle.OtaImageCurrentLength);
    do {
#if (gEepromType_d != gEepromDevice_None_c) && (!gEnableOTAServer_d || (gEnableOTAServer_d && gUpgradeImageOnCurrentDevice_d))
        /* Cannot commit a image without a prior call to OTA_StartImage() */
        if( !mHandle.LoadOtaImageInEepromInProgress )
            RAISE_ERROR(status, gOtaInvalidOperation_c);
        /* If the currently written image length in EEPROM is not the same with
            the one initially set, commit operation fails */
        if(mHandle.OtaImageCurrentLength != mHandle.OtaImageTotalLength)
            RAISE_ERROR(status, gOtaInvalidOperation_c);

#if gOtaEepromPostedOperations_d
        /* Writes the pending data to flash */
        OTA_WritePendingData();
        /* After flushing the remainder the written length must match the queued length */
        if(mHandle.OtaImageLengthWritten != mHandle.OtaImageCurrentLength)
            RAISE_ERROR(status, gOtaInvalidOperation_c);
#endif  /* gOtaEepromPostedOperations_d */
#if ( gEepromParams_bufferedWrite_c == 0)
#if !gBootData_None_c
#if (gEepromType_d == gEepromDevice_InternalFlash_c)
        uint8_t start_marker[gBootData_Marker_Size_c] = {gBootData_StartMarker_Value_c};

        /* Write the Start marker at the beginning of the internal storage. */
        OTA_MaskInt;
        if(EEPROM_WriteData(gBootData_Marker_Size_c, gBootData_StartMarker_Offset_c,&start_marker) != ee_ok)
        {
            OTA_UnMaskInt;
            RAISE_ERROR(status, gOtaExternalFlashError_c);
        }
        OTA_UnMaskInt;
#endif /* gEepromDevice_InternalFlash_c */
        /* To write image length into the EEPROM */
        if(EEPROM_WriteData(sizeof(uint32_t), gBootData_ImageLength_Offset_c,(uint8_t *)&mHandle.OtaImageCurrentLength) != ee_ok)
        {
            RAISE_ERROR(status, gOtaExternalFlashError_c);
        }
        /* To write the sector bitmap into the EEPROM */
        if(EEPROM_WriteData(gBootData_SectorsBitmap_Size_c, gBootData_SectorsBitmap_Offset_c, pBitmap) != ee_ok)
        {
            RAISE_ERROR(status, gOtaExternalFlashError_c);
        }
#endif /* !gBootData_None_c */
#else
        EEPROM_FlashWriteBufferedTailEndBytes(mHandle.OtaImageCurrentLength);
        OTA_SetInvalidateFlag(FALSE);
        OTA_InvalidateImageHeader(TRUE,0);
#endif
        /* Flash flags will be written in next instance of idle task */
        mHandle.NewImageReady = TRUE;
        /* End the load of OTA in EEPROM process */
        mHandle.LoadOtaImageInEepromInProgress = FALSE;
#endif  /* (gEepromType_d != gEepromDevice_None_c) && (!gEnableOTAServer_d || (gEnableOTAServer_d && gUpgradeImageOnCurrentDevice_d)) */
    } while (0);
    return status;
}


/*! *********************************************************************************
* \brief  Set the boot flags, to trigger the Bootloader at the next CPU reset.
*
********************************************************************************** */
void OTA_SetNewImageFlag(void)
{
    OTA_DEBUG_TRACE("%s\r\n", __FUNCTION__);
    OTA_DBG_LOG("");
#if (gEepromType_d != gEepromDevice_None_c) && (!gEnableOTAServer_d || (gEnableOTAServer_d && gUpgradeImageOnCurrentDevice_d))
    /* OTA image successfully written into the non-volatile storage.
       Set the boot flag to trigger the Bootloader at the next CPU Reset. */
    bool val = TRUE;
    do {
        if(! mHandle.NewImageReady ) break;

#if gBootData_None_c && defined CPU_JN518X
  #if gEepromType_d == gEepromDevice_InternalFlash_c
        uint32_t address = OTA_GetInternalStorageAddress();

    #ifdef SOTA_ENABLED
        if (OTA_SetNewPsectorOTAEntry(address, BIT(0)))
        {
            val = FALSE;
            break;
        }
    #else /* SOTA_ENABLED */
        /* Indicate to ROM code to select second partition through Flash */
        if (psector_SetEscoreImageData(address, psector_Read_MinVersion()) == WRITE_OK)
        {
            val = FALSE;
            break;
        }
    #endif /* SOTA_ENABLED */
  #else  /* gEepromType_d == gEepromDevice_InternalFlash_c */
        uint8_t flags;
        switch (mHandle.ciphered_mode) {
        case CIPHER_NONE:
            flags = BIT(0); /* bootable bit - image in plain text */
            break;
        case CIPHER_EFUSE_KEY:
            flags = (BIT(0) | BIT(7)); /* bootable bit + AES ciphering with fused key */
            break;
        case CIPHER_SW_KEY:
            flags = (BIT(0) | BIT(6)); /* bootable bit + AES ciphering with SW key */
            break;
        default:
            flags = 0;
            break;
        }
        if (OTA_SetNewPsectorOTAEntry(FSL_FEATURE_SPIFI_START_ADDR, flags))
        {
            val = FALSE;
            break;
        }
  #endif /* gEepromType_d == gEepromDevice_InternalFlash_c */
#else  /* gBootData_None_c && defined CPU_JN518X */
        uint32_t status;
        union{
            uint32_t value;
            uint8_t aValue[FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE];
        }bootFlag;
        NV_Init();

        bootFlag.value = gBootValueForTRUE_c;

        status = NV_FlashProgramUnaligned((uint32_t)&gBootFlags.newBootImageAvailable,
                                          sizeof(bootFlag),
                                          bootFlag.aValue);

        if( (status != kStatus_FLASH_Success)
            break;
        if (!FLib_MemCmpToVal(gBootFlags.internalStorageAddr, 0xFF, sizeof(gBootFlags.internalStorageAddr)) ))
            break;
        bootFlag.value = gEepromParams_StartOffset_c + gBootData_ImageLength_Offset_c;
        status = NV_FlashProgramUnaligned((uint32_t)&gBootFlags.internalStorageAddr,
                                            sizeof(bootFlag),
                                            bootFlag.aValue);
        if( status != kStatus_FLASH_Success )
            break;
        val = FALSE;
#endif  /* gBootData_None_c && defined CPU_JN518X */
    } while (0);
    mHandle.NewImageReady = val;
#endif /* (gEepromType_d != gEepromDevice_None_c) && (!gEnableOTAServer_d || (gEnableOTAServer_d && gUpgradeImageOnCurrentDevice_d)) */
}


/*! *********************************************************************************
* \brief  Cancels the process of writing a new image to the external EEPROM.
*
********************************************************************************** */
void OTA_CancelImage(void)
{
    OTA_DEBUG_TRACE("%s\r\n", __FUNCTION__);
    OTA_DBG_LOG("");

#if gEnableOTAServer_d || gUpgradeImageOnCurrentDevice_d
    mHandle.TotalUpdateSize = 0;
#endif
#if (gEepromType_d != gEepromDevice_None_c) && (!gEnableOTAServer_d || (gEnableOTAServer_d && gUpgradeImageOnCurrentDevice_d))
    mHandle.LoadOtaImageInEepromInProgress = FALSE;
#endif

#if defined CPU_JN518X && gOtaEepromPostedOperations_d
    OTA_TransactionQueuePurge();
#endif
#if ( gEepromParams_bufferedWrite_c == 1)
    OTA_SetInvalidateFlag(FALSE);
    mCurrentWriteOffset  = gEepromParams_StartOffset_c;
#endif
}

/*! *********************************************************************************
* \brief  Compute CRC over a data chunk.
*
* \param[in] pData        pointer to the data chunk
* \param[in] length       the length of the data chunk
* \param[in] crcValueOld  current CRC value
*
* \return  computed CRC.
*
********************************************************************************** */
uint16_t OTA_CrcCompute(uint8_t *pData, uint16_t lenData, uint16_t crcValueOld)
{
    uint8_t i;

    while(lenData--)
    {
        crcValueOld ^= (uint16_t)((uint16_t)*pData++ << 8);
        for( i = 0; i < 8; ++i )
        {
            if( crcValueOld & 0x8000 )
            {
                crcValueOld = (crcValueOld << 1) ^ 0x1021U;
            }
            else
            {
                crcValueOld = crcValueOld << 1;
            }
        }
    }
    return crcValueOld;
}

otaResult_t OTA_ClientInit(void)
{
    otaResult_t res = gOtaSuccess_c;

    mHandle.LoadOtaImageInEepromInProgress = FALSE;
    mHandle.OtaImageTotalLength = 0;
    mHandle.OtaImageCurrentLength = 0;
    mHandle.CurrentEepromAddress = 0;
    mHandle.NewImageReady = FALSE;

#if gEnableOTAServer_d || gUpgradeImageOnCurrentDevice_d
    mHandle.TotalUpdateSize = 0;
#endif
#if gUpgradeImageOnCurrentDevice_d
    mHandle.NextPushChunkSeq = 0;
#endif
#if gOtaEepromPostedOperations_d
    mHandle.q_sz = 0;
    mHandle.q_max = 0;
    mHandle.EepromAddressWritten = 0;
    mHandle.OtaImageLengthWritten = 0;
    mHandle.cnt_idle_op = 0;
    mHandle.max_cnt_idle = 0;
#endif
#if ( gEepromParams_bufferedWrite_c == 1)
    mHandle.gOtaInvalidateHeader = FALSE;
#endif

#if gOtaEepromPostedOperations_d
    MSG_InitQueue(&mHandle.op_queue);
#endif
#if gExternalFlashIsCiphered_d
  mHandle.ciphered_mode =  (efuse_AESKeyPresent()) ? CIPHER_EFUSE_KEY : CIPHER_SW_KEY;
#else
  mHandle.ciphered_mode =  CIPHER_NONE;
#endif
    res = OTA_InitExternalMemory();
    return res;
}

/*! *********************************************************************************
* \brief  Initializes the image storage (external memory or internal flash)
*
* \return  error code.
*
********************************************************************************** */
otaResult_t OTA_InitExternalMemory(void)
{
    OTA_DEBUG_TRACE("%s\r\n", __FUNCTION__);
    OTA_DBG_LOG("");
    otaResult_t status = gOtaExternalFlashError_c;
#if (gEepromType_d != gEepromDevice_None_c) && (gEepromType_d != gEepromDevice_InternalFlash_c)

    if(EEPROM_Init() == ee_ok)
    {
        status = gOtaSuccess_c;
    }
#if defined CPU_QN908X
#elif defined CPU_JN518X
#else
    {
        uint32_t err;
        /* Update the offset to the internal storage if necessary */
        err = NV_ReadHWParameters(&gHardwareParameters);
        if( err || (gHardwareParameters.gInternalStorageAddr != gEepromParams_StartOffset_c) )
        {
            gHardwareParameters.gInternalStorageAddr = gEepromParams_StartOffset_c;
            NV_WriteHWParameters(&gHardwareParameters);
        }
    }
#endif
#endif
    return status;
}


/*! *********************************************************************************
* \brief  This function is called in order to erase the image storage
*         (external memory or internal flash)
*
* \return  error code.
*
********************************************************************************** */
otaResult_t OTA_EraseExternalMemory(void)
{

    OTA_DEBUG_TRACE("%s\r\n", __FUNCTION__);
    OTA_DBG_LOG("");

#if (gEepromType_d != gEepromDevice_None_c && gEepromType_d != gEepromDevice_InternalFlash_c)
    otaResult_t status = gOtaSuccess_c;
    if (EEPROM_ChipErase() != ee_ok)
    {
        status = gOtaExternalFlashError_c;
    }

    return status;
#else
    return gOtaExternalFlashError_c;
#endif
}
/*****************************************************************************
*   OTA_MakeHeadRoomForNextBlock
*
*  This function is called in order to erase enough blocks to receive next OTA window
*
*****************************************************************************/
otaResult_t OTA_MakeHeadRoomForNextBlock(uint32_t size, ota_op_completion_cb_t cb, uint32_t param)
{
    otaResult_t status = gOtaSuccess_c;

#if gOtaEepromPostedOperations_d
#if  (gEepromType_d != gEepromDevice_None_c) && (gEepromType_d != gEepromDevice_InternalFlash_c)
    OTA_DEBUG_TRACE("%s\r\n", __FUNCTION__);

    FLASH_TransactionOp_t * pMsg;

    do {
        if (size == 0)
        {
            status = gOtaInvalidParam_c;
            break;
        }
        pMsg = MEM_BufferAllocWithId(sizeof(FLASH_TransactionOp_t),
                                            gOtaMemPoolId_c,
                                           (void*)__get_LR());
        if (pMsg == NULL)
        {
            status = gOtaError_c;
            panic(ID_PANIC(1,0), (uint32_t) OTA_MakeHeadRoomForNextBlock, 0, 0);
            break;
        }

        pMsg->flash_addr = mHandle.CurrentEepromAddress;
        pMsg->sz = size;
        pMsg->op_type = FLASH_OP_ERASE_NEXT_BLOCK;
        *(uint32_t*)(&pMsg->buf[0]) = (uint32_t)cb;
        *(uint32_t*)(&pMsg->buf[4]) = param;

        OTA_MsgQueue(pMsg);
        /* Always take head of queue */
        OTA_TransactionResume();

    } while (0);
#endif
#endif /* gOtaEepromPostedOperations_d */

    return status;

}

/*****************************************************************************
*  OTA_EraseBlock
*
*  This function is called in order to erase a block of memory (flash sector)
*
*****************************************************************************/
otaResult_t OTA_EraseBlock(uint32_t address)
{
    OTA_DEBUG_TRACE("%s\r\n", __FUNCTION__);
    OTA_DBG_LOG("address=%x", address);

#if (gEepromType_d != gEepromDevice_None_c)
    otaResult_t status = gOtaSuccess_c;

    OTA_MaskInt;
    if( EEPROM_EraseBlock(address, gEepromParams_SectorSize_c)!= ee_ok )
    {
        status = gOtaExternalFlashError_c;
    }
    OTA_UnMaskInt;

    return status;
#else
    return gOtaExternalFlashError_c;
#endif
}

/*! *********************************************************************************
* \brief  Read from the image storage (external memory or internal flash)
*
* \param[in] pData    pointer to the data chunk
* \param[in] length   the length of the data chunk
* \param[in] address  image storage address
*
* \return  error code.
*
********************************************************************************** */
otaResult_t OTA_ReadExternalMemory(uint8_t* pData, uint16_t length, uint32_t address)
{

    OTA_DEBUG_TRACE("%s\r\n", __FUNCTION__);
    OTA_DBG_LOG("address=%x length=%x", address, length);

#if (gEepromType_d != gEepromDevice_None_c)
#if gExternalFlashIsCiphered_d
    return OTA_ReadDecipher(length, address, pData);
#else
    otaResult_t status = gOtaSuccess_c;
    if(ee_ok != EEPROM_ReadData(length, address, pData))
    {
        status = gOtaExternalFlashError_c;
    }
    return status;
#endif
#else
    return gOtaExternalFlashError_c;
#endif
}


/*! *********************************************************************************
* \brief  Write into the image storage (external memory or internal flash)
*
* \param[in] pData    pointer to the data chunk
* \param[in] length   the length of the data chunk
* \param[in] address  image storage address
*
* \return  error code.
*
********************************************************************************** */
otaResult_t OTA_WriteExternalMemory(uint8_t* pData, uint16_t length, uint32_t address)
{
    OTA_DEBUG_TRACE("%s\r\n", __FUNCTION__);
    OTA_DBG_LOG("address=%x length=%x", address, length);

#if (gEepromType_d != gEepromDevice_None_c)
#if gExternalFlashIsCiphered_d
    return OTA_CipherWrite(length, address, pData);
#else
    return OTA_WriteToFlash(length, address, pData);
#endif
#else
    return gOtaExternalFlashError_c;
#endif
}

#ifdef CPU_JN518X
bool psector_GetPageContents(psector_page_data_t * page, psector_partition_id_t id)
{
    bool res = false;
    psector_page_state_t page_state;
    do {
        page_state = psector_ReadData(id, 0, 0, sizeof(psector_page_t), page);
        if (page_state < PAGE_STATE_DEGRADED)
        {
            panic(0,0,0,0);
            break;
        }
        res = true;
    } while (0);

    return res;
}

#if gEepromType_d != gEepromDevice_InternalFlash_c || defined(SOTA_ENABLED)


static bool_t OTA_SetNewPsectorOTAEntry(uint32_t imgAddr, uint8_t flag)
{
    bool_t entrySet = FALSE;
    image_directory_entry_t ota_entry = {
         .img_base_addr = imgAddr,
         .img_nb_pages = INT_FLASH_PAGES_NB(mHandle.OtaImageTotalLength),
         .flags = flag,
    };
    if (psector_SetOtaEntry(&ota_entry, true) == 0)
    {
        entrySet = TRUE;
    }
    return entrySet;
}
#endif /* gEepromType_d != gEepromDevice_InternalFlash_c || defined(SOTA_ENABLED) */



#if (gEepromType_d != gEepromDevice_InternalFlash_c)
uint32_t OTA_ImageValidate(uint32_t image_addr, void * imgValidationArgs)
{

    ImageValidationArg_t *args = (ImageValidationArg_t*)imgValidationArgs;

    typedef uint32_t (*crc_update_t)(uint32_t crc, const void* data, size_t data_len);
    typedef uint32_t (*boot_CheckVectorSum_t)(const IMG_HEADER_T *image);
    const static crc_update_t crc_update = (crc_update_t)ROM_API_crc_update;
    const static boot_CheckVectorSum_t boot_CheckVectorSum = (boot_CheckVectorSum_t)ROM_API_boot_CheckVectorSum;
    uint32_t result_addr = IMAGE_INVALID_ADDR;
    const IMG_HEADER_T *imageHeader = (IMG_HEADER_T *)image_addr;
    do {
        BOOT_BLOCK_T *psBootBlock;
        TRY
        {
            uint8_t * start = (uint8_t*)image_addr;
            uint8_t * end   = start + sizeof(IMG_HEADER_T);
            if (FLASH_BlankCheck(FLASH, start, end) == FLASH_DONE)
                break;
            if ((imageHeader->imageSignature < IMAGE_SIGNATURE) ||
            		(imageHeader->imageSignature >= (IMAGE_SIGNATURE+8)))
                break;
            uint32_t run_addr;

            /* Check that entry point is within tested archive */
            run_addr = (imageHeader->vectors[1] & ~0xfffUL);
            if (!args->in_ota_check)
            {
                if (run_addr != image_addr) break;
            }
            if (imageHeader->bootBlockOffset % sizeof(uint32_t) ) break;

            if (imageHeader->bootBlockOffset + sizeof(BOOT_BLOCK_T) >= UPPER_TEXT_LIMIT) break;

            psBootBlock = (BOOT_BLOCK_T *)(imageHeader->bootBlockOffset + image_addr);

            if (!(  (psBootBlock->header_marker >= BOOT_BLOCK_HDR_MARKER) &&
                    (psBootBlock->header_marker <= BOOT_BLOCK_HDR_MARKER+2))) break;
            if (psBootBlock->target_addr != run_addr) break;
            if (run_addr < args->min_valid_addr) break;
            if (psBootBlock->stated_size < (imageHeader->bootBlockOffset + sizeof(BOOT_BLOCK_T))) break;

            ImageSignature_t * img_signature = NULL;

            if (psBootBlock->certificate_offset != 0)
            {
                ImageCertificate_t * img_cert;
                /* If there is a certificate is must comply with the expectations */
                /* There must be a trailing ImageAuthTrailer_t appended to boot block */
                img_cert = (ImageCertificate_t *)(image_addr + psBootBlock->certificate_offset);

                if (img_cert->certificate.certificate_marker != CERTIFICATE_MARKER) break;
                /* Accesses to certificate header, certificate signature and image signature fields
                 * indirectly allow their correct presence via the Bus Fault TRY-CATCH.
                 */
                if (img_cert->certificate.public_key[0] == img_cert->certificate.public_key[SIGNATURE_WRD_LEN-1])
                {
                    break;
                }
                const uint32_t * cert_sign = (uint32_t*)&img_cert->signature[0];
                if (cert_sign[0] == cert_sign[SIGNATURE_WRD_LEN-1])
                {
                    break;
                }
                img_signature = (ImageSignature_t *)(img_cert + 1);
            }
            else
            {
                img_signature = (ImageSignature_t *)(psBootBlock + 1);
            }
            if (args->root_cert != NULL)
            {
                if (img_signature == NULL) break;
                const uint32_t * img_sign = (uint32_t*)&img_signature->signature[0];
                if (img_sign[0] == img_sign[SIGNATURE_WRD_LEN-1])
                {
                    break;
                }
            }

            /* compute CRC of the header */
            uint32_t crc = ~0UL;
            crc = crc_update(crc, imageHeader, sizeof(IMG_HEADER_T)-sizeof(imageHeader->header_crc));
            crc = CRC_FINALIZE(crc);

            if (imageHeader->header_crc != crc) break;

            if (boot_CheckVectorSum(imageHeader) != 0) break;
            if (args->root_cert != NULL)
            {
                /* If the root certificate is passed, it means the authentication is mandated */
                if (!secure_VerifyImage(image_addr, (const IMAGE_CERT_T*)args->root_cert)) break;
            }
            result_addr = run_addr;
        }
        CATCH (BUS_EXCEPTION)
        {
            break;
        }
        YRT;
    } while (0);

    return result_addr;
}


#endif

#if gExternalFlashIsCiphered_d
static otaResult_t OTA_CipherWrite(uint16_t NoOfBytes, uint32_t Addr, uint8_t *outbuf)
{
    OTA_DEBUG_TRACE("%s - Addr=%x NoOfBytes=%d\r\n", __FUNCTION__, Addr , NoOfBytes);

    otaResult_t status = gOtaError_c;
    do {
        bool check_ok = false;

        size_t nb_blocks;
        if (NoOfBytes & 0xf)
        {
            /*Then let's add padding and round up the size */
            nb_blocks =  (NoOfBytes + 0xf) >> 4;
            int padding_sz = (nb_blocks<<4) - NoOfBytes;
            if (padding_sz)
            {
                /* Fill with ISO padding */
                outbuf[NoOfBytes++] = 0x80;
                for (uint8_t i = 0; i < padding_sz-1; i++ )
                {
                    outbuf[NoOfBytes++] = 0;
                }
            }
        }
        else
        {
            /* If data to be encrypted is a multiple of 16 bytes */
            nb_blocks = (NoOfBytes>>4);
        }
#if (gOtaVerifyWrite_d >= 2)
        uint8_t temp_buf[PROGRAM_PAGE_SZ];
        FLib_MemCpy(&temp_buf[0], &outbuf[0],  NoOfBytes); /* keep plain text copy */
#endif  /*gOtaVerifyWrite_d */

        /* Perform ciphering in place */
        switch (mHandle.ciphered_mode) {
        case CIPHER_EFUSE_KEY:
            efuse_LoadUniqueKey();
            aesLoadKeyFromOTP(AES_KEY_128BITS);
            aesMode(AES_MODE_ECB_ENCRYPT, AES_INT_BSWAP | AES_OUTT_BSWAP);
            aesProcess((uint32_t*)&outbuf[0], (uint32_t*)&outbuf[0], nb_blocks);
            check_ok = true;
            break;
        case CIPHER_SW_KEY:
            aesLoadKeyFromSW(AES_KEY_128BITS, (uint32_t*)&mHandle.aes_key);
            aesMode(AES_MODE_ECB_ENCRYPT, AES_INT_BSWAP | AES_OUTT_BSWAP);
            aesProcess((uint32_t*)&outbuf[0], (uint32_t*)&outbuf[0], nb_blocks);
            check_ok = true;
            break;
        default:
            break;
        }
        if (!check_ok)
        {
            RAISE_ERROR(status, gOtaInvalidParam_c);
        }
        status = OTA_WriteToFlash(NoOfBytes, Addr, &outbuf[0]);
        if (status != gOtaSuccess_c) break;

#if (gOtaVerifyWrite_d >= 2)
        uint8_t  readback_buf[PROGRAM_PAGE_SZ];

        status = OTA_ReadDecipher(NoOfBytes, Addr, &readback_buf[0]);
        if (status != gOtaSuccess_c) break;

        if (FLib_MemCmp ((const void*)&temp_buf[0],
                        (const void*)&readback_buf[0],
                        NoOfBytes))
        {
            check_ok = true;
        }
        else
        {
            OTA_WARNING_TRACE("Deciphering error at offset %x\r\n", Addr);
            RAISE_ERROR(status, gOtaError_c);
        }
#endif  /*gOtaVerifyWrite_d */
        status = gOtaSuccess_c;

    } while (0);

    return status;

}

static otaResult_t OTA_ReadDecipher(uint16_t NoOfBytes, uint32_t Addr, uint8_t *inbuf)
{

    otaResult_t status = gOtaError_c;
    uint8_t alignedBufferStart[16];
    uint8_t alignedBufferEnd[16];
    uint16_t nbByteToRead = NoOfBytes;
    uint8_t *pBuf = inbuf;

    uint32_t lastAddrToRead = Addr+NoOfBytes-1;

    /* Encrypted reads require to have an addr aligned on 16 bytes */
    uint16_t nbByteToAlignStart = Addr % 16;
    uint16_t nbByteToAlignEnd = (16*(lastAddrToRead/16) + 15) - lastAddrToRead;
    uint16_t nbByteToMoveInAlignedBufferStart = 0;
    uint16_t nbByteToMoveInAlignedBufferEnd = 0;
    uint16_t nbByteToReadBeforeEndAlignBuffer = 0;

    Addr -= nbByteToAlignStart;

    do {

        /* Get the number of block that we will need to read */
        if ((nbByteToRead + nbByteToAlignStart + nbByteToAlignEnd)%16 != 0)
            break;
        int nb_blocks =  (nbByteToRead + nbByteToAlignStart + nbByteToAlignEnd)/16;

        if (nbByteToAlignStart)
        {
            if (EEPROM_ReadData(sizeof(alignedBufferStart),  Addr, &alignedBufferStart[0]) != ee_ok)
            {
                break;
            }
            else
            {
                Addr+= sizeof(alignedBufferStart);
            }
        }

        /* Check if we need to read more bytes */
        if (Addr < lastAddrToRead)
        {
            if (nbByteToAlignStart)
            {
                nbByteToMoveInAlignedBufferStart = sizeof(alignedBufferStart) - nbByteToAlignStart;
                pBuf += nbByteToMoveInAlignedBufferStart;
            }

            if (nbByteToAlignEnd)
            {
                nbByteToMoveInAlignedBufferEnd = sizeof(alignedBufferEnd) - nbByteToAlignEnd;
            }
            nbByteToReadBeforeEndAlignBuffer = nbByteToRead - nbByteToMoveInAlignedBufferStart - nbByteToMoveInAlignedBufferEnd;
            if (nbByteToReadBeforeEndAlignBuffer%16 != 0)
                break;
            if (EEPROM_ReadData(nbByteToReadBeforeEndAlignBuffer,  Addr, pBuf) != ee_ok)
            {
                break;
            }
            Addr += nbByteToReadBeforeEndAlignBuffer;
            if (nbByteToAlignEnd && EEPROM_ReadData(sizeof(alignedBufferEnd),  Addr, alignedBufferEnd) != ee_ok)
            {
                break;
            }
        }
        else
        {
            /* The asked buffer is too small and can fit in alignedBufferStart */
            nbByteToAlignEnd = 0;
        }

        if (mHandle.ciphered_mode == CIPHER_EFUSE_KEY)
        {
            efuse_LoadUniqueKey();
            aesLoadKeyFromOTP(AES_KEY_128BITS);
        }
        else if ( mHandle.ciphered_mode == CIPHER_SW_KEY)
        {
            aesLoadKeyFromSW(AES_KEY_128BITS, (uint32_t*)&mHandle.aes_key);
        }
        else
        {
            break;
        }

        aesMode(AES_MODE_ECB_DECRYPT, AES_INT_BSWAP | AES_OUTT_BSWAP);
        if (nbByteToAlignStart)
        {
            aesProcess((void*)alignedBufferStart, (void*)alignedBufferStart,  1);
            nb_blocks -=1;
        }
        if (nbByteToAlignEnd)
        {
            aesProcess((void*)pBuf, (void*)pBuf,  nb_blocks-1);
            aesProcess((void*)alignedBufferEnd, (void*)alignedBufferEnd,  1);
        }
        else
        {
            aesProcess((void*)pBuf, (void*)pBuf,  nb_blocks);
        }

        /* Fill missing pBuf bytes */
        pBuf-=nbByteToMoveInAlignedBufferStart;

        if (nbByteToAlignStart)
        {
            uint16_t i;
            uint16_t t=0;
            for (i=nbByteToAlignStart; i<sizeof(alignedBufferStart); i++)
            {
                pBuf[t++] = alignedBufferStart[i];
            }
        }

        if (nbByteToAlignEnd)
        {
            uint16_t i;
            for (i=0; i<nbByteToMoveInAlignedBufferEnd; i++)
            {
                *(pBuf+nbByteToMoveInAlignedBufferStart+nbByteToReadBeforeEndAlignBuffer+i) = alignedBufferEnd[i];
            }
        }

        status = gOtaSuccess_c;
    } while (0);
    return status;

}

#endif  /* gExternalFlashIsCiphered_d */

#endif /* CPU_JN518X */

/*****************************************************************************
*  OTA_GetMaxAllowedArchSize
*
*  This function is called in order to get the maximum allowed archive size.
*
*
*****************************************************************************/
static uint32_t OTA_GetMaxAllowedArchSize(void)
{
#ifdef CPU_JN518X
    uint32_t current_app_stated_size = (*(uint32_t *)((*(uint32_t *)BOOT_BLOCK_OFFSET_VALUE)+APP_STATED_SIZE_OFFSET));
    return gFlashMaxStatedSize*2 - current_app_stated_size;
#else
    return gFlashParams_MaxImageLength_c;
#endif
}

/*****************************************************************************
*  OTA_ProgressDisplay
*
*  This function displays the percentage of the OTA file that has been
*  received already at the debug console.
*
*
*****************************************************************************/
static void OTA_ProgressDisplay(uint32_t current_length)
{
#ifdef gOTADisplayProgress_d
    #define BS 0x08
    #define SP 0x20
	/* Display progress */
    uint32_t percentage =  (current_length*100 / mHandle.OtaImageTotalLength);
    static uint32_t prev_percentage = 0;
    if (current_length == 0)
    {
        PRINTF("\r\nOTA progress:  0%%");
    }
    else if (percentage > prev_percentage)
    {
        PUTCHAR(BS); PUTCHAR(BS); PUTCHAR(SP); PUTCHAR(BS); PUTCHAR(BS); PUTCHAR(SP); PUTCHAR(BS);
        if (percentage < 10)
            PRINTF(" %d%%", percentage);
        else
            PRINTF("%d%%", percentage);

        prev_percentage = percentage;
    }
    if (current_length == mHandle.OtaImageTotalLength)
        PRINTF("\r\n");
#endif
}

#ifdef CPU_JN518X
#if gBootData_None_c && (gEepromType_d == gEepromDevice_InternalFlash_c)
static uint32_t OTA_GetInternalStorageAddress(void)
{
#ifndef SOTA_ENABLED
    /* Address is equivalent to the stated size of this image. No need to
       account for hardware remapping as psector_SetEscoreImageData() does
       that itself */
    uint32_t address = *(uint32_t *)((*(uint32_t *)BOOT_BLOCK_OFFSET_VALUE)+APP_STATED_SIZE_OFFSET);

    return address;
#else
    return gEepromParams_StartOffset_c;
#endif
}
#endif
#endif

#if (gOtaVerifyWrite_d > 0)
static otaResult_t OTA_CheckVerifyFlash(uint8_t * pData, uint32_t flash_addr, uint16_t length)
{
    otaResult_t status = gOtaSuccess_c;
    uint8_t readData[gOtaVerifyWriteBufferSize_d];
    uint16_t readLen;
    uint16_t i = 0;
    while(i < length)
    {
        readLen = length - i;

        if( readLen > sizeof(readData) )
        {
            readLen = sizeof(readData);
        }
        OTA_MaskInt;
        EEPROM_ReadData(readLen, flash_addr + i, readData);
        OTA_UnMaskInt;

        if( !FLib_MemCmp(&pData[i], readData, readLen) )
        {
        	OTA_DEBUG_TRACE("%s - Flash address=%x\r\n", __FUNCTION__,  flash_addr);
            RAISE_ERROR(status, gOtaExternalFlashError_c);
        }

        i += readLen;
    }
    return status;
}
#endif /* gOtaVerifyWrite_d */



#if gOtaEepromPostedOperations_d
static bool OTA_IsTransactionPending(void)
{
    return  MSG_Pending(&mHandle.op_queue);
}
static void OTA_MsgQueue(FLASH_TransactionOp_t * pMsg)
{
    OSA_DisableIRQGlobal();
    MSG_Queue(&mHandle.op_queue, pMsg);
    mHandle.q_sz ++;
    if (mHandle.q_sz > mHandle.q_max)  { mHandle.q_max = mHandle.q_sz; }
    OSA_EnableIRQGlobal();
}

static void OTA_MsgDequeue(void)
{
    OSA_DisableIRQGlobal();
    MSG_DeQueue(&mHandle.op_queue);
    mHandle.q_sz --;
    OSA_EnableIRQGlobal();
}

#if (gOtaErasePolicy_c == gOtaEraseAtImageStart_c)
static int OTA_EraseStorageArea(uint32_t Addr, int32_t size)
{
    otaResult_t status = gOtaSuccess_c;
    OTA_DEBUG_TRACE("%s - Addr=%x size=%d\r\n", __FUNCTION__, Addr , size);

    FLASH_TransactionOp_t * pMsg;

    do {
        pMsg = MEM_BufferAllocWithId(sizeof(FLASH_TransactionOp_t),
                                     gOtaMemPoolId_c,
                                     (void*)__get_LR());
        if (pMsg == NULL)
        {
            status = gOtaError_c;
            panic(ID_PANIC(1,0), (uint32_t) OTA_EraseStorageArea, 0, 0);
            break;
        }

        pMsg->flash_addr = Addr;
        pMsg->sz = size;
        pMsg->op_type = FLASH_OP_ERASE_AREA;
        OTA_MsgQueue(pMsg);
#if 1
        /* Always take head of queue */
        OTA_TransactionResume();
#endif

    } while (0);

    return status;
}
#endif
#endif
static otaResult_t OTA_WriteToFlash(uint16_t NoOfBytes, uint32_t Addr, uint8_t *outbuf)
{
    otaResult_t status = gOtaSuccess_c;
    do {
        OTA_MaskInt;
        /* Try to write the data chunk into the external EEPROM */
        if(EEPROM_WriteData(NoOfBytes, Addr, outbuf) != ee_ok)
        {
            OTA_DBG_LOG("OTA_WriteToFlash - FAILURE");
            OTA_UnMaskInt;
            RAISE_ERROR(status, gOtaExternalFlashError_c);
        }
        OTA_UnMaskInt;

#if (gOtaVerifyWrite_d > 0)
        status = OTA_CheckVerifyFlash(outbuf, Addr, NoOfBytes);
        if (status != gOtaSuccess_c)
            RAISE_ERROR(status, gOtaExternalFlashError_c);
#endif  /* gOtaVerifyWrite_d */
    } while (0);
    return status;
}
#if gEnableOTAServer_d
/*! *********************************************************************************
* \brief  This function sends a request for a new image chunk over the serial interface
*
* \param[in] offset  image offset
* \param[in] len     requested chunk length
* \param[in] devId   Id of the device
*
********************************************************************************** */
void OTA_QueryImageReq(uint16_t devId, uint16_t manufacturer, uint16_t imgType, uint32_t fileVersion)
{
    uint8_t idx;
    clientPacket_t *pPkt;

    OTA_DEBUG_TRACE("%s\r\n", __FUNCTION__);

    /* compute payload len */
    idx = sizeof(devId) + sizeof(manufacturer) + sizeof(imgType) + sizeof(fileVersion);
    pPkt = MEM_BufferAlloc(sizeof(clientPacketHdr_t) + idx + 2);

    if( NULL == pPkt )
    {
        return;
    }

    pPkt->structured.header.opGroup = gFSCI_ReqOpcodeGroup_c;
    pPkt->structured.header.opCode = mFsciOtaSupportQueryImageReq_c;
    pPkt->structured.header.len = idx;

    /* Copy data into the payload buffer */
    idx = 0;
    FLib_MemCpy(&pPkt->structured.payload[idx], &devId, sizeof(devId));
    idx +=sizeof(devId);
    FLib_MemCpy(&pPkt->structured.payload[idx], &manufacturer, sizeof(manufacturer));
    idx +=sizeof(manufacturer);
    FLib_MemCpy(&pPkt->structured.payload[idx], &imgType, sizeof(imgType));
    idx +=sizeof(imgType);
    FLib_MemCpy(&pPkt->structured.payload[idx], &fileVersion, sizeof(fileVersion));

    FSCI_transmitFormatedPacket(pPkt, mHandle.OtaFsciInterface);
}


/*! *********************************************************************************
* \brief  This function sends over the serial interface a query for a specific image
*
* \param[in] devId          Id of the device
* \param[in] manufacturer   The manufacturer code
* \param[in] imgType        Type of the image
* \param[in] fileVersion    Minimum version of the image
*
********************************************************************************** */
void OTA_ImageChunkReq(uint32_t offset, uint16_t len, uint16_t devId)
{
    uint8_t idx;
    clientPacket_t *pPkt;

    PRINTF("%s\r\n", __FUNCTION__);

    idx = sizeof(offset) + sizeof(len) + sizeof(devId);
    pPkt = MEM_BufferAlloc(sizeof(clientPacketHdr_t) + idx + 2);

    if( NULL == pPkt )
    {
        return;
    }

    pPkt->structured.header.opGroup = gFSCI_ReqOpcodeGroup_c;
    pPkt->structured.header.opCode = mFsciOtaSupportImageChunkReq_c;
    pPkt->structured.header.len = idx;

    /* Copy data into the payload buffer */
    idx = 0;
    FLib_MemCpy(&pPkt->structured.payload[idx], &devId, sizeof(devId));
    idx +=sizeof(devId);
    FLib_MemCpy(&pPkt->structured.payload[idx], &offset, sizeof(offset));
    idx +=sizeof(offset);
    FLib_MemCpy(&pPkt->structured.payload[idx], &len, sizeof(len));

    FSCI_transmitFormatedPacket(pPkt, mHandle.OtaFsciInterface);
}


/*! *********************************************************************************
* \brief
*
* \param[in] pClientAddr    Pointer to the client address (16 bytes)
* \param[in] offset
* \param[in] devId          Id of the device
*
********************************************************************************** */
void OTA_ClientInfoCnf(uint8_t* pClientAddr, uint32_t offset, uint16_t devId)
{
    clientPacket_t *pPkt;
    uint8_t idx;

    OTA_DEBUG_TRACE("%s\r\n", __FUNCTION__);

    if (pClientAddr)
    {
      pPkt = MEM_BufferAlloc(sizeof(clientPacketHdr_t) + 16 + sizeof(uint16_t) + sizeof(uint32_t) + 2);

      if( NULL == pPkt )
      {
          return;
      }

      pPkt->structured.header.opGroup = gFSCI_CnfOpcodeGroup_c;
      pPkt->structured.header.opCode = mFsciOtaSupportGetClientInfo_c;
      pPkt->structured.header.len = 16 + sizeof(uint32_t) + sizeof(uint16_t);

      /* Copy data into the payload buffer */
      idx = 0;
      FLib_MemCpy(&pPkt->structured.payload[idx], pClientAddr, 16);
      idx += 16;
      FLib_MemCpy(&pPkt->structured.payload[idx], &offset, sizeof(uint32_t));
      idx += sizeof(uint32_t);
      FLib_MemCpy(&pPkt->structured.payload[idx], &devId, sizeof(devId));

      FSCI_transmitFormatedPacket(pPkt, mHandle.OtaFsciInterface);
    }
}
#endif


#if gEnableOTAServer_d || gUpgradeImageOnCurrentDevice_d
/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief  This function handles the FSCI messages for the OTA Server
*
* \param[in] pData    pointer to the received FSCI packet
*
* \return  TRUE if the received message should be recycled.
*
********************************************************************************** */
static bool_t OtaSupportCallback( clientPacket_t* pData )
{
    otaResult_t status = gOtaInvalidOperation_c;

    switch( pData->structured.header.opCode )
    {
    case mFsciOtaSupportStartImageReq_c:
        if( mHandle.TotalUpdateSize )
        {
            break;
        }

        OTA_DEBUG_TRACE("%s - mFsciOtaSupportStartImageReq_c\r\n", __FUNCTION__);

        FLib_MemCpy(&mHandle.TotalUpdateSize, pData->structured.payload, sizeof(mHandle.TotalUpdateSize));
        pData->structured.header.len = 3*sizeof(uint8_t);
        pData->structured.payload[0] = gOtaSuccess_c;
        pData->structured.payload[1] = gOtaVersion_c;
        /* Check if image storage is available */
        if( mHandle.TotalUpdateSize > gEepromParams_TotalSize_c )
            pData->structured.payload[2] = FALSE; /* The available storage cannot be used */
        else
            pData->structured.payload[2] = (gEepromType_d != gEepromDevice_None_c);
#if gUpgradeImageOnCurrentDevice_d
        mHandle.NextPushChunkSeq = 0;
#endif
        return TRUE;

    case mFsciOtaSupportSetModeReq_c:
        OTA_DEBUG_TRACE("%s - mFsciOtaSupportSetModeReq_c\r\n", __FUNCTION__);
        gUpgradeMode = (otaMode_t)pData->structured.payload[0];

        if( gUpgradeMode > gDoNotUseExternalMemoryForOtaUpdate_c )
        {
            status = gOtaInvalidParam_c;
        }
#if (gEepromType_d == gEepromDevice_None_c)
        else if( gUpgradeMode == gUseExternalMemoryForOtaUpdate_c )
        {
            status = gOtaInvalidOperation_c;
        }
#endif
#if gUpgradeImageOnCurrentDevice_d
        else if( gUpgradeMode == gUpgradeImageOnCurrentDevice_c )
        {
            status = OTA_StartImage(mHandle.TotalUpdateSize);
        }
#endif
#if gEnableOTAServer_d
        else if( mHandle.pOTA_AppCB && mHandle.pOTA_AppCB->otaServerSetModeCnf )
        {
            status = mHandle.pOTA_AppCB->otaServerSetModeCnf(pData->structured.payload, pData->structured.header.len);
        }
#endif
        break;

    case mFsciOtaSupportPushImageChunkReq_c:
        OTA_DEBUG_TRACE("%s - mFsciOtaSupportPushImageChunkReq_c\r\n", __FUNCTION__);
        if( !mHandle.TotalUpdateSize )
        {
            status = gOtaInvalidOperation_c;
        }
#if gUpgradeImageOnCurrentDevice_d
        else if( gUpgradeMode == gUpgradeImageOnCurrentDevice_c )
        {
            if( mHandle.NextPushChunkSeq == pData->structured.payload[0] )
            {
                status = OTA_PushImageChunk(&pData->structured.payload[1], pData->structured.header.len-1, NULL, NULL);
                if( gOtaSuccess_c == status )
                {
                    mHandle.NextPushChunkSeq++;
                }
            }
        }
#endif
#if gEnableOTAServer_d
        else if( mHandle.pOTA_AppCB && mHandle.pOTA_AppCB->otaServerPushChunkCnf )
        {
            status = mHandle.pOTA_AppCB->otaServerPushChunkCnf(pData->structured.payload, pData->structured.header.len);
        }
#endif
        break;

    case mFsciOtaSupportCommitImageReq_c:
        OTA_DEBUG_TRACE("%s - mFsciOtaSupportCommitImageReq_c\r\n", __FUNCTION__);
#if gUpgradeImageOnCurrentDevice_d
        mHandle.TotalUpdateSize = 0;

        if( gUpgradeMode == gUpgradeImageOnCurrentDevice_c )
        {
            status = OTA_CommitImage(pData->structured.payload);
            OTA_SetNewImageFlag();
        }
#endif
        break;

    case mFsciOtaSupportCancelImageReq_c:
        OTA_DEBUG_TRACE("%s - mFsciOtaSupportCancelImageReq_c\r\n", __FUNCTION__);
#if gEnableOTAServer_d
        if( mHandle.pOTA_AppCB && mHandle.pOTA_AppCB->otaServerCancelImgCnf )
        {
            status = mHandle.pOTA_AppCB->otaServerCancelImgCnf(pData->structured.payload, pData->structured.header.len);
        }
#endif
        OTA_CancelImage();
        break;

#if gEnableOTAServer_d
    case mFsciOtaSupportQueryImageRsp_c:
        OTA_DEBUG_TRACE("%s - mFsciOtaSupportQueryImageRsp_c\r\n", __FUNCTION__);
        if( mHandle.pOTA_AppCB && mHandle.pOTA_AppCB->otaServerQueryImageCnf )
        {
            status = mHandle.pOTA_AppCB->otaServerQueryImageCnf(pData->structured.payload, pData->structured.header.len);
        }
        break;

    case mFsciOtaSupportImageNotifyReq_c:
        OTA_DEBUG_TRACE("%s - mFsciOtaSupportImageNotifyReq_c\r\n", __FUNCTION__);
        if( mHandle.pOTA_AppCB && mHandle.pOTA_AppCB->otaServerImgNotifyCnf )
        {
            status = mHandle.pOTA_AppCB->otaServerImgNotifyCnf(pData->structured.payload, pData->structured.header.len);
        }
        break;
    case mFsciOtaSupportSetFileVerPoliciesReq_c:
        OTA_DEBUG_TRACE("%s - mFsciOtaSupportSetFileVerPoliciesReq_c\r\n", __FUNCTION__);
        if( mHandle.pOTA_AppCB && mHandle.pOTA_AppCB->otaServerSetFileVersPoliciesCnf )
        {
            status = mHandle.pOTA_AppCB->otaServerSetFileVersPoliciesCnf(pData->structured.payload, pData->structured.header.len);
        }
        break;
    case mFsciOtaSupportAbortOTAUpgradeReq_c:
        OTA_DEBUG_TRACE("%s - mFsciOtaSupportAbortOTAUpgradeReq_c\r\n", __FUNCTION__);
        if( mHandle.pOTA_AppCB && mHandle.pOTA_AppCB->otaServerAbortProcessCnf )
        {
            status = mHandle.pOTA_AppCB->otaServerAbortProcessCnf(pData->structured.payload, pData->structured.header.len);
        }
        break;
    case mFsciOtaSupportGetClientInfo_c:
        OTA_DEBUG_TRACE("%s - mFsciOtaSupportGetClientInfo_c\r\n", __FUNCTION__);
         if( mHandle.pOTA_AppCB && mHandle.pOTA_AppCB->otaServerClientInfoCnf)
         {
            status = mHandle.pOTA_AppCB->otaServerClientInfoCnf(pData->structured.payload, pData->structured.header.len);
         }
        break;
#endif
    }

    /* Default response */
    pData->structured.header.len = sizeof(uint8_t);
    pData->structured.payload[0] = status;
    return TRUE;
}
#endif
#if ( gEepromParams_bufferedWrite_c == 1)
/*****************************************************************************
*  OTA_InvalidateImageHeader
*
*
*
*****************************************************************************/
otaResult_t OTA_InvalidateImageHeader( bool_t bCurrentImage, uint32_t Offset )
{
    uint32_t    i = 0;
    otaResult_t status = gOtaSuccess_c;
    uint8_t StandaloneBuffer[gEepromParams_SectorSize_c];
    if (!bCurrentImage)
    {
        mCurrentWriteOffset  = gEepromParams_CurrentOffset;
    }
    else
    {
        mCurrentWriteOffset  = gEepromParams_StartOffset_c;
    }
    EEPROM_ReadData(gEepromParams_SectorSize_c, Offset, StandaloneBuffer);
    if (gInvalidateHeaderLength > gEepromParams_SectorSize_c)
    {
        status =  gOtaInvalidParam_c;
    }
    else
    {
        while( i < gInvalidateHeaderLength )
        {
            StandaloneBuffer[i] ^= 0xFB;
            i++;
        }
        EEPROM_WriteData(gEepromParams_SectorSize_c, Offset, StandaloneBuffer);
#if ( gEepromParams_bufferedWrite_c == 1)
        OTA_SetInvalidateFlag(FALSE);
#endif
    }
    return status;
}
/*****************************************************************************
*  OTA_SetInvalidateFlag
*
*
*
*****************************************************************************/
void OTA_SetInvalidateFlag ( bool_t bInvalidateFlag )
{
    mHandle.gOtaInvalidateHeader = bInvalidateFlag;
}
#endif

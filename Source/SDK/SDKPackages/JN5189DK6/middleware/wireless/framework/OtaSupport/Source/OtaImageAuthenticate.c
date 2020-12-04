/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2019 NXP
* All rights reserved.
*
* \file
*
* This file is the source file for the BLE OTAP Client ATT application
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */


#include "fsl_debug_console.h"
#include "rom_api.h"
#include "rom_secure.h"
#include "rom_psector.h"
#include "OtaSupport.h"


/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
/* Offset value of image publick key */
#define IMAGE_PK_VALID_OFFSET                ((size_t)&((psector_page_data_t*)0)->page0_v2.img_pk_valid)

#if 0
#define AUTH_LOG    PRINTF
#else
#define AUTH_LOG(...)
#endif

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

static int ReconstructRootCert(IMAGE_CERT_T *cert, psector_page_data_t* pPage0, psector_page_data_t* pFlash_page, uint8_t * key)
{
    int st = -1;
    if (!key)
    {
        int key_valid = 0;
        /* Read nxp public key from flash */
        st = psector_Read_ImagePubKey(&key_valid, (uint8_t*)&cert->public_key[0], false);
    }
    else
    {
        memcpy(&cert->public_key, key, SIGNATURE_LEN);
        st = 0;
    }

    if (!pFlash_page)
    {
        cert->customer_id = psector_Read_CustomerId();
        cert->min_device_id = psector_Read_MinDeviceId();
        cert->max_device_id = psector_Read_MaxDeviceId();
    }
    else
    {
        cert->customer_id   = pFlash_page->pFlash.customer_id;
        cert->min_device_id = pFlash_page->pFlash.min_device_id;
        cert->max_device_id = pFlash_page->pFlash.max_device_id;
    }
    cert->certificate_marker = CERTIFICATE_MARKER;
    cert->certificate_id = 0UL;
    cert->usage_flags = 0UL;

    return st;
}

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief        Image authenticate
*
* \param[in]    newImageAddress    Address of new image received by OTA
* \param[out]   Authenticate pass or not, gOtaImageAuthPass_c/gOtaImageAuthFail_c
********************************************************************************** */
otaImageAuthResult_t OTA_ImageAuthenticate(uint32_t newImageAddress)
{
    otaImageAuthResult_t imageVerifyRet = gOtaImageAuthFail_c;
    uint32_t img_pk_valid = 0;  /* 0: image pubkey invalid, 1: image pubkey valid */
    IMAGE_CERT_T cert;          /* for certificate */
    psector_page_state_t state;

    uint16_t authLevelVal = psector_Read_ImgAuthLevel();
    AUTH_LOG("auth level in psector=%d, imageAddr=%X\r\n", authLevelVal, newImageAddress);
    if(authLevelVal == AUTH_ON_FW_UPDATE || authLevelVal == AUTH_ALWAYS)
    {
        /* Read image public key valid value */
        state = psector_ReadData(PSECTOR_PAGE0_PART, 0, IMAGE_PK_VALID_OFFSET, sizeof(uint32_t), &img_pk_valid);

        if((state == PAGE_STATE_OK) && img_pk_valid)
        {
            int st = ReconstructRootCert(&cert, NULL, NULL, NULL);
            if(st == 0)    /* reconstruct root certifier success */
            {
                imageVerifyRet = (otaImageAuthResult_t)secure_VerifyImage(newImageAddress, &cert);
                AUTH_LOG("auth result=%d\r\n", imageVerifyRet);
            }
        }
    }

    return imageVerifyRet;
}


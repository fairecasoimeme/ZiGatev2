/*
* Copyright 2019 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "EmbeddedTypes.h"
#include "ModuleInfo.h"
#include <string.h>

#include "FunctionLib.h"
#include "Messaging.h"
#include "TimersManager.h"
#include "RNG_Interface.h"
#include "SecLib.h"

#include "network_utils.h"

#include <openthread/ip6.h>
#include "utils/wrap_string.h"

/*==================================================================================================
Private macros
==================================================================================================*/
#define __assert_func 
/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
/* Const Ip addresses */
const otIp6Address in6addr_any = IN6ADDR_ANY_INIT;
const otIp6Address in6addr_loopback = IN6ADDR_LOOPBACK_INIT;
const otIp6Address in6addr_nodelocal_allnodes = IN6ADDR_NODELOCAL_ALLNODES_INIT;
const otIp6Address in6addr_linklocal_allnodes = IN6ADDR_LINKLOCAL_ALLNODES_INIT;
const otIp6Address in6addr_linklocal_allrouters = IN6ADDR_LINKLOCAL_ALLROUTERS_INIT;
const otIp6Address in6addr_linklocal_allv2routers = IN6ADDR_LINKLOCAL_ALLV2ROUTERS_INIT;
const otIp6Address in6addr_sitelocal_alldhcpservers = IN6ADDR_SITELOCAL_ALLDHCPSERVERS;
const otIp6Address in6addr_realmlocal_allnodes = IN6ADDR_REALMLOCAL_ALLNODES_INIT;
const otIp6Address in6addr_realmlocal_allrouters = IN6ADDR_REALMLOCAL_ALLROUTERS_INIT;
const otIp6Address in6addr_realmlocal_allleasequeryservers = IN6ADDR_REALMLOCAL_ALL_DHCP_LEASEQUERY_SERVERS;
const otIp6Address in6addr_realmlocal_mcast_3ead = IN6ADDR_REALMLOCAL_MCAST_3EAD;
const otIp6Address in6addr_realmlocal_allmplforwarders = IN6ADDR_REALMLOCAL_ALLMPLFORWARDERS;
const otIp6Address in6addr_sitelocal_allnodes = IN6ADDR_SITELOCAL_ALLNODES_INIT;
const otIp6Address in6addr_sitelocal_allrouters = IN6ADDR_SITELOCAL_ALLROUTERS_INIT;
const otIp6Address in6addr_link_local_prefix = IN6ADDR_LINK_LOCAL_PREFIX_INIT;
const otIp6Address in6addr_linklocal_allcoapnodes = IN6ADDR_LINKLOCAL_ALL_COAP_NODES_INIT;
const otIp6Address in6addr_realmlocal_allcoapnodes = IN6ADDR_REALMLOCAL_ALL_COAP_NODES_INIT;
const otIp6Address in6addr_adminlocal_allcoapnodes = IN6ADDR_ADMINLOCAL_ALL_COAP_NODES_INIT;
const otIp6Address in6addr_sitelocal_allcoapnodes = IN6ADDR_SITELOCAL_ALL_COAP_NODES_INIT;

/* RAM global addresses - updated when the device join the network */
otIp6Address in6addr_linklocal_allthreadnodes = IN6ADDR_ANY_INIT;
otIp6Address in6addr_realmlocal_allthreadnodes = IN6ADDR_ANY_INIT;
otIp6Address in6addr_realmlocal_threadleaderanycast = IN6ADDR_ANY_INIT;

/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn    otIp6Address *NWKU_GetSpecificMcastAddr(otInstance *pOtInstance, thrMcastAllThrNodes_t type)
\brief This function is used to get a specific multicast address (Mesh Local All nodes multicast or
       link local All nodes multicast)

\param [in]    pOtInstance         Pointer to the OpenThread instance
\param [in]    type                Ip address type: gMcastLLAddrAllThrNodes_c, gMcastMLAddrAllThrNodes_c

\retval        otIp6Address        Pointer to requested multicast address
***************************************************************************************************/

otIp6Address *NWKU_GetSpecificMcastAddr
(
    otInstance *pOtInstance,
    thrMcastAllThrNodes_t type
)
{
    otIp6Address *pAddr = NULL;
    const otNetifMulticastAddress *pMCastAddr = otIp6GetMulticastAddresses(pOtInstance);

    while(pMCastAddr != NULL)
    {
        if((type == gMcastLLAddrAllThrNodes_c) && (pMCastAddr->mAddress.mFields.m8[1] == 0x32))
        {
            pAddr = (otIp6Address *)&pMCastAddr->mAddress;
            break;
        }
        else if((type == gMcastMLAddrAllThrNodes_c) && (pMCastAddr->mAddress.mFields.m8[1] == 0x33))
        {
            pAddr = (otIp6Address *)&pMCastAddr->mAddress;
            break;
        }

        pMCastAddr = pMCastAddr->mNext;
    }
    
    return pAddr;
}

/*!*************************************************************************************************
\fn    bool_t NWKU_MemCmpToVal(uint8_t *pAddr, uint8_t val, uint32_t len)
\brief Compare each octet of a given location to a value.

\param [in]     pAddr      location to be compared
\param [in]     val        reference value
\param [in]     len        length of location to be compared

\return         TRUE       If match
\return         FALSE      Otherwise
***************************************************************************************************/
bool_t NWKU_MemCmpToVal
(
    uint8_t     *pAddr,
    uint8_t     val,
    uint32_t    len
)
{
    while(len)
    {
        len--;

        if(pAddr[len] != val)
        {
            return FALSE;
        }
    }

    return TRUE;
}

/*!*************************************************************************************************
\fn     bool_t NWKU_BitCmp(uint8_t *pStr1, uint8_t *pStr2, uint8_t startBit, uint8_t stopBit)
\brief  Compare two strings bit by bit

\param [in]   pStr1            the start address of the first string to be compared
\param [in]   pStr2            the start address of the second string to be compared
\param [in]   startBit         the start bit number in the the 2 strings
\param [in]   stopBit          the stop bit number in the the 2 strings

\retval       bool_t           TRUE - if the strings match
                               FALSE - if the strings don't match
***************************************************************************************************/
bool_t NWKU_BitCmp
(
    uint8_t *pStr1,
    uint8_t *pStr2,
    uint8_t startBit,
    uint8_t stopBit
)
{
    uint8_t mask;

    /* Advance to the first byte that contains a bit to be compared */
    while (startBit >= 8)
    {
        pStr1++;
        pStr2++;
        startBit -= 8;
        stopBit -= 8;
    }

    /* Compare the first byte */
    if (startBit != 0)
    {
        mask = ((uint8_t)THR_ALL_FFs8 >> startBit);

        if (stopBit < 8)
        {
            mask = mask ^ ((uint8_t)THR_ALL_FFs8 >> stopBit);
        }

        if ((*pStr1 ^ *pStr2) & mask)
        {
            return FALSE;
        }

        /* If the stopBit and the startBit are in the same byte the comparison is over */
        if (stopBit < 8)
        {
            return TRUE;
        }

        stopBit -= 8;
        pStr1++;
        pStr2++;
    }

    while (stopBit >= 8)
    {
        if (*pStr1 != *pStr2)
        {
            return FALSE;
        }

        stopBit -= 8;
        pStr1++;
        pStr2++;
    }

    /* Compare the last byte */
    if (stopBit != 0)
    {
        mask = THR_ALL_FFs8 << (8 - stopBit);

        if ((*pStr1 ^ *pStr2) & mask)
        {
            return FALSE;
        }
    }

    return TRUE;
}

/*!*************************************************************************************************
\fn     bool_t NWKU_IsLLAddrEqual(uint8_t *pFirstLlAddr, uint32_t firstLlAddrSize,
                               uint8_t *pSecondLlAddr,uint32_t secondLlAddrSize)
\brief  Compare two Link Layer addresses

\param [in]   pFirstLlAddr      the start address of the first address to be compared
\param [in]   firstLlAddrSize   the size of the first address to be compared
\param [in]   pSecondLlAddr     the start address of the second address to be compared
\param [in]   secondLlAddrSize  the size of the second address to be compared

\retval       bool_t           TRUE - if the Link Layer addresses are the same
                               FALSE - if the Link Layer addresses are different
***************************************************************************************************/
bool_t NWKU_IsLLAddrEqual
(
    uint8_t *pFirstLlAddr,
    uint32_t firstLlAddrSize,
    uint8_t *pSecondLlAddr,
    uint32_t secondLlAddrSize
)
{
    bool_t bRetValue = FALSE;

    if ((firstLlAddrSize == secondLlAddrSize)
            && FLib_MemCmp(pFirstLlAddr, pSecondLlAddr, firstLlAddrSize))
    {
        bRetValue = TRUE;
    }

    return bRetValue;
}

/*!*************************************************************************************************
\fn     uint16_t NWKU_Revert16(uint16_t value)
\brief  Reverts a 16 bit numeric value.

\param [in]    value            the value to be converted

\return        uint16_t         the converted value
 ***************************************************************************************************/
uint16_t NWKU_Revert16(uint16_t value)
{
    return (uint16_t)((value << 8) | (value >> 8));
}

/*!*************************************************************************************************
\fn     uint32_t NWKU_Revert32(uint32_t value)
\brief  Reverts a 32 bit numeric value.

\param [in]    value            the value to be converted

\return       uint32_t         the converted value
 ***************************************************************************************************/
uint32_t NWKU_Revert32(uint32_t value)
{
    return (uint32_t)(
               (value >> 24) |
               (value << 24) |
               ((value >> 8) & 0x0000FF00) |
               ((value << 8) & 0x00FF0000));
}

/*!*************************************************************************************************
\fn     uint64_t NWKU_Revert64(uint64_t value)
\brief  Reverts a 64 bit numeric value.

\param [in]    value            the value to be converted

\return        uint64_t         the converted value
 ***************************************************************************************************/
uint64_t NWKU_Revert64(uint64_t value)
{
    uuint64_t in;
    uuint64_t out;
    in.u64 = value;

    out.u8[0] = in.u8[7];
    out.u8[1] = in.u8[6];
    out.u8[2] = in.u8[5];
    out.u8[3] = in.u8[4];
    out.u8[4] = in.u8[3];
    out.u8[5] = in.u8[2];
    out.u8[6] = in.u8[1];
    out.u8[7] = in.u8[0];

    return out.u64;
}

/*!*************************************************************************************************
\fn     uint16_t NWKU_TransformArrayToUint16(uint8_t *pArray)
\brief  Converts an big endian array to a 16 bit numeric value.

\param [in]    pArray           the start address of the array

\return        uint16_t         the converted value
 ***************************************************************************************************/
uint16_t NWKU_TransformArrayToUint16(uint8_t *pArray)
{
    uuint16_t out;

    out.u8[1] = *pArray++;
    out.u8[0] = *pArray;

    return out.u16;
}

/*!*************************************************************************************************
\fn    uint32_t NWKU_TransformArrayToUint32(uint8_t *pArray)
\brief  Converts an big endian array to a 32 bit numeric value.

\param [in]    pArray           the start address of the array

\return        uint32_t         the converted value
 ***************************************************************************************************/
uint32_t NWKU_TransformArrayToUint32
(
    uint8_t *pArray
)
{
    uuint32_t out;

    out.u8[3] = *pArray++;
    out.u8[2] = *pArray++;
    out.u8[1] = *pArray++;
    out.u8[0] = *pArray;

    return out.u32;

}

/*!*************************************************************************************************
\fn     uint64_t NWKU_TransformArrayToUint64(uint8_t *pArray)
\brief  Converts an big endian array to a 64 bit numeric value.

\param [in]     pArray           the start address of the array

\return         uint64_t         the converted value
 ***************************************************************************************************/
uint64_t NWKU_TransformArrayToUint64
(
    uint8_t *pArray
)
{
    uuint64_t out;

    out.u8[7] = *pArray++;
    out.u8[6] = *pArray++;
    out.u8[5] = *pArray++;
    out.u8[4] = *pArray++;
    out.u8[3] = *pArray++;
    out.u8[2] = *pArray++;
    out.u8[1] = *pArray++;
    out.u8[0] = *pArray;

    return out.u64;
}

/*!*************************************************************************************************
\fn     void NWKU_TransformUint16ToArray(uint8_t *pArray, uint16_t value)
\brief  Converts a 16 bit numeric value to array.

\param [in]    value            the value to be converted
\param [out]   pArray           the start address of the array

\return         none
 ***************************************************************************************************/
void NWKU_TransformUint16ToArray
(
    uint8_t *pArray,
    uint16_t value
)
{
    *pArray++ = (uint8_t)(value >> 8);
    *pArray   = (uint8_t)(value);
}

/*!*************************************************************************************************
\fn     void NWKU_TransformUint32ToArray(uint8_t *pArray, uint32_t value)
\brief  Converts a 32 bit numeric value to array.

\param [in]    value            the value to be converted
\param [out]   pArray           the start address of the array

\return        none
 ***************************************************************************************************/
void NWKU_TransformUint32ToArray
(
    uint8_t *pArray,
    uint32_t value
)
{
    *pArray++ = (uint8_t)(value >> 24);
    *pArray++ = (uint8_t)(value >> 16);
    *pArray++ = (uint8_t)(value >> 8);
    *pArray   = (uint8_t)(value);
}

/*!*************************************************************************************************
\fn    void NWKU_TransformUint64ToArray(uint8_t *pArray, uint64_t value)
\brief  Converts a 64 bit numeric value to array.

\param [in]    value            the value to be converted
\param [out]   pArray           the start address of the array

\return        none
 ***************************************************************************************************/
void NWKU_TransformUint64ToArray
(
    uint8_t *pArray,
    uint64_t value
)
{
    uuint64_t in;
    in.u64 = value;

    *pArray++ = in.u8[7];
    *pArray++ = in.u8[6];
    *pArray++ = in.u8[5];
    *pArray++ = in.u8[4];
    *pArray++ = in.u8[3];
    *pArray++ = in.u8[2];
    *pArray++ = in.u8[1];
    *pArray   = in.u8[0];
}

/*!*************************************************************************************************
\fn    uint64_t NWKU_TransformArrayToValue
\brief  Converts an array to a numeric value.

\param [in]   pArray          the start address of the array
\param [in]   nbOfBytes       the length of the data to be converted

\return       uint32_t        the value converted from the array
 ***************************************************************************************************/
uint64_t NWKU_TransformArrayToValue
(
    uint8_t *pArray,
    uint32_t nbOfBytes
)
{
    uuint64_t  value;
    uint32_t   iCount;
    value.u64 = 0;

    for (iCount = 0; iCount < nbOfBytes; iCount ++)
    {
        value.u8[nbOfBytes - iCount - 1] = pArray[iCount];
    }

    return value.u64;
}

/*!*************************************************************************************************
\fn    void NWKU_TransformValueToArray(uint64_t value, uint8_t *pArray, uint32_t nbOfBytes)
\brief  Converts a numeric value to array.

\param [in]    value            the value to be converted
\param [out]   pArray           the start address of the array
\param [in]    nbOfBytes        the length of the data to be converted

\retval        none
 ***************************************************************************************************/
void NWKU_TransformValueToArray
(
    uint64_t value,
    uint8_t *pArray,
    uint32_t nbOfBytes
)
{
    uint32_t   iCount;

    uuint64_t  u_value;
    u_value.u64 = value;

    for (iCount = 0; iCount < nbOfBytes; iCount ++)
    {
        pArray[nbOfBytes - iCount - 1] = u_value.u8[iCount];
    }
}

/*!*************************************************************************************************
\fn    bool_t NWKU_GetLut8(lut8_t *pLutTable, uint8_t lutTableSize, uint8_t type,
                           uint8_t *pEntryIndex)
\brief  Searches an entry in the lookup table indicated by pLutTable.

\param [in]    pLutTable     pointer to the lookup table
\param [in]    lutTableSize  lookup table size
\param [in]    type          type to find

\param [out]   pEntryIndex   index of the entry in case the entry is found

\retval        TRUE          returned when the entry is found
\retval        FALSE         otherwise
 ***************************************************************************************************/
bool_t NWKU_GetLut8
(
    lut8_t *pLutTable,
    uint8_t lutTableSize,
    uint8_t type,
    uint8_t *pEntryIndex
)
{
    uint8_t iCount;
    bool_t bRetValue = FALSE;

    if ((NULL != pLutTable) &&
            (NULL != pEntryIndex) &&
            (lutTableSize > 0U))
    {
        for (iCount = 0U; iCount < lutTableSize; iCount ++)
        {
            if ((pLutTable + iCount)->type == type)
            {
                /* Entry found */
                bRetValue = TRUE;
                *pEntryIndex = (pLutTable + iCount)->idx;
                break;
            }
        }
    }

    return bRetValue;
}

/*!*************************************************************************************************
\fn    int32_t NWKU_atoi(char *str)
\brief  Converts a string into an integer.

\param [in]    pStr       pointer to string

\retval     int32_t       integer converted from string.
***************************************************************************************************/
int32_t NWKU_atoi
(
    char *pStr
)
{
    int32_t res = 0;

    while ((*pStr != '\0') && (*pStr != ' ') && (*pStr >= '0') && (*pStr <= '9'))
    {
        res = res * 10 + *pStr - '0';
        pStr++;
    }

    return res;
}

/*!*************************************************************************************************
\fn    int64_t NWKU_atol(char *str)
\brief  Converts a string into an long integer.

\param [in]    pStr       pointer to string

\retval     int64_t       integer converted from string.
***************************************************************************************************/
int64_t NWKU_atol
(
    char *pStr
)
{
    int64_t res = 0;

    while ((*pStr != '\0') && (*pStr != ' ') && (*pStr >= '0') && (*pStr <= '9'))
    {
        res = res * 10 + *pStr - '0';
        pStr++;
    }

    return res;
}
/*!*************************************************************************************************
\fn    void NWKU_PrintDec(uint64_t value, uint8_t *pString, uint32_t nbOfDigits, bool_t bLeadingZeros)
\brief  Prints in a string decimal values.

\param [in]         value           integer value
\param [in/out]     pString         pointer to output location
\param [in]         nbPrintDigits   number of digits to be printed
\param [in]         bLeadingZeros   indicate if leading zeros are put or omitted
                                    TRUE - print leading zeros
                                    FALSE - do not print leading zeros
***************************************************************************************************/
void NWKU_PrintDec
(
    uint64_t value,
    uint8_t *pString,
    uint32_t nbPrintDigits,
    bool_t bLeadingZeros
)
{
    uint32_t nbOfDigits = 0;
    uint64_t tempValue = value;
    int32_t delta = 0;

    /* Get the number of digits for the number */
    if (0 == value)
    {
        /* If value == 0, print only one char */
        if (nbPrintDigits == THR_ALL_FFs8)
        {
            nbPrintDigits = 1;
        }

        FLib_MemSet(pString, '0', nbPrintDigits);
    }
    else
    {
        while (tempValue)
        {
            nbOfDigits++;
            tempValue = tempValue / 10;
        }

        delta = nbPrintDigits - nbOfDigits;
        tempValue = value;

        FLib_MemSet(pString, '0', nbOfDigits);

        /* Write only the number of digits wanted */
        while (nbPrintDigits && nbOfDigits)
        {
            if ((TRUE == bLeadingZeros) || (delta == 0))
            {
                *(pString + nbPrintDigits - 1) = (tempValue % 10) + '0';
            }
            else if (delta > 0)
            {
                *(pString + nbOfDigits - 1) = (tempValue % 10) + '0';
            }

            tempValue = tempValue / 10;
            nbPrintDigits--;
            nbOfDigits--;
        }
    }
}

/*!*************************************************************************************************
\fn     ptoll(uint8_t *pIn, uint32_t len, llAddr_t *pLlAddr)
\brief  Converts a string into an llAddr_t. Presentation to ll function.

\param [in]    pIn      pointer to the input buffer
\param [in]    len      size of the input buffer
\param [in]    pLlAddr  pointer to the start of the allocated llAddr_t structure

\retval     TRUE on success
            FALSE on error
***************************************************************************************************/
bool_t ptoll
(
    uint8_t *pIn,
    uint32_t len,
    llAddr_t *pLlAddr
)
{
    bool_t res = TRUE;

    /* EUI64 0xaabbccddeeff1122 */
    if ((pIn[0] == '0') && (pIn[1] == 'x') && (len == 18))
    {
        uint8_t idx;

        pIn += 2;
        pLlAddr->addrSize = gLlayerAddrEui64_c;
        pLlAddr->eui[0] = (NWKU_ByteToDec(*pIn) << 4) | NWKU_ByteToDec(*(pIn + 1));

        for (idx = 1; idx < 8; idx++)
        {
            pIn += 2;
            pLlAddr->eui[idx] = (NWKU_ByteToDec(*pIn) << 4) | NWKU_ByteToDec(*(pIn + 1));
        }
    }
    /* EUI64 aa:bb:cc:dd:ee:ff:11:22 */
    else if ((len == 23) && (pIn[2] == ':'))
    {
        uint8_t idx;

        pLlAddr->addrSize = gLlayerAddrEui64_c;
        pLlAddr->eui[0] = (NWKU_ByteToDec(*pIn) << 4) | NWKU_ByteToDec(*(pIn + 1));

        for (idx = 1; idx < 8; idx++)
        {
            pIn += 3;
            pLlAddr->eui[idx] = (NWKU_ByteToDec(*pIn) << 4) | NWKU_ByteToDec(*(pIn + 1));
        }
    }
    /* Unknown */
    else
    {
        res = FALSE;
    }

    return res;
}

/*!*************************************************************************************************
\fn     uint32_t NWKU_AsciiToHex(uint8_t *pString, uint32_t strLen)
\brief  Converts a string into hex.

\param  [in]    pString     pointer to string
\param  [in]    strLen      string length

\return uint32_t value in hex
***************************************************************************************************/
uint32_t NWKU_AsciiToHex
(
    uint8_t *pString,
    uint32_t strLen
)
{
    uint32_t length = strLen;
    uint32_t retValue = 0, hexDig = 0;
    bool_t validChar;

    while (length && pString)
    {
        hexDig = 0;
        validChar = FALSE;

        if (*pString >= '0' && *pString <= '9')
        {
            hexDig = *pString - '0';
            validChar = TRUE;
        }

        if (*pString >= 'a' && *pString <= 'f')
        {
            hexDig = *pString - 'a' + 10;
            validChar = TRUE;
        }

        if (*pString >= 'A' && *pString <= 'F')
        {
            hexDig = *pString - 'A' + 10;
            validChar = TRUE;
        }

        if (validChar == TRUE)
        {
            retValue = (retValue << 4) ^ hexDig;
        }

        pString++;
        length--;
    }

    return retValue;
}

/*!*************************************************************************************************
\fn     uint32_t NWKU_AsciiToDec(uint8_t *pString, uint32_t strLen)
\brief  Converts a string into hex.

\param  [in]    pString     pointer to string
\param  [in]    strLen      string length

\return uint32_t value in dec
***************************************************************************************************/
uint32_t NWKU_AsciiToDec
(
    uint8_t *pString,
    uint32_t strLen
)
{
    uint32_t length = strLen;
    uint32_t retValue = 0, hexDig = 0;

    while (length && pString)
    {
        hexDig = 0;

        if (*pString >= '0' && *pString <= '9')
        {
            hexDig = *pString - '0';

            retValue = (retValue * 10) + hexDig;
        }

        pString++;
        length--;
    }

    return retValue;
}

/*!*************************************************************************************************
\fn     uint8_t NWKU_ByteToDec(uint8_t byte)
\brief  Converts a byte from ASCII to decimal.

\param  [in]    byte     byte value in ASCII

\return uint8_t value in decimal
***************************************************************************************************/
uint8_t NWKU_ByteToDec
(
    uint8_t byte
)
{
    if (byte >= '0' && byte <= '9')
    {
        byte -= '0';
    }
    else if ((byte >= 'a' && byte <= 'f'))
    {
        byte = byte - 'a' + 10;
    }
    else if ((byte >= 'A' && byte <= 'F'))
    {
        byte = byte - 'A' + 10;
    }

    return byte;
}
/*!*************************************************************************************************
\fn     void NWKU_HexToAscii(uint8_t *pInputBuff,uint32_t inputBuffLen,uint8_t *pOutputBuffer,
                                uint32_t outputBuffLen, bool_t useUpperCase)
\brief  Converts a byte to ASCII.

\param  [in]    pInputBuff      pointer to input buffer
\param  [in]    inputBuffLen    length of the input buffer
\param  [in]    pOutputBuffer   pointer to output buffer
\param  [in]    outputBuffLen   length of the output buffer
\param  [in]    useUpperCase    indicate if the output shall be in upper/lower case

***************************************************************************************************/
void NWKU_HexToAscii
(
    uint8_t *pInputBuff,
    uint32_t inputBuffLen,
    uint8_t *pOutputBuffer,
    uint32_t outputBuffLen,
    bool_t useUpperCase
)
{
    if ((outputBuffLen >= 2 * inputBuffLen) || ((inputBuffLen == 1) && (outputBuffLen == 1)))
    {
        for (uint32_t i = 0; i < inputBuffLen; i++)
        {
            int8_t nib0 = 0;
            int8_t nib1 = 0;

            nib0 = (0xF0 & pInputBuff[i]) >> 4;
            nib1 = 0x0F & pInputBuff[i];

            /* if the first nib is 0 and there is only one byte for output,
            the first nib is elided */
            if ((nib0 == 0) && (outputBuffLen == 1))
            {
                pOutputBuffer[2 * i] = NWKU_NibToAscii(nib1, useUpperCase);
            }
            else
            {
                pOutputBuffer[2 * i] = NWKU_NibToAscii(nib0, useUpperCase);
                pOutputBuffer[2 * i + 1] = NWKU_NibToAscii(nib1, useUpperCase);
            }
        }
    }
}

/*!*************************************************************************************************
\fn     uint8_t NWKU_NibToAscii(uint8_t nib)
\brief  Converts a nib from hex to ASCII.

\param  [in]    nib     nib value in hex

\return uint8_t value in ascii
***************************************************************************************************/
uint8_t NWKU_NibToAscii
(
    int8_t nib,
    bool_t useUpperCase
)
{
    if ((nib >= 0) && (nib <= 0x09))
    {
        nib += '0';
    }


    if ((nib >= 0x0A) && (nib <= 0x0F))
    {
        if (TRUE == useUpperCase)
        {
            nib += 'A' - 0x0A;
        }
        else
        {
            nib += 'a' - 0x0A;
        }
    }


    return nib;
}

/*!*************************************************************************************************
\fn    uint32_t NWKU_TmrRtcGetElapsedTimeInSeconds(uint32_t timestamp)
\brief Calculates the time passed in seconds from the provided timestamp.

\param [in]    timestamp     timestamp in seconds

\return        uint32_t      number of seconds that have passed since the provided timestamp
 ***************************************************************************************************/
uint32_t NWKU_TmrRtcGetElapsedTimeInSeconds
(
    uint32_t timestamp
)
{
    return (uint32_t)(TmrMicrosecondsToSeconds(TMR_GetTimestamp()) - timestamp);
}

/*!*************************************************************************************************
\fn    bool_t NWKU_IsNUmber(char *pString)
\brief Check if a string is a number.

\param [in]    pString      pointer to the string

\return        bool_t       TRUE if the string represents a number
                            FALSE if the string does not represent a number
***************************************************************************************************/
bool_t NWKU_IsNUmber
(
    char *pString
)
{
    bool_t ret = TRUE;

    while (*pString)
    {
        if (*pString < '0' || *pString > '9')
        {
            ret = FALSE;
            break;
        }

        pString++;
    }

    return ret;
}

/*!*************************************************************************************************
\fn    uint32_t NWKU_GetRandomNoFromInterval(uint32_t startInterval, uint32_t endInterval)
\brief This function returns a random number from a given interval.

\param    [in]  startInterval   Start value of the interval
\param    [in]  endInterval     End value of the interval

\return   uint32_t - random value
***************************************************************************************************/
uint32_t NWKU_GetRandomNoFromInterval
(
    uint32_t startInterval,
    uint32_t endInterval
)
{
    uint32_t rand;

    RNG_GetRandomNo(&rand);

    rand = startInterval + rand % (endInterval - startInterval + 1);

    return rand;
}

/*!*************************************************************************************************
\fn     uint32_t NWKU_RightRotate(uint32_t val, uint8_t amount)
\brief  This function rotate a 32bit number to the right with an amount of bits.

\param    [in]  val     number
\param    [in]  amount  number of bits to rotate

\return   uint32_t      32bit number rotated to the right with an amount of bits
***************************************************************************************************/
uint32_t NWKU_RightRotate
(
    uint32_t val,
    uint8_t amount
)
{
    return (val >> amount) | (val << (uint8_t)(32 - amount));
}

/*!*************************************************************************************************
\fn     void NWKU_GetIIDFromLLADDR(llAddr_t *pLlAddr, uint16_t panId, uint8_t *pIID)
\brief  The function returns the IID from a Link-Layer address.

\param  [in]    pLlAddr         Pointer to the Link-Layer address
\param  [in]    panId           PAN ID
\param  [out]   pIID            Pointer to the variable which will hold the IID
***************************************************************************************************/
void NWKU_GetIIDFromLLADDR
(
    llAddr_t *pLlAddr,
    uint16_t panId,
    uint8_t *pIID
)
{
    if (gLlayerAddrEui64_c == pLlAddr->addrSize)
    {
        FLib_MemCpy(pIID, &pLlAddr->eui, pLlAddr->addrSize);
        *(pIID) ^= 0x02;
    }
    else
    {
        htonas(pIID, panId);
        htonas(pIID + 2, 0x00FF);
        htonas(pIID + 4, 0xFE00);
        FLib_MemCpy(pIID + 6, pLlAddr->eui, gLlayerAddrEui16_c);
    }
}

/*!*************************************************************************************************
\fn     bool_t NWKU_GetBit(uint32_t bitNr, uint8_t *pArray)
\brief  This function returns the value of a bit in an array.

\param  [in]    bitNr           bit number in the whole array
\param  [in]    pArray          pointer to the start of the array

\retval         TRUE            if the bit is set
\retval         FALSE           if the bit is not set
***************************************************************************************************/
bool_t NWKU_GetBit
(
    uint32_t bitNr,
    uint8_t *pArray
)
{
    return ((pArray[bitNr / 8] & (1 << (bitNr % 8))) ? TRUE : FALSE);
}

/*!*************************************************************************************************
\fn     void NWKU_SetBit(uint32_t bitNr, uint8_t* pArray)
\brief  This function sets a bit in an array.

\param  [in]    bitNr           bit number in the whole array
\param  [in]    pArray          pointer to the start of the array
***************************************************************************************************/
void NWKU_SetBit
(
    uint32_t bitNr,
    uint8_t *pArray
)
{
    pArray[bitNr / 8] |= (1 << (bitNr % 8));
}


/*!*************************************************************************************************
\fn     void NWKU_ClearBit(uint32_t bitNr, uint8_t* pArray)
\brief  This function clears a bit in an array.

\param  [in]    bitNr           bit number in the whole array
\param  [in]    pArray          pointer to the start of the array
***************************************************************************************************/
void NWKU_ClearBit
(
    uint32_t bitNr,
    uint8_t *pArray
)
{
    pArray[bitNr / 8] &= ~(1 << (bitNr % 8));
}

/*!*************************************************************************************************
\fn     uint32_t NWKU_GetFirstBitValueInRange(uint8_t* pArray, uint32_t lowBitNr, uint32_t
        highBitNr, bool_t bitValue)
\brief  This function returns the first bit with value=bitValue in a range in the array.

\param  [in]    pArray          pointer to the start of the array
\param  [in]    lowBitNr        starting bit number
\param  [in]    highBitNr       ending bit number
\param  [in]    bitValue        bit value

\return         uint32_t        bit number
***************************************************************************************************/
uint32_t NWKU_GetFirstBitValueInRange
(
    uint8_t *pArray,
    uint32_t lowBitNr,
    uint32_t highBitNr,
    bool_t bitValue
)
{
    for (; lowBitNr < highBitNr; lowBitNr++)
    {
        if (bitValue == NWKU_GetBit(lowBitNr, pArray))
        {
            return lowBitNr;
        }
    }

    return ((uint32_t) - 1); // invalid
}

/*!*************************************************************************************************
\fn     uint32_t getFirstBitValue(uint8_t* pArray, uint32_t arrayBytes, bool_t bitValue)
\brief  This function returns the index of the first bit with value=bitValue.

\param  [in]    pArray          pointer to the start of the array
\param  [in]    arrayBytes      number of bytes in the array
\param  [in]    bitValue        bit value

\return         uint32_t        bit value
***************************************************************************************************/
uint32_t NWKU_GetFirstBitValue
(
    uint8_t *pArray,
    uint32_t arrayBytes,
    bool_t bitValue
)
{
    return NWKU_GetFirstBitValueInRange(pArray, 0, (arrayBytes * 8), bitValue);
}

/*!*************************************************************************************************
\fn     uint32_t NWKU_GetNumOfBits(uint8_t* pArray, uint32_t arrayBytes, bool_t bitValue);
\brief  This function returns number of bits of value bitValue from an array

\param  [in]    pArray          pointer to the start of the array
\param  [in]    arrayBytes      number of bytes in the array
\param  [in]    bitValue        bit value

\return         uint32_t        bit value
***************************************************************************************************/
uint32_t NWKU_GetNumOfBits
(
    uint8_t *pArray,
    uint32_t arrayBytes,
    bool_t bitValue
)
{
    uint32_t i, j, nrOfBits = 0;

    for (i = 0; i < arrayBytes; i++)
    {
        for (j = 0; j < 8; j++)
        {
            if (((pArray[i] >> j) & 0x01) == bitValue)
            {
                nrOfBits++;
            }
        }
    }

    return nrOfBits;
}
/*!*************************************************************************************************
\fn     uint32_t NWKU_ReverseBits(uint32_t num);
\brief  This function returns uint32_t bit in reverse order

\param  [in]    num             uint32_t number

\return         uint32_t        uint32_t number with the bits in reverse order
***************************************************************************************************/
uint32_t NWKU_ReverseBits
(
    uint32_t num
)
{
    uint32_t count = sizeof(num) * 8 - 1;
    uint32_t reverseNum = num;

    num >>= 1;

    while (num)
    {
        reverseNum <<= 1;
        reverseNum |= num & 1;
        num >>= 1;
        count--;
    }

    reverseNum <<= count;
    return reverseNum;
}

/*!*************************************************************************************************
\fn     uint32_t NWKU_AddTblEntry(uint32_t entry, uint32_t *pTable, uint32_t tableSize)
\brief  This function adds a new entry in a table. The table needs to have uint32_t elements.

\param  [in]    entry       entry value
\param  [in]    pTable      pointer to the start of the table
\param  [in]    tableSize   the size of the table

\return         entry index or -1(0xFFFFFFFF) in case of error
***************************************************************************************************/
uint32_t NWKU_AddTblEntry
(
    uint32_t entry,
    uint32_t *pTable,
    uint32_t tableSize
)
{
    uint32_t iEntry = tableSize;
    bool_t status = FALSE;

    for (iEntry = 0; iEntry < tableSize; iEntry++)
    {
        if (!pTable[iEntry])
        {
            pTable[iEntry] = entry;
            status = TRUE;
            break;
        }
    }

    return (status == TRUE) ? iEntry : (uint32_t)(-1);
}

/*!*************************************************************************************************
\fn     uint32_t NWKU_GetTblEntry(uint32_t entry, uint32_t *pTable, uint32_t tableSize)
\brief  This function search for an element in a table.

\param  [in]    entry       entry value
\param  [in]    pTable      pointer to the start of the table
\param  [in]    tableSize   the size of the table

\return         entry index or NULL in case of error
***************************************************************************************************/
uint32_t NWKU_GetTblEntry
(
    uint32_t index,
    uint32_t *pTable,
    uint32_t tableSize
)
{
    return (index < tableSize) ? pTable[index] : 0;
}

/*!*************************************************************************************************
\fn     void NWKU_SwapArrayBytes(uint8_t *pByte, uint8_t numOfBytes)
\brief  This function swaps the bytes in an array and puts the result in the same array.

\param  [in/out]    pByte       pointer to the start of the array
\param  [in]        numOfBytes  size of the array
***************************************************************************************************/
void NWKU_SwapArrayBytes
(
    uint8_t *pByte,
    uint8_t numOfBytes
)
{
    uint8_t tmp, i;

    for (i = 0; i < (numOfBytes >> 1); i++)
    {
        tmp = pByte[i];
        pByte[i] = pByte[numOfBytes - i - 1];
        pByte[numOfBytes - i - 1] = tmp;
    }
}

/*!*************************************************************************************************
\fn     void NWKU_GenRand(uint8_t *pRand, uint8_t randLen)
\brief  This function generates a random value in the desired array.

\param  [out]   pRand     Pointer to the start of the output array
\param  [in]    randLen   Size of the array
***************************************************************************************************/
void NWKU_GenRand
(
    uint8_t *pRand,
    uint8_t randLen
)
{
    while (randLen != 0)
    {
        if (randLen < 4)
        {
            uint32_t randNb;
            RNG_GetRandomNo((uint32_t *)&randNb);
            FLib_MemCpy(pRand, (uint8_t *)&randNb, randLen);
            randLen = 0;
        }
        else
        {
            randLen -= 4;
            RNG_GetRandomNo((uint32_t *)&pRand[randLen]);
        }

    }
}

/*!*************************************************************************************************
\fn     NWKU_GetTimestampMs
\brief  Get the timestamp in milliseconds.

\return timestamp in milliseconds
***************************************************************************************************/
uint64_t NWKU_GetTimestampMs
(
    void
)
{
    return (TMR_GetTimestamp() / 1000);
}

/*!*************************************************************************************************
\fn     NWKU_isArrayGreater
\brief  Compare tow numbers represented as array
\param  [in] a first array
\param  [in] b second array
\param  [in] length - how many bytes to compare
\return 0 - are equal
        1 - a > b
        -1 - b < a
***************************************************************************************************/
int8_t NWKU_isArrayGreater
(
    const uint8_t *a,
    const uint8_t *b,
    uint8_t length
)
{
    int i;

    for (i = length - 1; i >= 0; --i)
    {
        if (a[i] > b[i])
        {
            return 1;
        }

        if (a[i] < b[i])
        {
            return -1;
        }
    }

    return 0;
}

size_t strlcpy(char *dst, const char *src, size_t dstsize)
{
    return missing_strlcpy(dst, src, dstsize);
}

/*==================================================================================================
Private functions
==================================================================================================*/

/*==================================================================================================
Private debug functions
==================================================================================================*/

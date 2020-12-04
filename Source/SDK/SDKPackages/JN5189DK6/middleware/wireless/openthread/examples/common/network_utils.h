/*
* Copyright 2019 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _NETWORK_UTILS_H
#define _NETWORK_UTILS_H

/*!
\file       network_utils.h
\brief      This is a header file for the Network Utils module.
*/

/*==================================================================================================
Include Files
==================================================================================================*/
/* General Includes */
#include "EmbeddedTypes.h"
#include <openthread/ip6.h>

#include "Messaging.h"
#include "fsl_os_abstraction.h"

/*==================================================================================================
Public macros
==================================================================================================*/
#if __ICCARM__
    #define gLittleEndian_c __LITTLE_ENDIAN__
#elif __GNUC__
    #if __BYTE_ORDER == __LITTLE_ENDIAN
        #define gLittleEndian_c 1
    #else
        #define gLittleEndian_c 0
    #endif
#else
    #warning "No Compiler was set"
#endif

/*! Max unsigned 64bit integers value */
#define THR_ALL_FFs64                       0xFFFFFFFFFFFFFFFF

/*! Max unsigned 32bit integers value */
#define THR_ALL_FFs32                       0xFFFFFFFF

/*! Max unsigned 16bit integers value */
#define THR_ALL_FFs16                       0xFFFF

/*! Max unsigned 8bit integers value */
#define THR_ALL_FFs8                        0xFF

/*! IPV6 any address */
#define IN6ADDR_ANY_INIT                                    \
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

/*! IPV6 loopback address */
#define IN6ADDR_LOOPBACK_INIT                               \
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }

/*! IPV6 node local all nodes address */
#define IN6ADDR_NODELOCAL_ALLNODES_INIT                     \
        { 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }

/*! IPV6 interface local all nodes address */
#define IN6ADDR_INTFACELOCAL_ALLNODES_INIT                  \
        { 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }

/*! IPV6 link local all nodes address */
#define IN6ADDR_LINKLOCAL_ALLNODES_INIT                     \
        { 0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }

/*! IPV6 link local all routers address */
#define IN6ADDR_LINKLOCAL_ALLROUTERS_INIT                   \
        { 0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 }

/*! IPV6 link local all v2 routers address */
#define IN6ADDR_LINKLOCAL_ALLV2ROUTERS_INIT                 \
        { 0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   \
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16 }

/*! IPV6 link local all DHCP routers and relay agents address */
#define IN6ADDR_LINKLOCAL_ALL_DHCP_ROUTERS_AND_RELAY_AGENTS  \
        {0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02}

/*! IPV6 realm local all DHCP lease query servers address */
#define IN6ADDR_REALMLOCAL_ALL_DHCP_LEASEQUERY_SERVERS       \
        {0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03}

/*! IPV6 realm local multicast 3ead address */
#define IN6ADDR_REALMLOCAL_MCAST_3EAD \
        {0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0xAD}

/*! IPV6 realm local multicast 3ead address */
#define IN6ADDR_REALMLOCAL_ALLMPLFORWARDERS \
        {0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc}

/*! IPV6 site local all DHCP servers address */
#define IN6ADDR_SITELOCAL_ALLDHCPSERVERS                                  \
        {0xff, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03}

/*! IPV6 realm local all nodes address */
#define IN6ADDR_REALMLOCAL_ALLNODES_INIT \
        {0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}

/*! IPV6 realm local all routers address */
#define IN6ADDR_REALMLOCAL_ALLROUTERS_INIT \
        {0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}

/*! IPV6 site local all nodes address */
#define IN6ADDR_SITELOCAL_ALLNODES_INIT \
        {0xff, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}

/*! IPV6 site local all routers address */
#define IN6ADDR_SITELOCAL_ALLROUTERS_INIT \
        {0xff, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}

/*! IPV6 link local prefix address */
#define IN6ADDR_LINK_LOCAL_PREFIX_INIT \
        {0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

/*! IPV6 all FFs address */
#define IN6ADDR_ALL_FFs                                    \
        { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   \
          0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }

/*! IPV6 link local all CoAP nodes address */
#define IN6ADDR_LINKLOCAL_ALL_COAP_NODES_INIT  \
        {0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd}

/*! IPV6 realm local all CoAP nodes address */
#define IN6ADDR_REALMLOCAL_ALL_COAP_NODES_INIT \
        {0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd}

/*! IPV6 admin local all CoAP nodes address */
#define IN6ADDR_ADMINLOCAL_ALL_COAP_NODES_INIT \
        {0xff, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd}

/*! IPV6 realm local all CoAP nodes address */
#define IN6ADDR_SITELOCAL_ALL_COAP_NODES_INIT       \
        {0xff, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     \
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd}

/*! Macro for IP address copy */
#define IP_AddrCopy(dst, src) \
        (dst)->m32[0] = (src)->m32[0]; \
        (dst)->m32[1] = (src)->m32[1]; \
        (dst)->m32[2] = (src)->m32[2]; \
        (dst)->m32[3] = (src)->m32[3];

/*! Macro for IP address copy */
#define IP_AddrCopyFromArray(ip, buf, len) \
        for (uint8_t idx = 0; idx < len; idx++) \
            (ip)->m8[idx] = (buf)[idx];

/*! Macro for IP address copy */
#define IP_AddrCopyToArray(ip, buf, len) \
        for (uint8_t idx = 0; idx < len; idx++) \
            (buf)[idx] = (ip)->m8[idx];


/*! Macro for IP address conversion to uint32_t */
#define IP4_AddrToUint32(addr) (ntohal(&(addr)->m8[12]))

/*! Macro for IPV6 address comparison */
#define IP_IsAddrEqual(addr1, addr2) \
        (((addr1)->m32[0] == (addr2)->m32[0]) && \
         ((addr1)->m32[1] == (addr2)->m32[1]) && \
         ((addr1)->m32[2] == (addr2)->m32[2]) && \
         ((addr1)->m32[3] == (addr2)->m32[3]))

/*! Macro for unspecified IPV6 address inquiry */
#define IP6_IsUnspecifiedAddr(addr) \
        (((addr)->addr64[0] == 0U) && \
         ((addr)->addr64[1] == 0U))

/*! Macro for link local IPV6 address inquiry */
#define IP6_IsLinkLocalAddr(addr) \
        (((addr)->m8[0] == 0xFE) && (((addr)->m8[1] & 0xC0) == 0x80))

/*! Macro for site local IPV6 address inquiry */
#define IP6_IsSiteLocalAddr(addr) \
        (((addr)->m8[0] == 0xFE) && (((addr)->m8[1] & 0xC0) == 0xC0))

/*! Macro for unique local IPV6 address inquiry */
#define IP6_IsUniqueLocalAddr(addr) \
        (((addr)->m8[0] == 0xFD)||((addr)->m8[0] == 0xFC) )

/*! Macro for global IPV6 address inquiry */
#define IP6_IsGlobalAddr(addr) \
        ((((addr)->m8[0] & 0xF0) == 0x20)||(((addr)->m8[0] & 0xF0) == 0x30))

/*! Macro for multicast IPV6 address inquiry */
#define IP6_IsMulticastAddr(addr) \
        ((addr)->m8[0] == THR_ALL_FFs8)          

/*! Macro for anycast IPV6 address inquiry */
#define IP6_IsAnycastAddr(addr) \
        (((addr)->m8[11] == THR_ALL_FFs8) && ((addr)->m8[12] == 0xFE) && ((addr)->m8[14] == 0xFC))

/*! Macro for loopback IPV6 address inquiry */
#define IP6_IsLoopbackAddr(addr) \
        (((addr)->addr64[0] == 0U) && \
         ((addr)->addr64[1] == 0x0100000000000000U))

/*! Macro for local multicast all nodes IPV6 address inquiry */
#define IP6_IsLocalMulticastAllNodes(addr) \
        (((addr)->addr64[0] == 0x00000000000002FFU) && \
         ((addr)->addr64[1] == 0x0100000000000000U))

/*! Macro for local multicast all routers IPV6 address inquiry */
#define IP6_IsLocalMulticastAllRouters(addr) \
        (((addr)->addr64[0] == 0x00000000000002FFU) && \
         ((addr)->addr64[1] == 0x0200000000000000U))

/*! Macro for mesh multicast all nodes IPV6 address inquiry */
#define IP6_IsMeshMulticastAllNodes(addr) \
        (((addr)->addr64[0] == 0x00000000000003FFU) && \
         ((addr)->addr64[1] == 0x0100000000000000U))

/*! Macro for EUI64 IPV6 address inquiry */
#define IP6_IsAddrEui64(addr) \
        (!(((addr)->addr8[10] == 0x00) && ((addr)->addr8[11] == THR_ALL_FFs8) && \
        ((addr)->addr8[12] == 0xFE) && ((addr)->addr8[13] == 0x00)))

/*! Macro for values to IP address array transformation */
#define IP_ADDR(a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16) \
        { (a1), (a2), (a3), (a4), (a5), (a6), (a7), (a8),       \
    (a9), (a10), (a11), (a12), (a13), (a14), (a15), (a16) }

/*! Mask for IPV4 address identification(RFC4291: 2.5.5.2) */
#define IPV4_Mask32_g   (THR_ALL_FFs16)

/*! Macro for IPV4 in IPv6 address inquiry */
#define IP_IsAddrIPv4(addr) \
        ((addr)->addr64[0] == 0U &&         \
         (addr)->addr16[4] == 0U &&         \
         (addr)->addr16[5] == IPV4_Mask32_g)

/*! Macro for IPV4 unspecified address inquiry */
#define IP4_IsUnspecifiedAddr(addr) \
        (((addr)->addr32[3] == 0U))

/*! Macro for IPV6 address inquiry */
#define IP_IsAddrIPv6(addr) (!(IP_IsAddrIPv4(addr)))

/*! Macro for appending network buffer */
#define NWKU_AppendNwkBuffer(dst, src) \
        (dst)->next = (src);

/*! Macro for link layer address validity inquiry */
#define NWKU_IsLlAddrValid(llAddr) (gLlayerAddrNoAddr_c != llAddr.addrSize)

/*! Macro for retrieving the last index of an array */
#define NWKU_GetLastArrayIndex(arraySize) ((arraySize) - 1U)

/*! Macro for host variable to 24 bit network array conversion */
#define htona24(p, x)\
    do\
    {\
        *((uint8_t*)(p)) = ((x) >> 16) & THR_ALL_FFs8;\
        htonas((uint8_t*)(p + 1), ((x) & THR_ALL_FFs16));\
    } while(0);

/*! Macro for 24 bit network array to host variable conversion */
#define ntoha24(p)\
    ((((uint32_t) *(((unsigned char*)(p)))) << 16)    \
   | (((uint32_t) *(((unsigned char*)(p))+1)) << 8) \
   | (((uint32_t) *(((unsigned char*)(p))+2))))

/*! Macro for host variable to 48 bit network array conversion */
#define htona48(p, x)\
    do\
    {\
        htonal((uint8_t*)p, ((x >> 16) & THR_ALL_FFs32));\
        htonas((uint8_t*)(p + 4), (x & THR_ALL_FFs16));\
    } while(0);

/*! Macro for 48 bit network array to host variable conversion */
#define ntoha48(p)\
    ((uint64_t)ntohal(p) << 16) | \
    (ntohas(p + 4) & THR_ALL_FFs16)

#if gLittleEndian_c

/*! Macro for network to host short conversion. Network byte order is OTA order */
#ifndef ntohs
#define ntohs(val)                      NWKU_Revert16((uint16_t)(val))
#endif

/*! Macro for host short to network conversion. Network byte order is OTA order */
#ifndef htons
#define htons(val)                      NWKU_Revert16((uint16_t)(val))
#endif

/*! Macro for network to host 32bit conversion. Network byte order is OTA order */
#ifndef ntohl
#define ntohl(val)                      NWKU_Revert32((uint32_t)(val))
#endif

/*! Macro for host 32bit to network conversion. Network byte order is OTA order */
#ifndef htonl
#define htonl(val)                      NWKU_Revert32((uint32_t)(val))
#endif

/*! Macro for network to host 64bit conversion. Network byte order is OTA order */
#ifndef ntohll
#define ntohll(val)                     NWKU_Revert64((uint64_t)(val))
#endif

/*! Macro for host 64bit to network conversion. Network byte order is OTA order */
#ifndef htonll
#define htonll(val)                     NWKU_Revert64((uint64_t)(val))
#endif

/*! Macro for network array to host short conversion. Network byte order is OTA order */
#ifndef ntohas
#define ntohas(p)                       NWKU_TransformArrayToUint16(p)
#endif

/*! Macro for host short to network array conversion. Network byte order is OTA order */
#ifndef htonas
#define htonas(p, x)                    NWKU_TransformUint16ToArray(p, (uint16_t)(x))
#endif

/*! Macro for network array to host 32bit conversion. Network byte order is OTA order */
#ifndef ntohal
#define ntohal(p)                       NWKU_TransformArrayToUint32(p)
#endif

/*! Macro for host 32bit to network array conversion. Network byte order is OTA order */
#ifndef htonal
#define htonal(p, x)                    NWKU_TransformUint32ToArray(p, (uint32_t)(x))
#endif

/*! Macro for network array to host 64bit conversion. Network byte order is OTA order */
#ifndef ntohall
#define ntohall(p)                      NWKU_TransformArrayToUint64(p)
#endif

/*! Macro for host 64bit to network array conversion. Network byte order is OTA order */
#ifndef htonall
#define htonall(p, x)                   NWKU_TransformUint64ToArray(p, (uint64_t)(x))
#endif

#else /*gBigEndian_c */
/*! Macro for network to host short conversion */
#ifndef ntohs
#define ntohs(val)                      (uint16_t)(val)
#endif

/*! Macro for host short to network conversion */
#ifndef htons
#define htons(val)                      (uint16_t)(val)
#endif

/*! Macro for network to host 32bit conversion */
#ifndef ntohl
#define ntohl(val)                      (uint32_t)(val)
#endif

/*! Macro for host 32bit to network conversion */
#ifndef htonl
#define htonl(val)                      (uint32_t)(val)
#endif

/*! Macro for network to host 64bit conversion*/
#ifndef ntohll
#define ntohll(val)                     (uint64_t)(val)
#endif

/*! Macro for host 64bit to network conversion */
#ifndef htonll
#define htonll(val)                     (uint64_t)(val)
#endif

/*! Macro for network array to host short conversion */
#ifndef ntohas
#define ntohas(p)                       (*((uint16_t*)(p)))
#endif

/*! Macro for host short to network array conversion */
#ifndef htonas
#define htonas(p, x)                    *((uint16_t*)(p)) = x
#endif

/*! Macro for network array to host 32bit conversion */
#ifndef ntohal
#define ntohal(p)                       (*((uint32_t*)(p)))
#endif

/*! Macro for host 32bit to network array conversion */
#ifndef htonal
#define htonal(p, x)                    *((uint32_t*)(p)) = x
#endif

/*! Macro for network array to host 64bit conversion */
#ifndef ntohall
#define ntohall(p)                      (*((uint64_t*)(p)))
#endif

/*! Macro for host 64bit to network array conversion */
#ifndef htonall
#define htonall(p, x)                   *((uint64_t*)(p)) = x
#endif

#endif

/*! Macro for obtaining the minimum value variable between two input variables */
#ifndef MIN
#   define MIN(a,b)                     (((a) < (b))?(a):(b))
#endif

/*! Generic Message Event */
#define NWKU_GENERIC_MSG_EVENT 1

#define gNoIPv6FlowInfo_c               THR_ALL_FFs32
/*==================================================================================================
Public type definitions
==================================================================================================*/
/*! Generic structure for holding uint16 values */
typedef union uuint16_tag
{
    uint16_t    u16;                 /*!< 16bit variable */
    uint8_t     u8[2];               /*!< 8bit array */
} uuint16_t;

/*! Generic structure for holding uint32 values */
typedef union uuint32_tag
{
    uint32_t    u32;                 /*!< 32bit variable */
    uint16_t    u16[2];              /*!< 16bit array */
    uint8_t     u8[4];               /*!< 8bit array */
} uuint32_t;

/*! Generic structure for holding uint64 values */
typedef union uuint64_tag
{
    uint64_t    u64;                 /*!< 64bit variable */
    uint32_t    u32[2];              /*!< 32bit array */
    uint16_t    u16[4];              /*!< 16bit array */
    uint8_t     u8[8];               /*!< 8bit array */
} uuint64_t;

/*! Enumeration for address size */
typedef enum
{
    gLlayerAddrNoAddr_c     = 0x00,     /*!< No address (addressing fields omitted) */
    gLlayerAddrReserved_c   = 0x01,     /*!< Reserved value*/
    gLlayerAddrEui16_c      = 0x02,     /*!< 16-bit short Link Layer address (size 2 bytes) */
    gLlayerAddrEui48_c      = 0x06,     /*!< 48-bit Ethernet MAC Address (size 6 bytes) */
    gLlayerAddrEui64_c      = 0x08,     /*!< 64-bit extended Link Layer address (size 8 bytes) */
} llAddrSize_t;

/*! IP Address Types */
typedef enum nwkIPAddrType_tag
{
    gLL64Addr_c = 0x00,         /*!< Link-Local 64 address (the IID is MAC Extended address Which is not the factory-assigned IEEE EUI-64,)*/
    gMLEIDAddr_c = 0x01,        /*!< Mesh-Local Endpoint Identifier address (the IID is randmon) */
    gRLOCAddr_c = 0x02,         /*!< Routing Locator address (the IID encodes the Router and Child IDs.)*/
    gGUAAddr_c  = 0x03,         /*!< Global Unicast Address*/
    gAnycastAddr_c = 0x04,      /*!< Anycast IPv6 addresses */
    gDUAAddr_c = 0x05,          /*!< Domain Unicast Address */
    gAnyIpv6_c = 0x06,          /*!< All IPv6 address */
    gAllThreadNodes_c = 0x07,   /*!< All Thread nodes address */
} nwkIPAddrType_t;

/*! Generic structure for link layer address */
typedef struct llAddr_tag
{
    uint8_t         eui[8];          /*!< Destination address: short/extended */
    llAddrSize_t    addrSize;        /*!< Destination address type: short/extended */
} llAddr_t;

/*! Lookup tables with 8 bits elements */
typedef struct lut8_tag
{
    uint8_t type;    /*!< Type */
    uint8_t idx;     /*!< Index */
}lut8_t;

/*! Network generic status enumeration */
typedef enum nwkStatus_tag
{
    gNwkStatusSuccess_c      = 0,              /*!< Network Status: Success */
    gNwkStatusMemAllocErr_c  = 1,              /*!< Network Status: Memory allocation error */
    gNwkStatusNotAllowed_c   = 2,              /*!< Operation was not allowed */
    gNwkStatusInvalidParam_c = 3,              /*!< Input parameters are invalid */
    gNwkStatusFail_c         = THR_ALL_FFs8,   /*!< Network Status: Fail */
}nwkStatus_t;

/*! Sequence number arithmetic comparison status */
typedef enum nwkSeqNbStatus_tag
{
    gNwkSeqNbLower_c     = 0,              /*!< Sequence number is lower status */
    gNwkSeqNbEqual_c     = 1,              /*!< Sequence number is equal status */
    gNwkSeqNbHigher_c    = 2,              /*!< Sequence number is higher status*/
}nwkSeqNbStatus_t;

/*! Multicast all thread nodes */
typedef enum thrMcastAllThrNodes_tag
{
    gMcastLLAddrAllThrNodes_c,  /*!< Multicast link local - all thread nodes */
    gMcastMLAddrAllThrNodes_c,  /*!< Multicast mesh local - all thread nodes */
} thrMcastAllThrNodes_t;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
extern const otIp6Address in6addr_any;
extern const otIp6Address in6addr_loopback;
extern const otIp6Address in6addr_nodelocal_allnodes;
extern const otIp6Address in6addr_linklocal_allnodes;
extern const otIp6Address in6addr_linklocal_allrouters;
extern const otIp6Address in6addr_linklocal_allv2routers;
extern const otIp6Address in6addr_sitelocal_alldhcpservers;
extern const otIp6Address in6addr_realmlocal_allnodes;
extern const otIp6Address in6addr_realmlocal_allrouters;
extern const otIp6Address in6addr_realmlocal_allleasequeryservers;
extern const otIp6Address in6addr_realmlocal_mcast_3ead;
extern const otIp6Address in6addr_realmlocal_allmplforwarders;
extern const otIp6Address in6addr_sitelocal_allnodes;
extern const otIp6Address in6addr_sitelocal_allrouters;
extern const otIp6Address in6addr_link_local_prefix;
extern const otIp6Address in6addr_linklocal_allcoapnodes;
extern const otIp6Address in6addr_realmlocal_allcoapnodes;
extern const otIp6Address in6addr_adminlocal_allcoapnodes;
extern const otIp6Address in6addr_sitelocal_allcoapnodes;
extern const uint32_t in4addr_any;
/* RAM global addresses - updated when the device join the network */
extern otIp6Address in6addr_linklocal_allthreadnodes;
extern otIp6Address in6addr_realmlocal_allthreadnodes;
extern otIp6Address in6addr_realmlocal_threadleaderanycast;

/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn    otIp6Address *NWKU_GetSpecificMcastAddr(otInstance *pOtInstance, thrMcastAllThrNodes_t type)
\brief This function is used to get a specific multicast address (Mesh Local All nodes multicast or
       link local All nodes multicast)

\param [in]    pOtInstance         Pointer to the OpenThread instance
\param [in]    type                Ip address type: gMcastLLAddrAllThrNodes_c, gMcastMLAddrAllThrNodes_c

\retval        otIp6Address        Pointer to requested multicast address
***************************************************************************************************/
otIp6Address *NWKU_GetSpecificMcastAddr(otInstance *pOtInstance, thrMcastAllThrNodes_t type);

/*!*************************************************************************************************
\fn     bool_t NWKU_CmpAddrPrefix6(uint8_t * addr1, uint8_t *addr2, uint32_t prefixLen)
\brief  Compares first "prefixLen" bits of the ipv6 addresses.

\param  [in]  addr1      First prefix to compare
\param  [in]  addr2      Second prefix to compare
\param  [in]  prefixLen  Length in bits to compare

\return       TRUE       If match
\return       FALSE      Otherwise
****************************************************************************************************/
bool_t NWKU_CmpAddrPrefix6(uint8_t * addr1, uint8_t *addr2, uint32_t prefixLen);

/*!*************************************************************************************************
\fn    bool_t NWKU_CmpAddr4(uint32_t destAddr, uint32_t netAddr, uint8_t prefixLen)
\brief Compare two IPv4 addresses using netMask

\param [in]  destAddr   destination address
\param [in]  netAddr    network address
\param [in]  prefixLen  network mask

\return      bool_t     TRUE if match
                        FALSE otherwise
***************************************************************************************************/
bool_t NWKU_CmpAddr4(uint32_t destAddr, uint32_t netAddr, uint8_t prefixLen);

/*!*************************************************************************************************
\fn    bool_t NWKU_MemCmpToVal(uint8_t *pAddr, uint8_t val, uint32_t len)
\brief Compare each octet of a given location to a value.

\param [in]     pAddr      location to be compared
\param [in]     val        reference value
\param [in]     len        length of location to be compared

\return         TRUE       If match
\return         FALSE      Otherwise
***************************************************************************************************/
bool_t NWKU_MemCmpToVal(uint8_t *pAddr, uint8_t val, uint32_t len);

/*!*************************************************************************************************
\fn     bool_t NWKU_BitCmp(uint8_t *pStr1, uint8_t *pStr2, uint8_t startBit, uint8_t stopBit)
\brief  Compare two strings bit by bit

\param  [in]   pStr1            The start address of the first string to be compared
\param  [in]   pStr2            The start address of the second string to be compared
\param  [in]   startBit         The start bit number in the the 2 strings
\param  [in]   stopBit          The stop bit number in the the 2 strings

\return        TRUE             If the strings match
\return        FALSE            If the strings don't match
***************************************************************************************************/
bool_t NWKU_BitCmp(uint8_t *pStr1, uint8_t *pStr2, uint8_t startBit, uint8_t stopBit);

/*!*************************************************************************************************
\fn     bool_t NWKU_IsLLAddrEqual(uint8_t *pFirstLlAddr, uint32_t firstLlAddrSize,
                                  uint8_t *pSecondLlAddr,uint32_t secondLlAddrSize)
\brief  Compare two Link Layer addresses

\param  [in]   pFirstLlAddr      The start address of the first address to be compared
\param  [in]   firstLlAddrSize   The size of the first address to be compared
\param  [in]   pSecondLlAddr     The start address of the second address to be compared
\param  [in]   secondLlAddrSize  The size of the second address to be compared

\return        TRUE              If the Link Layer addresses are the same
\return        FALSE             If the Link Layer addresses are different
***************************************************************************************************/
bool_t NWKU_IsLLAddrEqual(uint8_t *pFirstLlAddr, uint32_t firstLlAddrSize, uint8_t *pSecondLlAddr,
                                    uint32_t secondLlAddrSize);

/*!*************************************************************************************************
\fn     uint64_t NWKU_TransformArrayToValue(uint8_t* pArray, uint32_t nbOfBytes)
\brief  Converts an array to a numeric value.

\param  [in]   pArray          The start address of the array
\param  [in]   nbOfBytes       The length of the data to be converted

\return                        The value converted from the array
****************************************************************************************************/
uint64_t NWKU_TransformArrayToValue(uint8_t* pArray, uint32_t nbOfBytes);

/*!*************************************************************************************************
\fn     void NWKU_TransformValueToArray(uint64_t value, uint8_t* pArray, uint32_t nbOfBytes)
\brief  Converts a numeric value to array.

\param  [in]    value            The value to be converted
\param  [out]   pArray           The start address of the array
\param  [in]    nbOfBytes        The length of the data to be converted
****************************************************************************************************/
void NWKU_TransformValueToArray(uint64_t value, uint8_t* pArray, uint32_t nbOfBytes);

/*!*************************************************************************************************
\fn     uint16_t NWKU_Revert16(uint16_t value)
\brief  Reverts a 16 bit numeric value.

\param  [in]    value       The value to be converted

\return                     The converted value
 ***************************************************************************************************/
uint16_t NWKU_Revert16(uint16_t value);

/*!*************************************************************************************************
\fn     uint32_t NWKU_Revert32(uint32_t value)
\brief  Reverts a 32 bit numeric value.

\param  [in]    value      The value to be converted

\return                    The converted value
 ***************************************************************************************************/
uint32_t NWKU_Revert32(uint32_t value);

/*!*************************************************************************************************
\fn     uint64_t NWKU_Revert64(uint64_t value)
\brief  Reverts a 64 bit numeric value.

\param  [in]    value      The value to be converted

\return                    The converted value
****************************************************************************************************/
uint64_t NWKU_Revert64(uint64_t value);

/*!*************************************************************************************************
\fn     uint16_t NWKU_TransformArrayToUint16(uint8_t* pArray)
\brief  Converts an big endian array to a 16 bit numeric value.

\param  [in]    pArray      The start address of the array

\return                     The converted value
 ***************************************************************************************************/
uint16_t NWKU_TransformArrayToUint16(uint8_t* pArray);

/*!*************************************************************************************************
\fn     uint32_t NWKU_TransformArrayToUint32(uint8_t* pArray)
\brief  Converts an big endian array to a 32 bit numeric value.

\param  [in]    pArray     The start address of the array

\return                    The converted value
****************************************************************************************************/
uint32_t NWKU_TransformArrayToUint32(uint8_t* pArray);

/*!*************************************************************************************************
\fn     uint64_t NWKU_TransformArrayToUint64(uint8_t* pArray)
\brief  Converts an big endian array to a 64 bit numeric value.

\param  [in]    pArray      The start address of the array

\return                     The converted value
****************************************************************************************************/
uint64_t NWKU_TransformArrayToUint64(uint8_t* pArray);

/*!*************************************************************************************************
\fn     void NWKU_TransformUint16ToArray(uint8_t* pArray, uint16_t value)
\brief  Converts a 16 bit numeric value to array.

\param  [in]    value            The value to be converted
\param  [out]   pArray           The start address of the array
 ***************************************************************************************************/
void NWKU_TransformUint16ToArray(uint8_t* pArray, uint16_t value);

/*!*************************************************************************************************
\fn     void NWKU_TransformUint32ToArray(uint8_t* pArray, uint32_t value)
\brief  Converts a 32 bit numeric value to array.

\param  [in]    value            The value to be converted
\param  [out]   pArray           The start address of the array
****************************************************************************************************/
void NWKU_TransformUint32ToArray(uint8_t* pArray, uint32_t value);

/*!*************************************************************************************************
\fn     void NWKU_TransformUint64ToArray(uint8_t* pArray, uint64_t value)
\brief  Converts a 64 bit numeric value to array.

\param  [in]    value            The value to be converted
\param  [out]   pArray           The start address of the array
****************************************************************************************************/
void NWKU_TransformUint64ToArray(uint8_t* pArray, uint64_t value);

/*!*************************************************************************************************
\fn     bool_t NWKU_GetLut8(lut8_t* pLutTable, uint8_t lutTableSize, uint8_t type,
                           uint8_t* pEntryIndex)
\brief  Searches an entry in the lookup table indicated by pLutTable.

\param  [in]    pLutTable     Pointer to the lookup table
\param  [in]    lutTableSize  Lookup table size
\param  [in]    type          Type to find

\param  [out]   pEntryIndex   Index of the entry in case the entry is found

\return         TRUE          Returned when the entry is found
\return         FALSE         Otherwise
 ***************************************************************************************************/
bool_t NWKU_GetLut8(lut8_t* pLutTable, uint8_t lutTableSize, uint8_t type, uint8_t* pEntryIndex);

/*!*************************************************************************************************
\fn     int32_t NWKU_atoi(char *str)
\brief  Converts a string into an integer.

\param  [in]    pStr       Pointer to string

\return                    Integer converted from string.
***************************************************************************************************/
int32_t NWKU_atoi(char *pStr);

/*!*************************************************************************************************
\fn    int64_t NWKU_atol(char *str)
\brief  Converts a string into an long integer.

\param [in]    pStr       pointer to string

\retval     int64_t       integer converted from string.
***************************************************************************************************/
int64_t NWKU_atol(char *pStr);

/*!*************************************************************************************************
\fn     void NWKU_PrintDec(uint64_t value, uint8_t* pString, uint32_t nbOfDigits, bool_t bLeadingZeros)
\brief  Prints in a string decimal values.

\param  [in]         value           Integer value
\param  [in/out]     pString         Pointer to output location
\param  [in]         nbPrintDigits   Number of digits to be printed
\param  [in]         bLeadingZeros   Indicate if leading zeros are put or omitted
                                     TRUE - print leading zeros
                                     FALSE - do not print leading zeros
***************************************************************************************************/
void NWKU_PrintDec(uint64_t value, uint8_t* pString, uint32_t nbPrintDigits, bool_t bLeadingZeros);

/*!*************************************************************************************************
\fn    ptoll(uint8_t *pIn, uint32_t len, llAddr_t *pLlAddr)
\brief Converts a string into an llAddr_t. Presentation to ll function.

\param [in]    pIn      Pointer to the input buffer
\param [in]    len      Size of the input buffer
\param [in]    pLlAddr  Pointer to the start of the allocated llAddr_t structure

\return        TRUE     On success
\return        FALSE    On error
***************************************************************************************************/
bool_t ptoll(uint8_t *pIn, uint32_t len, llAddr_t *pLlAddr);

/*!*************************************************************************************************
\fn     uint32_t NWKU_AsciiToHex(uint8_t* pString, uint32_t strLen)
\brief  Converts a string into hex.

\param  [in]    pString     Pointer to string
\param  [in]    strLen      String length

\return                     Value in hex
***************************************************************************************************/
uint32_t NWKU_AsciiToHex(uint8_t* pString, uint32_t strLen);

/*!*************************************************************************************************
\fn     uint32_t NWKU_AsciiToDec(uint8_t* pString, uint32_t strLen)
\brief  Converts a string into hex.

\param  [in]    pString     Pointer to string
\param  [in]    strLen      String length

\return                     Value in decimal
***************************************************************************************************/
uint32_t NWKU_AsciiToDec(uint8_t* pString, uint32_t strLen);

/*!*************************************************************************************************
\fn     uint8_t NWKU_ByteToDec(uint8_t byte)
\brief  Converts a byte from ASCII to decimal.

\param  [in]    byte     Byte value in ASCII

\return                  Value in decimal
***************************************************************************************************/
uint8_t NWKU_ByteToDec(uint8_t byte);

/*!*************************************************************************************************
\fn     uint8_t NWKU_NibToAscii(int8_t nib, bool_t useUpperCase)
\brief  Converts a nib from hex to ASCII.

\param  [in]    nib            Nib value in hex
\param  [in]    useUpperCase   Flag to specify if conversion is to ASCII uppercase

\return                        Value in ASCII
***************************************************************************************************/
uint8_t NWKU_NibToAscii(int8_t nib, bool_t useUpperCase);

/*!*************************************************************************************************
\fn     void NWKU_HexToAscii(uint8_t* pInputBuff,uint32_t inputBuffLen,uint8_t* pOutputBuffer,
                             uint32_t outputBuffLen, bool_t useUpperCase)
\brief  Converts a byte to ASCII.

\param  [in]    pInputBuff      Pointer to input buffer
\param  [in]    inputBuffLen    Length of the input buffer
\param  [in]    pOutputBuffer   Pointer to output buffer
\param  [in]    outputBuffLen   Length of the output buffer
\param  [in]    useUpperCase    Indicate if the output shall be in upper/lower case
***************************************************************************************************/
void NWKU_HexToAscii(uint8_t* pInputBuff,uint32_t inputBuffLen,uint8_t* pOutputBuffer,
                                uint32_t outputBuffLen, bool_t useUpperCase);

/*!*************************************************************************************************
\fn     uint32_t NWKU_TmrRtcGetElapsedTimeInSeconds(uint32_t timestamp)
\brief  Calculates the time passed in seconds from the provided timestamp.

\param  [in]    timestamp     Timestamp in seconds

\return                       Number of seconds that have passed since the provided timestamp
 ***************************************************************************************************/
uint32_t NWKU_TmrRtcGetElapsedTimeInSeconds(uint32_t timestamp);

/*!*************************************************************************************************
\fn     bool_t NWKU_IsNUmber(char *pString)
\brief  Check if a string is a number.

\param  [in]   pString     Pointer to the string

\return        TRUE        If the string represents a number
\return        FALSE       If the string does not represent a number
***************************************************************************************************/
bool_t NWKU_IsNUmber(char *pString);

/*!*************************************************************************************************
\fn     uint32_t NWKU_GetRandomNoFromInterval(uint32_t startInterval, uint32_t endInterval)
\brief  This function returns a random number from a given interval.

\param  [in]  startInterval   Start value of the interval
\param  [in]  endInterval     End value of the interval

\return                       Random value
***************************************************************************************************/
uint32_t NWKU_GetRandomNoFromInterval(uint32_t startInterval, uint32_t endInterval);

/*!*************************************************************************************************
\fn     uint32_t NWKU_RightRotate(uint32_t val, uint8_t amount)
\brief  This function rotates a 32bit number to the right with an amount of bits.

\param  [in]  val       Number
\param  [in]  amount    Number of bits to rotate

\return                 Result of the rotation
***************************************************************************************************/
uint32_t NWKU_RightRotate(uint32_t val, uint8_t amount);

/*!*************************************************************************************************
\fn     bool_t NWKU_GetBit(uint32_t bitNr, uint8_t* pArray)
\brief  This function returns the value of a bit in an array.

\param  [in]    bitNr           Bit number in the whole array
\param  [in]    pArray          Pointer to the start of the array

\return         TRUE            If the bit is set
\return         FALSE           If the bit is not set
***************************************************************************************************/
bool_t NWKU_GetBit(uint32_t bitNr, uint8_t* pArray);

/*!*************************************************************************************************
\fn     void NWKU_SetBit(uint32_t bitNr, uint8_t* pArray)
\brief  This function sets a bit in an array.

\param  [in]    bitNr           Bit number in the whole array
\param  [in]    pArray          Pointer to the start of the array
***************************************************************************************************/
void NWKU_SetBit(uint32_t bitNr, uint8_t* pArray);

/*!*************************************************************************************************
\fn     void NWKU_ClearBit(uint32_t bitNr, uint8_t* pArray)
\brief  This function clears a bit in an array.

\param  [in]    bitNr           Bit number in the whole array
\param  [in]    pArray          Pointer to the start of the array
***************************************************************************************************/
void NWKU_ClearBit(uint32_t bitNr, uint8_t* pArray);

/*!*************************************************************************************************
\fn     uint32_t NWKU_GetFirstBitValueInRange(uint8_t* pArray, uint32_t lowBitNr, uint32_t
                                              highBitNr, bool_t bitValue)
\brief  This function returns the first bit with value=bitValue in a range in the array.

\param  [in]    pArray          Pointer to the start of the array
\param  [in]    lowBitNr        Starting bit number
\param  [in]    highBitNr       Ending bit number
\param  [in]    bitValue        Bit value

\return         uint32_t        Bit number
***************************************************************************************************/
uint32_t NWKU_GetFirstBitValueInRange(uint8_t* pArray, uint32_t lowBitNr, uint32_t highBitNr, bool_t bitValue);

/*!*************************************************************************************************
\fn     uint32_t NWKU_GetFirstBitValue(uint8_t* pArray, uint32_t arrayBytes, bool_t bitValue)
\brief  This function returns the index of the first bit with value=bitValue.

\param  [in]    pArray          Pointer to the start of the array
\param  [in]    arrayBytes      Number of bytes in the array
\param  [in]    bitValue        Bit value

\return                         Bit value
***************************************************************************************************/
uint32_t NWKU_GetFirstBitValue(uint8_t* pArray, uint32_t arrayBytes, bool_t bitValue);

/*!*************************************************************************************************
\fn     uint32_t NWKU_GetNumOfBits(uint8_t* pArray, uint32_t arrayBytes, bool_t bitValue)
\brief  This function returns number of bits of value bitValue from an array

\param  [in]    pArray          Pointer to the start of the array
\param  [in]    arrayBytes      Number of bytes in the array
\param  [in]    bitValue        Bit value

\return                         Bit value
***************************************************************************************************/
uint32_t NWKU_GetNumOfBits(uint8_t* pArray, uint32_t arrayBytes, bool_t bitValue);

/*!*************************************************************************************************
\fn     uint32_t NWKU_ReverseBits(uint32_t num)
\brief  Reverse bits

\param  [in]  num       The bits to reverse

\return                 The reversed bits
***************************************************************************************************/
uint32_t NWKU_ReverseBits(uint32_t num);

/*!*************************************************************************************************
\fn     uint32_t NWKU_AddTblEntry(uint32_t entry, uint32_t *pTable, uint32_t tableSize)
\brief  This function adds a new entry in a table. The table needs to have uint32_t elements.

\param  [in]    entry       Entry value
\param  [in]    pTable      Pointer to the start of the table
\param  [in]    tableSize   The size of the table

\return                     Entry index or -1(0xFFFFFFFF) in case of error
***************************************************************************************************/
uint32_t NWKU_AddTblEntry(uint32_t entry, uint32_t *pTable, uint32_t tableSize);

/*!*************************************************************************************************
\fn     uint32_t NWKU_GetTblEntry(uint32_t entry, uint32_t *pTable, uint32_t tableSize)
\brief  This function search for an element in a table.

\param  [in]    entry       Entry value
\param  [in]    pTable      Pointer to the start of the table
\param  [in]    tableSize   The size of the table

\return                     Entry index or NULL in case of error
***************************************************************************************************/
uint32_t NWKU_GetTblEntry(uint32_t index, uint32_t *pTable, uint32_t tableSize);

/*!*************************************************************************************************
\fn     void NWKU_SwapArrayBytes(uint8_t* pByte, uint8_t numOfBytes)
\brief  This function swaps the bytes in an array and puts the result in the same array.

\param  [in/out]    pByte        Pointer to the start of the array
\param  [in]        numOfBytes   Size of the array
***************************************************************************************************/
void NWKU_SwapArrayBytes(uint8_t* pByte, uint8_t numOfBytes);

/*!*************************************************************************************************
\fn     void NWKU_GenRand(uint8_t *pRand, uint8_t randLen)
\brief  This function generates a random value in the desired array.

\param  [out]   pRand     Pointer to the start of the output array
\param  [in]    randLen   Size of the array
***************************************************************************************************/
void NWKU_GenRand(uint8_t *pRand, uint8_t randLen);

/*!*************************************************************************************************
\fn     uint64_t NWKU_GetTimestampMs(void)
\brief  Get the timestamp in milliseconds.

\return                Timestamp in milliseconds
***************************************************************************************************/
uint64_t NWKU_GetTimestampMs(void);

/*!*************************************************************************************************
\fn     int8_t NWKU_isArrayGreater(const uint8_t *a, const uint8_t *b, uint8_t length)
\brief  Compare two numbers represented as array.

\param  [in]  a        First array
\param  [in]  b        Second array
\param  [in]  length   How many bytes to compare

\return                0 - are equal
\return                1 - a > b
\return                -1 - b < a
***************************************************************************************************/
int8_t NWKU_isArrayGreater(const uint8_t *a, const uint8_t *b, uint8_t length);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _NETWORK_UTILS_H */

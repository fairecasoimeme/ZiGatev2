/****************************************************************************
 *
 * MODULE:             Mac Scheduler
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5148, JN5142, JN5139].
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
 * Copyright NXP B.V. 2022. All rights reserved
 *
 ***************************************************************************/

#ifndef MAC_SCHED_H
#define MAC_SCHED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jendefs.h"

typedef enum
{
    E_SCHED_OK,
    E_SCHED_FAILED
} sched_status;

typedef enum
{
    E_SCHED_PROTO_OFF,      /* disabled */
    E_SCHED_PROTO_INACTIVE, /* waiting for radio ownership */
    E_SCHED_PROTO_PAUSING,  /* interrupted by switch timer */
    E_SCHED_PROTO_ACTIVE    /* current radio owner */
} proto_state;

typedef enum
{
    E_SCHED_NO_POLICY,  /* Internal use reserved */
    E_SCHED_DUTY_CYCLE, /* Round robin manner with a configured time slice for each protocol */
    E_SCHED_LOCKED,     /* Locking only one protocol until the policy is changed or timeout */
    E_SCHED_PRIORITY,   /* Defines a priority for each protocol and scheduler will select the highest priority to run
                           based on pending operations*/
} sched_policy;

/* Defines to bound the priorities numbers */
#define E_SCHED_MAX_PRIORITY (5)
#define E_SCHED_MIN_PRIORITY (0)

/* Convert ms to radio timer ticks */
#define MILISECONDS_TO_SCHEDTICKS(ms) (((uint32)(ms)*125) / 2)

/* Number of protocols supported by the scheduler */
#define E_SCHED_MAX_PROTO (2)

typedef uint8 proto_tag;                           /* Used for outside world as protocol identifier */
typedef void (*restore_ctx_fn)(void *custom_data); /* Function defined by protocol to restore context */
typedef void (*save_ctx_fn)(void *custom_data);    /* Function defined by protocol to save context */
typedef void (*bbc_isr_handler_fn)(uint32 isr,
                                   void *custom_data); /* Function defined by protocol to handle mac/bbc isr */

struct register_ctx
{
    restore_ctx_fn restore_ctx;
    save_ctx_fn save_ctx;
    bbc_isr_handler_fn bbc_isr_handler;
    void *custom_data;
};

struct sched_policy_duty_cycle
{
    uint32 slice;
};

struct sched_policy_prio
{
    uint8 priority;
};

struct sched_policy_locked
{
    uint32 timeout;
};

struct sched_policy_config
{
    sched_policy policy;
    union
    {
        struct sched_policy_duty_cycle duty_cycle;
        struct sched_policy_locked locked;
        struct sched_policy_prio prio;
    };
};

struct sched_stats {
    uint32_t tx_req_deny;
    uint32_t tx_req_grant;
    uint32_t rx_req_deny;
    uint32_t rx_req_grant;
    /* releases == req_grant */
    uint32_t rel;
    uint32_t abort;
    uint32_t ble2l54;
    uint32_t l542ble;
};

extern void sched_enable(void);
extern void sched_disable(void);
extern bool sched_enabled(void);
void sched_trx_done();

extern sched_status sched_register_proto(proto_tag tag, struct register_ctx *ctx);
extern sched_status sched_unregister_proto(proto_tag tag);
extern sched_status sched_unregister_all(void);

extern proto_state sched_get_state(proto_tag tag);
extern sched_status sched_get_ctx(proto_tag tag, struct register_ctx **ctx);

extern void sched_bbc_int_handler(uint32 isr);

#if defined BUILD_OPT_DYNAMIC_SCHED_PRINTF_DEBUG
extern void sched_debug(void);
#endif

void sched_isr(uint32 *isr);

/* Policy related API */
sched_status sched_set_policy(proto_tag tag, sched_policy policy);
sched_status sched_configure_policy(proto_tag tag, struct sched_policy_config *config);

/* External function to implement the scheduler time slice */
extern void sched_start_timer(uint32 time);
extern void sched_stop_timer(void);
bool_t sched_timer_expired();

/* Implementation depended critical section for scheduler */
extern void enter_sched_critical_section();
extern void exit_sched_critical_section();

sched_status sched_get_stats(struct sched_stats *stats);

/* dynamic mode API: 15.4 + BLE */
typedef enum
{
    E_DYN_SLAVE,            /* 15.4 */
    E_DYN_MASTER,           /* BLE */
    E_DYN_PROTOCOL_COUNT
} teDynProtocol;

typedef enum
{
    E_DYN_OK,
    E_DYN_REFUSED
} teDynStatus;

typedef enum
{
    E_DYN_STATE_OFF,            /* disabled */
    E_DYN_STATE_INACTIVE,       /* waiting for radio ownership */
    E_DYN_STATE_PAUSING,        /* interrupted by switch timer */
    E_DYN_STATE_ACTIVE,         /* current radio owner */
} teDynState;

teDynState eDynGetProtocolState(teDynProtocol proto);
void vDynRequestState(teDynProtocol proto, teDynState state);
void vMac_DynRadioAvailable(uint32 dt);
teDynStatus eMac_DynActivityAdded(uint32 dt);
void vDynStopAll();
void vDynEnableCoex(void *coexReg, void *coexReqAcces, void *coexChangeAccess, void *relAccess, void *changeAccess);

#ifdef __cplusplus
};
#endif

#endif /* _mac_dynamic_sched_h_ */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

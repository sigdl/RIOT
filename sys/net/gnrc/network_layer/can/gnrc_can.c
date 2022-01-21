/*
 * Copyright (C) 2022 Grr <gebbet00@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    
 * @ingroup     
 * @brief       
 *
 * @{
 *
 * @file
 * @brief       
 *
 *
 * @author      Grr <gebbet00@gmail.com>
 */

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/
#include "sched.h"
#include "net/gnrc/can.h"
#include "net/sock/can.h"

#define ENABLE_DEBUG        1
#include "debug.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                 Private Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                          Private Function Prototypes                         *
 *------------------------------------------------------------------------------*/
static void *_event_loop(void *args);
static void process_rcv(void *ptr);

/*------------------------------------------------------------------------------*
 *                                Private Data                                  *
 *------------------------------------------------------------------------------*/
static char _stack[GNRC_CAN_STACK_SIZE + DEBUG_EXTRA_STACKSIZE];

/*------------------------------------------------------------------------------*
 *                               Private Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief CAN Protocol event loop
 */
static void *_event_loop(void *args)
{
    msg_t msg, reply, msg_q[GNRC_CAN_MSG_QUEUE_SIZE];

    (void)args;

    msg_init_queue(msg_q, GNRC_CAN_MSG_QUEUE_SIZE);

    /* preinitialize ACK */
    reply.type = GNRC_NETAPI_MSG_TYPE_ACK;

    /* start event loop */
    while (1) {
        DEBUG("CAN: waiting for incoming message.\n");
        msg_receive(&msg);

        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                DEBUG("CAN: GNRC_NETAPI_MSG_TYPE_RCV received\n");
                process_rcv(msg.content.ptr);
                break;

            case GNRC_NETAPI_MSG_TYPE_SND:
                DEBUG("CAN: GNRC_NETAPI_MSG_TYPE_SND received\n");
                /*_send(msg.content.ptr, true);*/
                break;

            case GNRC_NETAPI_MSG_TYPE_GET:
            case GNRC_NETAPI_MSG_TYPE_SET:
                DEBUG("CAN: reply to unsupported get/set\n");
                reply.content.value = -ENOTSUP;
                msg_reply(&msg, &reply);
                break;

            default:
                break;
        }
    }

    return NULL;
}

/**
 * @brief CAN Protocol event loop
 */
static void process_rcv(void *ptr)
{
    netdev_t           *netdev   = (netdev_t *)ptr;
    socketcan_params_t *scparams = container_of(netdev,   socketcan_params_t, netdev);
    can_netdev_t       *dev      = container_of(scparams, can_netdev_t,       sparams);
    sock_can_t         *sock;


    /*Load first sock of this iface */
    sock = dev->sparams.first_sock;

    /* If no sock */
    if(sock == NULL) {

        /* Return failure */
        return;
    }

    /* Cycle through sock list */
    while(sock != NULL) {

        /*  */

        /*Load next filter of this iface */
        sock = sock->next_sock;
    }

    return;
}

/*------------------------------------------------------------------------------*
 *                                 Public Data                                  *
 *------------------------------------------------------------------------------*/
kernel_pid_t gnrc_can_pid = KERNEL_PID_UNDEF;

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief   Ready the device for initialization through it's netdev interface
 *
 * @param[in]            .
 * @param[in]         
 * @param[in]          
 *                          
 */
kernel_pid_t gnrc_can_init(void)
{
    if (gnrc_can_pid == KERNEL_PID_UNDEF) {
        gnrc_can_pid = thread_create(_stack,
                                      sizeof(_stack),
                                      GNRC_CAN_PRIO,
                                      THREAD_CREATE_STACKTEST,
                                      _event_loop,
                                      NULL,
                                      "can");
    }

    return gnrc_can_pid;
}

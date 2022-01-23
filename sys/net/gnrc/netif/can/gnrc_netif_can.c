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
#include <assert.h>
#include <string.h>

#include "net/gnrc.h"
#include "net/gnrc/netif/can_nd.h"

#define ENABLE_DEBUG 0
#include "debug.h"

#if defined(MODULE_OD) && ENABLE_DEBUG
#include "od.h"
#endif

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                 Private Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                          Private Function Prototypes                         *
 *------------------------------------------------------------------------------*/
static int can_netif_send(gnrc_netif_t *netif, gnrc_pktsnip_t *pkt);
static gnrc_pktsnip_t *can_netif_recv(gnrc_netif_t *netif);


/*------------------------------------------------------------------------------*
 *                                Private Data                                  *
 *------------------------------------------------------------------------------*/
static const gnrc_netif_ops_t can_ops = {
    .init = gnrc_netif_default_init,
    .send = can_netif_send,
    .recv = can_netif_recv,
    .get  = gnrc_netif_get_from_netdev,
    .set  = gnrc_netif_set_from_netdev,
};
extern kernel_pid_t gnrc_can_pid;

/*------------------------------------------------------------------------------*
 *                               Private Functions                              *
 *------------------------------------------------------------------------------*/
static int can_netif_send(gnrc_netif_t *netif, gnrc_pktsnip_t *pkt)
{

    netdev_t *dev = netif->dev;

    (void)dev;
    (void)pkt;

    return 0;
}

static gnrc_pktsnip_t *can_netif_recv(gnrc_netif_t *netif)
{
    int       resp;
    netdev_t *netdev = netif->dev;
    msg_t     msg;


    /* Msg type is RCV */
    msg.type = GNRC_NETAPI_MSG_TYPE_RCV;

    /* Msg content is netif descriptor */
    msg.content.ptr = (void *)netdev;

    /* send message */
    resp = msg_try_send(&msg, gnrc_can_pid);

    if (resp < 1) {
        DEBUG("gnrc_netapi: dropped message to %" PRIkernel_pid " (%s)\n", gnrc_can_pid,
              (resp == 0) ? "receiver queue is full" : "invalid receiver");
    }

    /* Returning null cancells the rest of default pkt processing in cb function */
    return NULL;
}


/*------------------------------------------------------------------------------*
 *                                 Public Data                                  *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief   .
 *
 * @param[in]               .
 * @param[in]               .
 * @param[in]               .
 *                          
 */
int gnrc_netif_can_create(gnrc_netif_t  *netif, 
                          char          *stack,
                          int            stacksize,
                          char           priority,
                          char          *name,
                          netdev_t      *netdev
                         )
{
    return gnrc_netif_create(netif,
                             stack,
                             stacksize,
                             priority,
                             name,
                             netdev,
                            &can_ops
                            );
}

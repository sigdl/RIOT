/*
 * Copyright (C) 2021 Grr <gebbet00@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     sys_auto_init_gnrc_netif
 * @{
 *
 * @file
 * @brief       Auto initialization for SocketCAN MCP2515 devices
 *
 * @author      Grr <gebbet00@gmail.com>
 */

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/
#include "log.h"
#include "mcp2515net.h"
#include "mcp2515net_params.h"
#include "net/gnrc/netif/can.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
/**
 * @brief   Find out how many of these devices we need to care for
 */
#define MCP2515NET_NUM              ARRAY_SIZE(mcp2515net_params)

/**
 * @brief   Define stack parameters for the MAC layer thread
 * @{
 */
#define MCP2515NET_MAC_STACKSIZE    (THREAD_STACKSIZE_DEFAULT)
#ifndef MCP2515NET_MAC_PRIO
#define MCP2515NET_MAC_PRIO         (GNRC_NETIF_PRIO)
#endif

/*------------------------------------------------------------------------------*
 *                                 Private Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                          Private Function Prototypes                         *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Private Data                                  *
 *------------------------------------------------------------------------------*/
/**
 * @brief   Allocate memory for the device descriptors
 * @{
 */
static mcp2515net_t         dev[MCP2515NET_NUM];
static gnrc_netif_t         _netif[MCP2515NET_NUM];

/** @} */

/**
 * @brief   Stacks for the MAC layer threads
 */
static char stack[MCP2515NET_NUM][MCP2515NET_MAC_STACKSIZE];


/*------------------------------------------------------------------------------*
 *                               Private Functions                              *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                 Public Data                                  *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
void auto_init_mcp2515net(void)
{
    for (unsigned i = 0; i < MCP2515NET_NUM; i++) {
        LOG_DEBUG("[auto_init_netif] initializing MCP2515net #%u\n", i);

        /* setup netdev device */
        mcp2515net_setup(&dev[i], &mcp2515net_params[i], &regs[i], i);

        /* Create network interface */
        gnrc_netif_can_create(&_netif[i], stack[i], MCP2515NET_MAC_STACKSIZE,
                                   MCP2515NET_MAC_PRIO, "mcp2515",
                                   &dev[i].netdev);
    }
}

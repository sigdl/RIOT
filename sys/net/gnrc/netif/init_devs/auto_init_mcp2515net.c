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

#include "log.h"
#include "mcp2515net.h"
#include "mcp2515net_params.h"
#include "net/gnrc/netif/ethernet.h"


void auto_init_mcp2515net(void)
{
    for (unsigned i = 0; i < ENC28J60_NUM; i++) {
        LOG_DEBUG("[auto_init_netif] initializing enc28j60 #%u\n", i);

        /* setup netdev device */
        mcp2515net_setup(&dev[i], &enc28j60_params[i], i);
        gnrc_netif_ethernet_create(&_netif[i], stack[i], ENC28J60_MAC_STACKSIZE,
                                   ENC28J60_MAC_PRIO, "enc28j60",
                                   &dev[i].netdev);
    }
}

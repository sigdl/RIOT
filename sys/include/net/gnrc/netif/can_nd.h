/*
 * Copyright (C) 2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup net_gnrc_netif
 * @{
 *
 * @file
 * @brief   CAN adaption for @ref net_gnrc_netif
 *
 * @author  Grr <gebbet00@gmail.com>
 */
#ifndef NET_GNRC_NETIF_CAN_ND_H
#define NET_GNRC_NETIF_CAN_ND_H

#include "net/gnrc/netif.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Creates a CAN network interface
 *
 * @param[out] netif    The interface. May not be `NULL`.
 * @param[in] stack     The stack for the network interface's thread.
 * @param[in] stacksize Size of @p stack.
 * @param[in] priority  Priority for the network interface's thread.
 * @param[in] name      Name for the network interface. May be NULL.
 * @param[in] netdev    Device for the interface.
 *
 * @see @ref gnrc_netif_create()
 *
 * @return  0 on success
 * @return  negative number on error
 */
int gnrc_netif_can_create(gnrc_netif_t  *netif,
                          char          *stack,
                          int            stacksize,
                          char           priority,
                          char          *name,
                          netdev_t      *netdev);

#ifdef __cplusplus
}
#endif

#endif /* NET_GNRC_NETIF_CAN_ND_H */
/** @} */

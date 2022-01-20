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
#include "net/sock/can.h"
#include "net/netif.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                 Private Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                          Private Function Prototypes                         *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Private Data                                  *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                               Private Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief STM32 Socketcan driver Init function
 *
 *
 * @param[in]  
 *
 * @return     
 * @return     
 */

/*------------------------------------------------------------------------------*
 *                                 Public Data                                  *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief   .Create a CAN sock from a socketcan iface
 *
 * @param[in] iface         SocketCAN iface
 * @param[in] sock          Pointer to CAN sock structure
 * @param[in] filter        Pointer to filter to use
 * @param[in] buffer        Pointer to frame buffer
 * @param[in] protocol      CAN upper protocol
 * @param[in] cb            App's call back function
 *                          
 */
int sock_can_create(sock_can_t          *sock,
                    char                *iface_name,
                    can_filter_t        *filter,
                    socketcan_buffer_t  *buffer,
                    socketcan_protocol_t protocol,
                    sock_can_cb_t        cb
                    )
{
    /*netif_t             *ndevice;
    socketcan_params_t  *params;*/

    /* Evaluate parameters */
    assert(sock);
    assert(filter);
    assert(buffer);
    assert(protocol);

    (void)cb;
    (void)iface_name;
#if 0
    /* Get netif device */
    ndevice = netif_get_by_name_buffer(iface_name, CONFIG_NETIF_NAMELENMAX);

    /* Get netif device */
    gnrc_netif_t netiface = container_of(ndevice, gnrc_netif_t, netif);

    /* Get socketcan device */
    can_netdev_t *dev = container_of(netiface, can_netdev_t, netdev);
#endif
    
    return 0;
}

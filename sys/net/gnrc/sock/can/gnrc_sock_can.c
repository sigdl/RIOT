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
#include "can_netdev.h"

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
int sock_can_create(sock_can_t *sock, char *iface_name)
{
    netif_t        *ndevice;
    gnrc_netif_t   *netiface;
    can_netdev_t   *dev;
    uint8_t         i;
    int             resp;
    uint8_t         iface_type;


    /* Evaluate parameters */
    assert(sock);

    /* Get netif device */
    ndevice = netif_get_by_name_buffer(iface_name, CONFIG_NETIF_NAMELENMAX);

    /* If no device found */
    if(ndevice == NULL) {
        return -ENODEV;
    }

    /* Get containers */
    netiface = container_of(ndevice, gnrc_netif_t, netif);
    sock->scparams = container_of(netiface, socketcan_params_t, netif);
    dev = container_of(sock->scparams, can_netdev_t, sparams);

    /* Get iface type */
    iface_type = sock->scparams->iface & CAN_IFACE_TYPE_Msk;

    /* Loop through filters */
    for(i = 0; i < sock->num_filters; i++) {

        switch(iface_type) {

            case CAN_IFACE_TYPE_STM32:
                resp = pcan_filterconf(dev, &sock->filters[i]);
                break;
        }

        /* If there is an error */
        if(resp < 0) {

            /* Return error */
            return resp;
        }
    }



    return 0;
}

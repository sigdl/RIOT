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
 * @brief   Create a CAN sock from a socketcan iface
 *
 * @param[in] sock          Pointer to CAN sock structure
 * @param[in] iface_name    Ascii name of iface
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
    sock_can_t     *last = NULL;

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

        /* Evaluate parameters */
        /*assert(sock->filters[i]);*/

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

    /* Make sure current sock is last */
    sock->next_sock = NULL;

    /* If it's at the beginning of list */
    if(sock->scparams->first_sock == NULL) {

        /* Register current sock as fist sock */
        sock->scparams->first_sock = sock;
    }

    /* If it's NOT at the beginning of list */
    else {

        /* Add sock to list */
        sock_find(sock->scparams, last, SOCK_LAST);
        last->next_sock = sock;
    }
    
    return 0;
}

/**
 * @brief   Search a sock in a sock list
 *
 * @param[in]  scparams   Pointer to SocketCAN device's params structure
 * @param[out] sock       Response pointer
 * @param[in]  mode       Type of search
 *                          
 */
int sock_find(socketcan_params_t *scparams, sock_can_t *sock, sock_find_t type)
{
    switch (type)
    {
        /* Find last sock in dev's list */
        case SOCK_LAST:

                /* Load beginning of sock list */
                sock = scparams->first_sock;

                while(sock->next_sock != NULL) {

                    /* Load next sock in list */
                    sock = sock->next_sock;
                }

                return 0;
            break;
    
        default:
            break;
    }

    return 0;
}
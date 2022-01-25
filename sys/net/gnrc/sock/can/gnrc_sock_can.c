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
#include "can_nd.h"
#include "can_netdev/can_netdev.h"

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
    netif_t      *ndevice;
    gnrc_netif_t *netiface;
    can_nd_t     *dev;
    uint8_t       i;
    int           resp;
    uint8_t       iface_type;
    sock_can_t   *last = NULL;


    /* Evaluate parameters */
    assert(sock);

    /* Get netif device */
    ndevice = netif_get_by_name_buffer(iface_name, CONFIG_NETIF_NAMELENMAX);

    /* If no device found */
    if(ndevice == NULL) {
        return -ENODEV;
    }

    /* Get containers */
    netiface       = container_of(ndevice,        gnrc_netif_t,       netif);
    sock->scparams = container_of(netiface,       socketcan_params_t, netif);
    dev            = container_of(sock->scparams, can_nd_t,           scparams);

    /* Get iface type */
    iface_type = sock->scparams->iface & CAN_IFACE_TYPE_Msk;

    /* Loop through filter banks */
    for(i = 0; i < sock->num_fbanks; i++) {

        /* Evaluate parameters */
        /*assert(sock->filters[i]);*/

        /* Search for filter with same parameters */
        resp = nd_filter_find(sock->scparams, &sock->filterbanks[i], CAN_FILTERFIND_SAME);

        /* If there's a filter with same parameters */
        if(resp == -EEXIST) {
            return resp;
        }

        /* Config num of existing filter banks as filter bank number */
        sock->filterbanks[i].fbank_num = resp;

        switch(iface_type) {

            case CAN_IFACE_TYPE_STM32:
                resp = pcan_filterconf(dev, &sock->filterbanks[i]);
                break;
        }

        /* If there is an error */
        if(resp < 0) {

            /* Return error */
            return resp;
        }
    }

    /* Make sure current sock is last in list */
    sock->next_sock = NULL;

    /* If it's at the beginning of list */
    if(sock->scparams->first_sock == NULL) {

        /* Register current sock as fist sock */
        sock->scparams->first_sock = sock;
    }

    /* If it's NOT at the beginning of list */
    else {

        /* Add sock to list */
        sock_can_find(sock->scparams, last, SOCK_LAST, 0);
        last->next_sock = sock;
    }
    
    return 0;
}

/**
 * @brief   Search some sock in a sock list
 *
 * @param[in]  scparams   Pointer to device's SocketCAN params structure
 * @param[out] sock       Response pointer
 * @param[in]  type       Type of search
 * @param[in]  data       Extra parameter
 *                          
 */
int sock_can_find(socketcan_params_t *scparams, sock_can_t *sock, sock_find_t type, uint8_t data)
{
    switch (type)
    {
        /* Find last sock in list */
        case SOCK_LAST:

            /* Load beginning of sock list */
            sock = scparams->first_sock;

            /* If no sock */
            if(sock == NULL) {

                /* Return failure */
                return -ENODATA;
            }

            while(sock->next_sock != NULL) {

                /* Load next sock in list */
                sock = sock->next_sock;
            }

            return 0;
            break;

        /* Find sock with filter number 'data' */
        case SOCK_FILTER:

            /*Load first sock of this iface */
            sock = scparams->first_sock;

            /* If no sock */
            if(sock == NULL) {

                /* Return failure */
                return -ENODATA;
            }

            /* Cycle through sock list */
            while(sock != NULL) {

                /* Cycle through fbanks */
                for(uint8_t i = 0; i < sock->num_fbanks; i++) {

                    /* if filter is found */
                    if(sock->filterbanks[i].fbank_num == data) {

                        /* Return sucess */
                        return 0;
                    }
                }

                /*Load next filter of this iface */
                sock = sock->next_sock;
            }

            /* Return failure */
            return -ENODATA;
            break;
    
        default:
            break;
    }

    return 0;
}

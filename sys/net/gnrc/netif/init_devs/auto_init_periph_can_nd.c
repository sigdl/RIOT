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
 * @brief       Auto initialization for SocketCAN CAN peripheral device
 *
 * @author      Grr <gebbet00@gmail.com>
 */

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/
#include "log.h"
#include "can_nd.h"
#include "can_nd_params.h"
#include "net/gnrc/netif/can_nd.h"
#include "pm_layered.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
/**
 * @brief   Get number of periph can devices
 */
#define CAN_NETDEV_NUM              ARRAY_SIZE(pcan_ifparams)

/**
 * @brief   Define stack parameters for the MAC layer thread
 * @{
 */
#define CAN_NETDEV_MAC_STACKSIZE    (THREAD_STACKSIZE_DEFAULT)
#ifndef CAN_NETDEV_MAC_PRIO
#define CAN_NETDEV_MAC_PRIO         (GNRC_NETIF_PRIO)
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
 * @brief   Array of device descriptors
 * @{
 */
static can_nd_t pcan_arr[CAN_NETDEV_NUM];

/** @} */

/**
 * @brief   Array of stacks for MAC layer threads
 */
static char pcan_stack[CAN_NETDEV_NUM][CAN_NETDEV_MAC_STACKSIZE];


/*------------------------------------------------------------------------------*
 *                               Private Functions                              *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                 Public Data                                  *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
void auto_init_pcan_nd(void)
{
    uint8_t i;
    uint8_t pm_level = 0;
    char    iface_name[CONFIG_NETIF_NAMELENMAX + 1];


    /* Copy basic name lefting space for device index and NULL */
    strncpy(iface_name, CAN_NETDEV_BNAME, CONFIG_NETIF_NAMELENMAX - 2);

    for (i = 0; i < CAN_NETDEV_NUM; i++) {
        LOG_DEBUG("[auto_init_netif] initializing PERIPH CAN #%u\n", i);

        /* Configure name */
        pcan_arr[i].scparams.name     = (char *)&pcan_name[i];

        /* Config iface type and number */
        pcan_arr[i].scparams.iface    = pcan_iface[i];

        /* Populate pointers */
        pcan_arr[i].scparams.ifparams = &pcan_ifparams[i];
        pcan_arr[i].scparams.timing   = &pcan_timing[i];
        pcan_arr[i].scparams.pm       = &pcan_pm[i];
        pcan_arr[i].eparams           = &pcan_eparams[i];

        /* Add index to name */
        snprintf(iface_name, CONFIG_NETIF_NAMELENMAX, "%s%d", CAN_NETDEV_BNAME, i);

        /* setup netdev device */
        pcan_nd_setup(&pcan_arr[i], i);

        /* Create network interface */
        gnrc_netif_can_create(&pcan_arr[i].scparams.netif,
                               pcan_stack[i],
                               CAN_NETDEV_MAC_STACKSIZE,
                               CAN_NETDEV_MAC_PRIO,
                               pcan_arr[i].scparams.name,
                              &pcan_arr[i].scparams.netdev
                             );

        /* If the configured PM level is above the previous one */
        if(pcan_pm[i].pm_level > pm_level) {

            /* Adopt new higher level for the whole driver */
            pm_level = pcan_pm[i].pm_level;
        }
    }

    /* Set the minimum power level */
    pm_block(pm_level);
}

inline can_nd_t * get_can_netdev(int8_t device)
{
    return &pcan_arr[device];
}
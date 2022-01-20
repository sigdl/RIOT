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
#include "can_netdev.h"
#include "can_netdev_params.h"
#include "net/gnrc/netif/can_netdev.h"
#include "pm_layered.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
/**
 * @brief   Find out how many of these devices we need to care for
 */
#define CAN_NETDEV_NUM              ARRAY_SIZE(can_netdev_params)

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
 * @brief   Allocate memory for the device descriptors
 * @{
 */
static can_netdev_t can_netdev_arr[CAN_NETDEV_NUM];
static gnrc_netif_t can_netdev_netif[CAN_NETDEV_NUM];

/** @} */

/**
 * @brief   Stacks for the MAC layer threads
 */
static char can_netdev_stack[CAN_NETDEV_NUM][CAN_NETDEV_MAC_STACKSIZE];


/*------------------------------------------------------------------------------*
 *                               Private Functions                              *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                 Public Data                                  *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
void auto_init_can_netdev(void)
{
    uint8_t i;
    uint8_t pm_level = 0;
    char    iface_name[CONFIG_NETIF_NAMELENMAX + 1];


    /* Copy basic name lefting space for device index and NULL */
    strncpy(iface_name, CAN_NETDEV_BNAME, CONFIG_NETIF_NAMELENMAX - 2);

    for (i = 0; i < CAN_NETDEV_NUM; i++) {
        LOG_DEBUG("[auto_init_netif] initializing PERIPH CAN #%u\n", i);

        /* If the configured PM level is above the previous one */
        if(can_netdev_params[i].pm.pm_level > pm_level) {

            /* Adopt new higher level for the whole driver */
            pm_level = can_netdev_params[i].pm.pm_level;
        }

        /* Add index to name */
        snprintf(iface_name, CONFIG_NETIF_NAMELENMAX, "%s%d", CAN_NETDEV_BNAME, i);

        /* setup netdev device */
        can_netdev_setup(&can_netdev_arr[i], &can_netdev_params[i], &can_netdev_eparams[i], i);

        /* Create network interface */
        gnrc_netif_can_create(&can_netdev_netif[i],
                               can_netdev_stack[i],
                               CAN_NETDEV_MAC_STACKSIZE,
                               CAN_NETDEV_MAC_PRIO,
                               iface_name,
                              &can_netdev_arr[i].netdev);
    }

    /* Set the minimum power level */
    pm_block(pm_level);
}

inline can_netdev_t * get_can_netdev(int8_t device)
{
    return &can_netdev_arr[device];
}
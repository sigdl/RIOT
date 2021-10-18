/*
 * Copyright (C) 2021 Grr
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_mcp2515 MCP2515
 * @ingroup     drivers_can
 * @brief       SocketCAN Driver for the Microchip MCP2515 can controller.
 *
 * @{
 *
 * @file
 * @brief       Implementation of SocketCAN controller driver.
 *
 *
 * @author      Grr <gebbet00@gmail.com>
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/
#include <errno.h>
#include <string.h>

#include "mutex.h"
#include "xtimer.h"
#include "assert.h"
#include "net/ethernet.h"
#include "net/eui_provider.h"
#include "net/netdev/eth.h"

#include "mcp2515net.h"

#define ENABLE_DEBUG    0
#include "debug.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/


/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/
static int mcp2515_init(netdev_t *netdev);
static void mcp2515_isr(netdev_t *netdev);
static int mcp2515_send(netdev_t *netdev, const iolist_t *iolist);
static int mcp2515_recv(netdev_t *netdev, void *buf, size_t max_len, void *info);
static int mcp2515_get(netdev_t *netdev, netopt_t opt, void *value, size_t max_len);
static int mcp2515_set(netdev_t *netdev, netopt_t opt, const void *value, size_t value_len);


/************************************************************************************
 * Private Data
 ************************************************************************************/
static const netdev_driver_t mcp2515net_driver = {
    .init = mcp2515_init,
    .isr  = mcp2515_isr,
    .send = mcp2515_send,
    .recv = mcp2515_recv,
    .get  = mcp2515_get,
    .set  = mcp2515_set,
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/
/**
 * @brief MCP2515 SocketCAN driver Init function
 *
 * The MCP2515 device is initializad
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int mcp2515_init(netdev_t *netdev)
{
    mcp2515net_t *dev;
    
    dev = (mcp2515net_t *)netdev;
    dev++;
#if 0
    int         res;
    uint8_t     tmp;

    /* get exclusive access of the device */
    mutex_lock(&dev->lock);

    mutex_unlock(&dev->lock);
#endif

    return 0;
}


/**
 * @brief MCP2515 SocketCAN driver ISR
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static void mcp2515_isr(netdev_t *netdev)
{
    netdev++;
}

/**
 * @brief MCP2515 SocketCAN driver send function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int mcp2515_send(netdev_t *netdev, const iolist_t *iolist)
{
    netdev++;
    iolist++;

    return 0;
}

/**
 * @brief MCP2515 SocketCAN driver receive function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int mcp2515_recv(netdev_t *netdev, void *buf, size_t max_len, void *info)
{

    netdev++;
    buf++;
    max_len++;
    info++;

    return 0;
}

/**
 * @brief MCP2515 SocketCAN driver get function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int mcp2515_get(netdev_t *netdev, netopt_t opt, void *value, size_t max_len)
{
    netdev++;
    opt++;
    value++;
    max_len++;

    return 0;
}

/**
 * @brief MCP2515 SocketCAN driver set function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int mcp2515_set(netdev_t *netdev, netopt_t opt, const void *value, size_t value_len)
{
    netdev++;
    opt++;
    value++;
    value_len++;

    return 0;
}


/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
/**
 * @brief MCP2515 SocketCAN driver setup
 *
 * The MCP2515 device is setup
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
void mcp2515net_setup(mcp2515net_t *dev, const mcp2515net_t *config, uint8_t index)
{
    dev++;
    config++;
    index++;
    dev->netdev.driver = &mcp2515net_driver;
#if 0

    dev->p = *params;
    mutex_init(&dev->lock);
    dev->tx_time = 0;

    netdev_register(&dev->netdev, NETDEV_MCP2515, index);
#endif
}

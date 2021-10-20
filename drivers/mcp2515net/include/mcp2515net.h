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
 * @brief       Definition of implementation of SocketCAN controller driver.
 *
 *
 * @author      Grr <gebbet00@gmail.com>
 */

#ifndef MCP2515NET_H
#define MCP2515NET_H

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/
#include "mutex.h"
#include "periph/spi.h"
#include "periph/gpio.h"
#include "net/netdev.h"
#include "can/socketcan.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/
/**
 * @brief MCP2515 general configuration descriptor
 */
typedef struct {
    mutex_t             lock;       /* Exclusive access mutex                   */
    socketcan_params_t  config;     /* SPI bus config                           */
    netdev_t            netdev;     /* Netdev config                            */
} mcp2515net_t;


/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
void mcp2515net_setup(mcp2515net_t *dev, const socketcan_iface_t *config, uint8_t index);

#endif /* MCP2515NET_H */

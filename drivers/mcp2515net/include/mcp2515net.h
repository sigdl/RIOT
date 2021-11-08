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
#define MCP2515NET_FLAGRXB_MASK     0x1
#define MCP2515NET_FLAGRXB_1        0x1

#define MCP2515NET_FLAGTXB_MASK     0x3
#define MCP2515NET_FLAGTXB_SHIFT    0x1
#define MCP2515NET_FLAGTXB_1        0x1

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/
/**
 * @brief MCP2515 registers descriptor
 */
typedef struct {
    uint8_t canctrl;

} mcp2515net_regs_t;

/**
 * @brief MCP2515 general configuration descriptor
 *
 * Flags 
 *
 * 7654 3210
 *       ttr
 *
 * r = Receive buffer
 * 0   RXB0
 * 1   RXB1
 *
 * tt = Trasmit buffer
 * 00   TXB0
 * 01   TXB1
 * 10   TXB2
 *
 */
typedef struct {
    mutex_t                   lock;     /* Exclusive access mutex               */
    uint8_t                   flags;    /* Flags for RXB number, etc            */
    const socketcan_params_t *params;   /* CAN config                           */
    can_frame_t               rxb[2];   /* RX Buffers                           */
    can_frame_t               txb[3];   /* TX Buffers                           */
    mcp2515net_regs_t        *regs;     /* MCP2515 registers                    */
    netdev_t                  netdev;   /* Netdev config                        */
} mcp2515net_t;

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
void mcp2515net_setup(mcp2515net_t *dev, const socketcan_params_t *config, mcp2515net_regs_t *regs, uint8_t index);

#endif /* MCP2515NET_H */

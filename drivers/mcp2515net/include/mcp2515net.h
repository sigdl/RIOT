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
#define MCP2515NET_INT_RXB0         0x01
#define MCP2515NET_INT_RXB1         0x02
#define MCP2515NET_INT_TXB0         0x04
#define MCP2515NET_INT_TXB1         0x08
#define MCP2515NET_INT_TXB2         0x10

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/
/**
 * @brief MCP2515 ouput pins enum
 */
typedef enum {
    MCP2515_RX0BF,
    MCP2515_RX1BF
} mcp2515net_outputs_t;

/**
 * @brief MCP2515 ouput action enum
 */
typedef enum {
    MCP2515_OUTSET,
    MCP2515_OUTRESET,
    MCP2515_TOGGLE
} mcp2515net_outact_t;

/**
 * @brief MCP2515 registers descriptor
 */
typedef struct {
    uint8_t canctrl;
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
    uint8_t bfpctrl;
} mcp2515net_regs_t;

/**
 * @brief MCP2515 general configuration descriptor
 *
 * Flags 
 *
 * 7654 3210
 *    t ttrr
 *    2 1010
 *
 * r0   RXB0
 * r1   RXB1
 * t0   TXB0
 * t1   TXB1
 * t2   TXB2
 *
 */
typedef struct {
    mutex_t                   lock;     /* Exclusive access mutex               */
    uint8_t                   flags;    /* Flags for RXB number, etc            */
    const socketcan_params_t *params;   /* CAN config                           */
    mcp2515net_regs_t        *regs;     /* MCP2515 registers                    */
    can_frame_t               rxb[2];   /* RX Buffers                           */
    can_frame_t               txb[3];   /* TX Buffers                           */
    netdev_t                  netdev;   /* Netdev config                        */
} mcp2515net_t;

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
void mcp2515net_setup(mcp2515net_t *dev, const socketcan_params_t *config, mcp2515net_regs_t *regs, uint8_t index);

#endif /* MCP2515NET_H */

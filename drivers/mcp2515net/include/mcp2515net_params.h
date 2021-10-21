/*
 * Copyright (C) 2021 Grr
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_mcp2515net MCP2515NET
 * @ingroup     drivers_can
 * @brief       SocketCAN Driver for the Microchip MCP2515NET can controller.
 *
 * @{
 *
 * @file
 * @brief       Definition of implementation of SocketCAN controller driver.
 *
 *
 * @author      Grr <gebbet00@gmail.com>
 */

#ifndef MCP2515NET_PARAMS_H
#define MCP2515NET_PARAMS_H

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/
#include "board.h"


/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Default configuration parameters for interface 0
 * @{
 */

 /*     ----- Interface Parameters -----     */
#ifndef MCP2515NET_IFACE0_SPI
#define MCP2515NET_IFACE0_SPI       (SPI_DEV(0))
#endif
#ifndef MCP2515NET_IFACE0_SPI_MODE
#define MCP2515NET_IFACE0_SPI_MODE  SPI_MODE_0
#endif
#ifndef MCP2515NET_IFACE0_SPI_CLK
#define MCP2515NET_IFACE0_SPI_CLK   SPI_CLK_10MHZ
#endif
#ifndef MCP2515NET_IFACE0_CS
#define MCP2515NET_IFACE0_CS        (GPIO_PIN(0, 0))
#endif
#ifndef MCP2515NET_IFACE0_INT
#define MCP2515NET_IFACE0_INT       (GPIO_PIN(0, 1))
#endif
#ifndef MCP2515NET_IFACE0_RESET
#define MCP2515NET_IFACE0_RESET     (GPIO_PIN(0, 2))
#endif

/*     ----- CAN Parameters -----     
 *
 * Default Values 
 *
 *  Nominal Bus Length              100 m
 *  Nominal Bit Rate                500 KHz
 *  Tq per Bit                      16
 *  Nominal Bit Time                2 us
 *  Clock Frequency                 16 MHz
 *  Tosc                            62.5 ns
 *  Tq                              125 ns
 *
 *  
 *  BRP = Tq /(2 * Tosc)            1
 *  
 *  SyncSeg                         1 Tq
 *  PropSeg                         2 Tq
 *  PhaseSeg1                       7 Tq
 *  PhaseSeg2                       6 Tq
 *  Synchronization Jump Width      1 Tq
 *
 */

#ifndef MCP2515NET_TIMING0_NBR      /* Nominal Bit Rate in bits/sec */
#define MCP2515NET_TIMING0_NBR      500000
#endif
#ifndef MCP2515NET_TIMING0_CLOCK    /* Clock for CAN device in Hz */
#define MCP2515NET_TIMING0_CLOCK    (8000000ul)
#endif
#ifndef MCP2515NET_TIMING0_PROP     /* Propagation segment in TQs */
#define MCP2515NET_TIMING0_PROP     2
#endif
#ifndef MCP2515NET_TIMING0_PS1      /* Phase segment 1 in TQs */
#define MCP2515NET_TIMING0_PS1      7
#endif
#ifndef MCP2515NET_TIMING0_PS2      /* Phase segment 2 in TQs */
#define MCP2515NET_TIMING0_PS2      6
#endif
#ifndef MCP2515NET_TIMING0_SJW      /*  */
#define MCP2515NET_TIMING0_SJW      1
#endif

#ifndef MCP2515NET_IFACE0
#define MCP2515NET_IFACE0           { \
                                      .spi      = MCP2515NET_IFACE0_SPI, \
                                      .spi_mode = MCP2515NET_IFACE0_SPI_MODE, \
                                      .spi_clk  = MCP2515NET_IFACE0_SPI_CLK, \
                                      .cs_pin   = MCP2515NET_IFACE0_CS, \
                                      .int_pin  = MCP2515NET_IFACE0_INT, \
                                      .rst_pin  = MCP2515NET_IFACE0_RESET \
                                    }
#endif
#ifndef MCP2515NET_TIMING0
#define MCP2515NET_TIMING0          { \
                                      .clock    = MCP2515NET_TIMING0_CLOCK, \
                                      .prop_seg = MCP2515NET_TIMING0_PROP, \
                                      .ps1      = MCP2515NET_TIMING0_PS1, \
                                      .ps2      = MCP2515NET_TIMING0_PS2, \
                                      .sjw      = MCP2515NET_TIMING0_SJW \
                                    }
#endif
#ifndef MCP2515NET_PARAMS0
#define MCP2515NET_PARAMS0          { \
                                      .timing   = MCP2515NET_TIMING0, \
                                      .iface    = MCP2515NET_IFACE0, \
                                    }
#endif



/**
 * @name    Array of ALL Interfaces' parameters
 * @{
 */
#ifndef MCP2515NET_PARAMS
#define MCP2515NET_PARAMS           { \
                                      MCP2515NET_PARAMS0, \
                                    }
#endif

/** @} */

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/
static socketcan_params_t mcp2515net_params[] = MCP2515NET_PARAMS;

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MCP2515NET_PARAMS_H */

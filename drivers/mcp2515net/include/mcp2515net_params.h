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
#include "mcp2515common.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Default configuration parameters for interface 0
 * @{
 *
 *
 * Default Values 
 *
 *  Clock Frequency (Cf)            16 MHz
 *  Tosc = 1/Cf                     62.5 ns
 *
 *  Nominal Bus Length              100 m
 *  Nominal Bit Rate (NBR)          500 KHz
 *  Nominal Bit Time (NBT) = 1/NBR  2 us
 *
 * Bit Definition
 *
 *  SyncSeg                         1 Tq
 *  PropSeg                         2 Tq
 *  PhaseSeg1                       7 Tq
 *  PhaseSeg2                       6 Tq
 *  Synchronization Jump Width      1 Tq
 *
 *  Tq per Bit (QPB)                16
 *  Tq = NBT / QPB                  125 ns
 *  
 *  BRP = Tq /(2 * Tosc)            1
 *  
 */

/*      -----  Control Parameters  -----        */
#ifndef MCP2515NET_REG0_WAKFIL      /* WAKFIL signal bit */
#define MCP2515NET_REG0_WAKFIL      1       /* Wakeup filter enabled            */
#endif
#ifndef MCP2515NET_REG0_SOF         /* SOF signal bit */
#define MCP2515NET_REG0_SOF         1       /* CLKOUT enabled for SOF           */
#endif

/*       -----  Timing Parameters  -----        */
#ifndef MCP2515NET_TIMING0_NBR      /* Nominal Bit Rate in bits/sec */
#define MCP2515NET_TIMING0_NBR      500000
#endif
#ifndef MCP2515NET_TIMING0_CLOCK    /* Clock for CAN device in Hz */
#define MCP2515NET_TIMING0_CLOCK    (16000000ul)
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

 /*     ----- Operation Parameters -----     */
#ifndef MCP2515NET_OP0_INITMODE     
#define MCP2515NET_OP0_INITMODE     MCP2515_CANCTRL_REQOP_NORMAL
#endif



/* Variable parameters that must be in RAM */
static uint32_t nom_bitrate_0;
static uint32_t r_bitrate_0;
static uint32_t brp_0;


#ifndef MCP2515NET_TIMING0
#define MCP2515NET_TIMING0          { \
                                      .nom_bitrate  = &nom_bitrate_0, \
                                      .r_bitrate    = &r_bitrate_0, \
                                      .brp          = &brp_0, \
                                      .clock        = MCP2515NET_TIMING0_CLOCK, \
                                      .prseg        = MCP2515NET_TIMING0_PROP, \
                                      .phseg1       = MCP2515NET_TIMING0_PS1, \
                                      .phseg2       = MCP2515NET_TIMING0_PS2, \
                                      .sjw          = MCP2515NET_TIMING0_SJW \
                                    }
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
#ifndef MCP2515NET_PARAMS0
#define MCP2515NET_PARAMS0          { \
                                      .timing   = MCP2515NET_TIMING0, \
                                      .iface    = MCP2515NET_IFACE0, \
                                    }
#endif
#ifndef MCP2515NET_REG0
#define MCP2515NET_REG0             { \
                                      .canctrl  = MCP2515_CANCTRL_RESET | \
                                                  ((MCP2515NET_OP0_INITMODE & MCP2515_CANCTRL_REQOP_MASK) \
                                                                           << MCP2515_CANCTRL_REQOP_SHIFT ), \
                                      .cnf3     = ((MCP2515NET_REG0_WAKFIL & MCP2515_CNF3_WAKFIL_MASK) \
                                                                          << MCP2515_CNF3_WAKFIL_SHIFT ) | \
                                                  ((MCP2515NET_REG0_SOF & MCP2515_CNF3_SOF_MASK) \
                                                                       << MCP2515_CNF3_SOF_SHIFT), \
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

/**
 * @name    Array of ALL Interfaces' control parameters
 * @{
 */
#ifndef MCP2515NET_REGS
#define MCP2515NET_REGS             { \
                                      MCP2515NET_REG0, \
                                    }
#endif

/** @} */

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/

static const socketcan_params_t mcp2515net_params[] = MCP2515NET_PARAMS;
static       mcp2515net_regs_t  regs[]              = MCP2515NET_REGS;

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MCP2515NET_PARAMS_H */

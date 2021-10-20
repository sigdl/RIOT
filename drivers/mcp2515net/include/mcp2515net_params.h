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

#ifndef MCP2515NET_IFACE0
#define MCP2515NET_IFACE0           { .spi      = MCP2515NET_IFACE0_SPI, \
                                      .spi_mode = MCP2515NET_IFACE0_SPI_MODE, \
                                      .spi_clk  = MCP2515NET_IFACE0_SPI_CLK, \
                                      .cs_pin   = MCP2515NET_IFACE0_CS, \
                                      .int_pin  = MCP2515NET_IFACE0_INT, \
                                      .rst_pin  = MCP2515NET_IFACE0_RESET }
#endif

#ifndef MCP2515NET_IFACE
#define MCP2515NET_IFACE            MCP2515NET_IFACE0
#endif

/** @} */

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/
static const socketcan_iface_t  mcp2515net_params[] = {
    MCP2515NET_IFACE
};

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MCP2515NET_PARAMS_H */

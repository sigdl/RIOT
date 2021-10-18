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

/********************************************************************************
 * Included Files
 ********************************************************************************/
#include "board.h"


/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the MCP2515NET driver
 * @{
 */
#ifndef MCP2515NET_PARAM_SPI
#define MCP2515NET_PARAM_SPI       (SPI_DEV(0))
#endif
#ifndef MCP2515NET_PARAM_CS
#define MCP2515NET_PARAM_CS        (GPIO_PIN(0, 0))
#endif
#ifndef MCP2515NET_PARAM_INT
#define MCP2515NET_PARAM_INT       (GPIO_PIN(0, 1))
#endif
#ifndef MCP2515NET_PARAM_RESET
#define MCP2515NET_PARAM_RESET     (GPIO_PIN(0, 2))
#endif

#ifndef MCP2515NET_PARAMS
#define MCP2515NET_PARAMS       { .spi = MCP2515NET_PARAM_SPI,     \
                                  .cs_pin = MCP2515NET_PARAM_CS,   \
                                  .int_pin = MCP2515NET_PARAM_INT, \
                                  .rst_pin = MCP2515NET_PARAM_RESET }
#endif
/** @} */


/********************************************************************************
 * Public Types
 ********************************************************************************/
static const  mcp2515net_params_t mcp2515net_params[] = {
    MCP2515NET_PARAMS
};
/** @} */


/********************************************************************************
 * Public Functions
 ********************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* MCP2515NET_PARAMS_H */

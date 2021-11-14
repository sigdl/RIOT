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

#ifndef MCP2515NET_SPI_H
#define MCP2515NET_SPI_H

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
void mcp2515_spi_reset(mcp2515net_t *dev);
void mcp2515_spi_transf(mcp2515net_t *dev, uint8_t *out, uint8_t *in, uint16_t len);
int  mcp2515_spi_bitmod(mcp2515net_t *dev, uint8_t reg, uint8_t mask, uint8_t value);
void mcp2515_spi_toggleclk(mcp2515net_t *dev);
void mcp2515_spi_setoutput(mcp2515net_t *dev, mcp2515net_outputs_t output, mcp2515net_outact_t action);

#endif /* MCP2515NET_SPI_H */

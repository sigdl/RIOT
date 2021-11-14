/*
 * Copyright (C) 2021 Grr <gebbet00@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_mcp2515 MCP2515
 * @ingroup     drivers_can
 * @brief       Microchip MCP2515 SPI functions
 *
 * @{
 *
 * @file
 * @brief       Implementation of SocketCAN controller driver.
 *
 *
 * @author      Grr <gebbet00@gmail.com>
 */

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/
#include <errno.h>
#include <string.h>

#include "mutex.h"
#include "xtimer.h"
#include "irq.h"
#include "assert.h"

#include "periph/gpio.h"
#include "periph/spi.h"
#include "mcp2515net.h"
#include "mcp2515common.h"

#define ENABLE_DEBUG 0
#include "debug.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                 Private Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                          Private Function Prototypes                         *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Private Data                                  *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                               Private Functions                              *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                 Public Data                                  *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief   Reset MCP2515 device through SPI command
 *
 * @param[in] dev           device descriptor
 */
void mcp2515_spi_reset(mcp2515net_t *dev)
{
    spi_acquire(dev->params->iface.spi,
                dev->params->iface.cs_pin,
                dev->params->iface.spi_mode,
                dev->params->iface.spi_clk
               );

    spi_transfer_byte(dev->params->iface.spi,
                      dev->params->iface.cs_pin,
                      false,
                      MCP2515_SPI_RESET
                     );
    spi_release(dev->params->iface.spi);

    /* Load Reset values into registers' image */
    dev->regs->canctrl = MCP2515_CANCTRL_RESET;
}

/**
 * @brief Do a MCP2515 transfer
 *
 * @param[in]  dev          device descriptor
 * @param[in]  in           input buffer
 * @param[out] out          output buffer
 * @param[in]  num          number of bytes to transfer
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
void mcp2515_spi_transf(mcp2515net_t *dev,
                        uint8_t      *out,
                        uint8_t      *in,
                        uint16_t      len)
{
    spi_acquire(dev->params->iface.spi,
                dev->params->iface.cs_pin,
                dev->params->iface.spi_mode,
                dev->params->iface.spi_clk
               );

    spi_transfer_bytes(dev->params->iface.spi,
                      dev->params->iface.cs_pin,
                      false,
                      out,
                      in,
                      len
                     );

    spi_release(dev->params->iface.spi);

}

/**
 * @brief Modify MCP2515 register bits of @p addr
 *
 * @param[in]  dev          device descriptor
 * @param[in]  addr         register addr
 * @param[out] mask         mask of bits to modify
 * @param[in]  value        data bits
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int mcp2515_spi_bitmod(mcp2515net_t *dev,
                       uint8_t       addr,
                       uint8_t       mask,
                       uint8_t       value)
{
    uint8_t bytes[4];

    /* Load sequence of bytes to transfer */
    bytes[0] = MCP2515_SPI_BITMOD;
    bytes[1] = addr;
    bytes[2] = mask;
    bytes[3] = value;

    spi_acquire(dev->params->iface.spi,
                dev->params->iface.cs_pin,
                dev->params->iface.spi_mode,
                dev->params->iface.spi_clk
               );

    spi_transfer_bytes(dev->params->iface.spi,
                       dev->params->iface.cs_pin,
                       false,
                       bytes,
                       NULL,
                       4
                      );

    spi_release(dev->params->iface.spi);

    return 0;
}

/**
 * @brief Toggle MCP2515 CLKOUT pin enable
 *
 * This function toggles clock output
 *
 * Can be used to signal that driver behaviour is OK so far wherever
 * it's needed for fast debug purposes
 *
 * @param[in] dev           device descriptor
 */
void mcp2515_spi_toggleclk(mcp2515net_t *dev)
{
    /* Toggle CLKEN */
    dev->regs->canctrl ^= MCP2515_CANCTRL_CLKEN;

    /* Transfer value */    
    mcp2515_spi_bitmod(dev,
                       MCP2515_CANCTRL,
                       MCP2515_CANCTRL_CLKEN,
                       dev->regs->canctrl
                      );

}

/**
 * @brief Set MCP2515 RXnBF outputs
 *
 * This function sets RXnBF outputs as needed
 *
 * @param[in] dev           device descriptor
 * @param[in] dev           device descriptor
 * @param[in] dev           device descriptor
 */
void mcp2515_spi_setoutput(mcp2515net_t         *dev,
                           mcp2515net_outputs_t  output,
                           mcp2515net_outact_t   action)
{
    uint8_t outputbit;

    if(output == MCP2515_RX0BF) {
        outputbit = MCP2515_BFPCTRL_B0BFS;
    }

    else {
        outputbit = MCP2515_BFPCTRL_B1BFS;
    }

    /* If output must be set */
    if(action == MCP2515_OUTSET) {

        /* Set corresponding bit */
        dev->regs->bfpctrl |= outputbit;
    }

    /* If output must be reset */
    else if(action == MCP2515_OUTRESET) {

        /* Reset corresponding bit */
        dev->regs->bfpctrl &= ~outputbit;
    }

    /* If output must be toggled */
    else {

        /* Toggle corresponding bit */
        dev->regs->bfpctrl ^= outputbit;
    }

    /* Transfer value */    
    mcp2515_spi_bitmod(dev,
                       MCP2515_BFPCTRL,
                       outputbit,
                       dev->regs->bfpctrl
                      );

}

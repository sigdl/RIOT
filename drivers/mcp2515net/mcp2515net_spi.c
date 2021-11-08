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
    dev->regs->canctrl = MCP2515_RESET_CANCTRL;
}

/**
 * @brief   Signal MCP2515 driver if functional so far
 *
 * This function turns off clock output to signal that driver behaviour
 * is OK so far. Can be used wherever it's needed for fast debug purposes
 *
 * @param[in] dev           device descriptor
 */
int mcp2515_spi_signal(mcp2515net_t *dev)
{
    spi_acquire(dev->params->iface.spi,
                dev->params->iface.cs_pin,
                dev->params->iface.spi_mode,
                dev->params->iface.spi_clk
               );

    spi_transfer_byte(dev->params->iface.spi,
                      dev->params->iface.cs_pin,
                      true,
                      MCP2515_SPI_WRITE
                     );
#if 0
    spi_transfer_regs(dev->params->iface.spi,
                      dev->params->iface.cs_pin,
                      MCP2515_CANCTRL,
                      NULL,
                      (void *)buf,
                      len
                     );
#endif
    spi_release(dev->params->iface.spi);

    return 0;
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
 * @brief Read MCP2515 register @p addr
 *
 * @param[in]  dev          device descriptor
 * @param[in]  addr         register addr
 * @param[out] buf          buffer to receive register value
 * @param[in]  len          length of register value
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int mcp2515_spi_readrxb(mcp2515net_t *dev,
                        uint8_t addr,
                        uint8_t *buf,
                        unsigned int len)
{
    spi_acquire(dev->params->iface.spi,
                dev->params->iface.cs_pin,
                dev->params->iface.spi_mode,
                dev->params->iface.spi_clk
               );

    spi_transfer_byte(dev->params->iface.spi,
                      dev->params->iface.cs_pin,
                      true,
                      MCP2515_SPI_READ_RXBUF |
                      ((dev->flags & MCP2515NET_FLAGRXB_MASK) << MCP2515_RXBUF_SHIFT)
                     );

    spi_transfer_regs(dev->params->iface.spi,
                      dev->params->iface.cs_pin,
                      addr,
                      NULL,
                      (void *)buf,
                      len
                     );

    spi_release(dev->params->iface.spi);

    return 0;
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
                       uint8_t addr,
                       uint8_t mask,
                       uint8_t value)
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
 * @brief Get MCP2515 interrupt flags
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int mcp2515_spi_getint(mcp2515net_t *dev)
{
    dev++;
    
    return 0;
}
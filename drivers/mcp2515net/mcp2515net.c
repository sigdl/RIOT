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
 * @brief       SocketCAN Driver for the Microchip MCP2515 can controller.
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
#include "assert.h"

#include "mcp2515common.h"
#include "mcp2515net.h"
#include "mcp2515net_spi.h"

#define ENABLE_DEBUG            0
#include "debug.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
#define TRANSF_BUFFER_SIZE      5

/*------------------------------------------------------------------------------*
 *                                 Private Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                          Private Function Prototypes                         *
 *------------------------------------------------------------------------------*/
static int  mcp2515net_init(netdev_t *netdev);
static void mcp2515net_irq_h(void *arg);
static void mcp2515net_isr(netdev_t *netdev);
static int  mcp2515net_send(netdev_t *netdev, const iolist_t *iolist);
static int  mcp2515net_recv(netdev_t *netdev, void *buf, size_t max_len, void *info);
static int  mcp2515net_get(netdev_t *netdev, netopt_t opt, void *value, size_t max_len);
static int  mcp2515net_set(netdev_t *netdev, netopt_t opt, const void *value, size_t value_len);

/*------------------------------------------------------------------------------*
 *                                Private Data                                  *
 *------------------------------------------------------------------------------*/
static const netdev_driver_t mcp2515net_driver = {
    .init = mcp2515net_init,
    .isr  = mcp2515net_isr,
    .send = mcp2515net_send,
    .recv = mcp2515net_recv,
    .get  = mcp2515net_get,
    .set  = mcp2515net_set,
};

/*------------------------------------------------------------------------------*
 *                               Private Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief MCP2515 SocketCAN driver Init function
 *
 * The MCP2515 device is initializad
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int mcp2515net_init(netdev_t *netdev)
{
    mcp2515net_t *dev = container_of(netdev, mcp2515net_t, netdev);
    int     resp;
    uint8_t in_buf[TRANSF_BUFFER_SIZE];
    uint8_t out_buf[TRANSF_BUFFER_SIZE];
    uint8_t bitquanta;

    /* Lock device */

    /* initialize CS */
    resp = spi_init_cs(dev->params->iface.spi,
                       dev->params->iface.cs_pin
                      );
    if (resp != SPI_OK) {
        DEBUG("MCP2515: error initializing SPI_%i device (code %i)\n",
              dev->params->iface.spi,
              resp
             );
        return -1;
    }

    /* Initialize INT */
    resp = gpio_init_int(dev->params->iface.int_pin,
                         GPIO_IN,
                         GPIO_FALLING,
                         mcp2515net_irq_h,
                         dev
                        );
    if (resp != 0) {
        DEBUG("MCP2515: error setting interrupt pin!\n");
        return -1;
    }

    /* Reset device */
    mcp2515_spi_reset(dev);

    /* Test if device is functional */
    out_buf[0] = MCP2515_SPI_READ;
    out_buf[1] = MCP2515_CANCTRL;
    out_buf[2] = MCP2515_STUFFING;
    mcp2515_spi_transf(dev,
                       out_buf,
                       in_buf,
                       3
                      );
    if (in_buf[2] != MCP2515_RESET_CANCTRL) {
        DEBUG("MCP2515: error communicating with device\n");
        return -1;
    }

    /* Obtain the number of Tq per bit */
    bitquanta = 1 + \
                dev->params->timing.prop + \
                dev->params->timing.ps1  + \
                dev->params->timing.ps2;

    /* Calculate Bitrate Prescaler */
    *dev->params->timing.brp = \
                (int8_t)((float)dev->params->timing.clock / \
                ((float)(2 * bitquanta * *dev->params->timing.nom_bitrate)) - 1);

    /* Configure masks and filters */


    /* Enable interrupts */
    out_buf[0] = MCP2515_SPI_WRITE;
    out_buf[1] = MCP2515_CANINTE;
    out_buf[2] = MCP2515_CANINTE_RX0IE |
                 MCP2515_CANINTE_RX1IE |
                 MCP2515_CANINTE_ERRIE |
                 MCP2515_CANINTE_WAKIE;
    mcp2515_spi_transf(dev,
                       out_buf,
                       in_buf,
                       3
                      );

    return 0;
}

/**
 * @brief MCP2515 SocketCAN IRQ Handler
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 */
static void mcp2515net_irq_h(void *arg)
{
    mcp2515net_t *dev = arg;

    /* Signal event to driver's ISR */
    netdev_trigger_event_isr(&dev->netdev);
}

/**
 * @brief MCP2515 SocketCAN driver ISR
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static void mcp2515net_isr(netdev_t *netdev)
{
    mcp2515net_t *dev = container_of(netdev, mcp2515net_t, netdev);
    uint8_t irq_flags;
    uint8_t in_buf[TRANSF_BUFFER_SIZE];
    uint8_t out_buf[TRANSF_BUFFER_SIZE];

    /* Read interrupt flags */
    out_buf[0] = MCP2515_SPI_READ;
    out_buf[1] = MCP2515_CANINTF;
    out_buf[2] = MCP2515_STUFFING;
    mcp2515_spi_transf(dev,
                       out_buf,
                       in_buf,
                       3
                      );
    irq_flags = out_buf[2];

    /* Reset interrupt flags */
    out_buf[0] = MCP2515_SPI_BITMOD;
    out_buf[1] = MCP2515_CANINTF;
    out_buf[2] = irq_flags;
    out_buf[2] = 0;
    mcp2515_spi_transf(dev,
                       out_buf,
                       in_buf,
                       4
                      );

    /* Process Wake Interrupt */
    if( irq_flags & MCP2515_CANINTF_WAKIF) {
        DEBUG("[MCP2515net] isr: wake up\n");
        netdev->event_callback(netdev, NETDEV_EVENT_WAKEUP);
    }

    /* Process Error Interrupt */
    if( irq_flags & MCP2515_CANINTF_ERRIF) {

    }

    /* Process Message Error Interrupt */
    if( irq_flags & MCP2515_CANINTF_MERRF) {

    }

    /* Process RX0 Interrupt */
    if( irq_flags & MCP2515_CANINTF_RX0IF) {
        DEBUG("[MCP2515NET] isr: RXB0 frame\n");

        /* Erase field before new information */
        dev->flags &= ~MCP2515NET_FLAGRXB_MASK;

        /* Since RXB0 is value 0, nothing extra to be done */

        /* Event callback */
        netdev->event_callback(netdev, NETDEV_EVENT_RX_COMPLETE);
    }

    /* Process RX1 Interrupt */
    if( irq_flags & MCP2515_CANINTF_RX1IF) {
        DEBUG("[MCP2515NET] isr: RXB1 frame\n");

        /* Erase field before new information */
        dev->flags &= ~MCP2515NET_FLAGRXB_MASK; 

        /* Flag RXB1 */
        dev->flags |= 1;
        
        /* Event callback */
        netdev->event_callback(netdev, NETDEV_EVENT_RX_COMPLETE);
    }

    /* Process TX0 Interrupt */
    if( irq_flags & MCP2515_CANINTF_TX0IF) {

        /* Erase field before new information */
        dev->flags &= ~(MCP2515NET_FLAGTXB_MASK << MCP2515NET_FLAGTXB_SHIFT); 

        /* Since TXB0 is value 0, nothing extra to be done */

        /* Event callback */
        netdev->event_callback(netdev, NETDEV_EVENT_TX_COMPLETE);
    }

    /* Process TX1 Interrupt */
    if( irq_flags & MCP2515_CANINTF_TX1IF) {

    }

    /* Process TX2 Interrupt */
    if( irq_flags & MCP2515_CANINTF_TX2IF) {

    }
}

/**
 * @brief MCP2515 SocketCAN driver receive function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int mcp2515net_recv(netdev_t *netdev, void *buf, size_t max_len, void *info)
{
    mcp2515net_t *dev = container_of(netdev, mcp2515net_t, netdev);
    uint8_t buf_num;
    uint8_t in_buf[MCP2515_RXB_BYTES + 1];
    uint8_t out_buf[MCP2515_RXB_BYTES + 1];

    (void)buf;
    (void)max_len;
    (void)info;

    /* Obtain buffer number to work on */
    buf_num = (dev->flags & MCP2515NET_FLAGRXB_MASK);

    /* Construct instruction according to buffer number */
    out_buf[0] = MCP2515_SPI_READ_RXBUF |
                 (buf_num << MCP2515_RXBUF_SHIFT);

    /* Read buffer */
    mcp2515_spi_transf(dev,
                       out_buf,
                       in_buf,
                       MCP2515_RXB_BYTES + 1
                      );

    /* If it's an extended frame */
    if (in_buf[MCP2515_RXBUF_SIDL + 1] & MCP2515_RX_IDE) {

        /* REG+1 because buffer is shifted due to byte 0 being the instruction */
        dev->rxb[buf_num].id = ((uint32_t)in_buf[MCP2515_RXBUF_SIDH + 1] << 21) +
                              (((uint32_t)in_buf[MCP2515_RXBUF_SIDL + 1] & 0xE0) << 13) +
                              (((uint32_t)in_buf[MCP2515_RXBUF_SIDL + 1] & 0x03) << 16) +
                               ((uint32_t)in_buf[MCP2515_RXBUF_EID8 + 1] << 8) +
                                          in_buf[MCP2515_RXBUF_EID0 + 1];
        dev->rxb[buf_num].id |= CAN_FLAG_EFF;
    }

    /* If it's an standard frame */
    else {

        /* REG+1 because buffer is shifted due to byte 0 being the instruction */
        dev->rxb[buf_num].id = ((uint32_t)in_buf[MCP2515_RXBUF_SIDH + 1] << 3) +
                              (((uint32_t)in_buf[MCP2515_RXBUF_SIDL + 1] & 0xE0) >> 5);
    }

    /* Get DLC field */
    dev->rxb[buf_num].dlc = in_buf[MCP2515_RXBUF_DLC + 1];

    /* Get payload */
    memcpy(dev->rxb[buf_num].data, in_buf + 6, dev->rxb[buf_num].dlc);

    return 0;
}

/**
 * @brief MCP2515 SocketCAN driver send function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int mcp2515net_send(netdev_t *netdev, const iolist_t *iolist)
{
    netdev++;
    iolist++;

    return 0;
}

/**
 * @brief MCP2515 SocketCAN driver get function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int mcp2515net_get(netdev_t *netdev, netopt_t opt, void *value, size_t max_len)
{
    netdev++;
    opt++;
    value++;
    max_len++;

    return 0;
}

/**
 * @brief MCP2515 SocketCAN driver set function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int mcp2515net_set(netdev_t *netdev, netopt_t opt, const void *value, size_t value_len)
{
    netdev++;
    opt++;
    value++;
    value_len++;

    return 0;
}

/*------------------------------------------------------------------------------*
 *                                 Public Data                                  *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief   Ready the device for initialization through it's netdev interface
 *
 * @param[in] dev           device descriptor
 * @param[in] params        peripheral configuration to use
 * @param[in]   index       Index of @p params in a global parameter struct array.
 *                          If initialized manually, pass a unique identifier instead.
 */
void mcp2515net_setup(mcp2515net_t *dev,
                      const socketcan_params_t *params,
                      mcp2515net_regs_t *regs,
                      uint8_t index
                     )
{
    /* Load parameters' address */
    dev->params = params;

    /* Load registers' address */
    dev->regs = regs;

    /* Load driver's interface */
    dev->netdev.driver = &mcp2515net_driver;

    /* Initialize mutex */
    mutex_init(&dev->lock);

    /* Register device */
    netdev_register(&dev->netdev, NETDEV_MCP2515, index);
}

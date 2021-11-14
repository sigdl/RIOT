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
    /*out_buf[2] = MCP2515_STUFFING;*/
    mcp2515_spi_transf(dev,
                       out_buf,
                       in_buf,
                       3
                      );

    if (in_buf[2] != MCP2515_CANCTRL_RESET) {
        DEBUG("MCP2515: error communicating with device\n");
        return -1;
    }

    /* Obtain the number of Tq per bit */
    bitquanta = 1 + \
                dev->params->timing.prseg + \
                dev->params->timing.phseg1  + \
                dev->params->timing.phseg2;

    /* Calculate Bitrate Prescaler */
    *dev->params->timing.brp = \
                (int8_t)((float)dev->params->timing.clock / \
                ((float)(2 * bitquanta * *dev->params->timing.nom_bitrate)) - 1);

    /* Configure CNF1 register */
    dev->regs->cnf1 |= (*dev->params->timing.brp & MCP2515_CNF1_BRP_MASK) |
                       ((dev->params->timing.sjw  & MCP2515_CNF1_SJW_MASK) << MCP2515_CNF1_SJW_SHIFT);
    out_buf[0] = MCP2515_SPI_WRITE;
    out_buf[1] = MCP2515_CNF1;
    out_buf[2] = dev->regs->cnf1;
    mcp2515_spi_transf(dev,
                       out_buf,
                       in_buf,
                       3
                      );

    /* Configure CNF2 register */
    dev->regs->cnf2 |= (dev->params->timing.prseg  & MCP2515_CNF2_PRSEG_MASK) |
                      ((dev->params->timing.phseg1 & MCP2515_CNF2_PHSEG1_MASK) << MCP2515_CNF2_PHSEG1_SHIFT);
    out_buf[0] = MCP2515_SPI_WRITE;
    out_buf[1] = MCP2515_CNF2;
    out_buf[2] = dev->regs->cnf2;
    mcp2515_spi_transf(dev,
                       out_buf,
                       in_buf,
                       3
                      );

    /* Configure CNF3 register */
    dev->regs->cnf3 |= dev->params->timing.phseg2 & MCP2515_CNF3_PHSEG2_MASK;
    out_buf[0] = MCP2515_SPI_WRITE;
    out_buf[1] = MCP2515_CNF3;
    out_buf[2] = dev->regs->cnf3;
    mcp2515_spi_transf(dev,
                       out_buf,
                       in_buf,
                       3
                      );

    /* Configure BFPCTRL register for using RXnBF pins as digital outputs */
    dev->regs->bfpctrl |= MCP2515_BFPCTRL_B0BFE | MCP2515_BFPCTRL_B1BFE;
    out_buf[0] = MCP2515_SPI_WRITE;
    out_buf[1] = MCP2515_BFPCTRL;
    out_buf[2] = dev->regs->bfpctrl;
    mcp2515_spi_transf(dev,
                       out_buf,
                       in_buf,
                       3
                      );

    /* Configure masks and filters */


    /* Enable default interrupts */
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

    /* Set operating mode */
    mcp2515_spi_bitmod(dev,
                       MCP2515_CANCTRL,
                       MCP2515_CANCTRL_REQOP_MASK << MCP2515_CANCTRL_REQOP_SHIFT,
                       dev->regs->canctrl
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

    DEBUG("[MCP2515NET] isr: Entering ISR in thread context\n");

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

    /* ---------- Wake Interrupt ---------- */
    if( irq_flags & MCP2515_CANINTF_WAKIF) {
        DEBUG("[MCP2515net] isr: wake up\n");
        netdev->event_callback(netdev, NETDEV_EVENT_WAKEUP);
    }

    /* ---------- Error Interrupt ---------- */
    if( irq_flags & MCP2515_CANINTF_ERRIF) {

    }

    /* ---------- Message Error Interrupt ---------- */
    if( irq_flags & MCP2515_CANINTF_MERRF) {

    }

    /* ---------- RX0 Interrupt ---------- */
    if( irq_flags & MCP2515_CANINTF_RX0IF) {
        DEBUG("[MCP2515NET] isr: RXB0 frame\n");

        /* Flag RXB0 */
        dev->flags |= MCP2515NET_INT_RXB0;
        
        /* Event callback */
        netdev->event_callback(netdev, NETDEV_EVENT_RX_COMPLETE);
    }

    /* ---------- RX1 Interrupt ---------- */
    if( irq_flags & MCP2515_CANINTF_RX1IF) {
        DEBUG("[MCP2515NET] isr: RXB1 frame\n");

        /* Flag RXB1 */
        dev->flags |= MCP2515NET_INT_RXB1;
        
        /* Event callback */
        netdev->event_callback(netdev, NETDEV_EVENT_RX_COMPLETE);
    }

    /* ---------- TX0 Interrupt ---------- */
    if( irq_flags & MCP2515_CANINTF_TX0IF) {

        /* Flag TXB0 */
        dev->flags |= MCP2515NET_INT_TXB0;
        
        /* Event callback */
        netdev->event_callback(netdev, NETDEV_EVENT_TX_COMPLETE);
    }

    /* ---------- TX1 Interrupt ---------- */
    if( irq_flags & MCP2515_CANINTF_TX1IF) {

        /* Flag TXB1 */
        dev->flags |= MCP2515NET_INT_TXB1;
        
        /* Event callback */
        netdev->event_callback(netdev, NETDEV_EVENT_TX_COMPLETE);
    }

    /* ---------- TX2 Interrupt ---------- */
    if( irq_flags & MCP2515_CANINTF_TX2IF) {

        /* Flag TXB2 */
        dev->flags |= MCP2515NET_INT_TXB2;
        
        /* Event callback */
        netdev->event_callback(netdev, NETDEV_EVENT_TX_COMPLETE);
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
    uint8_t dlc;
    uint8_t out_buf[MCP2515_RXB_BYTES + 1];
    uint8_t in_buf[MCP2515_RXB_BYTES + 1];

    (void)buf;
    (void)max_len;
    (void)info;

    /* Cycle thru receive buffers */
    for(buf_num = 0; buf_num < MCP2515_RXBUF_NUM; buf_num++) {

        /* If corresponding buffer's flag is NOT set */
        if(!(1<<buf_num & dev->flags)) {

            /* Jump to next flag */
            continue;
        }

        /* Clear flag */
        dev->flags &= ~(1<<buf_num);

        /* Construct instruction according to buffer number */
        out_buf[0] = MCP2515_SPI_READ_RXBUF | (buf_num << MCP2515_RXBUF_SHIFT);

        /* Read buffer */
        mcp2515_spi_transf(dev,
                           out_buf,
                           in_buf,
                           MCP2515_RXB_BYTES + 1
                          );

        /* Clear previous flags to start fresh */
        dev->rxb[buf_num].flags = 0;

        /* If it's an extended frame */
        if (in_buf[MCP2515_RXBUF_SIDL + 1] & MCP2515_RX_IDE) {

            /* Signal extended frame */
            dev->rxb[buf_num].flags |= CAN_FLAG_IDE_EXT;

            /* REG+1 because buffer is shifted due to byte 0 being the instruction */
            dev->rxb[buf_num].id = ((uint32_t)in_buf[MCP2515_RXBUF_SIDH + 1] << 21) +
                                  (((uint32_t)in_buf[MCP2515_RXBUF_SIDL + 1] & 0xE0) << 13) +
                                  (((uint32_t)in_buf[MCP2515_RXBUF_SIDL + 1] & 0x03) << 16) +
                                   ((uint32_t)in_buf[MCP2515_RXBUF_EID8 + 1] << 8) +
                                              in_buf[MCP2515_RXBUF_EID0 + 1];
        }

        /* If it's an standard frame */
        else {

            /* Signal standard frame */
            dev->rxb[buf_num].flags &= ~CAN_FLAG_IDE_MASK;

            /* REG+1 because buffer is shifted due to byte 0 being the instruction */
            dev->rxb[buf_num].id = ((uint32_t)in_buf[MCP2515_RXBUF_SIDH + 1] << 3) +
                                  (((uint32_t)in_buf[MCP2515_RXBUF_SIDL + 1] & 0xE0) >> 5);
        }

        /* If it is a remote frame */
        if(in_buf[MCP2515_RXBUF_DLC + 1] & MCP2515_RXB0CTRL_RXRTR) {

            /* Store RTR flag */
            dev->rxb[buf_num].flags |=  CAN_FLAG_RTR_REM;

        }

        /* If it's a data frame */
        else {

            /* Get DLC field */
            dlc = in_buf[MCP2515_RXBUF_DLC + 1] & CAN_FLAG_DLC_MASK;

            /* Store DLC value */
            dev->rxb[buf_num].flags |= dlc << CAN_FLAG_DLC_SHIFT;

            /* Get payload */
            memcpy(dev->rxb[buf_num].data, in_buf + 6, dlc);
        }

    }

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

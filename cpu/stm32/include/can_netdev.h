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
#include "board.h"

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
 * @brief STM32 CAN extended parameters
 */
typedef struct {
    CAN_TypeDef     *device;                /**< CAN device registers           */
    uint32_t         master_rcc_enable;     /**< Enable bit for master CAN periph */
    uint32_t         rcc_enable;            /**< Enable bit for current CAN periph */
    uint8_t          start_bank;            /**< CAN2 start bank                */
    uint8_t          ttcm : 1;              /**< Time triggered communication mode */
    uint8_t          abom : 1;              /**< Automatic bus-off management   */
    uint8_t          awum : 1;              /**< Automatic wakeup mode          */
    uint8_t          nart : 1;              /**< No automatic retransmission    */
    uint8_t          rflm : 1;              /**< Receive FIFO locked mode       */
    uint8_t          txfp : 1;              /**< Transmit FIFO priority         */
    uint8_t          lbkm : 1;              /**< Loopback mode                  */
    uint8_t          silm : 1;              /**< Silent mode                    */
#if defined(CPU_FAM_STM32F0)
    uint8_t          irq;                   /**< CAN single IRQ vector          */
#else
    uint8_t          irq_rx0;               /**< RX0 IRQ vector                 */
    uint8_t          irq_rx1;               /**< RX1 IRQ vector                 */
    uint8_t          irq_tx;                /**< TX IRQ vector                  */
    uint8_t          irq_sce;               /**< SCE IRQ vector                 */
#endif

} can_netdev_eparams_t;

/**
 * @brief STM32 CAN general configuration descriptor
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
    mutex_t                     lock;     /**< Exclusive access mutex           */    
    uint8_t                     flags;    /**< Flags for RXB number, etc        */
    const socketcan_params_t   *params;   /**< CAN config                       */
    const can_netdev_eparams_t *eparams;  /**< CAN extra config                 */

    can_frame_t                 rxb[2];   /**< RX Buffers                       */
    can_frame_t                 txb[3];   /**< TX Buffers                       */
    netdev_t                    netdev;   /**< Netdev config                    */
} can_netdev_t;

extern const uint8_t can_netdev_num;

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
void can_netdev_setup(can_netdev_t *dev,
                      const socketcan_params_t   *params,
                      const can_netdev_eparams_t *eparams,
                      uint8_t index
                     );

#endif /* MCP2515NET_H */

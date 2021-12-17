/*
 * Copyright (C) 2021 Grr
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    cpu_stm32 socketcan
 * @ingroup     cpu_stm32
 * @brief       SocketCAN Driver for the STM32 Socketcan.
 *
 * @{
 *
 * @file
 * @brief       Definition of implementation of SocketCAN controller driver.
 *
 *
 * @author      Grr <gebbet00@gmail.com>
 */

#ifndef CAN_NETDEV_H
#define CAN_NETDEV_H

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
#define CAN_NETDEV_SFF_SHIFT    21
#define CAN_NETDEV_EFF_SHIFT    3
#define CAN_NETDEV_FMP_MASK     0x03
#define CAN_NETDEV_FOVR_MASK    0x10
#define CAN_NETDEV_RFOM_MASK    0x20

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/
typedef enum {
    CAN_CONFMODE_OP,                        /**< Normal operation mode          */
    CAN_CONFMODE_NTEST                      /**< Network diagnostic mode        */
} can_confmode_t;

/**
 * @brief STM32 CAN extended parameters
 */
typedef struct {
    CAN_TypeDef     *device;                /**< CAN device registers           */

#if defined(CPU_FAM_STM32F0)
    uint8_t          irq;                   /**< CAN single IRQ vector          */
#else
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
    netdev_t                    netdev;   /**< Netdev config                    */
} can_netdev_t;

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
void can_netdev_setup(can_netdev_t *dev,
                      const socketcan_params_t   *params,
                      const can_netdev_eparams_t *eparams,
                      uint8_t index
                     );
int can_netdev_bconfig(can_netdev_t *dev, can_confmode_t mode);
int can_netdev_opconfig(can_netdev_t *dev);
can_netdev_t * get_can_netdev(uint8_t device);
#endif /* CAN_NETDEV_H */

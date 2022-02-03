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

#ifndef CAN_ND_H
#define CAN_ND_H

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
/**
 * @brief Basic name less than CONFIG_NETIF_NAMELENMAX - 2 to leave space for index
 * 
 */
#define CAN_NETDEV_BNAME        "pcan"

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
    uint8_t          start_bank;            /**< CAN2 filter bank start         */
    uint8_t          ttcm     : 1;          /**< Time triggered communication mode */
    uint8_t          abom     : 1;          /**< Automatic bus-off management   */
    uint8_t          awum     : 1;          /**< Automatic wakeup mode          */
    uint8_t          nart     : 1;          /**< No automatic retransmission    */
    uint8_t          rflm     : 1;          /**< Receive FIFO locked mode       */
    uint8_t          txfp     : 1;          /**< Transmit FIFO priority         */
    uint8_t          lbkm     : 1;          /**< Loopback mode                  */
    uint8_t          silm     : 1;          /**< Silent mode                    */
    uint8_t          irq_rx0;               /**< RX0 IRQ vector                 */
    uint8_t          irq_rx1;               /**< RX1 IRQ vector                 */
    uint8_t          irq_tx;                /**< TX  IRQ vector                 */
    uint8_t          irq_sce;               /**< SCE IRQ vector                 */
#endif /* CPU_FAM_STM32F0 */
} can_nd_eparams_t;

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
    mutex_t          lock;                /**< Exclusive access mutex           */    
    uint8_t          flag_rx0 : 1;        /**< Signals RX0 interrupt occured    */
    uint8_t          flag_rx1 : 1;        /**< Signals RX1 interrupt occured    */
    uint8_t          flag_tx  : 1;        /**< Signals TX  interrupt occured    */
    uint8_t          flag_sce : 1;        /**< Signals SCE interrupt occured    */
    socketcan_params_t    scparams;       /**< CAN config                       */
    can_nd_eparams_t     *eparams;        /**< CAN extra config                 */
} can_nd_t;

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
void pcan_nd_setup(can_nd_t *dev, uint8_t index);
int  pcan_nd_basicconf(can_nd_t *dev, can_confmode_t mode);
int  pcan_nd_opconf(can_nd_t *dev);
int  pcan_filterconf(can_nd_t *dev, socketcan_filterbank_t *filter);
can_nd_t * get_can_netdev(int8_t device);
#endif /* CAN_ND_H */

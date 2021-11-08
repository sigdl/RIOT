/*
 * Copyright (C) 2016 Grr <gebbet00@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for
 * more details.
 */

/**
 * @defgroup    drivers_socketcan SocketCAN device driver interface
 * @ingroup     drivers_socketcan
 * @brief       Definitions for low-level SocketCAN driver interface
 * @{
 *
 * This is the SocketCAN controller generic driver interface
 *
 * @file
 * @brief       Definitions for low-level SocketCAN driver interface
 *
 * @author      Grr <gebbet00@gmail.com>
 */

#ifndef CAN_SOCKETCAN_H
#define CAN_SOCKETCAN_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/
#include "mutex.h"
#include "periph/spi.h"
#include "periph/gpio.h"
#include "net/netdev.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
/**
 * @brief Max data length for a CAN frame
 */
#define CAN_PAYLOAD             (8)
/**
 * @name CAN_ID flags and masks
 * @{
 */
/* special address description flags for the CAN_ID */
#define CAN_FLAG_EFF            (0x80000000U) /**< EFF/SFF is set in the MSB    */
#define CAN_FLAG_RTR            (0x40000000U) /**< remote transmission request  */
#define CAN_FLAG_ERR            (0x20000000U) /**< error message frame          */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK            (0x000007FFU) /**< standard frame format (SFF)  */
#define CAN_EFF_MASK            (0x1FFFFFFFU) /**< extended frame format (EFF)  */
#define CAN_ERR_MASK            (0x1FFFFFFFU) /**< omit EFF, RTR, ERR flags     */
/** @} */

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/
/**
 * @brief   CAN Frame types
 */
 enum ca_frame_types {
     CAN_FRAME_STANDARD,
     CAN_FRAME_EXTENDED
 };

/**
 * @brief   Definition for CAN bittiming struct
 */
typedef struct {
    uint32_t *nom_bitrate;      /**< Nominal Bit rate in bits/sec               */
    uint32_t *r_bitrate;        /**< Real Bit rate in bits/sec                  */
    uint32_t *brp;              /**< Bit-rate prescaler                         */
    uint32_t clock;             /**< Clock for CAN device in Hz                 */
    uint32_t tq;                /**< Time quanta (TQ) in nanoseconds            */
    uint8_t  prseg;             /**< Propagation segment in TQs                 */
    uint8_t  phseg1;            /**< Phase segment 1 in TQs                     */
    uint8_t  phseg2;            /**< Phase segment 2 in TQs                     */
    uint8_t  sjw;               /**< Synchronisation jump width in TQs          */
} socketcan_timing_t;

/**
 * @brief   Definition for CAN interface struct
 */
typedef struct {
    spi_t       spi;            /**< Interface for SPI devices                  */
    spi_mode_t  spi_mode;       /**< SPI mode                                   */
    spi_clk_t   spi_clk;        /**< SPI clock frequency                        */
    gpio_t      cs_pin;         /**< CS pin                                     */
    gpio_t      int_pin;        /**< INT pin                                    */
    gpio_t      rst_pin;        /**< RST pin                                    */
    gpio_t      rx_pin;         /**< RX pin                                     */
    gpio_t      tx_pin;         /**< TX pin                                     */
    gpio_af_t   af;             /**< Alternate pin function to use              */
} socketcan_iface_t;

/**
 * @brief   Definition for CAN parameters struct
 */
typedef struct {
    uint8_t             frame_type;  /**< CAN frame type                        */
    socketcan_timing_t  timing; /**< CAN timing parameters                      */
    socketcan_iface_t   iface;  /**< CAN interface parameters                   */
} socketcan_params_t;

/**
 * @brief   Definition for CAN frame struct
 */
typedef struct {
    uint32_t id;                /**< CAN_ID + EFF/RTR/ERR flags                 */
    uint8_t  dlc;               /**< Payload length in bytes                    */
    uint8_t  __pad;             /**< padding                                    */
    uint8_t  __res0;            /**< reserved / padding                         */
    uint8_t  __res1;            /**< reserved / padding                         */
    uint8_t  data[CAN_PAYLOAD] __attribute__((aligned(8)));
} can_frame_t;

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* CAN_SOCKETCAN_H */
/** @} */

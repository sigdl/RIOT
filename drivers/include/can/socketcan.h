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

/** @} */

/* CAN frame flags
 *
 * 7654 3210
 * dddd  tri
 *
 *  i = ID Extension
 *  0   Standard frame
 *  1   Extended frame
 *
 *  r = Remote Trasmit Request
 *  0   data frame
 *  1   remote frame
 *
 *  t = Trasmit Global Time
 *  0   No timestamp
 *  1   Timestamp trasmitted
 *
 *  dddd = Data Length Code
 *  0 - 8
 *
 */
#define CAN_FLAG_IDE_MASK       0x01
#define CAN_FLAG_IDE_SHIFT      0x00
#define CAN_FLAG_IDE_STD        0x00
#define CAN_FLAG_IDE_EXT        0x01

#define CAN_FLAG_RTR_MASK       0x02
#define CAN_FLAG_RTR_SHIFT      0x01
#define CAN_FLAG_RTR_DAT        0x00
#define CAN_FLAG_RTR_REM        0x01

#define CAN_FLAG_TGT_MASK       0x01
#define CAN_FLAG_TGT_SHIFT      0x02
#define CAN_FLAG_TGT_NO         0x00
#define CAN_FLAG_TGT_YES        0x01

#define CAN_FLAG_DLC_MASK       0x0F
#define CAN_FLAG_DLC_SHIFT      0x04

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/
typedef enum {
    CAN_OPMODE_INIT,
    CAN_OPMODE_NORMAL,
    CAN_OPMODE_SLEEP,
} can_opmode_t;

/**
 * @brief   CAN Frame types
 */
enum ca_frame_types {
    CAN_FRAME_STANDARD,
    CAN_FRAME_EXTENDED
};

/**
 * @brief   Definition for CAN bittiming struct
 * 
 *                   Nominal bit time (nbt) composed of 8 time quantum (tq)
 * |<------------------------------------------------------------------------------------->|
 * |                                                                                       |
 * +----------+----------+-------------------------------------------+---------------------+
 * | SYNC_SEG | PROP_SEG |                PHASE_SEG_1                |     PHASE_SEG_2     |
 * +----------+----------+-------------------------------------------+---------------------+
 * |                                                                 ^                     |
 * |                                                    Sample point | at 75%              |
 * |----------|----------|----------|----------|----------|----------|----------|----------|
 * |   Time quanta                                                 6 | 2                   |
 *
 * Synchronization segment = always 1 tq
 * 
 *                SYNC_SEG + PROP_SEG + PHASE_SEG1
 * Sample point = --------------------------------
 *                             nbt
 *
 * tseg1 = PROP_SEG + PHASE_SEG_1
 * tseg2 = PHASE_SEG_2
 * tseg  = tseg1 + tseg2
 * nbt   = tseg + SYNC_SEG
 *
 */

/**
 * @brief   Definition for CAN frame struct
 *      
 */
typedef struct {
    uint32_t id;                /**< CAN ID                                     */
    uint8_t  flags;             /**< CAN frame flags                            */
    uint8_t  data[CAN_PAYLOAD] __attribute__((aligned(8)));
} can_frame_t;

typedef struct {
    uint8_t      rxbuf_num;     /**< Num of RX buffer                           */
    uint8_t     *rxbuf_wr;      /**< Pointer of last available RX frame         */
    uint8_t     *rxbuf_rd;      /**< Pointer of last processed RX frame         */
    can_frame_t *rxbuf;         /**< RX frame circular buffer                   */
    uint8_t      txbuf_num;     /**< Num of TX buffer                           */
    uint8_t     *txbuf_cnt;     /**< Counter to next available TX buffer        */
    can_frame_t *txbuf;         /**< TX Buffers                                 */
} socketcan_buffer_t;

typedef struct {
    uint32_t    *nom_bitrate;   /**< Nominal Bit rate in bits/sec               */
    uint32_t    *r_bitrate;     /**< Real Bit rate in bits/sec                  */
    uint32_t    *brp;           /**< Bit-rate prescaler                         */
    uint32_t    clock;          /**< Clock for CAN device in Hz                 */
    uint32_t    tq;             /**< Time quanta (TQ) in nanoseconds            */
    uint8_t     prseg;          /**< Propagation segment in TQs                 */
    uint8_t     phseg1;         /**< Phase segment 1 in TQs                     */
    uint8_t     phseg2;         /**< Phase segment 2 in TQs                     */
    uint8_t     sjw;            /**< Synchronisation jump width in TQs          */
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
    gpio_af_t   af_op;          /**< Alt pin function for normal operation      */
    gpio_af_t   af_ndiag;       /**< Alt pin function for network diagnostics   */
} socketcan_iface_t;

/**
 * @brief   Definition for CAN power management struct
 */
typedef struct {
    uint8_t     pm_level;       /**< PM block level                             */
} socketcan_pm_t;

/**
 * @brief   Definition for CAN parameters struct
 */
typedef struct {
    socketcan_timing_t  timing;     /**< CAN timing parameters                  */
    socketcan_iface_t   iface;      /**< CAN interface parameters               */
    socketcan_buffer_t  buffers;    /**< CAN buffers                            */
    socketcan_pm_t      pm;         /**< CAN power management parameters        */
} socketcan_params_t;

/**
 * @brief   Definition for CAN identification struct
 */
typedef struct {
    uint32_t            id;     /**< Full CAN ID                                */
} can_id_t;

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* CAN_SOCKETCAN_H */
/** @} */
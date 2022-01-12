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
 * @brief       SocketCAN Driver for the STM32 CAN peripheral.
 *
 * @{
 *
 * @file
 * @brief       Definition of implementation of SocketCAN controller driver.
 *
 *
 * @author      Grr <gebbet00@gmail.com>
 */

#ifndef CAN_NETDEV_PARAMS_H
#define CAN_NETDEV_PARAMS_H

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/
#include "board.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Default configuration parameters for interface 0
 * @{
 *
 *
 * Default Values 
 *
 *  Clock Frequency (Cf)              CLOCK_APB1
 *  Tosc = 1/Cf                       62.5 ns
 *
 *  Nominal Bus Length                100 m
 *  Nominal Bit Rate (NBR)            500 KHz
 *  Nominal Bit Time (NBT) = 1/NBR    2 us
 *
 * Bit Definition
 *
 *  SyncSeg                           1 Tq
 *  PropSeg                           2 Tq
 *  PhaseSeg1                         7 Tq
 *  PhaseSeg2                         6 Tq
 *  Synchronization Jump Width        1 Tq
 *
 *  Tq per Bit (QPB)                  16
 *  Tq = NBT / QPB = 1 / (NBR * QPB)  125 ns
 *
 *  BRP = Tq / Tosc - 1 = Cf / (NBR * QPB) - 1
 *  
 */

/*      -----       Extra Parameters      -----        */
#if defined(CPU_FAM_STM32F0)
  #ifndef CAN_NETDEV0_DEVICE          /* Device registers base address          */
  #define CAN_NETDEV0_DEVICE          CAN
  #endif
#else /* CPU_FAM_STM32F0 */
  #ifndef CAN_NETDEV0_DEVICE
  #define CAN_NETDEV0_DEVICE          CAN1
  #endif

  #ifndef CAN_NETDEV0_MRCCEN          /* Master channel RCC Enable Bit          */
  #define CAN_NETDEV0_MRCCEN          RCC_APB1ENR_CAN1EN
  #endif
  #ifndef CAN_NETDEV0_RCCEN           /* Slave channel RCC Enable Bit           */
  #define CAN_NETDEV0_RCCEN           RCC_APB1ENR_CAN1EN
  #endif
  #ifndef CAN_NETDEV0_SBANK           /* CAN2 start bank                        */
  #define CAN_NETDEV0_SBANK           14
  #endif
  #ifndef CAN_NETDEV0_TTCM            /* Time Triggered Communication Mode      */
  #define CAN_NETDEV0_TTCM            0
  #endif
  #ifndef CAN_NETDEV0_ABOM            /* Automatic bus-off management           */
  #define CAN_NETDEV0_ABOM            0
  #endif
  #ifndef CAN_NETDEV0_AWUM            /* Automatic wakeup mode                  */
  #define CAN_NETDEV0_AWUM            0
  #endif
  #ifndef CAN_NETDEV0_NART            /* No automatic retransmissio             */
  #define CAN_NETDEV0_NART            0
  #endif
  #ifndef CAN_NETDEV0_RFLM            /* Receive FIFO locked mode               */
  #define CAN_NETDEV0_RFLM            0
  #endif
  #ifndef CAN_NETDEV0_TXFP            /* Transmit FIFO priority                 */
  #define CAN_NETDEV0_TXFP            0
  #endif
  #ifndef CAN_NETDEV0_LBKM            /* Loopback mode                          */
  #define CAN_NETDEV0_LBKM            0
  #endif
  #ifndef CAN_NETDEV0_SILM            /* Silent mode                            */
  #define CAN_NETDEV0_SILM            0
  #endif

#endif


#if defined(CPU_FAM_STM32F0)
  #ifndef CAN_NETDEV0_IRQ             /* CAN IRQ                                */
  #define CAN_NETDEV0_IRQ             CEC_CAN_IRQn
  #endif
#else /* CPU_FAM_STM32F0 */
  #ifndef CAN_NETDEV0_IRQRX0          /* CAN RX0 IRQ                            */
  #define CAN_NETDEV0_IRQRX0          CAN1_RX0_IRQn
  #endif
  #ifndef CAN_NETDEV0_IRQRX1          /* CAN RX1 IRQ                            */
  #define CAN_NETDEV0_IRQRX1          CAN1_RX1_IRQn
  #endif
  #ifndef CAN_NETDEV0_IRQTX           /* CAN TX IRQ                             */
  #define CAN_NETDEV0_IRQTX           CAN1_TX_IRQn
  #endif
  #ifndef CAN_NETDEV0_IRQSCE          /* CAN RX0 IRQ                            */
  #define CAN_NETDEV0_IRQSCE          CAN1_SCE_IRQn
  #endif
#endif

/*      -----  Power Management Parameters  -----        */
#ifndef CAN_NETDEV0_PMLEVEL           /* PM Level to block                      */
#define CAN_NETDEV0_PMLEVEL           2
#endif

/*      -----      Control Parameters       -----        */
#ifndef CAN_NETDEV0_REG0_WAKFIL       /* WAKFIL signal bit */
#define CAN_NETDEV0_REG0_WAKFIL       1       /* Wakeup filter enabled          */
#endif
#ifndef CAN_NETDEV0_REG0_SOF          /* SOF signal bit */
#define CAN_NETDEV0_REG0_SOF          1       /* CLKOUT enabled for SOF         */
#endif

/*      -----      Timing Parameters        -----        */
#ifndef CAN_NETDEV0_TIMING_NBR        /* Nominal Bit Rate in bits/sec           */
#define CAN_NETDEV0_TIMING_NBR        50000
#endif
#ifndef CAN_NETDEV0_TIMING_CLOCK      /* Clock for CAN device in Hz             */
#define CAN_NETDEV0_TIMING_CLOCK      CLOCK_APB1
#endif
#ifndef CAN_NETDEV0_TIMING_PROP       /* Propagation segment in TQs             */
#define CAN_NETDEV0_TIMING_PROP       2
#endif
#ifndef CAN_NETDEV0_TIMING_PS1        /* Phase segment 1 in TQs                 */
#define CAN_NETDEV0_TIMING_PS1        7
#endif
#ifndef CAN_NETDEV0_TIMING_PS2        /* Phase segment 2 in TQs                 */
#define CAN_NETDEV0_TIMING_PS2        6
#endif
#ifndef CAN_NETDEV0_TIMING_SJW        /* Synchronization Jump Width             */
#define CAN_NETDEV0_TIMING_SJW        1
#endif

 /*     -----     Interface Parameters      -----     */
#ifndef CAN_NETDEV0_IFACE_RXPIN
#define CAN_NETDEV0_IFACE_RXPIN       GPIO_PIN(PORT_D, 0)
#endif
#ifndef CAN_NETDEV0_IFACE_TXPIN
#define CAN_NETDEV0_IFACE_TXPIN       GPIO_PIN(PORT_D, 1)
#endif
#ifndef CAN_NETDEV0_IFACE_AF_OP       /* AF for normal operation                */
#define CAN_NETDEV0_IFACE_AF_OP       GPIO_AF9
#endif
#ifndef CAN_NETDEV0_IFACE_AF_NDIAG    /* AF for network diagnostics             */
#define CAN_NETDEV0_IFACE_AF_NDIAG    GPIO_AF0
#endif

 /*     -----      Buffer Parameters       -----     */
#ifndef CAN_NETDEV0_BUFFER_RXNUM      /* Number of RX frame buffers             */
#define CAN_NETDEV0_BUFFER_RXNUM      3
#endif


#if 0
 /*     -----     Operation Parameters      -----     */
#ifndef CAN_NETDEV0_OP0_INITMODE      
#define CAN_NETDEV0_OP0_INITMODE      MCP2515_CANCTRL_REQOP_NORMAL
#endif
#endif


/* Variable parameters that must be in RAM */
static uint32_t     nom_bitrate_0 = CAN_NETDEV0_TIMING_NBR;
static uint32_t     r_bitrate_0;
static uint32_t     brp_0;

#ifndef CAN_NETDEV0_TIMING
#define CAN_NETDEV0_TIMING            { \
                                        .nom_bitrate  = &nom_bitrate_0, \
                                        .r_bitrate    = &r_bitrate_0, \
                                        .brp          = &brp_0, \
                                        .clock        = CAN_NETDEV0_TIMING_CLOCK, \
                                        .prseg        = CAN_NETDEV0_TIMING_PROP, \
                                        .phseg1       = CAN_NETDEV0_TIMING_PS1, \
                                        .phseg2       = CAN_NETDEV0_TIMING_PS2, \
                                        .sjw          = CAN_NETDEV0_TIMING_SJW \
                                      }
#endif

#ifndef CAN_NETDEV0_IFACE
#define CAN_NETDEV0_IFACE             { \
                                        .rx_pin       = CAN_NETDEV0_IFACE_RXPIN, \
                                        .tx_pin       = CAN_NETDEV0_IFACE_TXPIN, \
                                        .af_op        = CAN_NETDEV0_IFACE_AF_OP, \
                                        .af_ndiag     = CAN_NETDEV0_IFACE_AF_NDIAG \
                                      }
#endif

/* Variable parameters that must be in RAM */
static uint8_t      rxbuf_0_wr = 0;
static uint8_t      rxbuf_0_rd = 0;
static can_frame_t  rxbuf_0[CAN_NETDEV0_BUFFER_RXNUM];
#if 0
static uint8_t      txbuf_cnt_0 = 0;
static can_frame_t  txbuf_0[CAN_NETDEV0_BUFFER_TXNUM];
#endif

#ifndef CAN_NETDEV0_BUFFERS
#define CAN_NETDEV0_BUFFERS           { \
                                        .rxbuf_num    = CAN_NETDEV0_BUFFER_RXNUM, \
                                        .rxbuf_wr     = &rxbuf_0_wr, \
                                        .rxbuf_rd     = &rxbuf_0_rd, \
                                        .rxbuf        = (can_frame_t *)&rxbuf_0, \
                                      }
#endif

#ifndef CAN_NETDEV0_PM
#define CAN_NETDEV0_PM                { \
                                        .pm_level     = CAN_NETDEV0_PMLEVEL \
                                      }
#endif

#ifndef CAN_NETDEV0_PARAMS
#define CAN_NETDEV0_PARAMS            { \
                                        .timing       = CAN_NETDEV0_TIMING, \
                                        .iface        = CAN_NETDEV0_IFACE, \
                                        .buffers      = CAN_NETDEV0_BUFFERS, \
                                        .pm           = CAN_NETDEV0_PM \
                                      }
#endif

#if defined(CPU_FAM_STM32F0)
  #ifndef CAN_NETDEV0_EPARAMS
  #define CAN_NETDEV0_EPARAMS         { \
                                        .device            = CAN_NETDEV0_DEVICE, \
                                        .master_rcc_enable = CAN_NETDEV0_MRCCEN, \
                                        .rcc_enable        = CAN_NETDEV0_RCCEN, \
                                        .start_bank        = CAN_NETDEV0_SBANK, \
                                        .ttcm              = CAN_NETDEV0_TTCM, \
                                        .abom              = CAN_NETDEV0_ABOM, \
                                        .awum              = CAN_NETDEV0_AWUM, \
                                        .nart              = CAN_NETDEV0_NART, \
                                        .rflm              = CAN_NETDEV0_RFLM, \
                                        .txfp              = CAN_NETDEV0_TXFP, \
                                        .lbkm              = CAN_NETDEV0_LBKM, \
                                        .silm              = CAN_NETDEV0_SILM, \
                                        .irq               = CAN_NETDEV0_IRQ \
                                      }
  #endif
#else /* CPU_FAM_STM32F0 */
  #ifndef CAN_NETDEV0_EPARAMS
  #define CAN_NETDEV0_EPARAMS         { \
                                        .device            = CAN_NETDEV0_DEVICE, \
                                        .master_rcc_enable = CAN_NETDEV0_MRCCEN, \
                                        .rcc_enable        = CAN_NETDEV0_RCCEN, \
                                        .start_bank        = CAN_NETDEV0_SBANK, \
                                        .ttcm              = CAN_NETDEV0_TTCM, \
                                        .abom              = CAN_NETDEV0_ABOM, \
                                        .awum              = CAN_NETDEV0_AWUM, \
                                        .nart              = CAN_NETDEV0_NART, \
                                        .rflm              = CAN_NETDEV0_RFLM, \
                                        .txfp              = CAN_NETDEV0_TXFP, \
                                        .lbkm              = CAN_NETDEV0_LBKM, \
                                        .silm              = CAN_NETDEV0_SILM, \
                                        .irq_rx0           = CAN_NETDEV0_IRQRX0, \
                                        .irq_rx1           = CAN_NETDEV0_IRQRX1, \
                                        .irq_tx            = CAN_NETDEV0_IRQTX, \
                                        .irq_sce           = CAN_NETDEV0_IRQSCE \
                                      }
  #endif
#endif
/**
 * @name    Array of ALL Interfaces' parameters
 * @{
 */
#ifndef CAN_NETDEV_PARAMS
#define CAN_NETDEV_PARAMS             { \
                                        CAN_NETDEV0_PARAMS, \
                                      }
#endif

/**
 * @name    Array of ALL Interfaces' extra parameters
 * @{
 */
#ifndef CAN_NETDEV_EPARAMS
#define CAN_NETDEV_EPARAMS            { \
                                        CAN_NETDEV0_EPARAMS, \
                                      }
#endif


/** @} */

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/
static const socketcan_params_t   can_netdev_params[]  = CAN_NETDEV_PARAMS;
static const can_netdev_eparams_t can_netdev_eparams[] = CAN_NETDEV_EPARAMS;


/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* CAN_NETDEV_PARAMS_H */

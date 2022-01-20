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
#include "can/socketcan.h"

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
/*     -----     Interface Parameters      -----     */

#ifndef PCAN0_IFACE
#define PCAN0_IFACE                 CAN_IFACE_TYPE_STM32 | CAN_IFACE_NUM_0
#endif

#ifndef PCAN0_IFPARAMS_RXPIN
#define PCAN0_IFPARAMS_RXPIN        GPIO_PIN(PORT_D, 0)
#endif
#ifndef PCAN0_IFPARAMS_TXPIN
#define PCAN0_IFPARAMS_TXPIN        GPIO_PIN(PORT_D, 1)
#endif
#ifndef PCAN0_IFPARAMS_AF_OP        /* AF for normal operation                */
#define PCAN0_IFPARAMS_AF_OP        GPIO_AF9
#endif
#ifndef PCAN0_IFPARAMS_AF_NDIAG     /* AF for network diagnostics             */
#define PCAN0_IFPARAMS_AF_NDIAG     GPIO_AF0
#endif
#ifndef PCAN0_IFPARAMS
#define PCAN0_IFPARAMS                { \
                                        .rx_pin       = PCAN0_IFPARAMS_RXPIN, \
                                        .tx_pin       = PCAN0_IFPARAMS_TXPIN, \
                                        .af_op        = PCAN0_IFPARAMS_AF_OP, \
                                        .af_ndiag     = PCAN0_IFPARAMS_AF_NDIAG \
                                      }
#endif


/*      -----      Timing Parameters        -----        */
#ifndef PCAN0_TIMING_NBR        /* Nominal Bit Rate in bits/sec           */
#define PCAN0_TIMING_NBR        50000
#endif
#ifndef PCAN0_TIMING_CLOCK      /* Clock for CAN device in Hz             */
#define PCAN0_TIMING_CLOCK      CLOCK_APB1
#endif
#ifndef PCAN0_TIMING_PROP       /* Propagation segment in TQs             */
#define PCAN0_TIMING_PROP       2
#endif
#ifndef PCAN0_TIMING_PS1        /* Phase segment 1 in TQs                 */
#define PCAN0_TIMING_PS1        7
#endif
#ifndef PCAN0_TIMING_PS2        /* Phase segment 2 in TQs                 */
#define PCAN0_TIMING_PS2        6
#endif
#ifndef PCAN0_TIMING_SJW        /* Synchronization Jump Width             */
#define PCAN0_TIMING_SJW        1
#endif

/* Variable parameters that must be in RAM */
static uint32_t     nom_bitrate_0 = PCAN0_TIMING_NBR;
static uint32_t     r_bitrate_0;
static uint32_t     brp_0;

#ifndef PCAN0_TIMING
#define PCAN0_TIMING            { \
                                  .nom_bitrate  = &nom_bitrate_0, \
                                  .r_bitrate    = &r_bitrate_0, \
                                  .brp          = &brp_0, \
                                  .clock        = PCAN0_TIMING_CLOCK, \
                                  .prseg        = PCAN0_TIMING_PROP, \
                                  .phseg1       = PCAN0_TIMING_PS1, \
                                  .phseg2       = PCAN0_TIMING_PS2, \
                                  .sjw          = PCAN0_TIMING_SJW \
                                }
#endif


/*      -----  Power Management Parameters  -----        */

#ifndef PCAN0_PMLEVEL           /* PM Level to block                      */
#define PCAN0_PMLEVEL           2
#endif
#ifndef PCAN0_PM
#define PCAN0_PM                { \
                                  .pm_level     = PCAN0_PMLEVEL \
                                }
#endif

/*      -----       Extra Parameters      -----        */

#if defined(CPU_FAM_STM32F0)
  #ifndef PCAN0_DEVICE          /* Device registers base address          */
  #define PCAN0_DEVICE          CAN
  #endif
#else /* CPU_FAM_STM32F0 */
  #ifndef PCAN0_DEVICE
  #define PCAN0_DEVICE          CAN1
  #endif
  #ifndef PCAN0_MRCCEN          /* Master channel RCC Enable Bit          */
  #define PCAN0_MRCCEN          RCC_APB1ENR_CAN1EN
  #endif
  #ifndef PCAN0_RCCEN           /* Slave channel RCC Enable Bit           */
  #define PCAN0_RCCEN           RCC_APB1ENR_CAN1EN
  #endif
  #ifndef PCAN0_SBANK           /* CAN2 start bank                        */
  #define PCAN0_SBANK           14
  #endif
  #ifndef PCAN0_TTCM            /* Time Triggered Communication Mode      */
  #define PCAN0_TTCM            0
  #endif
  #ifndef PCAN0_ABOM            /* Automatic bus-off management           */
  #define PCAN0_ABOM            0
  #endif
  #ifndef PCAN0_AWUM            /* Automatic wakeup mode                  */
  #define PCAN0_AWUM            0
  #endif
  #ifndef PCAN0_NART            /* No automatic retransmissio             */
  #define PCAN0_NART            0
  #endif
  #ifndef PCAN0_RFLM            /* Receive FIFO locked mode               */
  #define PCAN0_RFLM            0
  #endif
  #ifndef PCAN0_TXFP            /* Transmit FIFO priority                 */
  #define PCAN0_TXFP            0
  #endif
  #ifndef PCAN0_LBKM            /* Loopback mode                          */
  #define PCAN0_LBKM            0
  #endif
  #ifndef PCAN0_SILM            /* Silent mode                            */
  #define PCAN0_SILM            0
  #endif
#endif

#if defined(CPU_FAM_STM32F0)
  #ifndef PCAN0_IRQ             /* CAN IRQ                                */
  #define PCAN0_IRQ             CEC_CAN_IRQn
  #endif
#else /* CPU_FAM_STM32F0 */
  #ifndef PCAN0_IRQRX0          /* CAN RX0 IRQ                            */
  #define PCAN0_IRQRX0          CAN1_RX0_IRQn
  #endif
  #ifndef PCAN0_IRQRX1          /* CAN RX1 IRQ                            */
  #define PCAN0_IRQRX1          CAN1_RX1_IRQn
  #endif
  #ifndef PCAN0_IRQTX           /* CAN TX IRQ                             */
  #define PCAN0_IRQTX           CAN1_TX_IRQn
  #endif
  #ifndef PCAN0_IRQSCE          /* CAN RX0 IRQ                            */
  #define PCAN0_IRQSCE          CAN1_SCE_IRQn
  #endif
#endif

#if defined(CPU_FAM_STM32F0)
  #ifndef PCAN0_EPARAMS
  #define PCAN0_EPARAMS         { \
                                  .device            = PCAN0_DEVICE, \
                                  .master_rcc_enable = PCAN0_MRCCEN, \
                                  .rcc_enable        = PCAN0_RCCEN, \
                                  .start_bank        = PCAN0_SBANK, \
                                  .ttcm              = PCAN0_TTCM, \
                                  .abom              = PCAN0_ABOM, \
                                  .awum              = PCAN0_AWUM, \
                                  .nart              = PCAN0_NART, \
                                  .rflm              = PCAN0_RFLM, \
                                  .txfp              = PCAN0_TXFP, \
                                  .lbkm              = PCAN0_LBKM, \
                                  .silm              = PCAN0_SILM, \
                                  .irq               = PCAN0_IRQ \
                                }
  #endif
#else /* CPU_FAM_STM32F0 */
  #ifndef PCAN0_EPARAMS
  #define PCAN0_EPARAMS         { \
                                  .device            = PCAN0_DEVICE, \
                                  .master_rcc_enable = PCAN0_MRCCEN, \
                                  .rcc_enable        = PCAN0_RCCEN, \
                                  .start_bank        = PCAN0_SBANK, \
                                  .ttcm              = PCAN0_TTCM, \
                                  .abom              = PCAN0_ABOM, \
                                  .awum              = PCAN0_AWUM, \
                                  .nart              = PCAN0_NART, \
                                  .rflm              = PCAN0_RFLM, \
                                  .txfp              = PCAN0_TXFP, \
                                  .lbkm              = PCAN0_LBKM, \
                                  .silm              = PCAN0_SILM, \
                                  .irq_rx0           = PCAN0_IRQRX0, \
                                  .irq_rx1           = PCAN0_IRQRX1, \
                                  .irq_tx            = PCAN0_IRQTX, \
                                  .irq_sce           = PCAN0_IRQSCE \
                                }
  #endif
#endif


/*      -----      Control Parameters       -----        */

#ifndef PCAN0_REG0_WAKFIL       /* WAKFIL signal bit */
#define PCAN0_REG0_WAKFIL       1       /* Wakeup filter enabled          */
#endif
#ifndef PCAN0_REG0_SOF          /* SOF signal bit */
#define PCAN0_REG0_SOF          1       /* CLKOUT enabled for SOF         */
#endif


 /*     -----     Operation Parameters      -----     */

#ifndef PCAN0_OP0_INITMODE      
#define PCAN0_OP0_INITMODE      MCP2515_CANCTRL_REQOP_NORMAL
#endif

/**
 * @name    Arrays of ALL Interfaces' parameters
 * @{
 */
#ifndef PCAN_IFPARAMS
#define PCAN_IFPARAMS           { \
                                  PCAN0_IFPARAMS, \
                                }
#endif

#ifndef PCAN_TIMING
#define PCAN_TIMING             { \
                                  PCAN0_TIMING, \
                                }
#endif

#ifndef PCAN_PM
#define PCAN_PM                 { \
                                  PCAN0_PM, \
                                }
#endif

#ifndef PCAN_EPARAMS
#define PCAN_EPARAMS            { \
                                  PCAN0_EPARAMS, \
                                }
#endif



/** @} */

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/
static const socketcan_ifparams_t pcan_ifparams[] = PCAN_IFPARAMS;
static const socketcan_timing_t   pcan_timing[]   = PCAN_TIMING;
static       socketcan_pm_t       pcan_pm[]       = PCAN_PM;
static       can_netdev_eparams_t pcan_eparams[]  = PCAN_EPARAMS;


/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* CAN_NETDEV_PARAMS_H */

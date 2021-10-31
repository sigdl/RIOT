/*
 * Copyright (C) 2016 OTA keys S.A.
 *               2021 Grr
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 *
 * @{
 *
 * @file
 * @brief       Defines for the MCP2515 can controller driver.
 *
 *  MCP2515 SPI CAN Controller Register & Configuration Constant
 *
 * @author      Toon Stegen <toon.stegen@altran.com>
 * @author      Grr <gebbet00@gmail.com>
 */

#ifndef MCP2515COMMON_H
#define MCP2515COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/

/**
 * @name MCP2515 Register Memory Map
 * {
 */
#define MCP2515_RXF0SIDH 0x00
#define MCP2515_RXF0SIDL 0x01
#define MCP2515_RXF0EID8 0x02
#define MCP2515_RXF0EID0 0x03

#define MCP2515_RXF1SIDH 0x04
#define MCP2515_RXF1SIDL 0x05
#define MCP2515_RXF1EID8 0x06
#define MCP2515_RXF1EID0 0x07

#define MCP2515_RXF2SIDH 0x08
#define MCP2515_RXF2SIDL 0x09
#define MCP2515_RXF2EID8 0x0A
#define MCP2515_RXF2EID0 0x0B

#define MCP2515_BFPCTRL   0x0C
#define MCP2515_TXRTSCTRL 0x0D
#define MCP2515_CANSTAT   0x0E
#define MCP2515_CANCTRL   0x0F

#define MCP2515_RXF3SIDH 0x10
#define MCP2515_RXF3SIDL 0x11
#define MCP2515_RXF3EID8 0x12
#define MCP2515_RXF3EID0 0x13

#define MCP2515_RXF4SIDH 0x14
#define MCP2515_RXF4SIDL 0x15
#define MCP2515_RXF4EID8 0x16
#define MCP2515_RXF4EID0 0x17

#define MCP2515_RXF5SIDH 0x18
#define MCP2515_RXF5SIDL 0x19
#define MCP2515_RXF5EID8 0x1A
#define MCP2515_RXF5EID0 0x1B

#define MCP2515_TEC      0x1C
#define MCP2515_REC      0x1D

#define MCP2515_RXM0SIDH 0x20
#define MCP2515_RXM0SIDL 0x21
#define MCP2515_RXM0EID8 0x22
#define MCP2515_RXM0EID0 0x23

#define MCP2515_RXM1SIDH 0x24
#define MCP2515_RXM1SIDL 0x25
#define MCP2515_RXM1EID8 0x26
#define MCP2515_RXM1EID0 0x27

#define MCP2515_CNF3     0x28
#define MCP2515_CNF2     0x29
#define MCP2515_CNF1     0x2A
#define MCP2515_CANINTE  0x2B

#define MCP2515_CANINTF  0x2C
#define MCP2515_EFLG     0x2D

#define MCP2515_TXB0CTRL 0x30
#define MCP2515_TXB0SIDH 0x31
#define MCP2515_TXB0SIDL 0x32
#define MCP2515_TXB0EID8 0x33
#define MCP2515_TXB0EID0 0x34
#define MCP2515_TXB0DLC  0x35
#define MCP2515_TXB0D0   0x36
#define MCP2515_TXB0D1   0x37
#define MCP2515_TXB0D2   0x38
#define MCP2515_TXB0D3   0x39
#define MCP2515_TXB0D4   0x3A
#define MCP2515_TXB0D5   0x3B
#define MCP2515_TXB0D6   0x3C
#define MCP2515_TXB0D7   0x3D

#define MCP2515_TXB1CTRL 0x40
#define MCP2515_TXB1SIDH 0x41
#define MCP2515_TXB1SIDL 0x42
#define MCP2515_TXB1EID8 0x43
#define MCP2515_TXB1EID0 0x44
#define MCP2515_TXB1DLC  0x45
#define MCP2515_TXB1D0   0x46
#define MCP2515_TXB1D1   0x47
#define MCP2515_TXB1D2   0x48
#define MCP2515_TXB1D3   0x49
#define MCP2515_TXB1D4   0x4A
#define MCP2515_TXB1D5   0x4B
#define MCP2515_TXB1D6   0x4C
#define MCP2515_TXB1D7   0x4D

#define MCP2515_TXB2CTRL 0x50
#define MCP2515_TXB2SIDH 0x51
#define MCP2515_TXB2SIDL 0x52
#define MCP2515_TXB2EID8 0x53
#define MCP2515_TXB2EID0 0x54
#define MCP2515_TXB2DLC  0x55
#define MCP2515_TXB2D0   0x56
#define MCP2515_TXB2D1   0x57
#define MCP2515_TXB2D2   0x58
#define MCP2515_TXB2D3   0x59
#define MCP2515_TXB2D4   0x5A
#define MCP2515_TXB2D5   0x5B
#define MCP2515_TXB2D6   0x5C
#define MCP2515_TXB2D7   0x5D

#define MCP2515_RXB0CTRL 0x60
#define MCP2515_RXB0SIDH 0x61
#define MCP2515_RXB0SIDL 0x62
#define MCP2515_RXB0EID8 0x63
#define MCP2515_RXB0EID0 0x64
#define MCP2515_RXB0DLC  0x65
#define MCP2515_RXB0D0   0x66
#define MCP2515_RXB0D1   0x67
#define MCP2515_RXB0D2   0x68
#define MCP2515_RXB0D3   0x69
#define MCP2515_RXB0D4   0x6A
#define MCP2515_RXB0D5   0x6B
#define MCP2515_RXB0D6   0x6C
#define MCP2515_RXB0D7   0x6D

#define MCP2515_RXB1CTRL 0x70
#define MCP2515_RXB1SIDH 0x71
#define MCP2515_RXB1SIDL 0x72
#define MCP2515_RXB1EID8 0x73
#define MCP2515_RXB1EID0 0x74
#define MCP2515_RXB1DLC  0x75
#define MCP2515_RXB1D0   0x76
#define MCP2515_RXB1D1   0x77
#define MCP2515_RXB1D2   0x78
#define MCP2515_RXB1D3   0x79
#define MCP2515_RXB1D4   0x7A
#define MCP2515_RXB1D5   0x7B
#define MCP2515_RXB1D6   0x7C
#define MCP2515_RXB1D7   0x7D
/** @} */

/**
 * @name MCP2515 Control Register bits
 * {
 */
#define MCP2515_BFPCTRL_B0BFM 0x01
#define MCP2515_BFPCTRL_B1BFM 0x02
#define MCP2515_BFPCTRL_B0BFE 0x04
#define MCP2515_BFPCTRL_B1BFE 0x08

#define MCP2515_BFPCTRL_B0BFS 0x10
#define MCP2515_BFPCTRL_B1BFS 0x20

#define MCP2515_TXRTSCTRL_B0RTSM 0x01
#define MCP2515_TXRTSCTRL_B1RTSM 0x02
#define MCP2515_TXRTSCTRL_B2RTSM 0x04
#define MCP2515_TXRTSCTRL_B0RTS 0x08
#define MCP2515_TXRTSCTRL_B1RTS 0x10
#define MCP2515_TXRTSCTRL_B2RTS 0x20

#define MCP2515_CANSTAT_ICOD0 0x02
#define MCP2515_CANSTAT_ICOD1 0x04
#define MCP2515_CANSTAT_ICOD2 0x08
#define MCP2515_CANSTAT_OPMOD0 0x20
#define MCP2515_CANSTAT_OPMOD1 0x40
#define MCP2515_CANSTAT_OPMOD2  0x80

#define MCP2515_CANSTAT_ICOD_MASK 0x0E

#define MCP2515_CANSTAT_OPMOD_MASK          0xE0
#define MCP2515_CANSTAT_OPMOD_CONFIGURATION MCP2515_CANSTAT_OPMOD2
#define MCP2515_CANSTAT_OPMOD_NORMAL        0x00
#define MCP2515_CANSTAT_OPMOD_SLEEP         MCP2515_CANSTAT_OPMOD0
#define MCP2515_CANSTAT_OPMOD_LOOPBACK      MCP2515_CANSTAT_OPMOD1
#define MCP2515_CANSTAT_OPMOD_LISTEN_ONLY   (MCP2515_CANSTAT_OPMOD1 | \
                                             MCP2515_CANSTAT_OPMOD0)

#define MCP2515_CANCTRL_CLKPRE0 0x01
#define MCP2515_CANCTRL_CLKPRE1 0x02
#define MCP2515_CANCTRL_CLKEN   0x04
#define MCP2515_CANCTRL_OSM     0x08
#define MCP2515_CANCTRL_ABAT    0x10
#define MCP2515_CANCTRL_REQOP0  0x20
#define MCP2515_CANCTRL_REQOP1  0x40
#define MCP2515_CANCTRL_REQOP2  0x80

#define MCP2515_CANCTRL_CLKPRE_MASK (MCP2515_CANCTRL_CLKPRE1 | \
                                     MCP2515_CANCTRL_CLKPRE0)

#define MCP2515_CANCTRL_REQOP_MASK          0xE0
#define MCP2515_CANCTRL_REQOP_CONFIGURATION MCP2515_CANCTRL_REQOP2
#define MCP2515_CANCTRL_REQOP_NORMAL        0x00
#define MCP2515_CANCTRL_REQOP_SLEEP         MCP2515_CANCTRL_REQOP0
#define MCP2515_CANCTRL_REQOP_LOOPBACK      MCP2515_CANCTRL_REQOP1
#define MCP2515_CANCTRL_REQOP_LISTEN_ONLY   (MCP2515_CANCTRL_REQOP1 | \
                                             MCP2515_CANCTRL_REQOP0)

#define MCP2515_CNF3_PHSEG20    0x01
#define MCP2515_CNF3_PHSEG21    0x02
#define MCP2515_CNF3_PHSEG22    0x04
#define MCP2515_CNF3_WAKFIL     0x40
#define MCP2515_CNF3_SOF        0x80

#define MCP2515_CNF3_PHSEG_MASK 0x07

#define MCP2515_CNF2_PRSEG0     0x01
#define MCP2515_CNF2_PRSEG1     0x02
#define MCP2515_CNF2_PRSEG2     0x04
#define MCP2515_CNF2_PHSEG10    0x08
#define MCP2515_CNF2_PHSEG11    0x10
#define MCP2515_CNF2_PHSEG12    0x20
#define MCP2515_CNF2_SAM        0x40
#define MCP2515_CNF2_BTLMODE    0x80

#define MCP2515_CNF2_PRSEG_MASK 0x07
#define MCP2515_CNF2_PHSEG_MASK 0x38

#define MCP2515_CNF1_BRP0       0x01
#define MCP2515_CNF1_BRP1       0x02
#define MCP2515_CNF1_BRP2       0x04
#define MCP2515_CNF1_BRP3       0x08
#define MCP2515_CNF1_BRP4       0x10
#define MCP2515_CNF1_BRP5       0x20
#define MCP2515_CNF1_SJW0       0x40
#define MCP2515_CNF1_SJW1       0x80

#define MCP2515_CNF1_BRP_MASK   0x3F
#define MCP2515_CNF1_SJW_MASK   0x3
#define MCP2515_CNF1_SJW_SHIFT  0x6

#define MCP2515_CANINTE_RX0IE   0x01
#define MCP2515_CANINTE_RX1IE   0x02
#define MCP2515_CANINTE_TX0IE   0x04
#define MCP2515_CANINTE_TX1IE   0x08
#define MCP2515_CANINTE_TX2IE   0x10
#define MCP2515_CANINTE_ERRIE   0x20
#define MCP2515_CANINTE_WAKIE   0x40
#define MCP2515_CANINTE_MERRE   0x80

#define MCP2515_CANINTF_RX0IF   0x01
#define MCP2515_CANINTF_RX1IF   0x02
#define MCP2515_CANINTF_TX0IF   0x04
#define MCP2515_CANINTF_TX1IF   0x08
#define MCP2515_CANINTF_TX2IF   0x10
#define MCP2515_CANINTF_ERRIF   0x20
#define MCP2515_CANINTF_WAKIF   0x40
#define MCP2515_CANINTF_MERRF   0x80
/** @} */

/**
 * @name MCP2515 error flags
 * {
 */
#define MCP2515_EFLG_EWARN                  0x01
#define MCP2515_EFLG_RXWAR                  0x02
#define MCP2515_EFLG_TXWAR                  0x04
#define MCP2515_EFLG_RXEP                   0x08
#define MCP2515_EFLG_TXEP                   0x10
#define MCP2515_EFLG_TXBO                   0x20
#define MCP2515_EFLG_RX0OVR                 0x40
#define MCP2515_EFLG_RX1OVR                 0x80
/** @} */

/**
 * @name MCP2515 Transmit and receive flags
 * @{
 */
#define MCP2515_TXBCTRL_TXP0                0x01
#define MCP2515_TXBCTRL_TXP1                0x02
#define MCP2515_TXBCTRL_TXREQ               0x08
#define MCP2515_TXBCTRL_TXERR               0x10
#define MCP2515_TXBCTRL_MLOA                0x20
#define MCP2515_TXBCTRL_ABTF                0x40

#define MCP2515_RXB0CTRL_FILHIT0                0x01
#define MCP2515_RXB0CTRL_BUKT1                  0x02
#define MCP2515_RXB0CTRL_BUKT                   0x04
#define MCP2515_RXB0CTRL_RXRTR                  0x08
#define MCP2515_RXB0CTRL_RXM0                   0x20
#define MCP2515_RXB0CTRL_RXM1                   0x40
#define MCP2515_RXB0CTRL_MODE_RECV_STD_OR_EXT   0x00
#define MCP2515_RXB0CTRL_MODE_RECV_STD          MCP2515_RXB0CTRL_RXM0
#define MCP2515_RXB0CTRL_MODE_RECV_EXT          MCP2515_RXB0CTRL_RXM1
#define MCP2515_RXB0CTRL_MODE_RECV_ALL          (MCP2515_RXB0CTRL_RXM1 | \
                                                 MCP2515_RXB0CTRL_RXM0)

#define MCP2515_RXB1CTRL_FILHIT0                0x01
#define MCP2515_RXB1CTRL_FILHIT1                0x02
#define MCP2515_RXB1CTRL_FILHIT2                0x04
#define MCP2515_RXB1CTRL_RXRTR                  0x08
#define MCP2515_RXB1CTRL_RXM0                   0x20
#define MCP2515_RXB1CTRL_RXM1                   0x40
#define MCP2515_RXB1CTRL_MODE_RECV_STD_OR_EXT   0x00
#define MCP2515_RXB1CTRL_MODE_RECV_STD          MCP2515_RXB1CTRL_RXM0
#define MCP2515_RXB1CTRL_MODE_RECV_EXT          MCP2515_RXB1CTRL_RXM1
#define MCP2515_RXB1CTRL_MODE_RECV_ALL          (MCP2515_RXB1CTRL_RXM1 | \
                                                 MCP2515_RXB1CTRL_RXM0)
/** @} */

/**
 * @name MCP2515 SPI commands
 * {
 */
#define MCP2515_SPI_RESET                   0xC0
#define MCP2515_SPI_READ                    0x03
#define MCP2515_SPI_READ_RXBUF              0x90
#define MCP2515_SPI_WRITE                   0x02
#define MCP2515_SPI_LOAD_TXBUF              0x40
#define MCP2515_SPI_RTS                     0x80
#define MCP2515_SPI_READ_STATUS             0xA0
#define MCP2515_SPI_RX_STATUS               0xB0
#define MCP2515_SPI_BITMOD                  0x05
/** @} */

/**
 * @name MCP2515 RX buffer id
 * {
 */
#define MCP2515_RXBUF_RXB0SIDH              0x00
#define MCP2515_RXBUF_RXB0D0                0x02
#define MCP2515_RXBUF_RXB1SIDH              0x04
#define MCP2515_RXBUF_RXB1D0                0x06
/** @} */

/**
 * @name MCP2515 TX buffer id
 * {
 */
#define MCP2515_TXBUF_TXB0SIDH              0x00
#define MCP2515_TXBUF_TXB0D0                0x01
#define MCP2515_TXBUF_TXB1SIDH              0x02
#define MCP2515_TXBUF_TXB1D0                0x03
#define MCP2515_TXBUF_TXB2SIDH              0x04
#define MCP2515_TXBUF_TXB2D0                0x05
/** @} */

/**
 * @name MCP2515 option ID for ioctl function
 * {
 */
#define MCP2515_OPTION_ROLLOVER             1
#define MCP2515_OPTION_ONESHOT              2
#define MCP2515_OPTION_ABORT                3
#define MCP2515_OPTION_CLOCKOUT             4
#define MCP2515_OPTION_LOOPBACK             5
#define MCP2515_OPTION_LISTEN_ONLY          6
#define MCP2515_OPTION_SLEEP                7
#define MCP2515_OPTION_MULTISAMPLE          8
#define MCP2515_OPTION_SOFOUT               9
#define MCP2515_OPTION_WAKE_GLITCH_FILTER   10
#define MCP2515_OPTION_WAKE                 11
/** @} */

/**
 * @name MCP2515 IRQ handling
 * {
 */
#define MCP2515_IRQ_FLAGGED                 0x80
#define MCP2515_IRQ_HANDLED                 0x40
#define MCP2515_IRQ_RX                      0x01
#define MCP2515_IRQ_TX                      0x02
#define MCP2515_IRQ_ERROR                   0x04
#define MCP2515_IRQ_WAKEUP                  0x08
/** @} */

/**
 * @name MCP2515 Extended ID bit
 * {
 */
#define MCP2515_RX_IDE                      0x08
/** @} */

/**
 * @name MCP2515 reset values
 * {
 */
#define MCP2515_RESET_CANCTRL               0x87
/** @} */

/**
 * @name MCP2515 miscelaneous values
 * {
 */
#define MCP2515_STUFFING                    0xFF
/** @} */

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* MCP2515COMMON_H */

/****************************************************************************
 * include/nuttx/can/mcp2515_upds.h
 *
 *   Copyright (C) 2021 Grr. All rights reserved.
 *   Author: Grr <gebbet00@gmail.com>
 *
 *  This file is free software: you may copy, redistribute and/or modify it  
 *  under the terms of the GNU General Public License as published by the  
 *  Free Software Foundation, either version 2 of the License, or (at your  
 *  option) any later version.  
 *  
 *  This file is distributed in the hope that it will be useful, but  
 *  WITHOUT ANY WARRANTY; without even the implied warranty of  
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU  
 *  General Public License for more details.  
 *  
 *  You should have received a copy of the GNU General Public License  
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.  
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_CAN_MCP2515_UPDS_H
#define __INCLUDE_NUTTX_CAN_MCP2515_UPDS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default Values 
 *
 *  Nominal Bit Rate                500 KHz
 *  Nominal Bit Time                2 us
 *  Nominal Bus Length              100 m
 *
 *  Tq                              125 ns
 *  Tq per Bit                      16
 *  
 *  Clock Frequency                 16 MHz
 *  Tosc                            62.5 ns
 *  BRP = Tq /(2 * Tosc)            1
 *  
 *  SyncSeg                         1 Tq
 *  PropSeg                         2 Tq
 *  PhaseSeg1                       7 Tq
 *  PhaseSeg2                       6 Tq
 *  Synchronization Jump Width      1 Tq
 *
 */

#define MCP2515_MODE                SPI_MODE0
#define MCP2515_WORDSIZE            8
#define MCP2515_SPI_FREQ            500000
#define MCP2515_AFTERWORD           10
#define MCP2515_AFTERSTART          5
#define MCP2515_AFTERCLOCK          10
#define MCP2515_CANCTRL             0x87


#define MCP2515_RESET_TIME          10000

#define MCP2515_STUFFING            0xff

/* Config Register 1 */

#define MCP2515_BRP_MASK            0x3f
#define MCP2515_SJW_SHIFT           6
#define MCP2515_SJW_MASK            3

/* Config Register 2 */

#define MCP2515_PROPSEG_MASK        0X7

#define MCP2515_PS1_MASK            0X7
#define MCP2515_PS1_SHIFT           3
#define MCP2515_PS2_MASK            0X7
#define MCP2515_SAM                 0       /* Single sampling of bit                 */
#define MCP2515_SAM_MASK            1
#define MCP2515_SAM_SHIFT           6
#define MCP2515_BTL                 1       /* PS2 by PHSEG2                          */
#define MCP2515_BTL_MASK            1
#define MCP2515_BTL_SHIFT           7
#define MCP2515_WAKFIL              0       /* Wake-up filter disabled                */
#define MCP2515_WAKFIL_MASK         1
#define MCP2515_WAKFIL_SHIFT        6
#define MCP2515_SOF                 0       /* CLKOUT enabled                         */
#define MCP2515_SOF_MASK            1
#define MCP2515_SOF_SHIFT           7

#define MCP2515_CNF1                ()

/* MCP2515 Commands */

#define MCP2515_CMD_RESET           0xC0
#define MCP2515_CMD_READ            0X03
#define MCP2515_CMD_RDSTATUS        0xA0
#define MCP2515_CMD_WRITE           0x02

/* MCP2515 Register Addresses */

#define MCP2515_ADDR_RXF0SIDH       0x00
#define MCP2515_ADDR_RXF0SIDL       0x01
#define MCP2515_ADDR_RXF0EID8       0x02
#define MCP2515_ADDR_RXF0EID0       0x03
#define MCP2515_ADDR_RXF1SIDH       0x04
#define MCP2515_ADDR_RXF1SIDL       0x05
#define MCP2515_ADDR_RXF1EID8       0x06
#define MCP2515_ADDR_RXF1EID0       0x07
#define MCP2515_ADDR_RXF2SIDH       0x08
#define MCP2515_ADDR_RXF2SIDL       0x09
#define MCP2515_ADDR_RXF2EID8       0x0a
#define MCP2515_ADDR_RXF2EID0       0x0b
#define MCP2515_ADDR_BFPCTRL        0x0c
#define MCP2515_ADDR_TXRTSCTRL      0x0d
#define MCP2515_ADDR_CANSTAT        0x0e
#define MCP2515_ADDR_CANCTRL        0x0f
#define MCP2515_ADDR_RXF3SIDH       0x10
#define MCP2515_ADDR_RXF3SIDL       0x11
#define MCP2515_ADDR_RXF3EID8       0x12
#define MCP2515_ADDR_RXF3EID0       0x13
#define MCP2515_ADDR_RXF4SIDH       0x14
#define MCP2515_ADDR_RXF4SIDL       0x15
#define MCP2515_ADDR_RXF4EID8       0x16
#define MCP2515_ADDR_RXF4EID0       0x17
#define MCP2515_ADDR_RXF5SIDH       0x18
#define MCP2515_ADDR_RXF5SIDL       0x19
#define MCP2515_ADDR_RXF5EID8       0x1a
#define MCP2515_ADDR_RXF5EID0       0x1b
#define MCP2515_ADDR_TEC            0x1c
#define MCP2515_ADDR_REC            0x1d
#define MCP2515_ADDR_RXM0SIDH       0x20
#define MCP2515_ADDR_RXM0SIDL       0x21
#define MCP2515_ADDR_RXM0EID8       0x22
#define MCP2515_ADDR_RXM0EID0       0x23
#define MCP2515_ADDR_RXM1SIDH       0x24
#define MCP2515_ADDR_RXM1SIDL       0x25
#define MCP2515_ADDR_RXM1EID8       0x26
#define MCP2515_ADDR_RXM1EID0       0x27
#define MCP2515_ADDR_CNF3           0x28
#define MCP2515_ADDR_CNF2           0x29
#define MCP2515_ADDR_CNF1           0x2a
#define MCP2515_ADDR_CANINTE        0x2b
#define MCP2515_ADDR_CANINTF        0x2c
#define MCP2515_ADDR_EFLG           0x2d
#define MCP2515_ADDR_TXB0CTRL       0x30
#define MCP2515_ADDR_TXB0SIDH       0x31
#define MCP2515_ADDR_TXB0SIDL       0x32
#define MCP2515_ADDR_TXB0EID8       0x33
#define MCP2515_ADDR_TXB0EID0       0x34
#define MCP2515_ADDR_TXB0DLC        0x35
#define MCP2515_ADDR_TXB0D0         0x36
#define MCP2515_ADDR_TXB0D1         0x37
#define MCP2515_ADDR_TXB0D2         0x38
#define MCP2515_ADDR_TXB0D3         0x39
#define MCP2515_ADDR_TXB0D4         0x3a
#define MCP2515_ADDR_TXB0D5         0x3b
#define MCP2515_ADDR_TXB0D6         0x3c
#define MCP2515_ADDR_TXB0D7         0x3d
#define MCP2515_ADDR_TXB1CTRL       0x40
#define MCP2515_ADDR_TXB1SIDH       0x41
#define MCP2515_ADDR_TXB1SIDL       0x42
#define MCP2515_ADDR_TXB1EID8       0x43
#define MCP2515_ADDR_TXB1EID0       0x44
#define MCP2515_ADDR_TXB1DLC        0x45
#define MCP2515_ADDR_TXB1D0         0x46
#define MCP2515_ADDR_TXB1D1         0x47
#define MCP2515_ADDR_TXB1D2         0x48
#define MCP2515_ADDR_TXB1D3         0x49
#define MCP2515_ADDR_TXB1D4         0x4a
#define MCP2515_ADDR_TXB1D5         0x4b
#define MCP2515_ADDR_TXB1D6         0x4c
#define MCP2515_ADDR_TXB1D7         0x4d
#define MCP2515_ADDR_TXB2CTRL       0x50
#define MCP2515_ADDR_TXB2SIDH       0x51
#define MCP2515_ADDR_TXB2SIDL       0x52
#define MCP2515_ADDR_TXB2EID8       0x53
#define MCP2515_ADDR_TXB2EID0       0x54
#define MCP2515_ADDR_TXB2DLC        0x55
#define MCP2515_ADDR_TXB2D0         0x56
#define MCP2515_ADDR_TXB2D1         0x57
#define MCP2515_ADDR_TXB2D2         0x58
#define MCP2515_ADDR_TXB2D3         0x59
#define MCP2515_ADDR_TXB2D4         0x5a
#define MCP2515_ADDR_TXB2D5         0x5b
#define MCP2515_ADDR_TXB2D6         0x5c
#define MCP2515_ADDR_TXB2D7         0x5d
#define MCP2515_ADDR_RXB0CTRL       0x60
#define MCP2515_ADDR_RXB0SIDH       0x61
#define MCP2515_ADDR_RXB0SIDL       0x62
#define MCP2515_ADDR_RXB0EID8       0x63
#define MCP2515_ADDR_RXB0EID0       0x64
#define MCP2515_ADDR_RXB0DLC        0x65
#define MCP2515_ADDR_RXB0D0         0x66
#define MCP2515_ADDR_RXB0D1         0x67
#define MCP2515_ADDR_RXB0D2         0x68
#define MCP2515_ADDR_RXB0D3         0x69
#define MCP2515_ADDR_RXB0D4         0x6a
#define MCP2515_ADDR_RXB0D5         0x6b
#define MCP2515_ADDR_RXB0D6         0x6c
#define MCP2515_ADDR_RXB0D7         0x6d
#define MCP2515_ADDR_RXB1CTRL       0x70
#define MCP2515_ADDR_RXB1SIDH       0x71
#define MCP2515_ADDR_RXB1SIDL       0x72
#define MCP2515_ADDR_RXB1EID8       0x73
#define MCP2515_ADDR_RXB1EID0       0x74
#define MCP2515_ADDR_RXB1DLC        0x75
#define MCP2515_ADDR_RXB1D0         0x76
#define MCP2515_ADDR_RXB1D1         0x77
#define MCP2515_ADDR_RXB1D2         0x78
#define MCP2515_ADDR_RXB1D3         0x79
#define MCP2515_ADDR_RXB1D4         0x7a
#define MCP2515_ADDR_RXB1D5         0x7b
#define MCP2515_ADDR_RXB1D6         0x7c
#define MCP2515_ADDR_RXB1D7         0x7d


/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif


#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CAN_MCP2515_H */

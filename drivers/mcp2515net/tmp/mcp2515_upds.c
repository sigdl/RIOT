/****************************************************************************
 * arch/arm/src/stm32/stm32_gpio_upds.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/gpio/gpio_upds.h>
#include <nuttx/spi/spi_upds.h>
#include <nuttx/can/mcp2515_upds.h>



/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
#define BUF_SIZE       4

/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/
static struct upds_spidev_list_s g_spidev_list =
{
  .num_elements     = 0,
  .start            = NULL
};

#ifdef CONFIG_MCP2515_UPDS_1
static struct upds_commparams_s g_comm1 =
{
  .master           = true,
  .mode             = MCP2515_MODE,
  .wordsize         = MCP2515_WORDSIZE,
  .lsbfirst         = 0
};

static struct upds_spitransfer_s g_transfer1 =
{
  .nwords           = 0,
  .deselect         = 0,
  .txcnt            = 0,
  .rxcnt            = 0,
  .txbuffer         = NULL,
  .rxbuffer         = NULL
};

static struct upds_spidev_s g_dev1 =
{
  .num              = 0,
  .status           = SPI_INACTIVE,
  .next             = NULL,
  .features         = 0,
  .comm             = &g_comm1,
  .transfer         = &g_transfer1,
  .cs               = NULL,
  .rst              = NULL,
  .irq              = NULL,
  .afterword        = MCP2515_AFTERWORD,
  .afterstart       = MCP2515_AFTERSTART,
  .afterclock       = MCP2515_AFTERCLOCK,
  .debug_level      = CONFIG_UPDS_SPI_DEBUG
};
#endif

#ifdef CONFIG_MCP2515_UPDS_2
static struct upds_commparams_s g_comm2 =
{
  .master           = true,
  .mode             = MCP2515_MODE,
  .wordsize         = MCP2515_WORDSIZE,
  .lsbfirst         = 0
};

static struct upds_spitransfer_s g_transfer2 =
{
  .nwords           = 0,
  .deselect         = 0,
  .txcnt            = 0,
  .rxcnt            = 0,
  .txbuffer         = NULL,
  .rxbuffer         = NULL
};

static struct upds_spidev_s g_dev2 =
{
  .num              = 0,
  .status           = SPI_INACTIVE,
  .next             = NULL,
  .features         = 0,
  .comm             = &g_comm2,
  .transfer         = &g_transfer2,
  .cs               = NULL,
  .rst              = NULL,
  .irq              = NULL,
  .afterword        = MCP2515_AFTERWORD,
  .afterstart       = MCP2515_AFTERSTART,
  .afterclock       = MCP2515_AFTERCLOCK,
  .debug_level      = CONFIG_UPDS_SPI_DEBUG
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: upds_mcp2515_initialize
 *
 * Description:
 *   Initialize MCP2515 devices
 *
 * Input Parameters:
 *   dev        - Pointer to GPIO device structure
 *   stdconfig  - Pointer to stdconfig variable
 *
 * Returned Value:
 *   OK on success
 *
 ************************************************************************************/
int upds_mcp2515_initialize(void)
{
  int       intret = SPIOK;
  uint8_t   i;
  uint8_t   cnf;
  uint8_t   fifo_out[BUF_SIZE];
  uint8_t   fifo_in[BUF_SIZE];
  uint8_t   ctrllerlist[CONFIG_MCP2515_UPDS_NUM];
  uint8_t   cslist[CONFIG_MCP2515_UPDS_NUM];
  uint8_t   irqlist[CONFIG_MCP2515_UPDS_NUM];
  uint32_t  clklist[CONFIG_MCP2515_UPDS_NUM];
  uint32_t  nbrlist[CONFIG_MCP2515_UPDS_NUM];
  uint8_t   propseglist[CONFIG_MCP2515_UPDS_NUM];
  uint8_t   ps1list[CONFIG_MCP2515_UPDS_NUM];
  uint8_t   ps2list[CONFIG_MCP2515_UPDS_NUM];
  uint8_t   sjwlist[CONFIG_MCP2515_UPDS_NUM];
  uint8_t   bitquanta;
  int8_t    brp;
  uint32_t  volatile delay;

  struct upds_spidev_s            *devdir[CONFIG_MCP2515_UPDS_NUM];
  FAR struct upds_spicontroller_s *ctrller;
  FAR struct upds_spidev_s        *dev;


#ifdef CONFIG_MCP2515_UPDS_1
  devdir[g_spidev_list.num_elements]  = &g_dev1;
  ctrllerlist[0]                      = CONFIG_MCP2515_UPDS_1_SPI;
  cslist[0]                           = CONFIG_MCP2515_UPDS_1_CS;
  irqlist[0]                          = CONFIG_MCP2515_UPDS_1_IRQ;
  clklist[0]                          = CONFIG_MCP2515_UPDS_1_CLK;
  nbrlist[0]                          = CONFIG_MCP2515_UPDS_1_NBR;
  propseglist[0]                      = CONFIG_MCP2515_UPDS_1_PROPSEG;
  ps1list[0]                          = CONFIG_MCP2515_UPDS_1_PS1;
  ps2list[0]                          = CONFIG_MCP2515_UPDS_1_PS2;
  sjwlist[0]                          = CONFIG_MCP2515_UPDS_1_SJW;
  g_spidev_list.num_elements++;
#endif

#ifdef CONFIG_MCP2515_UPDS_2
  devdir[g_spidev_list.num_elements]  = &g_dev2;
  ctrllerlist[1]                      = CONFIG_MCP2515_UPDS_2_SPI;
  cslist[1]                           = CONFIG_MCP2515_UPDS_2_CS;
  irqlist[1]                          = CONFIG_MCP2515_UPDS_2_IRQ;
  clklist[1]                          = CONFIG_MCP2515_UPDS_2_CLK;
  nbrlist[1]                          = CONFIG_MCP2515_UPDS_2_NBR;
  propseglist[1]                      = CONFIG_MCP2515_UPDS_2_PROPSEG;
  ps1list[1]                          = CONFIG_MCP2515_UPDS_2_PS1;
  ps2list[1]                          = CONFIG_MCP2515_UPDS_2_PS2;
  sjwlist[1]                          = CONFIG_MCP2515_UPDS_2_SJW;
  g_spidev_list.num_elements++;
#endif

for(i = 0; i < g_spidev_list.num_elements; i++)
  {
    /* Load corresponding SPI struct addr */

    dev = devdir[i];

    /* Get controller struct addr */

    intret = upds_spi_getctrller(ctrllerlist[i], &dev->ctrller);

    /* Obtain GPIO control struct for CS */

    intret = upds_gpio_getstruct(cslist[i], &dev->cs);

    /* Obtain GPIO control struct for IRQ */

    intret = upds_gpio_getstruct(irqlist[i], &dev->irq);

    /* Configure clock */

    dev->comm->clock = dev->ctrller->clock;

    /* Configure frequency */

    intret = upds_spi_calcfrequency(dev->comm, MCP2515_SPI_FREQ);

#if CONFIG_UPDS_SPI_DEBUG >= 4
    spiinfo("MCP2515 Device %d freq:%d   Dselect:%d   \n",
            i + 1,
            dev->comm->realfrequency,
            dev->transfer->deselect
          );
#endif

    /* Get bus control */

    intret = upds_spi_getbus(dev, true);

    /* Set buffer addresses */

    dev->transfer->txbuffer = fifo_out;
    dev->transfer->rxbuffer = fifo_in;

    /* Set reset cmd parameters */

    fifo_out[0]             = MCP2515_CMD_RESET;
    dev->transfer->nwords   = 1;

    /* Do the SPI transfer */

    intret = upds_spi_transfer(dev, SPI_TRANSFER_BUFFER);

    /*nxsig_usleep(MCP2515_RESET_TIME);*/
      /* Wait the specified cycles */

    for(delay = 0; delay < MCP2515_RESET_TIME; delay++)
      {
        /*__asm__ __volatile__("");*/
        /*asm("");*/
      }


    /* Set device probe parameters */

    fifo_out[0]             = MCP2515_CMD_READ;
    fifo_out[1]             = MCP2515_ADDR_CANCTRL;
    fifo_out[2]             = MCP2515_STUFFING;
    dev->transfer->nwords   = 3;

    /* Do the SPI transfer */

    intret = upds_spi_transfer(dev, SPI_TRANSFER_BUFFER);

    /* If no correct answer */

    if(fifo_in[2] != MCP2515_CANCTRL)
      {
        spiinfo("MCP2515 Device %d response:0x%x.   Failed to be detected in SPI Bus %d\n",
            i + 1,
            fifo_in[2],
            dev->ctrller->busnum
          );

        /* Release bus control */

        intret = upds_spi_getbus(dev, false);

        /* Leave this device in inactive status and jump to the next */

        continue;
      }

    /* Obtain the number of Tq per bit */

    bitquanta = 1 + propseglist[i] + ps1list[i] + ps2list[i];

    /* Calculate Bitrate Prescaler */

    brp = (int8_t)((float)clklist[i] / \
                  ((float)(2 * bitquanta * nbrlist[i])) - 1);

    /* Config CNF1 */

    cnf = brp & MCP2515_BRP_MASK;

    /* Loading SJW */

    cnf |= (sjwlist[i] & MCP2515_SJW_MASK) << MCP2515_SJW_SHIFT;

    /* Fill Transfer Buffer */

    fifo_out[0]             = MCP2515_CMD_WRITE;
    fifo_out[1]             = MCP2515_ADDR_CNF1;
    fifo_out[2]             = cnf;
    dev->transfer->nwords   = 3;

    /* Do SPI transfer for CNF1 */

    intret = upds_spi_transfer(dev, SPI_TRANSFER_BUFFER);

    /* Loading PropSeg */

    cnf = propseglist[i] & MCP2515_PROPSEG_MASK;

    /* Loading Phase Segment 1 */

    cnf |= (ps1list[i] &MCP2515_PS1_MASK) << MCP2515_PS1_SHIFT;

    /* Loading SAM */

    cnf |= (MCP2515_SAM & MCP2515_SAM_MASK) << MCP2515_SAM_SHIFT;

    /* Loading BTL */

    cnf |= (MCP2515_BTL & MCP2515_BTL_MASK) << MCP2515_BTL_SHIFT;

    /* Fill Transfer Buffer */

    fifo_out[0]             = MCP2515_CMD_WRITE;
    fifo_out[1]             = MCP2515_ADDR_CNF2;
    fifo_out[2]             = cnf;
    dev->transfer->nwords   = 3;

    /* Do SPI transfer for CNF2 */

    intret = upds_spi_transfer(dev, SPI_TRANSFER_BUFFER);

    /* Loading Phase Segment 2 */

    cnf = ps2list[i] & MCP2515_PS2_MASK;

    /* Loading WAKFIL */

    cnf |= (MCP2515_WAKFIL & MCP2515_WAKFIL_MASK) << MCP2515_WAKFIL_SHIFT;

    /* Loading SOF */

    cnf |= (MCP2515_SOF & MCP2515_SOF_MASK) << MCP2515_SOF_SHIFT;

    /* Fill Transfer Buffer */

    fifo_out[0]             = MCP2515_CMD_WRITE;
    fifo_out[1]             = MCP2515_ADDR_CNF3;
    fifo_out[2]             = cnf;
    dev->transfer->nwords   = 3;

    /* Do SPI transfer for CNF2 */

    intret = upds_spi_transfer(dev, SPI_TRANSFER_BUFFER);





    /* Flag device as ready */

    dev->status = SPI_ENABLED;

    /* Register device */

    intret = upds_spi_registerdev(dev);

#if CONFIG_UPDS_SPI_DEBUG > 0
    spiinfo("MCP2515 Device %d Initialized as SPI device %d of Bus %d\n",
            i + 1,
            dev->num,
            dev->ctrller->busnum
          );
#endif
  }

  return intret;
}

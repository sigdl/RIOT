/*
 * Copyright (C) 2021 Grr <gebbet00@gmail.com>
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
 * @brief       Implementation of SocketCAN controller driver.
 *
 *
 * @author      Grr <gebbet00@gmail.com>
 */

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/
#include <errno.h>
#include <string.h>

#include "mutex.h"
#include "xtimer.h"
#include "assert.h"

#include "can_netdev.h"

#define ENABLE_DEBUG            1
#include "debug.h"
#include "log.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
#define TRANSF_BUFFER_SIZE      5
#define MODE_MAX_DELAY          (100000U)

/*------------------------------------------------------------------------------*
 *                                 Private Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                          Private Function Prototypes                         *
 *------------------------------------------------------------------------------*/
static int  can_netdev_init(netdev_t *netdev);
/*static void can_netdev_irq_h(void *arg);*/
static void can_netdev_isr(netdev_t *netdev);
static int  can_netdev_send(netdev_t *netdev, const iolist_t *iolist);
static int  can_netdev_recv(netdev_t *netdev, void *buf, size_t max_len, void *info);
static int  can_netdev_get(netdev_t *netdev, netopt_t opt, void *value, size_t max_len);
static int  can_netdev_set(netdev_t *netdev, netopt_t opt, const void *value, size_t value_len);
static int  can_netdev_mode(can_netdev_t *dev, can_opmode_t mode);
static void rx_irq_h(void *arg, uint8_t mailbox);
static void tx_irq_h(void *arg);
static void sce_irq_h(void *arg);


/*------------------------------------------------------------------------------*
 *                                Private Data                                  *
 *------------------------------------------------------------------------------*/
static const netdev_driver_t can_netdev_driver = {
    .init = can_netdev_init,
    .isr  = can_netdev_isr,
    .send = can_netdev_send,
    .recv = can_netdev_recv,
    .get  = can_netdev_get,
    .set  = can_netdev_set,
};
extern can_netdev_t can_netdev_arr[];

/*------------------------------------------------------------------------------*
 *                               Private Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief STM32 Socketcan driver Init function
 *
 *
 * @param[in]  netdev       Netdev device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int can_netdev_init(netdev_t *netdev)
{
    can_netdev_t *dev = container_of(netdev, can_netdev_t, netdev);

    /* Do basic config */
    can_netdev_bconfig(dev, CAN_CONFMODE_OP);

    /* Do operational parameters config */
    can_netdev_opconfig(dev);

    return 0;
}


/**
 * @brief STM32 Socketcan IRQ Handlers
 *
 * 
 *
 *
 */
void ISR_CAN1_RX0(void)
{
    rx_irq_h(&can_netdev_arr[0], 0);

    cortexm_isr_end();
}

void ISR_CAN1_RX1(void)
{
    rx_irq_h(&can_netdev_arr[0], 1);

    cortexm_isr_end();
}

void ISR_CAN1_TX(void)
{
    tx_irq_h(&can_netdev_arr[0]);

    cortexm_isr_end();
}

void ISR_CAN1_SCE(void)
{
    sce_irq_h(&can_netdev_arr[0]);

    cortexm_isr_end();
}

static inline void rx_irq_h(void *arg, uint8_t mailbox)
{
    can_netdev_t *dev = arg;
    uint8_t       flags;
    uint8_t       dlc;
    uint8_t       i;
    volatile uint32_t *rfr;

#ifdef ENABLE_DEBUG
    LED0_TOGGLE;
#endif

    /* Select registers of given mailbox */
    if (mailbox == 0) {
        rfr = &(dev->eparams->device->RF0R);
    }
    else {
        rfr = &(dev->eparams->device->RF1R);
    }

    /* If there was Overrun */
    if(*rfr & CAN_NETDEV_FOVR_MASK) {

        /* Turn off OVR flag */
        *rfr |= CAN_NETDEV_FOVR_MASK;

    }

    /* Cycle through available frames */
    while(*rfr & CAN_NETDEV_FMP_MASK ) {

        /* Initialize flags */
        flags = 0;

        /* If EXTENDED FRAME */
        if(dev->eparams->device->sFIFOMailBox[mailbox].RIR & CAN_RI0R_IDE) {

            /* Save Extended ID */
            dev->params->buffers.rxbuf[*(dev->params->buffers.rxbuf_wr)].id =
                dev->eparams->device->sFIFOMailBox[mailbox].RIR >> CAN_NETDEV_EFF_SHIFT;

            /* Save corresponding flag */
            flags |= CAN_FLAG_IDE_EXT;
        }

        /* If STANDARD FRAME */
        else {

            /* Save Standard ID */
            dev->params->buffers.rxbuf[*(dev->params->buffers.rxbuf_wr)].id =
                dev->eparams->device->sFIFOMailBox[mailbox].RIR >> CAN_NETDEV_SFF_SHIFT;
        }

        /* If REMOTE FRAME */
        if(dev->eparams->device->sFIFOMailBox[mailbox].RIR & CAN_RI0R_RTR ) {

            /* Save remote flag */
            flags |= CAN_FLAG_RTR_MASK;
        }

        /* If DATA FRAME */
        else {

            /* Get DLC */
            dlc = (dev->eparams->device->sFIFOMailBox[mailbox].RDTR & CAN_RDT0R_DLC_Msk);

            /* Save DLC */
            flags |= dlc << CAN_FLAG_DLC_SHIFT;

            /* Save data */
            for(i = 0; i < 4; i++) {
                dev->params->buffers.rxbuf[*(dev->params->buffers.rxbuf_wr)].data[i] =
                    (dev->eparams->device->sFIFOMailBox[mailbox].RDLR >> (i * 8)) & 0xFF;
            }
            for(i = 4; i < 8; i++) {
                dev->params->buffers.rxbuf[*(dev->params->buffers.rxbuf_wr)].data[i] =
                    (dev->eparams->device->sFIFOMailBox[mailbox].RDHR >> ((i - 4) * 8)) & 0xFF;
            }

            /* Save flags */
            dev->params->buffers.rxbuf[*(dev->params->buffers.rxbuf_wr)].flags = flags;

            /* Inc frame pointer */
            (*(dev->params->buffers.rxbuf_wr))++;

            /* If got to end of buffer */
            if(*(dev->params->buffers.rxbuf_wr) == dev->params->buffers.rxbuf_num ) {

                /* Wrap to beginning of buffer */
                *(dev->params->buffers.rxbuf_wr) = 0;
            }
        }

        /* Release mailbox */
        *rfr |= CAN_NETDEV_RFOM_MASK;
    }

    /* Signal event to driver's ISR */
    netdev_trigger_event_isr(&dev->netdev);
}

static inline void tx_irq_h(void *arg)
{
    can_netdev_t *dev = arg;

    /* Signal event to driver's ISR */
    netdev_trigger_event_isr(&dev->netdev);
}

static inline void sce_irq_h(void *arg)
{
    can_netdev_t *dev = arg;

    /* Signal event to driver's ISR */
    netdev_trigger_event_isr(&dev->netdev);
}


/**
 * @brief STM32 Socketcan driver ISR
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static void can_netdev_isr(netdev_t *netdev)
{
    can_netdev_t *dev = container_of(netdev, can_netdev_t, netdev);

    (void)dev;
    DEBUG("[PERIPH CAN] isr: Entering ISR in thread context\n");

}

/**
 * @brief STM32 Socketcan driver receive function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int can_netdev_recv(netdev_t *netdev, void *buf, size_t max_len, void *info)
{
    can_netdev_t *dev = container_of(netdev, can_netdev_t, netdev);

    (void)dev;
    (void)buf;
    (void)max_len;
    (void)info;


    return 0;
}

/**
 * @brief STM32 Socketcan driver send function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int can_netdev_send(netdev_t *netdev, const iolist_t *iolist)
{
    netdev++;
    iolist++;

    return 0;
}

/**
 * @brief STM32 Socketcan driver get function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int can_netdev_get(netdev_t *netdev, netopt_t opt, void *value, size_t max_len)
{
    int resp;

    (void)netdev;
    (void)max_len;

    switch(opt) {

        /* Get device type */
        case NETOPT_DEVICE_TYPE:
        {
            uint16_t *devtype = (uint16_t *)value;

            *devtype = NETDEV_TYPE_CAN;
            resp = 2;
            break;
        }
        
        default:
            return -ENOTSUP;
    }

    return resp;
}

/**
 * @brief STM32 Socketcan driver set function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int can_netdev_set(netdev_t *netdev, netopt_t opt, const void *value, size_t value_len)
{
    netdev++;
    opt++;
    value++;
    value_len++;

    return 0;
}

/**
 * @brief STM32 set mode function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int can_netdev_mode(can_netdev_t *dev, can_opmode_t mode)
{
    uint32_t wait_cnt = MODE_MAX_DELAY;
    int resp = 0;

    DEBUG("can_netdev: Switching to mode %d\n", mode);
    switch (mode) {

        case CAN_OPMODE_INIT:
            /* Sleep off */
            dev->eparams->device->MCR &= ~CAN_MCR_SLEEP;

            /* Init request on */
            dev->eparams->device->MCR |= CAN_MCR_INRQ;

            /* wait for hardware confirmation */
            while ((!(dev->eparams->device->MSR & CAN_MSR_INAK) ||
                     (dev->eparams->device->MSR & CAN_MSR_SLAK)) &&
                      wait_cnt > 0) {
                wait_cnt--;
            }
            break;

        case CAN_OPMODE_NORMAL:
            /* Init & sleep off */
            dev->eparams->device->MCR &= ~(CAN_MCR_INRQ | CAN_MCR_SLEEP);

            /* wait for hardware confirmation */
            while (((dev->eparams->device->MSR & CAN_MSR_INAK) ||
                    (dev->eparams->device->MSR & CAN_MSR_SLAK)) &&
                     wait_cnt > 0) {
                wait_cnt--;
            }
            break;

        case CAN_OPMODE_SLEEP:
            /* Init request off */
            dev->eparams->device->MCR &= ~CAN_MCR_INRQ;

            /* Sleep request on */
            dev->eparams->device->MCR |= CAN_MCR_SLEEP;
            
            /* wait for hardware confirmation */
            while (((dev->eparams->device->MSR & CAN_MSR_INAK) ||
                   !(dev->eparams->device->MSR & CAN_MSR_SLAK)) &&
                     wait_cnt > 0) {
                wait_cnt--;
            }
            break;
    }

    if (wait_cnt == 0) {
        DEBUG("can_netdev: didn't switch to mode %d\n", mode);
        resp = -1;
    }

    return resp;
}


/*------------------------------------------------------------------------------*
 *                                 Public Data                                  *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief   Ready the device for initialization through it's netdev interface
 *
 * @param[in] dev           device descriptor
 * @param[in] params        peripheral configuration to use
 * @param[in] index         Index of @p params in a global parameter struct array.
 *                          If initialized manually, pass a unique identifier instead.
 */
void can_netdev_setup(can_netdev_t *dev,
                      const socketcan_params_t   *params,
                      const can_netdev_eparams_t *eparams,
                      uint8_t index
                     )
{
    /* Load parameters' address */
    dev->params = params;

    /* Load extra parameters' address */
    dev->eparams = eparams;

    /* Load driver's interface */
    dev->netdev.driver = &can_netdev_driver;

    /* Initialize mutex */
    mutex_init(&dev->lock);

    /* Register device */
    netdev_register(&dev->netdev, NETDEV_MCP2515, index);
}

/**
 * @brief STM32 CAN periph basic config
 *
 * This set of procedures is in a different function so it can be called anywhere
 * to change the basic configuration for normal operation, media diagnostic, etc
 *
 */
int can_netdev_bconfig(can_netdev_t *dev, can_confmode_t mode)
{
#ifdef ENABLE_DEBUG
    LED0_TOGGLE;
#endif

     /*Disable interrupts before changing anything*/
#if defined(CPU_FAM_STM32F0)
    NVIC_DisableIRQ(dev->eparams->irq);
#else
    NVIC_DisableIRQ(dev->eparams->irq_rx0);
    NVIC_DisableIRQ(dev->eparams->irq_rx1);
    NVIC_DisableIRQ(dev->eparams->irq_tx);
    NVIC_DisableIRQ(dev->eparams->irq_sce);
#endif /* CPU_FAM_STM32F0 */

   /* Master channel's clock must be always enabled */
    periph_clk_en(APB1, dev->eparams->master_rcc_enable);

#if CAN_NETDEV_NUM > 1
    /* Enable current channel's clock */
    periph_clk_en(APB1, dev->eparams->rcc_mask);
#endif

    /* Init rx pin */
    gpio_init(dev->params->iface.rx_pin, GPIO_IN);
    gpio_init_analog(dev->params->iface.rx_pin);
    gpio_init(dev->params->iface.rx_pin, GPIO_IN);

    /* Init tx pin */
    gpio_init(dev->params->iface.tx_pin, GPIO_IN);
    gpio_init_analog(dev->params->iface.tx_pin);
    gpio_init(dev->params->iface.tx_pin, GPIO_OUT);

    /* Configuring for normal operation
       It's possible to configure later for other modes, like diagnostic, etc */
    switch( mode ) {

        case CAN_CONFMODE_OP:

            /* Extra steps for pin configuration */
#ifdef CPU_FAM_STM32F1
            gpio_init_af(dev->params->iface.tx_pin, GPIO_AF_OUT_PP);
#else
            gpio_init_af(dev->params->iface.rx_pin, dev->params->iface.af_op);
            gpio_init_af(dev->params->iface.tx_pin, dev->params->iface.af_op);
#endif
            break;

        case CAN_CONFMODE_NTEST:

#ifdef CPU_FAM_STM32F1
            gpio_init_af(dev->params->iface.tx_pin, GPIO_AF_OUT_PP);
#else
            gpio_init_af(dev->params->iface.rx_pin, dev->params->iface.af_ndiag);
            gpio_init_af(dev->params->iface.tx_pin, dev->params->iface.af_ndiag);
#endif
            break;
    }

    return 0;
}

/**
 * @brief STM32 CAN periph operational config
 *
 * This set of procedures is in a different function so it can be called anywhere
 * to change the configuration of things like bitrate, mode, etc
 *
 */
int can_netdev_opconfig(can_netdev_t *dev)
{
    uint32_t      config;

    /*Disable interrupts before changing anything*/
#if defined(CPU_FAM_STM32F0)
    NVIC_DisableIRQ(dev->eparams->irq);
#else
    NVIC_DisableIRQ(dev->eparams->irq_rx0);
    NVIC_DisableIRQ(dev->eparams->irq_rx1);
    NVIC_DisableIRQ(dev->eparams->irq_tx);
    NVIC_DisableIRQ(dev->eparams->irq_sce);
#endif /* CPU_FAM_STM32F0 */

    /* Set init mode */
    can_netdev_mode(dev, CAN_OPMODE_INIT);

    /* Build config word */
    config = dev->eparams->ttcm << CAN_MCR_TTCM_Pos |
             dev->eparams->abom << CAN_MCR_ABOM_Pos |
             dev->eparams->awum << CAN_MCR_AWUM_Pos |
             dev->eparams->nart << CAN_MCR_NART_Pos |
             dev->eparams->rflm << CAN_MCR_RFLM_Pos |
             dev->eparams->txfp << CAN_MCR_TXFP_Pos;

    /* Load config word */
    dev->eparams->device->MCR |= config;

    /* Calculating BRP = Tq / Tosc - 1 = Cf / (NBR * QPB) - 1 */
    *(dev->params->timing.brp) = (uint32_t)CLOCK_APB1 / 
           (uint32_t)(*(dev->params->timing.nom_bitrate) *
                       (1 + 
                        dev->params->timing.prseg +
                        dev->params->timing.phseg1 +
                        dev->params->timing.phseg2)) - 1;

    /* Configuring bit timing parameters */
    dev->eparams->device->BTR =
         (((uint32_t)dev->eparams->silm              << CAN_BTR_SILM_Pos) & CAN_BTR_SILM_Msk) |
         (((uint32_t)dev->eparams->lbkm              << CAN_BTR_LBKM_Pos) & CAN_BTR_LBKM_Msk) |
        (((uint32_t)(dev->params->timing.sjw - 1)    << CAN_BTR_SJW_Pos) & CAN_BTR_SJW_Msk) |
        (((uint32_t)(dev->params->timing.phseg2 - 1) << CAN_BTR_TS2_Pos) & CAN_BTR_TS2_Msk) |
       (((uint32_t)((dev->params->timing.phseg1 +
                     dev->params->timing.prseg) - 1) << CAN_BTR_TS1_Pos) & CAN_BTR_TS1_Msk) |
       ((uint32_t)(*(dev->params->timing.brp)) & CAN_BTR_BRP_Msk);

    /* Configure filter init state ON */
    dev->eparams->device->FMR |= CAN_FMR_FINIT_Msk;

    /* Clear start bank */
    dev->eparams->device->FMR &= ~CAN_FMR_CAN2SB_Msk;

    /* Configure start bank */
    dev->eparams->device->FMR |=
        (((uint32_t)dev->eparams->start_bank << CAN_FMR_CAN2SB_Pos) & CAN_FMR_CAN2SB_Msk);

    /* TODO: filter configuration */

    /* Configure filter init state OFF */
    dev->eparams->device->FMR &= ~CAN_FMR_FINIT_Msk;

    /* Clear interrupt flags */
    dev->eparams->device->TSR  |= CAN_TSR_RQCP2 | CAN_TSR_RQCP1  | CAN_TSR_RQCP0;
    dev->eparams->device->RF0R |= CAN_RF0R_FMP0 | CAN_RF0R_FULL0 | CAN_RF0R_FOVR0;
    dev->eparams->device->RF1R |= CAN_RF1R_FMP1 | CAN_RF1R_FULL1 | CAN_RF1R_FOVR1;
    dev->eparams->device->ESR  |= CAN_ESR_EWGF  | CAN_ESR_EPVF   | CAN_ESR_BOFF;
    dev->eparams->device->MSR  |= CAN_MSR_WKUI;
    dev->eparams->device->IER   = CAN_IER_WKUIE  | CAN_IER_EPVIE  | CAN_IER_EWGIE  | 
                                  CAN_IER_ERRIE  | CAN_IER_BOFIE  | CAN_IER_FOVIE1 | 
                                  CAN_IER_FMPIE1 | CAN_IER_FOVIE0 | CAN_IER_FMPIE0 |
                                  CAN_IER_TMEIE;

    /*Enable interrupts */
#if defined(CPU_FAM_STM32F0)
    NVIC_EnableIRQ(dev->eparams->irq);
#else
    NVIC_EnableIRQ(dev->eparams->irq_rx0);
    NVIC_EnableIRQ(dev->eparams->irq_rx1);
    NVIC_EnableIRQ(dev->eparams->irq_tx);
    NVIC_EnableIRQ(dev->eparams->irq_sce);
#endif /* CPU_FAM_STM32F0 */

    /* Set normal mode. This will ONLY be successful if peripheral can detect 11
       consecutive recessive bits. That requieres correct pin assigment, proper
       connections, etc */
    can_netdev_mode(dev, CAN_OPMODE_NORMAL);

    return 0;
}



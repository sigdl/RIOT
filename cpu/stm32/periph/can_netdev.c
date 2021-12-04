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

#define ENABLE_DEBUG            0
#include "debug.h"
#include "log.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
#define TRANSF_BUFFER_SIZE      5
#define MODE_MAX_DELAY          (10000U)

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
static int  can_netdev_mode(can_netdev_t *dev, can_mode_t mode);
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
 * The MCP2515 device is initializad
 *
 * @param[in]  netdev       Netdev device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int can_netdev_init(netdev_t *netdev)
{
    can_netdev_t *dev = container_of(netdev, can_netdev_t, netdev);
    uint32_t      config;

    /* Master channel's clock must be always enabled */
    periph_clk_en(APB1, dev->eparams->master_rcc_enable);

#if CAN_NETDEV_NUM > 1
    /* Enable current channel's clock */
    periph_clk_en(APB1, dev->eparams->rcc_mask);
#endif

    /* Init rx pin */
    gpio_init(dev->params->iface.rx_pin, GPIO_IN);
    gpio_init_analog(dev->params->iface.rx_pin);

    /* Init tx pin */
    gpio_init(dev->params->iface.tx_pin, GPIO_IN);
    gpio_init_analog(dev->params->iface.tx_pin);
    gpio_init(dev->params->iface.tx_pin, GPIO_OUT);

    /* Extra steps for pin configuration */
#ifndef CPU_FAM_STM32F1
    gpio_init_af(dev->params->iface.rx_pin, dev->params->iface.af);
    gpio_init_af(dev->params->iface.tx_pin, dev->params->iface.af);
#else
    gpio_init_af(dev->params->iface.tx_pin, GPIO_AF_OUT_PP);
#endif
    
    /* Set init mode */
    can_netdev_mode(dev, CAN_MODE_INIT);

    /* Build config word */
    config    = dev->eparams->ttcm << CAN_MCR_TTCM_Pos |
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

    /* Clear flags */
    dev->eparams->device->TSR |= CAN_TSR_RQCP2 | CAN_TSR_RQCP1 | CAN_TSR_RQCP0;

    /*Enable interrupts */
#if defined(CPU_FAM_STM32F0)
    NVIC_EnableIRQ(dev->eparams->irq);
#else
    NVIC_EnableIRQ(dev->eparams->irq_rx0);
    NVIC_EnableIRQ(dev->eparams->irq_rx1);
    NVIC_EnableIRQ(dev->eparams->irq_tx);
    NVIC_EnableIRQ(dev->eparams->irq_sce);
#endif

    /* Set normal mode */
    can_netdev_mode(dev, CAN_MODE_NORMAL);

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

    (void)mailbox;

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
    netdev++;
    opt++;
    value++;
    max_len++;

    return 0;
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
static int can_netdev_mode(can_netdev_t *dev, can_mode_t mode)
{
    uint32_t wait_cnt = MODE_MAX_DELAY;
    int resp = 0;

    switch (mode) {

        case CAN_MODE_INIT:
            /* Sleep off */
            dev->eparams->device->MCR &= ~CAN_MCR_SLEEP;

            /* Init request on */
            dev->eparams->device->MCR |= CAN_MCR_INRQ;

            /* wait for hardware confirmation */
            while ((!(dev->eparams->device->MSR & CAN_MSR_INAK) ||
                     (dev->eparams->device->MSR & CAN_MSR_SLAK)) &&
                      wait_cnt != 0) {
                wait_cnt--;
            }
            break;

        case CAN_MODE_NORMAL:
            /* Init & sleep off */
            dev->eparams->device->MCR &= ~(CAN_MCR_INRQ | CAN_MCR_SLEEP);

            /* wait for hardware confirmation */
            while (((dev->eparams->device->MSR & CAN_MSR_INAK) ||
                    (dev->eparams->device->MSR & CAN_MSR_SLAK)) &&
                     wait_cnt != 0) {
                wait_cnt--;
            }
            break;

        case CAN_MODE_SLEEP:
            /* Init request off */
            dev->eparams->device->MCR &= ~CAN_MCR_INRQ;

            /* Sleep request on */
            dev->eparams->device->MCR |= CAN_MCR_SLEEP;
            
            /* wait for hardware confirmation */
            while (((dev->eparams->device->MSR & CAN_MSR_INAK) ||
                   !(dev->eparams->device->MSR & CAN_MSR_SLAK)) &&
                     wait_cnt != 0) {
                wait_cnt--;
            }
            break;
    }

    if (wait_cnt == 0) {
        DEBUG("candev_stm32: didn't switch mode %d\n", mode);
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
 * @param[in]   index       Index of @p params in a global parameter struct array.
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

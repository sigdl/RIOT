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

#include "can_nd.h"
#include "net/sock/can.h"
#include "net/l2filter.h"

#define ENABLE_DEBUG            1
#include "debug.h"
#include "log.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
#define TRANSF_BUFFER_SIZE      5
#define MODE_MAX_DELAY          (100000U)

/* Convenient alias for IRQ handlers */
#if defined(CPU_FAM_STM32F1)
    #define ISR_CAN1_TX         isr_usb_hp_can1_tx
    #define ISR_CAN1_RX0        isr_usb_lp_can1_rx0
    #define ISR_CAN1_RX1        isr_can1_rx1
    #define ISR_CAN1_SCE        isr_can1_sce
#else
    #define ISR_CAN1_TX         isr_can1_tx
    #define ISR_CAN1_RX0        isr_can1_rx0
    #define ISR_CAN1_RX1        isr_can1_rx1
    #define ISR_CAN1_SCE        isr_can1_sce
    #define ISR_CAN2_TX         isr_can2_tx
    #define ISR_CAN2_RX0        isr_can2_rx0
    #define ISR_CAN2_RX1        isr_can2_rx1
    #define ISR_CAN2_SCE        isr_can2_sce
    #define ISR_CAN3_TX         isr_can3_tx
    #define ISR_CAN3_RX0        isr_can3_rx0
    #define ISR_CAN3_RX1        isr_can3_rx1
    #define ISR_CAN3_SCE        isr_can3_sce
#endif

/*------------------------------------------------------------------------------*
 *                                 Private Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                          Private Function Prototypes                         *
 *------------------------------------------------------------------------------*/
int         pcan_nd_basicconf(can_nd_t *dev, can_confmode_t mode);
int         pcan_nd_opconf(can_nd_t *dev);
static int  pcan_nd_init(netdev_t *netdev);
/*static void pcan_nd_irq_h(void *arg);*/
static void pcan_nd_isr(netdev_t *netdev);
static int  pcan_nd_send(netdev_t *netdev, const iolist_t *iolist);
static int  pcan_nd_recv(netdev_t *netdev, void *buf, size_t max_len, void *info);
static int  pcan_nd_get(netdev_t *netdev, netopt_t opt, void *value, size_t max_len);
static int  pcan_nd_set(netdev_t *netdev, netopt_t opt, const void *value, size_t value_len);
static int  pcan_nd_mode(can_nd_t *dev, socketcan_opmode_t mode);
int         pcan_filterconf(can_nd_t *dev, l2filterbank_t *filter);
static inline void rx_isr(can_nd_t *dev, uint8_t mailbox);
static inline void tx_isr(can_nd_t *dev);
static inline void sce_isr(can_nd_t *dev);


/*------------------------------------------------------------------------------*
 *                                Private Data                                  *
 *------------------------------------------------------------------------------*/
static const netdev_driver_t pcan_nd_driver = {
    .init = pcan_nd_init,
    .isr  = pcan_nd_isr,
    .send = pcan_nd_send,
    .recv = pcan_nd_recv,
    .get  = pcan_nd_get,
    .set  = pcan_nd_set,
};

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
static int pcan_nd_init(netdev_t *netdev)
{
    socketcan_params_t *scparams = container_of(netdev,   socketcan_params_t, netdev);
    can_nd_t           *dev      = container_of(scparams, can_nd_t,       scparams);


    /*Disable interrupts before changing anything*/
#if defined(CPU_FAM_STM32F0)
    NVIC_DisableIRQ(dev->eparams->irq);
#else
    NVIC_DisableIRQ(dev->eparams->irq_rx0);
    NVIC_DisableIRQ(dev->eparams->irq_rx1);
    NVIC_DisableIRQ(dev->eparams->irq_tx);
    NVIC_DisableIRQ(dev->eparams->irq_sce);
#endif /* CPU_FAM_STM32F0 */

    /* Do basic config */
    pcan_nd_basicconf(dev, CAN_CONFMODE_OP);

    /* Do operational parameters config */
    pcan_nd_opconf(dev);

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
    /* Obtain device's structure */
    can_nd_t *dev = get_can_netdev(0);

    /* Turn IRQ off */
    NVIC_DisableIRQ(dev->eparams->irq_rx0);

    /* Turn interrupt flags OFF */
    dev->eparams->device->IER &= ~(CAN_IER_FOVIE0 |
                                   CAN_IER_FFIE0  |
                                   CAN_IER_FMPIE0);

    /* Signal RX0 interrupt */
    dev->flag_rx0 = 1;

    /* Signal event to driver's ISR */
    netdev_trigger_event_isr(&dev->scparams.netdev);

    cortexm_isr_end();
}

void ISR_CAN1_RX1(void)
{

    /* Obtain device's structure */
    can_nd_t *dev = get_can_netdev(0);

    /* Turn IRQ off */
    NVIC_DisableIRQ(dev->eparams->irq_rx1);

    /* Turn interrupt flags OFF */
    dev->eparams->device->IER &= ~(CAN_IER_FOVIE1 | 
                                   CAN_IER_FFIE1  |
                                   CAN_IER_FMPIE1);

    /* Signal RX1 interrupt */
    dev->flag_rx1 = 1;

    /* Signal event to driver's ISR */
    netdev_trigger_event_isr(&dev->scparams.netdev);

    cortexm_isr_end();
}

void ISR_CAN1_TX(void)
{
    /* Obtain device's structure */
    can_nd_t *dev = get_can_netdev(0);

    /* Turn IRQ off */
    NVIC_DisableIRQ(dev->eparams->irq_tx);

    /* Turn interrupt flag OFF */
    dev->eparams->device->IER &= ~CAN_IER_TMEIE;

    /* Signal TX interrupt */
    dev->flag_tx = 1;

    /* Signal event to driver's ISR */
    netdev_trigger_event_isr(&dev->scparams.netdev);

    cortexm_isr_end();
}

void ISR_CAN1_SCE(void)
{
    /* Obtain device's structure */
    can_nd_t *dev = get_can_netdev(0);

    /* Signal SCE interrupt */
    dev->flag_sce = 1;

    /* Signal event to driver's ISR */
    netdev_trigger_event_isr(&dev->scparams.netdev);

    cortexm_isr_end();
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
static void pcan_nd_isr(netdev_t *netdev)
{
    socketcan_params_t *scparams = container_of(netdev,   socketcan_params_t, netdev);
    can_nd_t       *dev      = container_of(scparams, can_nd_t,       scparams);

    DEBUG("[PERIPH CAN] isr(): Entering ISR in thread context\n");

    /**********     RX0 interrupt     **********/
    if(dev->flag_rx0) {

        /* Turn IRQ flagg off */
        dev->flag_rx0 = 0;

        /* Process RX0 isr */
        rx_isr(dev, 0);

        /* Turn interrupt flags ON */
        dev->eparams->device->IER |= CAN_IER_FOVIE0 |
                                     CAN_IER_FFIE0  |
                                     CAN_IER_FMPIE0;

        /* Turn IRQ on */
        NVIC_EnableIRQ(dev->eparams->irq_rx0);
    }

    /**********     RX1 interrupt     **********/
    if(dev->flag_rx1) {

        /* Turn IRQ flagg off */
        dev->flag_rx1 = 0;

        /* Process RX1 isr */
        rx_isr(dev, 1);

        /* Turn interrupt flags ON */
        dev->eparams->device->IER |= CAN_IER_FOVIE1 |
                                     CAN_IER_FFIE1  |
                                     CAN_IER_FMPIE1;

        /* Turn IRQ on */
        NVIC_EnableIRQ(dev->eparams->irq_rx1);
    }

    /**********     TX  interrupt     **********/
    if(dev->flag_tx) {

        /* Turn IRQ flagg off */
        dev->flag_tx = 0;

        /* Process TX isr */
        tx_isr(dev);

        /* Turn interrupt flag ON */
        dev->eparams->device->IER |= CAN_IER_TMEIE;

        /* Turn IRQ on */
        NVIC_EnableIRQ(dev->eparams->irq_rx0);
    }

    /**********     SCE interrupt     **********/
    if(dev->flag_sce) {

        /* Turn IRQ flagg off */
        dev->flag_sce = 0;

        /* Process TX isr */
        tx_isr(dev);

        /* Turn IRQ on */
        NVIC_EnableIRQ(dev->eparams->irq_sce);
    }
}

static inline void rx_isr(can_nd_t *dev, uint8_t mailbox)
{
    int           resp;
    uint8_t       flags;
    uint8_t       dlc;
    uint8_t       filter;
    sock_can_t   *sock = NULL;
    uint8_t       i;
    volatile uint32_t *rfr;

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

        /* Store matching filter */
        filter = (dev->eparams->device->sFIFOMailBox[mailbox].RDTR & CAN_RDT0R_FMI_Msk) >> CAN_RDT0R_FMI_Pos;

        /* Search for this filter */
        resp = sock_can_find(&dev->scparams, sock, SOCK_FILTER, filter);

        /* If no associated sock found */
        if(resp < 0) {

            /* There's no moro to do */
            return;
        }

        /* If EXTENDED FRAME */
        if(dev->eparams->device->sFIFOMailBox[mailbox].RIR & CAN_RI0R_IDE) {

            /* Save Extended ID */
            sock->buffer->rxbuf[sock->buffer->rxbuf_wr].id =
                dev->eparams->device->sFIFOMailBox[mailbox].RIR >> CAN_NETDEV_EFF_SHIFT;

            /* Save corresponding flag */
            flags |= CAN_FLAG_IDE_EXT;
        }

        /* If STANDARD FRAME */
        else {

            /* Save Standard ID */
            sock->buffer->rxbuf[sock->buffer->rxbuf_wr].id =
                dev->eparams->device->sFIFOMailBox[mailbox].RIR >> CAN_NETDEV_SFF_SHIFT;
        }

#ifdef ENABLE_DEBUG
        uint32_t debug_id = sock->buffer->rxbuf[sock->buffer->rxbuf_wr].id;
        uint8_t  debug_buffer = sock->buffer->rxbuf_wr;
#endif
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
            for(i = 0; i < 4 && i < dlc; i++) {
                sock->buffer->rxbuf[sock->buffer->rxbuf_wr].data[i] =
                    (dev->eparams->device->sFIFOMailBox[mailbox].RDLR >> (i * 8)) & 0xFF;
            }
            for(i = 0; i < 4 && i + 4 < dlc; i++) {
                sock->buffer->rxbuf[sock->buffer->rxbuf_wr].data[i + 4] =
                    (dev->eparams->device->sFIFOMailBox[mailbox].RDHR >> (i * 8)) & 0xFF;
            }

            /* Save flags */
            sock->buffer->rxbuf[sock->buffer->rxbuf_wr].flags = flags;

            /* Inc frame pointer */
            (sock->buffer->rxbuf_wr)++;

            /* If got to end of buffer */
            if(sock->buffer->rxbuf_wr == sock->buffer->rxbuf_num ) {

                /* Wrap to beginning of buffer */
                sock->buffer->rxbuf_wr = 0;
            }

#ifdef ENABLE_DEBUG
            DEBUG("[PERIPH CAN] rx_isr(): Received CAN frame ID:%lx to buffer:%d bytes:%d=",
                debug_id,
                debug_buffer,
                dlc
            );
            for(uint8_t k = 0; k < dlc; k++) {
                DEBUG(" %x", sock->buffer->rxbuf[debug_buffer].data[k]);
            }
            DEBUG("\n");
#endif
        }

        /* Release mailbox */
        *rfr |= CAN_NETDEV_RFOM_MASK;
    }

    /* Signal event */
    dev->scparams.netdev.event_callback(&dev->scparams.netdev, NETDEV_EVENT_RX_COMPLETE);
}

static inline void tx_isr(can_nd_t *dev)
{

    DEBUG("[PERIPH CAN] tx_isr(): packet transmitted\n");

    /* Signal event */
    dev->scparams.netdev.event_callback(&dev->scparams.netdev, NETDEV_EVENT_TX_COMPLETE);
}

static inline void sce_isr(can_nd_t *dev)
{
    dev++;
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
static int pcan_nd_recv(netdev_t *netdev, void *buf, size_t max_len, void *info)
{
    socketcan_params_t *scparams = container_of(netdev,   socketcan_params_t, netdev);
    can_nd_t       *dev      = container_of(scparams, can_nd_t,       scparams);

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
static int pcan_nd_send(netdev_t *netdev, const iolist_t *iolist)
{
    socketcan_params_t *scparams = container_of(netdev,   socketcan_params_t, netdev);
    can_nd_t       *dev      = container_of(scparams, can_nd_t,       scparams);
    can_frame_t        *frame;   
    uint8_t i;
    uint8_t dlc;
    uint32_t id;


    /* If mailbox 0 is ready */
    if (dev->eparams->device->TSR & CAN_TSR_TME0) {

        /* It will be mailbox 0 */
        i = 0;
    }

    /* If mailbox 1 is ready */
    else if(dev->eparams->device->TSR & CAN_TSR_TME1) {

        /* It will be mailbox 1 */
        i = 1;
    }

    /* If mailbox 2 is ready */
    else if(dev->eparams->device->TSR & CAN_TSR_TME2) {

        /* It will be mailbox 0 */
        i = 2;
    }

    /* If no mailbox is ready */
    else {
        return -EBUSY;
    }

    /* Obtaining frame */
    frame = (can_frame_t *)iolist->iol_base;

    /* If it's Extended frame */
    if((frame->flags & CAN_FLAG_IDE_MASK) == CAN_FLAG_IDE_EXT) {

        /* Config TIxR */
        dev->eparams->device->sTxMailBox[i].TIR =
            (frame->id << CAN_TI0R_EXID_Pos) & CAN_TI0R_EXID_Msk;
    }

    /* If it's Standard frame */
    else {

        /* Config TIxR */
        dev->eparams->device->sTxMailBox[i].TIR =
            (frame->id << CAN_TI0R_STID_Pos) & CAN_TI0R_STID_Msk;
        id = frame->id << CAN_TI0R_STID_Pos;
        id++;
    }

    /* If it's Remote frame */
    if(frame->flags & (CAN_FLAG_RTR_REM << CAN_FLAG_RTR_SHIFT)) {

        /* Add RTR */
        dev->eparams->device->sTxMailBox[i].TIR |= CAN_TI0R_RTR;
    }

    /* Set DLC */
    dlc = (frame->flags >> CAN_FLAG_DLC_SHIFT) & CAN_FLAG_DLC_MASK;
    dev->eparams->device->sTxMailBox[i].TDTR = (uint32_t)dlc;

    /* If TGT is indicated */
    if(frame->flags & (CAN_FLAG_TGT_YES << CAN_FLAG_TGT_SHIFT)) {

        /* Add TGT */
        dev->eparams->device->sTxMailBox[i].TDTR |= CAN_TDT0R_TGT;
    }

    /* Blank data registers */
    dev->eparams->device->sTxMailBox[i].TDLR = 0;
    dev->eparams->device->sTxMailBox[i].TDHR = 0;

    /* Load first 4 data bytes */
    for (int j = 0; j < 4 && j < dlc; j++) {
        dev->eparams->device->sTxMailBox[i].TDLR |=
            (uint32_t)(frame->data[j] << (8 * j));
    }

    /* Load last 4 data bytes */
    for (int j = 0; j < 4 && j + 4 < dlc; j++) {
        dev->eparams->device->sTxMailBox[i].TDHR |=
            (uint32_t)(frame->data[j + 4] << (8 * j));
    }

    /* Send frame */
    dev->eparams->device->sTxMailBox[i].TIR |= CAN_TI0R_TXRQ;

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
static int pcan_nd_get(netdev_t *netdev, netopt_t opt, void *value, size_t max_len)
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
        
        case NETOPT_ADDRESS:
            strncpy(value, CAN_ADDRESS, max_len);
            resp = strlen(value);
            break;

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
static int pcan_nd_set(netdev_t *netdev, netopt_t opt, const void *value, size_t value_len)
{
    netdev++;
    opt++;
    value++;
    value_len++;

    return 0;
}

/**
 * @brief STM32 CAN set mode function
 *
 * 
 *
 * @param[in]  dev          device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
static int pcan_nd_mode(can_nd_t *dev, socketcan_opmode_t mode)
{
    uint32_t wait_cnt = MODE_MAX_DELAY;
    int resp = 0;

    DEBUG("pcan_nd: Switching to mode %d\n", mode);
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
void pcan_nd_setup(can_nd_t *dev, uint8_t index)
{
    /* Load driver's interface */
    dev->scparams.netdev.driver = &pcan_nd_driver;

    /* Initialize mutex */
    mutex_init(&dev->lock);

    /* Register device */
    netdev_register(&dev->scparams.netdev, NETDEV_STM32_CAN, index);
}

/**
 * @brief STM32 CAN periph basic config
 *
 * Selects the mode (direct in/out or AF) of used pins and related configurations
 *
 */
int pcan_nd_basicconf(can_nd_t *dev, can_confmode_t mode)
{

     /*Disable interrupts before changing anything*/
#if defined(CPU_FAM_STM32F0)
    NVIC_DisableIRQ(dev->eparams->irq);
#else
    NVIC_DisableIRQ(dev->eparams->irq_rx0);
    NVIC_DisableIRQ(dev->eparams->irq_rx1);
    NVIC_DisableIRQ(dev->eparams->irq_tx);
    NVIC_DisableIRQ(dev->eparams->irq_sce);
#endif /* CPU_FAM_STM32F0 */

    /* Init rx pin */
    gpio_init(dev->scparams.ifparams->rx_pin, GPIO_IN);

    /* Init tx pin */
    gpio_init(dev->scparams.ifparams->tx_pin, GPIO_OUT);

    /* Selecting configuration */
    switch( mode ) {

        /* Configuring for normal operation */
        case CAN_CONFMODE_OP:

            /* Master channel's clock must be always enabled */
            periph_clk_en(APB1, dev->eparams->master_rcc_enable);

#if CAN_NETDEV_NUM > 1
            /* Enable current channel's clock */
            periph_clk_en(APB1, dev->eparams->rcc_mask);
#endif

            /* Extra steps for pin configuration */
#ifdef CPU_FAM_STM32F1
            gpio_init_af(dev->scparams.ifparams->tx_pin, GPIO_AF_OUT_PP);
#else
            gpio_init_af(dev->scparams.ifparams->rx_pin, dev->scparams.ifparams->af_op);
            gpio_init_af(dev->scparams.ifparams->tx_pin, dev->scparams.ifparams->af_op);
#endif
            break;

        /* Configuring for physical network testing */
        case CAN_CONFMODE_NTEST:

            /* Set TX pin to 0 */
            gpio_clear(dev->scparams.ifparams->tx_pin);

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
int pcan_nd_opconf(can_nd_t *dev)
{
    uint32_t      config;

    /* Set init mode */
    pcan_nd_mode(dev, CAN_OPMODE_INIT);

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
    *(dev->scparams.timing->brp) = (uint32_t)CLOCK_APB1 / 
           (uint32_t)(*(dev->scparams.timing->nom_bitrate) *
                       (1 + 
                        dev->scparams.timing->prseg +
                        dev->scparams.timing->phseg1 +
                        dev->scparams.timing->phseg2)) - 1;

    /* Configuring bit timing parameters */
    dev->eparams->device->BTR =
         (((uint32_t)dev->eparams->silm              << CAN_BTR_SILM_Pos) & CAN_BTR_SILM_Msk) |
         (((uint32_t)dev->eparams->lbkm              << CAN_BTR_LBKM_Pos) & CAN_BTR_LBKM_Msk) |
        (((uint32_t)(dev->scparams.timing->sjw - 1)    << CAN_BTR_SJW_Pos)  & CAN_BTR_SJW_Msk)  |
        (((uint32_t)(dev->scparams.timing->phseg2 - 1) << CAN_BTR_TS2_Pos)  & CAN_BTR_TS2_Msk)  |
       (((uint32_t)((dev->scparams.timing->phseg1 +
                     dev->scparams.timing->prseg) - 1) << CAN_BTR_TS1_Pos)  & CAN_BTR_TS1_Msk)  |
       ((uint32_t)(*(dev->scparams.timing->brp)) & CAN_BTR_BRP_Msk);

    /* Configure filter init state ON */
    dev->eparams->device->FMR |= CAN_FMR_FINIT_Msk;

    /* Initialize next filter index */
    dev->scparams.netdev.filter_idx = 0;

    /* Clear start bank */
    dev->eparams->device->FMR &= ~CAN_FMR_CAN2SB_Msk;

    /* Configure start bank */
    dev->eparams->device->FMR |=
        (((uint32_t)dev->eparams->start_bank << CAN_FMR_CAN2SB_Pos) & CAN_FMR_CAN2SB_Msk);

    /* Configure filter init state OFF */
    dev->eparams->device->FMR &= ~CAN_FMR_FINIT_Msk;

    /* Clear interrupt flags */
    dev->eparams->device->TSR  |= CAN_TSR_RQCP2 | CAN_TSR_RQCP1  | CAN_TSR_RQCP0;
    dev->eparams->device->RF0R |= CAN_RF0R_FMP0 | CAN_RF0R_FULL0 | CAN_RF0R_FOVR0;
    dev->eparams->device->RF1R |= CAN_RF1R_FMP1 | CAN_RF1R_FULL1 | CAN_RF1R_FOVR1;
    dev->eparams->device->ESR  |= CAN_ESR_EWGF  | CAN_ESR_EPVF   | CAN_ESR_BOFF;
    dev->eparams->device->MSR  |= CAN_MSR_WKUI;

    dev->eparams->device->IER   = CAN_IER_WKUIE  | CAN_IER_EPVIE  | CAN_IER_ERRIE  |
                                  CAN_IER_BOFIE  | CAN_IER_EWGIE  |
                                  CAN_IER_FOVIE1 | CAN_IER_FFIE1  | CAN_IER_FMPIE1 | 
                                  CAN_IER_FOVIE0 | CAN_IER_FFIE0  | CAN_IER_FMPIE0 | 
                                  CAN_IER_TMEIE;

    /* Clear interrupt flags */
    dev->flag_rx0 = 0;
    dev->flag_rx1 = 0;
    dev->flag_tx  = 0;
    dev->flag_sce = 0;

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
    pcan_nd_mode(dev, CAN_OPMODE_NORMAL);

    return 0;
}

/**
 * @brief STM32 CAN periph filter config
 *
 *
 */
int pcan_filterconf(can_nd_t *dev, l2filterbank_t *filter)
{
    uint32_t mask_on;
    uint32_t mask_off;
    uint8_t  idx_inc;

    /* Configure filter init state ON */
    dev->eparams->device->FMR |= CAN_FMR_FINIT_Msk;

    /* Calculate masks for turning ON and OFF  */
    mask_on  = 0x1 << filter->fbank_num;
    mask_off = 0xfffffffe << filter->fbank_num;

    /* Deactivate filter */
    dev->eparams->device->FA1R &= mask_off;

    /* If turning OFF filter */
    if(filter->mode == CAN_FILTERMODE_OFF) {

        /* Ending process */
        return 0;
    }

    /* If going for FIFO 1 */
    if(filter->fifo) {
        dev->eparams->device->FFA1R |= mask_on;
    }

    /* If going for FIFO 0 */
    else
    {
        dev->eparams->device->FFA1R &= mask_off;
    }

    /* Select FBM and FSC bits of FS1R and FM1R regs according to mode */
    switch (filter->mode) {

        case CAN_FILTERMODE_MSK32:
            /* FBM = 0 */
            dev->eparams->device->FM1R &= mask_off;

            /* FSC = 1 */
            dev->eparams->device->FS1R |= mask_on;

            /* One filter */
            idx_inc = 1;

            break;
    
        case CAN_FILTERMODE_ID32:
            /* FBM = 1 */
            dev->eparams->device->FM1R |= mask_on;

            /* FSC = 1 */
            dev->eparams->device->FS1R |= mask_on;

            /* Two filters */
            idx_inc = 2;
            
            break;
    
        case CAN_FILTERMODE_MSK16:
            /* FBM = 0 */
            dev->eparams->device->FM1R &= mask_off;

            /* FSC = 0 */
            dev->eparams->device->FS1R &= mask_off;

            /* Two filters */
            idx_inc = 2;
            
            break;
    
        case CAN_FILTERMODE_ID16:
            /* FBM = 1 */
            dev->eparams->device->FM1R |= mask_on;

            /* FSC = 0 */
            dev->eparams->device->FS1R &= mask_off;

            /* Four filters */
            idx_inc = 4;
            
            break;

        default:
            break;
    }

    /* Load Mask/ID values */
    dev->eparams->device->sFilterRegister[filter->fbank_num].FR1 =
        filter->can_id;
    dev->eparams->device->sFilterRegister[filter->fbank_num].FR2 =
        filter->can_mask;

    /* Make sure current filter is last in list */
    filter->next_fbank = NULL;

    /* If it's at the beginning of list */
    if(dev->scparams.netdev.first_fbank == NULL) {

        /* Register current filter as fist filter */
        dev->scparams.netdev.first_fbank = filter;
    }

    /* If it's NOT at the beginning of list */
    else {
        l2filterbank_t *last = NULL;

        /* Add filter to list */
        l2filterbank_find(&dev->scparams, last, CAN_FILTERFIND_LAST);
        last->next_fbank = filter;
    }

    /* Increment filter index */
    dev->scparams.netdev.filter_idx += idx_inc;

    /* Activate filter */
    dev->eparams->device->FA1R |= mask_on;

    /* Filter init state OFF */
    dev->eparams->device->FMR &= ~CAN_FMR_FINIT_Msk;

    return 0;
}


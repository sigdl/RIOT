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
#include <stdlib.h>
#include "ztimer.h"
#include "shell.h"
#include "shell_commands.h"

#include "can_netdev/can_netdev.h"

#define ENABLE_DEBUG            1
#include "debug.h"
#include "log.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
#define PARAM_MAX_SIZE          8

/*------------------------------------------------------------------------------*
 *                                 Private Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                          Private Function Prototypes                         *
 *------------------------------------------------------------------------------*/
int can_netdev_test(int argc, char **argv);
int can_netdev_start(int argc, char **argv);

/*------------------------------------------------------------------------------*
 *                                Private Data                                  *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                               Private Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief Parse parameters for param=value pairs
 *
 * @param[in]  argc     Number of CLI arguments
 * @param[in]  argv     Array of CLI arguments
 *
 */
int uparse_argv(char *cli_param , char *parameter, char *value)
{
    uint8_t clilen;
    uint8_t i;
    uint8_t j;

    /* Obtain number of characters in parameter */
    clilen = strlen(cli_param);

    /* Scan string */
    for(i = 0; i < clilen; i++) {

        /* If equal sign found, stop */
        if(cli_param[i] == '=') {
            break;
        }
    }

    /* If no equal sign found */
    if(i == clilen) {

        /* Return error */
        return -1;
    }

    /* Obtain parameter string */
    for(j = 0; j < i; j++) {
        parameter[j] = cli_param[j];
    }
    parameter[j] = '\0';

    /* Obtain value string */
    for(j = i + 1; j < clilen; j++) {
        value[j - i - 1] = cli_param[j];
    }
    value[j] = '\0';

    return 0;
}

/**
 * @brief STM32 CAN network test shell command
 *
 * @param[in]  argc     Number of CLI arguments
 * @param[in]  argv     Array of CLI arguments
 *
 */
int can_netdev_test(int argc, char **argv)
{
    uint32_t cnt;
    uint32_t period;
    uint32_t count;

    /* If not correctly called */
    if (argc != 3) {
        puts("Usage:can_netdev_test <iface (0-3)> <freq(KHz) (0,1,50,125,250,500)>");
        return 0;
    }

    /* Get device number */
    uint8_t device = (uint8_t)atoi(argv[1]);

    /* Reject incorrect device number */

    /* Get device structure */
    can_nd_t *dev = get_can_netdev(device);

    /* Get desired frequency */
    uint32_t freq = atoi(argv[2]);

    /* Only accept specified frequencies */
    switch(freq) {

        case CAN_TEST_0:
            break;

        case CAN_TEST_1:
            break;

        case CAN_TEST_50:
            count  = 145;
            period = 20;
            break;

        case CAN_TEST_125:
            count  = 48;
            period = 8;
            break;

        case CAN_TEST_250:
            count  = 15;
            period = 4;
            break;

        case CAN_TEST_500:
            count  = 1;
            period = 2;
            break;

        default:
            puts("Wrong freq. Must be 0,1,50,125,250,500");
            return -1;
    }

    /* Configure TX pin for testing */
    can_netdev_basicconf(dev, CAN_CONFMODE_NTEST);
#if 0
    /* Avoid divide by zero */
    if(freq != CAN_TEST_0 &&
       freq != CAN_TEST_1 ) {

        /* Get needed count in usec units*/
        count = (uint32_t)((uint32_t)1000 / freq - 1);
    
    } else {
        count = 0;
    }
#endif
    switch (freq)
    {
        case CAN_TEST_0:
            DEBUG("can_netdev_test: Setting TX pin to 0\n");

            /* Set TX pin to 0 */
            gpio_clear(dev->scparams.ifparams->tx_pin);
            break;
    
        case CAN_TEST_1:
            DEBUG("can_netdev_test: Setting TX pin to 1\n");

            /* Set TX pin to 1 */
            gpio_set(dev->scparams.ifparams->tx_pin);
            break;
    
        case CAN_TEST_500:
            DEBUG("can_netdev_test: Starting test with freq(KHz):%lu  period(useg):%lu  count:%lu\n", freq, period, count);

            /* Cycle */
            while (1) {
                /* Toggle output */
                gpio_toggle(dev->scparams.ifparams->tx_pin);
            }
            break;

        default:
            DEBUG("can_netdev_test: Starting test with freq(KHz):%lu  period(useg):%lu  count:%lu\n", freq, period, count);

            /* Cycle */
            while (1) {
        
                /* Wait needed period */
                for(cnt = 0; cnt < count; cnt++) {
                }

                /* Toggle output */
                gpio_toggle(dev->scparams.ifparams->tx_pin);
            }
    }

    return 0;
}

/**
 * @brief STM32 CAN start shell command
 *
 * @param[in]  argc     Number of CLI arguments
 * @param[in]  argv     Array of CLI arguments
 *
 */
int can_netdev_start(int argc, char **argv)
{
    /*uint8_t i;*/
    int8_t  device;
    int     resp;
    char    parameter[PARAM_MAX_SIZE];
    char    value[PARAM_MAX_SIZE];


    /* If not correctly called */
    if (argc < 3) {
        puts("Usage: cnds iface=[0-3] <param=value> <param=value>...");
        return 0;
    }

    /* Parse iface */
    resp = uparse_argv(argv[1], parameter, value);

    /* If wrong argument */
    if(resp < 0) {
        printf("Parameter incorrect: %s\n", argv[1]);
        return 0;
    }

    /* Param IFACE */
    if(!strcmp(parameter, "iface")) {

        /* Get device number */
        device = (uint8_t)atoi(value);

        /* If wrong number */
        if(device < 0 || device > 2) {

            printf("Wrong iface: %s\n", value);
            return 0;
        }
    }

    /* Get device structure */
    can_nd_t *dev = get_can_netdev(device);
#if 0
    /* Scan args */
    for (i = 2; i < argc; i++)
    {
        /* Parse argument */
        resp = uparse_argv(argv[i], parameter, value);

        /* If wrong argument */
        if(resp < 0) {
            printf("Parameter incorrect: %s\n", argv[i]);
            return 0;
        }

        printf("Param:%s   Value:%s\n", parameter, value);

        /* Param FREQ */
        if(!strcmp(parameter, "freq")) {

            /* Get frequency */
            uint16_t freq = (uint16_t)atoi(value);
            

        }

        /* Param  */
        if(!strcmp(parameter, "")) {

        }

        /* Param */
        if(!strcmp(parameter, "")) {

        }

        /* Param TTCM*/
        if(!strcmp(parameter, "ttcm")) {

        }

        /* Param ABOM */
        if(!strcmp(parameter, "abom")) {

        }

        /* Param AWUM */
        if(!strcmp(parameter, "awum")) {

        }

        /* Param NART */
        if(!strcmp(parameter, "nart")) {

        }

        /* Param RFLM */
        if(!strcmp(parameter, "rflm")) {

        }

        /* Param TXFP */
        if(!strcmp(parameter, "txfp")) {

        }

        /* Param LBKM */
        if(!strcmp(parameter, "lbkm")) {

        }

        /* Param SILM */
        if(!strcmp(parameter, "silm")) {

        }

    }
#endif

    /* Reject incorrect device number */

    /* Get desired frequency */
    uint32_t freq = atoi(argv[2]);

    /* Only accept specified frequencies */
    switch(freq) {

        case CAN_TEST_50:
        case CAN_TEST_125:
        case CAN_TEST_250:
        case CAN_TEST_500:
        case CAN_TEST_1000:
            break;

        default:
            puts("Wrong freq. Must be 50,125,250,500,1000");
            return -1;
    }

    /* Do basic config */
    can_netdev_basicconf(dev, CAN_CONFMODE_OP);

    /* Do operational parameters config */
    can_netdev_opconf(dev);

    return 0;
}

/**
 * @brief STM32 CAN filter config
 *
 * @param[in]  argc     Number of CLI arguments
 * @param[in]  argv     Array of CLI arguments
 *
 */
int can_netdev_filter(int argc, char **argv)
{
    uint8_t i;
    char    parameter[PARAM_MAX_SIZE];
    char    value[PARAM_MAX_SIZE];
    int     resp;
    int8_t  iface  = 0;    
    uint8_t filter = 0;
    uint8_t fifo   = 0;
    uint32_t val1  = 0;
    uint32_t val2  = 0;
    socketcan_filtermode_t mode = CAN_FILTERMODE_MSK32;
    can_nd_t *dev;


    /* If there are parameters */
    if (argc > 1) {

        /* Parse iface */
        resp = uparse_argv(argv[1], parameter, value);

        /* If wrong argument */
        if(resp < 0) {
            puts("Usage: cndl iface=[0-3] filter=x fifo=y mode=[mask32,id32] val1=w val2=z");
            return 0;
        }

        /* Param IFACE */
        if(!strcmp(parameter, "iface")) {

            /* Get device number */
            iface = (uint8_t)atoi(value);

            /* If wrong number */
            if(iface < 0 || iface > 2) {

                printf("Wrong iface: %s\n", value);
                return 0;
            }
        }

        /* If not iface specified */
        else {
            
            puts("First parameter MUST be iface\n");
            return 0;
        }

        /* Get device structure */
        dev = get_can_netdev(iface);

        /* Scan args */
        for (i = 2; i < argc; i++)
        {
            /* Parse argument */
            resp = uparse_argv(argv[i], parameter, value);

            /* If wrong argument */
            if(resp < 0) {
                printf("Parameter incorrect: %s\n", argv[i]);
                return 0;
            }

            DEBUG("Param:%s   Value:%s\n", parameter, value);

            /* Param FILTER */
            if(!strcmp(parameter, "filter")) {

                /* Get filter number */
                filter = (uint8_t)atoi(value);
            }

            /* Param FIFO */
            else if(!strcmp(parameter, "fifo")) {

                /* Get filter number */
                fifo = (uint8_t)atoi(value);
            }

            /* Param MODE */
            else if(!strcmp(parameter, "mode")) {

                /* If mode = mask32 */
                if(!strcmp(value, "mask32")) {

                    mode = CAN_FILTERMODE_MSK32;
                }

                else if(!strcmp(value, "id32")) {

                    mode = CAN_FILTERMODE_ID32;
                }

                else {

                    printf("Incorrect mode: %s\n", value);
                    return 0;
                }
            }

            /* Param VAL1 */
            else if(!strcmp(parameter, "val1")) {

                /* Get device number */
                val1 = (uint32_t)atoi(value);
            }

            /* Param VAL2 */
            else if(!strcmp(parameter, "val2")) {

                /* Get device number */
                val2 = (uint32_t)atoi(value);
            }

            /* If wrong argument */
            else {
                printf("Parameter incorrect: %s\n", argv[i]);
                return 0;
            }
        }
    }

    /* If no arguments */
    else {

        /* Just get device structure */
        dev = get_can_netdev(iface);
    }

    DEBUG("Configuring filter=%u for iface=%u fifo=%u in mode=%u with val1=%lu and val2=%lu\n",
           filter,
           iface,
           fifo,
           mode,
           val1,
           val2);

    /* Configure filter */
    /*pcan_filterconf(dev, filter, fifo, mode, val1, val2);*/
    dev++;

    return 0;
}

/**
 * @brief STM32 CAN send frame shell command
 *
 * @param[in]  argc     Number of CLI arguments
 * @param[in]  argv     Array of CLI arguments
 *
 */
int can_netdev_frame(int argc, char **argv)
{
    can_frame_t frame;
    iolist_t    iolist;


    /* If not correctly called */
    if (argc < 3) {
        puts("usage:cndf <iface (0-3)> <id> <data> <data>...");
        return 0;
    }

    /* Get device number */
    uint8_t  device = (uint8_t)atoi(argv[1]);

    /* Reject incorrect device number */

    /* Get device structure */
    can_nd_t *dev = get_can_netdev(device);

    /* Configure iolist */
    iolist.iol_next = NULL;
    iolist.iol_base = &frame;
    iolist.iol_len  = sizeof(frame);

    /* Configure frame */
    frame.id = (uint32_t)atoi(argv[2]);
    for(uint8_t i = 3; i < argc && i < 3+8; i++) {
        frame.data[i - 3] = (uint8_t)atoi(argv[i]);
    }
    uint8_t dlc = ((uint8_t)(argc - 3) & CAN_FLAG_DLC_MASK) << CAN_FLAG_DLC_SHIFT;
    frame.flags = dlc;

    DEBUG("Sending frame to iface %d with id %lu and dlc = %d\n", device, frame.id, argc - 3);

    dev->scparams.netdev.driver->send(&dev->scparams.netdev, &iolist);

    return 0;
}

/*------------------------------------------------------------------------------*
 *                                 Public Data                                  *
 *------------------------------------------------------------------------------*/
static const shell_command_t shell_cmds[] = {
    { "cndt", "Test CAN network", can_netdev_test},
    { "cnds", "Start CAN device", can_netdev_start},
    { "cndf", "Send CAN frame", can_netdev_frame},
    { "cndl", "Set CAN filter", can_netdev_filter},
    { NULL, NULL, NULL }
};

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
void can_netdev_cmds(void)
{
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    shell_run(shell_cmds, line_buf, SHELL_DEFAULT_BUFSIZE);

}
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

#include "can_netdev/can_netdev_utils.h"

#define ENABLE_DEBUG            1
#include "debug.h"
#include "log.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                 Private Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                          Private Function Prototypes                         *
 *------------------------------------------------------------------------------*/
int can_netdev_test(int argc, char **argv);

/*------------------------------------------------------------------------------*
 *                                Private Data                                  *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                               Private Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief STM32 CAN network test shell command
 *
 * @param[in]  netdev       Netdev device descriptor
 *
 */
int can_netdev_test(int argc, char **argv)
{

    /* If not correctly called */
    if (argc != 3) {
        puts("usage:can_netdev_test <iface (0-3)> <freq (50,125,250,500,1000)>");
        return 0;
    }

    /* Get device */
    uint8_t device = (uint8_t)atoi(argv[2]);

    /* Reject incorrect device number */

    /* Get device structure */
    can_netdev_t *dev = get_can_netdev(device);

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

    /* Configure TX pin for testing */
    can_netdev_bconfig(dev, CAN_CONFMODE_NTEST);

    /* Get needed count in usec units*/
    uint32_t count = (uint32_t)1000 / freq;

    /* Cycle */
    while (1) {
        
        /* Wait needed period */
        ztimer_sleep(ZTIMER_USEC, count);

        /* Toggle output */
        gpio_toggle(dev->params->iface.tx_pin);
    }

    return 0;
}


/*------------------------------------------------------------------------------*
 *                                 Public Data                                  *
 *------------------------------------------------------------------------------*/
static const shell_command_t shell_cmds[] = {
    { "cndt", "Test CAN network", can_netdev_test},
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
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

/*------------------------------------------------------------------------------*
 *                                 Public Data                                  *
 *------------------------------------------------------------------------------*/
static const shell_command_t shell_cmds[] = {
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

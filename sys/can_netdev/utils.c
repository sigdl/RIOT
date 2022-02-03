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
/**
 * @brief   Search filter filter in a filter list
 *
 * @param[in]    scparams   Pointer to SocketCAN device's params structure
 * @param[inout] filter     Filter pointer for input or output
 * @param[in]    type       Type of search
 *                          
 */
int nd_filter_find(socketcan_params_t *scparams, socketcan_filterbank_t *filter, filter_find_t type)
{
    socketcan_filterbank_t *tmp;
    int filter_count;

    switch (type)
    {
        /* Find last filter in list */
        case CAN_FILTERFIND_LAST:

            /* Load beginning of filter list */
            filter = scparams->first_fbank;

            /* If no filter */
            if(filter == NULL) {

                /* Return failure */
                return -ENODATA;
            }

            while(filter->next_fbank != NULL) {

                /* Load next filter in list */
                filter = filter->next_fbank;
            }

            return 0;
            break;
    
        /* Find same filter in list */
        case CAN_FILTERFIND_SAME:

            /* Load beginning of filter list */
            tmp = scparams->first_fbank;

            /* If no filter */
            if(tmp == NULL) {

                /* Return failure */
                return -ENODATA;
            }

            /* Initialize filter count */
            filter_count = 0;

            do {
                /* If it's the same filter */
                if(tmp->fifo     == filter->fifo   &&
                   tmp->mode     == filter->mode   &&
                   tmp->can_id   == filter->can_id &&
                   tmp->can_mask == filter->can_mask
                  ) {
                    return -EEXIST;
                }

                /* Load next filter in list */
                tmp = tmp->next_fbank;

                filter_count++;

            } while(tmp->next_fbank != NULL);

            return filter_count;
            break;

        default:
            break;
    }

    return 0;
}



void can_netdev_cmds(void)
{
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    shell_run(shell_cmds, line_buf, SHELL_DEFAULT_BUFSIZE);

}
/*
 * Copyright (C) 2022 Grr <gebbet00@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    
 * @ingroup     
 * @brief       
 *
 * @{
 *
 * @file
 * @brief       
 *
 *
 * @author      Grr <gebbet00@gmail.com>
 */

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/
#include "sched.h"
#include "net/gnrc/can.h"

#define ENABLE_DEBUG        1
#include "debug.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                 Private Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                          Private Function Prototypes                         *
 *------------------------------------------------------------------------------*/
static void *_event_loop(void *args);

/*------------------------------------------------------------------------------*
 *                                Private Data                                  *
 *------------------------------------------------------------------------------*/
static char _stack[GNRC_CAN_STACK_SIZE + DEBUG_EXTRA_STACKSIZE];

/*------------------------------------------------------------------------------*
 *                               Private Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief STM32 Socketcan driver Init function
 *
 *
 * @param[in]               .
 *
 * @return                  
 * @return                  
 */
static void *_event_loop(void *args)
{
    (void)args;


    return NULL;
}

/*------------------------------------------------------------------------------*
 *                                 Public Data                                  *
 *------------------------------------------------------------------------------*/
kernel_pid_t gnrc_can_pid = KERNEL_PID_UNDEF;

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
/**
 * @brief   Ready the device for initialization through it's netdev interface
 *
 * @param[in]            .
 * @param[in]         
 * @param[in]          
 *                          
 */
kernel_pid_t gnrc_can_init(void)
{
    if (gnrc_can_pid == KERNEL_PID_UNDEF) {
        gnrc_can_pid = thread_create(_stack,
                                      sizeof(_stack),
                                      GNRC_CAN_PRIO,
                                      THREAD_CREATE_STACKTEST,
                                      _event_loop,
                                      NULL,
                                      "can");
    }

    return gnrc_can_pid;
}

/*
 * Copyright (C) 2016 Grr <gebbet00@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for
 * more details.
 */

/**
 * @defgroup    .
 * @ingroup     
 * @brief       
 * @{
 *
 * This is the SocketCAN controller generic driver interface
 *
 * @file
 * @brief       
 *
 * @author      Grr <gebbet00@gmail.com>
 */

#ifndef NET_GNRC_CAN_H
#define NET_GNRC_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/
#include "sched.h"
#include "net/gnrc.h"
#include "thread.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
/**
 * @defgroup    net_gnrc_can_conf  GNRC CAN compile configurations
 * @ingroup     net_gnrc_can
 * @ingroup     net_gnrc_conf
 * @{
 */
/**
 * @brief   Default stack size to use for the IPv6 thread
 */
#ifndef GNRC_CAN_STACK_SIZE
#define GNRC_CAN_STACK_SIZE        (THREAD_STACKSIZE_DEFAULT)
#endif

/**
 * @brief   Default priority for the CAN thread
 */
#ifndef GNRC_CAN_PRIO
#define GNRC_CAN_PRIO              (THREAD_PRIORITY_MAIN - 3)
#endif

/**
 * @brief   Default message queue size to use for the CAN thread (as exponent
 *          of 2^n).
 *
 *          As the queue size ALWAYS needs to be power of two, this option
 *          represents the exponent of 2^n, which will be used as the size of
 *          the queue.
 */
#ifndef CONFIG_GNRC_CAN_MSG_QUEUE_SIZE_EXP
#define CONFIG_GNRC_CAN_MSG_QUEUE_SIZE_EXP    (3U)
#endif

/**
 * @brief Message queue size to use for the IPv6 thread.
 */
#ifndef GNRC_CAN_MSG_QUEUE_SIZE
#define GNRC_CAN_MSG_QUEUE_SIZE    (1 << CONFIG_GNRC_CAN_MSG_QUEUE_SIZE_EXP)
#endif

/**
 * @brief   The PID to the CAN thread.
 *
 * @note    Use @ref gnrc_ipv6_init() to initialize. **Do not set by hand**.
 *
 * @details This variable is preferred for CAN internal communication *only*.
 *          Please use @ref net_gnrc_netreg for external communication.
 */
extern kernel_pid_t gnrc_ipv6_pid;


/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/



#ifdef __cplusplus
}
#endif

#endif /* NET_GNRC_CAN_H */
/** @} */

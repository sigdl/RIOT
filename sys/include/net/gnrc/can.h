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

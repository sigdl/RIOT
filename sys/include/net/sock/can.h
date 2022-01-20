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
 * 
 *
 * @file
 * @brief       
 *
 * @author      Grr <gebbet00@gmail.com>
 */

#ifndef NET_SOCK_CAN_H
#define NET_SOCK_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/
#include "can/socketcan.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/
/**
 * @brief   Forward declaration for SocketCAN sock
 * 
 * Needed for recursive use in list of socks
 */
typedef struct sock_can sock_can_t;

/**
 * @brief   Event callback for @ref sock_can_t
 */
typedef void (*sock_can_cb_t)(sock_can_t *sock, void *arg);

/**
 * @brief   Definition for SocketCAN socket
 */
struct sock_can {
    sock_can_t           *next_sock;   /**< Next sock using this interface      */
    uint8_t               num_filters; /**< Number of filter for this socket    */
    can_filter_t         *filters;     /**< Filters array                       */
    socketcan_buffer_t   *buffer;      /**< Frame buffer                        */
    socketcan_protocol_t  protocol;    /**< Socket protocol                     */
    sock_can_cb_t         cb;          /**< App's call back function            */
};

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/



#ifdef __cplusplus
}
#endif

#endif /* NET_SOCK_CAN_H */
/** @} */

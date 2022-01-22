/*
 * Copyright (C) 2021 Grr
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    can_netdev_utils test
 * @ingroup     can_netdev_utils
 * @brief       Utility for testing CAN wiring
 *
 * @{
 *
 * @file
 * @brief       
 *
 *
 * @author      Grr <gebbet00@gmail.com>
 */

#ifndef CAN_NETDEV_TEST_H
#define CAN_NETDEV_TEST_H

/*------------------------------------------------------------------------------*
 *                                Included Files                                *
 *------------------------------------------------------------------------------*/
#include "can_nd.h"

/*------------------------------------------------------------------------------*
 *                           Pre-processor Definitions                          *
 *------------------------------------------------------------------------------*/
#define CAN_TEST_0          0
#define CAN_TEST_1          1
#define CAN_TEST_50         50
#define CAN_TEST_125        125
#define CAN_TEST_250        250
#define CAN_TEST_500        500
#define CAN_TEST_1000       1000

/*------------------------------------------------------------------------------*
 *                                  Public Types                                *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
 *                                Public Functions                              *
 *------------------------------------------------------------------------------*/
void can_netdev_cmds(void);

#endif /* CAN_NETDEV_TEST_H */

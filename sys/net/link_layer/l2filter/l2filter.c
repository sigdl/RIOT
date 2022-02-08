/*
 * Copyright (C) 2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     net_l2filter
 * @{
 *
 * @file
 * @brief       Link layer address filter implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <string.h>

#include "assert.h"
#include "net/l2filter.h"

#ifdef MODULE_L2FILTERBANK
  #include "can/socketcan.h"
#endif

#define ENABLE_DEBUG 0
#include "debug.h"

static inline bool match(const l2filter_t *filter,
                         const void *addr, size_t addr_len)
{
    return ((filter->addr_len == addr_len) &&
            (memcmp(filter->addr, addr, addr_len) == 0));
}

void l2filter_init(l2filter_t *list)
{
    assert(list);

    for (unsigned i = 0; i < CONFIG_L2FILTER_LISTSIZE; i++) {
        list[i].addr_len = 0;
    }
}

int l2filter_add(l2filter_t *list, const void *addr, size_t addr_len)
{
    assert(list && addr && (addr_len <= CONFIG_L2FILTER_ADDR_MAXLEN));

    int res = -ENOMEM;

    for (unsigned i = 0; i < CONFIG_L2FILTER_LISTSIZE; i++) {
        if (list[i].addr_len == 0) {
            list[i].addr_len = addr_len;
            memcpy(list[i].addr, addr, addr_len);
            res = 0;
            break;
        }
    }

    return res;
}

int l2filter_rm(l2filter_t *list, const void *addr, size_t addr_len)
{
    assert(list && addr && (addr_len <= CONFIG_L2FILTER_ADDR_MAXLEN));

    int res = -ENOENT;

    for (unsigned i = 0; i < CONFIG_L2FILTER_LISTSIZE; i++) {
        if (match(&list[i], addr, addr_len)) {
            list[i].addr_len = 0;
            res = 0;
            break;
        }
    }

    return res;
}

bool l2filter_pass(const l2filter_t *list, const void *addr, size_t addr_len)
{
    assert(list && addr && (addr_len <= CONFIG_L2FILTER_ADDR_MAXLEN));

#ifdef MODULE_L2FILTER_WHITELIST
    bool res = false;
    for (unsigned i = 0; i < CONFIG_L2FILTER_LISTSIZE; i++) {
        if (match(&list[i], addr, addr_len)) {
            DEBUG("[l2filter] whitelist: address match -> packet passes\n");
            res = true;
            break;
        }
    }
    DEBUG("[l2filter] whitelist: no match -> packet dropped\n");
#else
    bool res = true;
    for (unsigned i = 0; i < CONFIG_L2FILTER_LISTSIZE; i++) {
        if (match(&list[i], addr, addr_len)) {
            DEBUG("[l2filter] blacklist: address match -> packet dropped\n");
            res = false;
            break;
        }
    }
    DEBUG("[l2fitler] blacklist: no match -> packet passes\n");
#endif

    return res;
}

/**
 * @brief   Search filter filter in a filter list
 *
 * @param[in]    scparams   Pointer to SocketCAN device's params structure
 * @param[inout] filter     Filter pointer for input or output
 * @param[in]    type       Type of search
 *                          
 */
#ifdef MODULE_L2FILTERBANK
int l2filterbank_find(socketcan_params_t *scparams, l2filterbank_t *filter, l2filterbank_find_t type)
{
    l2filterbank_t *tmp;
    int filter_count;

    switch (type)
    {
        /* Find last filter in list */
        case CAN_FILTERFIND_LAST:

            /* Load beginning of filter list */
            filter = scparams->netdev.first_fbank;

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
            tmp = scparams->netdev.first_fbank;

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
#endif /* MODULE_L2FILTERBANK */

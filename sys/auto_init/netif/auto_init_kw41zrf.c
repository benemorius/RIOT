/*
 * Copyright (C) 2017 SKF AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/*
 * @ingroup auto_init_gnrc_netif
 * @{
 *
 * @file
 * @brief   Auto initialization for kw41zrf network interfaces
 *
 * @author  Joakim Nohlgård <joakim.nohlgard@eistec.se>
 */

#ifdef MODULE_KW41ZRF

#include "log.h"
#include "board.h"
#include "net/gnrc/netdev.h"
#include "net/gnrc/netdev/ieee802154.h"
#include "net/gnrc.h"

#include "kw41zrf.h"

/**
 * @brief   Define stack parameters for the MAC layer thread
 * @{
 */
#ifndef KW41ZRF_MAC_STACKSIZE
#define KW41ZRF_MAC_STACKSIZE     (THREAD_STACKSIZE_DEFAULT)
#endif
#ifndef KW41ZRF_MAC_PRIO
#define KW41ZRF_MAC_PRIO          (GNRC_NETDEV_MAC_PRIO)
#endif

/* There is only one memory mapped transceiver in the supported SoCs, the driver
 * does not try to take into account multiple instances of the hardware module */
#define KW41ZRF_NUMOF 1

static kw41zrf_t kw41zrf_devs[KW41ZRF_NUMOF];
static gnrc_netdev_t gnrc_adpt[KW41ZRF_NUMOF];
static char _kw41zrf_stacks[KW41ZRF_NUMOF][KW41ZRF_MAC_STACKSIZE];

void auto_init_kw41zrf(void)
{
    for (unsigned i = 0; i < KW41ZRF_NUMOF; i++) {
        LOG_DEBUG("[auto_init_netif] initializing kw41zrf #%u\n", i);
        kw41zrf_setup(&kw41zrf_devs[i]);
        if (gnrc_netdev_ieee802154_init(&gnrc_adpt[i], (netdev_ieee802154_t *)&kw41zrf_devs[i]) < 0) {
            LOG_ERROR("[auto_init_netif] error initializing kw41zrf #%u\n", i);
        }
        else {
            gnrc_netdev_init(_kw41zrf_stacks[i], KW41ZRF_MAC_STACKSIZE,
                             KW41ZRF_MAC_PRIO, "kw41zrf", &gnrc_adpt[i]);
        }
    }
}
#else
typedef int dont_be_pedantic;
#endif /* MODULE_KW41ZRF */

/** @} */

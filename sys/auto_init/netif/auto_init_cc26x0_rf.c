/*
 *    Copyright (c) 2016 Thomas Stilwell <stilwellt@openlabs.co>
 *
 *    Permission is hereby granted, free of charge, to any person
 *    obtaining a copy of this software and associated documentation
 *    files (the "Software"), to deal in the Software without
 *    restriction, including without limitation the rights to use,
 *    copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the
 *    Software is furnished to do so, subject to the following
 *    conditions:
 *
 *    The above copyright notice and this permission notice shall be
 *    included in all copies or substantial portions of the Software.
 *
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *    OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @ingroup     auto_init_gnrc_netif
 * @{
 *
 * @file
 * @brief       Auto initialization for the cc26x0 network interface
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 * @}
 */

#ifdef MODULE_CC26X0_RF

#include "net/gnrc/netdev2.h"
#include "net/gnrc/netdev2/ieee802154.h"
#include "cc26x0_rf.h"

#define ENABLE_DEBUG 0
#include "debug.h"

/**
 * @brief   Define stack parameters for the MAC layer thread
 * @{
 */
#define CC26X0_RF_MAC_STACKSIZE       (THREAD_STACKSIZE_DEFAULT)
#define CC26X0_RF_MAC_PRIO            (THREAD_PRIORITY_MAIN - 4)

static cc26x0_rf_t cc26x0_rf_dev;
static gnrc_netdev2_t gnrc_adpt;
static char _cc26x0_rf_stack[CC26X0_RF_MAC_STACKSIZE];

void auto_init_cc26x0_rf(void)
{
    int res;

    DEBUG("Initializing cc26x0 radio...\n");
    cc26x0_rf_setup(&cc26x0_rf_dev);
    res = gnrc_netdev2_ieee802154_init(&gnrc_adpt,
                                       (netdev2_ieee802154_t *)&cc26x0_rf_dev);

    if (res < 0) {
        DEBUG("Error initializing cc26x0 radio device!\n");
    }
    else {
        gnrc_netdev2_init(_cc26x0_rf_stack,
                          CC26X0_RF_MAC_STACKSIZE,
                          CC26X0_RF_MAC_PRIO,
                          "cc26x0_rf",
                          &gnrc_adpt);
    }
}

#else
typedef int dont_be_pedantic;
#endif /* MODULE_CC26X0_RF */
/** @} */

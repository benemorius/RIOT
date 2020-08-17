/*
 * Copyright (C) 2020 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */


/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       strain gauge application
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include <stdio.h>

#include "msg.h"
#include "util.h"
#include "xtimer.h"
#include "random.h"
#include "periph/pm.h"
#include "periph/adc.h"
#include "net/emcute.h"
#include "net/gnrc/netif.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifndef EMCUTE_ID
#define EMCUTE_ID           ("strain-gauge")
#endif
#define EMCUTE_PORT         (1883U)
#define EMCUTE_PRIO         (THREAD_PRIORITY_MAIN - 1)

#define NUMOFSUBS           (16U)
#define TOPIC_MAXLEN        (64U)

enum {
    MSG_TIMER_CB,
    MSG_EXPIRE_WAKE,
};

static char stack[THREAD_STACKSIZE_DEFAULT];

static void *emcute_thread(void *arg)
{
    (void)arg;
    emcute_run(EMCUTE_PORT, EMCUTE_ID);
    return NULL;    /* should never be reached */
}

void sleep_radio(bool sleep)
{
    netopt_state_t state = NETOPT_STATE_IDLE;
    if (sleep) {
        state = NETOPT_STATE_SLEEP;
    }
    gnrc_netif_t *netif = gnrc_netif_iter(NULL);
    gnrc_netapi_set(netif->pid, NETOPT_STATE, 0, &state, sizeof(netopt_state_t));
}

int main(void)
{
    msg_t msg_queue[4];
    msg_init_queue(msg_queue, 4);

    sleep_radio(true);

    adc_init(3);
    xtimer_usleep(10 * US_PER_MS);

    /* start the emcute thread */
    thread_create(stack, sizeof(stack), EMCUTE_PRIO, 0,
                  emcute_thread, NULL, "emcute");

    sock_udp_ep_t gw = { .family = AF_INET6, .port = 1885 };
    size_t len = 0;
    char addr[] = "fe80::d0f5:cc91:b049:83d5";

    /* parse address */
    if (ipv6_addr_from_str((ipv6_addr_t *)&gw.addr.ipv6, addr) == NULL) {
        printf("error parsing IPv6 address\n");
        return 1;
    }

    xtimer_sleep(1);
    while (1) {
        sleep_radio(false);
        emcute_discon();
        if (emcute_con(&gw, true, NULL, NULL, len, 0) != EMCUTE_OK) {
//             printf("error: unable to connect to [%s]:%i\n", addr, (int)gw.port);
            sleep_radio(true);
            xtimer_sleep(20);
            continue;
        }
        printf("connected\n");
        sleep_radio(true);
        break;
    }

    emcute_topic_t t;
    char topic[] = "strain/volts";
    char data[] = "10.000";
    unsigned flags = EMCUTE_QOS_1; /* request ack */

    while (1) {
        sleep_radio(false);
        /* step 1: get topic id */
        t.name = topic;
        if (emcute_reg(&t) != EMCUTE_OK) {
            puts("error: unable to obtain topic ID");
            sleep_radio(true);
            pm_reboot();
        }
        sleep_radio(true);
        printf("got topic ID\n");
        break;
    }

    unsigned retries = 0;
    while (1) {
        xtimer_usleep(1000 * 1000 * 0.5);
        sleep_radio(false);

        int32_t sample = adc_sample(3, ADC_RES_16BIT);
        float raw_voltage = (float)sample * ADC_REF_VOLTAGE / 65535;
        raw_voltage = raw_voltage * 21;
        int volts = raw_voltage;
        unsigned mvolts = (unsigned)(raw_voltage * 1000) % 1000;
        snprintf(data, sizeof(data), "%2i.%03u", volts, mvolts);

        /* step 2: publish data */
        if (emcute_pub(&t, data, strlen(data), flags) != EMCUTE_OK) {
            printf("error: unable to publish data to topic '%s [%i]'\n",
                   t.name, (int)t.id);
            sleep_radio(true);

            ++retries;
            if (retries == 10) {
                pm_reboot();
            }
        } else {
            retries = 0;
        }

        sleep_radio(true);
    }

    return 0;
}

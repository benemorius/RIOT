/*
 * Copyright (C) 2016 Luminița Lăzărescu <cluminita.lazarescu@gmail.com>
 *               2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_shell_commands
 * @{
 *
 * @file
 * @brief       Prints the real time offset from the system time
 *
 * @author      Luminița Lăzărescu <cluminita.lazarescu@gmail.com>
 * @author      Martine Lenders <m.lenders@fu-berlin.de>
 */

#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "net/sntp.h"
#include "net/ntp_packet.h"
#include "net/af.h"
#include "net/ipv6/addr.h"
#include "timex.h"
#include "periph/rtc.h"
#include "net/gnrc/ipv6/nib.h"

#define _DEFAULT_TIMEOUT (500000LU)

extern int settimeofday(const struct timeval *restrict tp, const void *restrict tzp);

static void _usage(char *cmd)
{
    printf("Usage: %s <server addr>[%%<interface>] [<timeout in us>] [-s]\n", cmd);
    printf("default: timeout = %lu\n", _DEFAULT_TIMEOUT);
    printf("-s: set system time\n");
    printf("-r: set RTC time\n");
}

void _get_gateway_ipv6_addr(ipv6_addr_t *addr, kernel_pid_t *iface)
{
    /* get the default router and use that for ntp */
    ipv6_addr_t dst;
    ipv6_addr_from_str(&dst, "::0");
    gnrc_ipv6_nib_ft_t entry;
    gnrc_ipv6_nib_ft_get(&dst, NULL, &entry);
    ipv6_addr_init_prefix(addr, &entry.next_hop, 128);
    *iface = entry.iface;
}

int _ntpdate(int argc, char **argv)
{
    uint32_t timeout = _DEFAULT_TIMEOUT;

    if (argc < 2) {
        _usage(argv[0]);
        return 1;
    }
    sock_udp_ep_t server = { .port = NTP_PORT, .family = AF_INET6 };
    ipv6_addr_t *addr = (ipv6_addr_t *)&server.addr;

    char *iface = ipv6_addr_split_iface(argv[1]);
    kernel_pid_t src_iface = iface ? atoi(iface) : KERNEL_PID_UNDEF;

    if (ipv6_addr_from_str(addr, argv[1]) == NULL) {
        // puts("error: malformed address");
        // return 1;
        _get_gateway_ipv6_addr(addr, &src_iface);
        char addr_str[IPV6_ADDR_MAX_STR_LEN];
        ipv6_addr_to_str(addr_str, addr, sizeof(addr_str));
//         printf("using address %s\n", addr_str);
        ipv6_addr_from_str(addr, "2001:470:4bb0:815::2");
    }

    if (ipv6_addr_is_link_local(addr) || (src_iface != KERNEL_PID_UNDEF)) {
        size_t ifnum = gnrc_netif_numof();

        if (src_iface == KERNEL_PID_UNDEF) {
            if (ifnum == 1) {
                src_iface = gnrc_netif_iter(NULL)->pid;
            }
            else {
                puts("error: link local target needs interface parameter (use \"<address>%<ifnum>\")\n");
                return 1;
            }
        }
        else {
            if (gnrc_netif_get_by_pid(src_iface) == NULL) {
                printf("error: %"PRIkernel_pid" is not a valid interface.\n", src_iface);
                return 1;
            }
        }
        server.netif = src_iface;
    }

    if (argc > 2) {
        timeout = atoi(argv[2]);
    }
    if (timeout == 0) {
        timeout = US_PER_SEC;
    }
    if (sntp_sync(&server, timeout) < 0) {
        puts("Error in synchronization");
        return 1;
    }
#ifdef MODULE_NEWLIB
    uint64_t unix_usec = sntp_get_unix_usec();
    time_t time = (time_t)(unix_usec / US_PER_SEC);
    struct tm tm;
    gmtime_r(&time, &tm);

    /* compare ntp time with system time */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t system_usec = tv.tv_usec + tv.tv_sec * US_PER_SEC;
    int32_t difference_usec = (int32_t)(system_usec - unix_usec);

    /* set system time */
    if (argc > 3 && strcmp(argv[3], "-s") == 0) {
        const struct timeval tv = {unix_usec / US_PER_SEC, unix_usec % US_PER_SEC};
        settimeofday(&tv, NULL);
    }

    /* set rtc time */
    if (argc > 3 && strcmp(argv[3], "-r") == 0) {
        // const struct timeval tv = {unix_usec / US_PER_SEC, unix_usec % US_PER_SEC};
        // settimeofday(&tv, NULL);

        /* wait until the end of the current second */
        tm.tm_sec += 1;
        uint32_t us_to_wait = US_PER_SEC - (unix_usec % US_PER_SEC);
        /* we need more time, so skip into the next second */
        if (us_to_wait < XTIMER_BACKOFF * 40) {
            xtimer_usleep(XTIMER_BACKOFF * 80);
            tm.tm_sec += 1;
            unix_usec = sntp_get_unix_usec();
            us_to_wait = US_PER_SEC - (unix_usec % US_PER_SEC);
        }
        us_to_wait -= XTIMER_BACKOFF * 225;

        /* check that we waited the right amount of time */
        uint64_t before = xtimer_now_usec64();
        xtimer_usleep64(us_to_wait);
        uint64_t after = xtimer_now_usec64();
        uint32_t waited_us = (uint32_t)(after - before);

        // /* wait until next rtc tick */
        // struct tm rtc_tm;
        // rtc_get_time(&rtc_tm);
        // int current_tm_sec = rtc_tm.tm_sec;
        // uint64_t before = xtimer_now_usec64();
        // while (rtc_tm.tm_sec == current_tm_sec) {
        //     if (rtc_get_time(&rtc_tm) != 0) {
        //         return 1;
        //     }
        // }
        // uint64_t after = xtimer_now_usec64();
        // uint32_t waited_us = (uint32_t)(after - before);
        // tm.tm_sec += 1;

        /* set rtc time */
        rtc_set_time(&tm);

        /* print how long we waited */
        unix_usec = sntp_get_unix_usec();
        struct tm tm_now;
        gmtime_r(&time, &tm_now);
        printf("wrote second %i at second %i\n", tm.tm_sec, tm_now.tm_sec);

        printf("waited %lu usecs (wanted %lu usecs) (error %li usecs)\n", waited_us, us_to_wait, (int32_t)waited_us - us_to_wait);
    }

    /* print time */
    printf("%04i-%02i-%02i %02i:%02i:%02i UTC (%lu ms) (%li us)\n",
           tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
           tm.tm_hour, tm.tm_min, tm.tm_sec,
           (uint32_t)(sntp_get_offset() / 1000),
           difference_usec);
#else
    uint64_t time = sntp_get_unix_usec();
    printf("%" PRIu32 ".%" PRIu32 " (%i us)\n",
           (uint32_t)(time / US_PER_SEC),
           (uint32_t)(time / US_PER_SEC),
           (int)sntp_get_offset());
#endif
    return 0;
}

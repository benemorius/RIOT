#if defined MODULE_SNTP || defined DOXYGEN

#include <sys/time.h>

#include "log.h"
#include "net/sntp.h"
#include "net/gnrc/ipv6/nib/ft.h"

#define DEFAULT_TIMEOUT (500000LU)

extern int settimeofday(const struct timeval *restrict tp, const void *restrict tzp);

int get_default_ipv6_router(ipv6_addr_t *addr, uint16_t *iface)
{
    ipv6_addr_t dst;
    ipv6_addr_from_str(&dst, "::0");
    gnrc_ipv6_nib_ft_t entry;

    if (gnrc_ipv6_nib_ft_get(&dst, NULL, &entry)) {
        return -ENETUNREACH;
    }

    ipv6_addr_init_prefix(addr, &entry.next_hop, 128);
    *iface = entry.iface;

    return 0;
}

int ntpdate(void)
{
    sock_udp_ep_t server = { .port = NTP_PORT, .family = AF_INET6 };

    /* use default router as ntp server */
    if (get_default_ipv6_router((ipv6_addr_t *)&server.addr, &server.netif)) {
//         LOG_DEBUG("[ntp] no gateway available\n");
        return 1;
    }

    ipv6_addr_from_str((ipv6_addr_t *)&server.addr, "2001:470:4bb0:815::2");

    char ip_str[IPV6_ADDR_MAX_STR_LEN];
    ipv6_addr_to_str(ip_str, (ipv6_addr_t *)&server.addr, IPV6_ADDR_MAX_STR_LEN);

    /* get time from ntp server */
    if (sntp_sync(&server, DEFAULT_TIMEOUT) < 0) {
        LOG_WARNING("could not synchronize with NTP server %s\n", ip_str);
        return 1;
    }
    uint64_t unix_usec = sntp_get_unix_usec();

    /* compare ntp time with system time */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t system_usec = tv.tv_usec + tv.tv_sec * US_PER_SEC;
    int64_t difference_usec = system_usec - unix_usec;
    int32_t difference_sec = difference_usec / US_PER_SEC;

    /* set system time */
    const struct timeval tv_now = {unix_usec / US_PER_SEC, unix_usec % US_PER_SEC};
    settimeofday(&tv_now, NULL);

    LOG_INFO("system time set from server %s (error was %li.%06lu seconds)\n",
             ip_str, difference_sec, (uint32_t)(abs(difference_usec) % US_PER_SEC));

    return 0;
}

#endif /* MODULE_SNTP */

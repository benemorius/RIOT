#if defined MODULE_GNRC_SOCK_UDP || defined DOXYGEN

#include "log.h"
#include "util.h"
#include "net/sock/udp.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

static char _udp_server_thread_stack [1024 + 1024];
static void *_udp_server_thread(void *arg);

#define SERVER_BUFFER_SIZE      (32)

#define MAIN_QUEUE_SIZE     (8)
static msg_t udp_server_msg_queue[MAIN_QUEUE_SIZE];

#define _IPV6_DEFAULT_PREFIX_LEN        (64U)

static uint8_t _get_prefix_len(char *addr)
{
    int prefix_len = ipv6_addr_split_prefix(addr);
    if (prefix_len < 1) {
        prefix_len = _IPV6_DEFAULT_PREFIX_LEN;
    }
    return prefix_len;
}

ssize_t udp_send(const char *dest_ipv6, uint16_t port, const char *data, size_t data_len)
{
    sock_udp_ep_t remote = SOCK_IPV6_EP_ANY;
    ipv6_addr_from_str((ipv6_addr_t *)&remote.addr.ipv6, dest_ipv6);
    remote.port = port;

    ssize_t res;
    if ((res = sock_udp_send(NULL, data, data_len, &remote)) < 0)
    {
        LOG_ERROR("sock_udp_send() returned %i\n", res);
    }

    return res;
}

static void *_udp_server_thread(void *arg)
{
    udp_server_t *srv = arg;
    char server_buffer[SERVER_BUFFER_SIZE];

    msg_init_queue(udp_server_msg_queue, MAIN_QUEUE_SIZE);

    sock_udp_ep_t local = SOCK_IPV6_EP_ANY;
    ipv6_addr_from_str((ipv6_addr_t *)&local.addr.ipv6, srv->address_str);
    local.port = srv->port;

    int res;
    if ((res = sock_udp_create(&srv->sock, &local, NULL, 0)) < 0) {
        LOG_ERROR("[udp server] error initializing socket (%i)\n", res);
        return NULL;
    }

    LOG_INFO("[udp server] started listening on [%s]:%" PRIu16 "\n",
             srv->address_str, srv->port);

    while (1) {
        ssize_t res;
        if ((res = sock_udp_recv(&srv->sock, server_buffer, sizeof(server_buffer),
            SOCK_NO_TIMEOUT, &srv->remote)) < 0)
        {
            LOG_WARNING("sock_udp_recv() returned %i\n", res);
            continue;
        }

        char remote_ip[IPV6_ADDR_MAX_STR_LEN];
        ipv6_addr_to_str(remote_ip, (const ipv6_addr_t *)srv->remote.addr.ipv6,
                         sizeof(remote_ip));
        DEBUG("received datagram from [%s]:%" PRIu16 "\n",
              remote_ip, srv->remote.port);

        srv->rssi = srv->sock.rssi;
        srv->callback(server_buffer, res);
    }

    return NULL;
}

int udp_start_server(udp_server_t *srv)
{
    static char thread_name[17];
    snprintf(thread_name, sizeof(thread_name), "udp server %" PRIu16, srv->port);

    if (thread_create(_udp_server_thread_stack, sizeof(_udp_server_thread_stack),
                      THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                      _udp_server_thread, srv, thread_name)
                <= KERNEL_PID_UNDEF) {
        LOG_ERROR("error creating thread\n");
        return 1;
    }

    /* set the listening address on each interface. This is dirty */
    ipv6_addr_t addr;
    gnrc_netif_t *netif = NULL;
    while ((netif = gnrc_netif_iter(netif)) != NULL) {
        if (ipv6_addr_from_str(&addr, srv->address_str) == NULL) {
            LOG_ERROR("error: unable to parse IPv6 address.");
            return 1;
        }

        if (ipv6_addr_is_unspecified(&addr)) {
            continue;
        }

        if (ipv6_addr_is_multicast(&addr)) {
            if (gnrc_netapi_set(netif->pid, NETOPT_IPV6_GROUP, 0, &addr,
                sizeof(addr)) < 0) {
                LOG_ERROR("error: unable to join IPv6 multicast group\n");
                }
                else {
                    LOG_INFO("joined IPv6 multicast group %s%%%u\n",
                             srv->address_str, netif->pid);
                }
        }
        else {
            uint16_t flags = GNRC_NETIF_IPV6_ADDRS_FLAGS_STATE_VALID;
            flags |= (_get_prefix_len(srv->address_str) << 8U);
            if (gnrc_netapi_set(netif->pid, NETOPT_IPV6_ADDR, flags, &addr,
                sizeof(addr)) < 0) {
                LOG_ERROR("error: unable to add IPv6 address\n");
            return 1;
                }
        }
    }

    return 0;
}

#endif

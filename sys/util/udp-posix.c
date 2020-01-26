#if (defined MODULE_GNRC_UDP && defined MODULE_POSIX_SOCKETS) || defined(DOXYGEN)

#include <stdio.h>
#include <arpa/inet.h>

#include "log.h"
#include "util.h"
#include "net/sock/udp.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

static char _udp_server_thread_stack [512 + 1024];
static void *_udp_server_thread(void *arg);

#define MAIN_QUEUE_SIZE     (8)
static msg_t udp_server_msg_queue[MAIN_QUEUE_SIZE];

#define SERVER_BUFFER_SIZE      (32)
static char server_buffer[SERVER_BUFFER_SIZE];
static int server_socket = -1;
struct sockaddr_in6 server_in6;
static struct sockaddr_in6 src_in6;

#define _IPV6_DEFAULT_PREFIX_LEN        (64U)

static uint8_t _get_prefix_len(char *addr)
{
    int prefix_len = ipv6_addr_split_prefix(addr);
    if (prefix_len < 1) {
        prefix_len = _IPV6_DEFAULT_PREFIX_LEN;
    }
    return prefix_len;
}

size_t udp_send(const char *dest_ipv6, uint16_t port, const char *data, size_t data_len)
{
    struct sockaddr_in6 dest_in6;
    inet_pton(AF_INET6, dest_ipv6, &dest_in6.sin6_addr);
    dest_in6.sin6_family = AF_INET6;
    dest_in6.sin6_port = htons(port);

    int s = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) {
        LOG_ERROR("[udp server] error initializing socket\n");
        return 1;
    }

    size_t bytes_sent = 0;
    uint8_t retries = 3;
    while (bytes_sent < data_len) {
        size_t bytes_to_send = data_len - bytes_sent;
        if (bytes_to_send > 100) {
            bytes_to_send = 100;
        }
        // DEBUG("[udp server] sending %u bytes\n", bytes_to_send);
        int ret = sendto(s, &data[bytes_sent], bytes_to_send, 0, (struct sockaddr *)&dest_in6, sizeof(struct sockaddr));
        if (ret < 0) {
            LOG_WARNING("[udp server] transmit failed %u bytes (%i) (%i)\n", bytes_to_send, ret, errno);
            if (--retries == 0) {
                LOG_WARNING("[udp server] could not send all %u bytes\n", data_len);
                break;
            }
        }
        else {
            DEBUG("[udp server] Success: send %u byte to [%s]:%u\n", (unsigned)data_len, dest_ipv6, port);
            bytes_sent += bytes_to_send;
        }
    }

    close(s);
    return bytes_sent;
}

static void *_udp_server_thread(void *arg)
{
    udp_server_t *srv = arg;
    msg_init_queue(udp_server_msg_queue, MAIN_QUEUE_SIZE);

    /* set source address and port */
    server_in6.sin6_family = AF_INET6;
    server_in6.sin6_port = htons(srv->port);
    memset(&server_in6.sin6_addr, 0, sizeof(server_in6.sin6_addr));
    inet_pton(AF_INET6, srv->address_str, &server_in6.sin6_addr);

    server_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);

    if (server_socket < 0) {
        LOG_ERROR("[udp server] error initializing socket (%i) (%i)\n", server_socket, errno);
        server_socket = 0;
        return NULL;
    }
    if (bind(server_socket, (struct sockaddr *)&server_in6, sizeof(server_in6)) < 0) {
        LOG_ERROR("[udp server] error binding socket (%i) (%i)\n", server_socket, errno);
        server_socket = -1;
        return NULL;
    }
    LOG_INFO("[udp server] started listening on port %" PRIu16 "\n", srv->port);
    while (1) {
        int res;
        socklen_t src_len = sizeof(struct sockaddr_in6);
        if ((res = recvfrom(server_socket, server_buffer, sizeof(server_buffer),
                            0, (struct sockaddr *)&src_in6, &src_len))
                        < 0) {
            LOG_WARNING("[udp server] Error on receive %i (errno %i)\n", res, errno);
        }
        else if (res == 0) {
            LOG_WARNING("[udp server] Peer did shut down");
        }
        else {
            char src_ip[INET6_ADDRSTRLEN];
            inet_ntop(AF_INET6, &src_in6.sin6_addr, src_ip, sizeof(src_ip));
            char dst_ip[INET6_ADDRSTRLEN];
            inet_ntop(AF_INET6, &server_in6.sin6_addr, dst_ip, sizeof(dst_ip));
            DEBUG("[udp server] Received data from [%s]:%u to [%s]:%u\n",
                   src_ip, ntohs(src_in6.sin6_port),
                   dst_ip, ntohs(server_in6.sin6_port));
            DEBUG("[udp server] data: <%s>\n", server_buffer);

            srv->callback(server_buffer, res);
        }
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
        LOG_ERROR("[udp server] error creating thread\n");
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

        if (ipv6_addr_is_multicast(&addr)) {
            if (gnrc_netapi_set(netif->pid, NETOPT_IPV6_GROUP, 0, &addr,
                sizeof(addr)) < 0) {
                LOG_ERROR("error: unable to join IPv6 multicast group\n");
            }
            else {
                LOG_INFO("joined IPv6 multicast group %s%%%u\n", srv->address_str, netif->pid);
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

#endif /* MODULE_GNRC_UDP */

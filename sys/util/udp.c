#if (defined MODULE_GNRC_SOCK_UDP && !defined MODULE_POSIX_SOCKETS) || defined DOXYGEN

#include "log.h"
#include "util.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

static char _udp_server_thread_stack [1024];
static void *_udp_server_thread(void *arg);

// #define MAIN_QUEUE_SIZE     (8)
// static msg_t udp_server_msg_queue[MAIN_QUEUE_SIZE];

size_t udp_send(const char *dest_ipv6, uint16_t port, const char *data, size_t data_len)
{
    (void)dest_ipv6;
    (void)port;
    (void)data;
    (void)data_len;
    LOG_ERROR("send not implemented\n");
    return 0;
}

static void *_udp_server_thread(void *arg)
{
    (void)arg;

//     udp_server_t *srv = arg;
//     const char source_ip[] = "::";
//     char *source_ip = srv->address_str;
//     msg_init_queue(udp_server_msg_queue, MAIN_QUEUE_SIZE);

    return NULL;
}

int udp_start_server(udp_server_t *srv)
{
    static char thread_name[17];
    snprintf(thread_name, sizeof(thread_name), "udp server %" PRIu16, srv->port);

    LOG_ERROR("UDP server started but not implemented\n");
    return 1;

    if (thread_create(_udp_server_thread_stack, sizeof(_udp_server_thread_stack),
                      THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                      _udp_server_thread, srv, thread_name)
                <= KERNEL_PID_UNDEF) {
        LOG_ERROR("error creating thread\n");
        return 1;
    }
    return 0;
}

#endif

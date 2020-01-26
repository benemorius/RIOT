/*
 * Copyright (C) 2015 Freie Universität Berlin
 * Copyright (C) 2018 Inria
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
 * @brief       Demonstrating the server side of TinyDTLS (Simple echo)
 *
 * @author      Raul A. Fuentes Samaniego <ra.fuentes.sam+RIOT@gmail.com>
 * @author      Olaf Bergmann <bergmann@tzi.org>
 * @author      Hauke Mehrtens <hauke@hauke-m.de>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 *
 * @}
 */

#if defined MODULE_TINYDTLS || defined DOXYGEN

#include <stdio.h>
#include <inttypes.h>

#include "msg.h"
#include "xtimer.h"
#include "net/nanocoap.h"
#include "net/sock/udp.h"
// #include "tinydtls_keys.h"

#define PSK_DEFAULT_IDENTITY "Client_identity"
// #define PSK_DEFAULT_KEY "secretPSK"
#define PSK_DEFAULT_KEY "MakeBurn"
#define PSK_OPTIONS "i:k:"
#define PSK_ID_MAXLEN 32
#define PSK_MAXLEN 32

/* TinyDTLS */
#include "dtls.h"
#include "dtls_debug.h"
#include "tinydtls.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#ifndef DTLS_DEFAULT_PORT
#define DTLS_DEFAULT_PORT 20220 /* DTLS default port */
#endif

#define DTLS_STOP_SERVER_MSG 0x4001 /* Custom IPC type msg. */

#define _IPV6_DEFAULT_PREFIX_LEN        (64U)

/*
 * This structure will be used for storing the sock and the remote into the
 * dtls_context_t variable.
 *
 * This is because remote must not have port set to zero on sock_udp_create()
 * making impossible to recover the remote with sock_udp_get_remote()
 *
 * An alternative is to modify dtls_handle_message () to receive the remote
 * from sock_udp_recv(). Also, it's required to modify _send_to_peer_handler()  for
 * parsing an auxiliary sock_udp_ep_t variable from the dls session.
 */
typedef struct {
    sock_udp_t *sock;
    sock_udp_ep_t *remote;
} dtls_remote_peer_t;

static kernel_pid_t _dtls_server_pid = KERNEL_PID_UNDEF;

bool _dtls_init_called = false;

#define READER_QUEUE_SIZE (8U)

/*  NOTE: Temporary patch for tinyDTLS 0.8.6 */
#ifndef TINYDTLS_EXTRA_BUFF
#define TINYDTLS_EXTRA_BUFF (512U)
#endif

char _dtls_server_stack[THREAD_STACKSIZE_MAIN +
THREAD_EXTRA_STACKSIZE_PRINTF +
TINYDTLS_EXTRA_BUFF];

#define COAP_INBUF_SIZE (512U)
static uint8_t _coap_in_buf[COAP_INBUF_SIZE];

static uint8_t _get_prefix_len(char *addr)
{
    int prefix_len = ipv6_addr_split_prefix(addr);
    if (prefix_len < 1) {
        prefix_len = _IPV6_DEFAULT_PREFIX_LEN;
    }
    return prefix_len;
}

/*
 * Handles all the packets arriving at the node and identifies those that are
 * DTLS records. Also, it determines if said DTLS record is coming from a new
 * peer or a currently established peer.
 */
static void dtls_handle_read(dtls_context_t *ctx)
{
    static session_t session;
    static uint8_t packet_rcvd[DTLS_MAX_BUF];

    assert(ctx);
    assert(dtls_get_app_data(ctx));

    if (!ctx) {
        DEBUG("No DTLS context!\n");
        return;
    }

    if (!dtls_get_app_data(ctx)) {
        DEBUG("No app_data stored!\n");
        return;
    }

    dtls_remote_peer_t *remote_peer;
    remote_peer = (dtls_remote_peer_t *)dtls_get_app_data(ctx);

    ssize_t res = sock_udp_recv(remote_peer->sock, packet_rcvd, DTLS_MAX_BUF,
                                1 * US_PER_SEC, remote_peer->remote);

    if (res <= 0) {
        if ((ENABLE_DEBUG) && (res != -EAGAIN) && (res != -ETIMEDOUT)) {
            DEBUG("sock_udp_recv unexepcted code error: %i\n", (int)res);
        }
        return;
    }

//     DEBUG("DBG-Server: Record Rcvd\n");

    /* (DTLS) session requires the remote peer address (IPv6:Port) and netif */
    session.size = sizeof(uint8_t) * 16 + sizeof(unsigned short);

#ifdef WITH_RIOT_GNRC
    ipv6_addr_t *addr = &session.addr;
    session.port = remote_peer->remote->port;
#else
    struct in6_addr *addr = &session.addr.sin6.sin6_addr;
    session.addr.sin6.sin6_port = remote_peer->remote->port;
    session.addr.sa.sa_family = AF_INET6;
#endif

    if (remote_peer->remote->netif ==  SOCK_ADDR_ANY_NETIF) {
        session.ifindex = SOCK_ADDR_ANY_NETIF;
    }
    else {
        session.ifindex = remote_peer->remote->netif;
    }

    if (memcpy(addr, &remote_peer->remote->addr.ipv6, 16) == NULL) {
        LOG_ERROR("ERROR: memcpy failed!");
        return;
    }

    dtls_handle_message(ctx, &session, packet_rcvd, (int)DTLS_MAX_BUF);

    return;
}

/* Reception of a DTLS Application data record. */
static int _read_from_peer_handler(struct dtls_context_t *ctx,
                                   session_t *session, uint8 *data, size_t len)
{
    coap_pkt_t pkt;
    ssize_t res = 0;

    memcpy(_coap_in_buf, data, len);

    if (coap_parse(&pkt, _coap_in_buf, len) < 0) {
        LOG_WARNING("error parsing packet\n");
        return 0;
    }
    if ((res = coap_handle_req(&pkt, _coap_in_buf, COAP_INBUF_SIZE)) > 0) {
        res = dtls_write(ctx, session, _coap_in_buf, res);
    }
    else {
        LOG_WARNING("error handling request %d\n", (int)res);
    }
    return res;
}

/* Handles the DTLS communication with the other peer. */
static int _send_to_peer_handler(struct dtls_context_t *ctx,
                                 session_t *session, uint8 *buf, size_t len)
{

    /*
     * It's possible to create a sock_udp_ep_t variable. But, it's required
     * to copy memory from the session variable to it.
     */
    (void) session;

    assert(ctx);
    assert(dtls_get_app_data(ctx));

    if (!dtls_get_app_data(ctx)) {
        return -1;
    }

    dtls_remote_peer_t *remote_peer;
    remote_peer = (dtls_remote_peer_t *)dtls_get_app_data(ctx);

//     DEBUG("DBG-Server: Sending record\n");

    /* don't send from the multicast address we're listening on */
//     ipv6_addr_set_unspecified((ipv6_addr_t *)remote_peer->sock->local.addr.ipv6);

    return sock_udp_send(remote_peer->sock, buf, len, remote_peer->remote);
}

#ifdef DTLS_PSK
static unsigned char psk_id[PSK_ID_MAXLEN] = PSK_DEFAULT_IDENTITY;
static size_t psk_id_length = sizeof(PSK_DEFAULT_IDENTITY) - 1;
static unsigned char psk_key[PSK_MAXLEN] = PSK_DEFAULT_KEY;
static size_t psk_key_length = sizeof(PSK_DEFAULT_KEY) - 1;

unsigned char psk_id_alt[PSK_ID_MAXLEN] = "ff9a83c49cb341f195ce1a5df89a6247";
size_t psk_id_alt_length = 32;
unsigned char psk_key_alt[PSK_MAXLEN] = PSK_DEFAULT_KEY;
size_t psk_key_alt_length = sizeof(PSK_DEFAULT_KEY) - 1;

/*
 * This function is the "key store" for tinyDTLS. It is called to retrieve a
 * key for the given identity within this particular session.
 */
static int _peer_get_psk_info_handler(struct dtls_context_t *ctx, const session_t *session,
                                      dtls_credentials_type_t type,
                                      const unsigned char *id, size_t id_len,
                                      unsigned char *result, size_t result_length)
{
    (void) ctx;
    (void) session;

    struct keymap_t {
        unsigned char *id;
        size_t id_length;
        unsigned char *key;
        size_t key_length;
    } psk[4] = {
        { (unsigned char *)psk_id, psk_id_length,
            (unsigned char *)psk_key, psk_key_length },
            { (unsigned char *)psk_id_alt, psk_id_alt_length,
                (unsigned char *)psk_key_alt, psk_key_alt_length },
                { (unsigned char *)"default identity", 16,
                    (unsigned char *)"\x11\x22\x33", 3 },
                    { (unsigned char *)"\0", 2,
                        (unsigned char *)"", 1 }
    };

    if (type != DTLS_PSK_KEY) {
        return 0;
    }

    if (id) {
        uint8_t i;
        for (i = 0; i < sizeof(psk) / sizeof(struct keymap_t); i++) {
            if (id_len == psk[i].id_length && memcmp(id, psk[i].id, id_len) == 0) {
                if (result_length < psk[i].key_length) {
                    dtls_warn("buffer too small for PSK");
                    return dtls_alert_fatal_create(DTLS_ALERT_INTERNAL_ERROR);
                }

                memcpy(result, psk[i].key, psk[i].key_length);
                return psk[i].key_length;
            }
            else {
                dtls_warn("id no match lengths %i %i\n", id_len, psk[i].id_length);
            }
        }
    }
    else {
        dtls_warn("no id\n");
    }

    return dtls_alert_fatal_create(DTLS_ALERT_DECRYPT_ERROR);
}
#endif /* DTLS_PSK */

#ifdef DTLS_ECC
static int _peer_get_ecdsa_key_handler(struct dtls_context_t *ctx,
                                       const session_t *session,
                                       const dtls_ecdsa_key_t **result)
{
    (void) ctx;
    (void) session;
    static const dtls_ecdsa_key_t ecdsa_key = {
        .curve = DTLS_ECDH_CURVE_SECP256R1,
        .priv_key = ecdsa_priv_key,
        .pub_key_x = ecdsa_pub_key_x,
        .pub_key_y = ecdsa_pub_key_y
    };

    /* TODO: Load the key from external source */

    *result = &ecdsa_key;
    return 0;
}

static int _peer_verify_ecdsa_key_handler(struct dtls_context_t *ctx,
                                          const session_t *session,
                                          const unsigned char *other_pub_x,
                                          const unsigned char *other_pub_y,
                                          size_t key_size)
{
    (void) ctx;
    (void) session;
    (void) other_pub_x;
    (void) other_pub_y;
    (void) key_size;

    /* TODO: As far for tinyDTLS 0.8.2 this is not used */

    return 0;
}
#endif /* DTLS_ECC */

/* DTLS variables and register are initialized. */
dtls_context_t *_server_init_dtls(dtls_remote_peer_t *remote_peer)
{
    dtls_context_t *new_context = NULL;

    static dtls_handler_t cb = {
        .write = _send_to_peer_handler,
        .read = _read_from_peer_handler,
        .event = NULL,
        #ifdef DTLS_PSK
        .get_psk_info = _peer_get_psk_info_handler,
        #endif  /* DTLS_PSK */
        #ifdef DTLS_ECC
        .get_ecdsa_key = _peer_get_ecdsa_key_handler,
        .verify_ecdsa_key = _peer_verify_ecdsa_key_handler
        #endif  /* DTLS_ECC */
    };

//     #ifdef DTLS_PSK
//     DEBUG("Server support PSK\n");
//     #endif
//     #ifdef DTLS_ECC
//     DEBUG("Server support ECC\n");
//     #endif

#ifdef TINYDTLS_LOG_LVL
    dtls_set_log_level(TINYDTLS_LOG_LVL);
#endif

    /*
     * The context for the server is different from the client.
     * This is because sock_udp_create() cannot work with a remote endpoint
     * with port set to 0. And even after sock_udp_recv(), sock_udp_get_remote()
     * cannot retrieve the remote.
     */
    new_context = dtls_new_context(remote_peer);

    if (new_context) {
        dtls_set_handler(new_context, &cb);
    }
    else {
        return NULL;
    }

    return new_context;
}

void *_dtls_server_wrapper(void *arg)
{
    /* NOTE: dtls_init() must be called previous to this */

    bool active = true;
    msg_t _reader_queue[READER_QUEUE_SIZE];
    msg_t msg;

    sock_udp_t udp_socket;
    sock_udp_ep_t local = SOCK_IPV6_EP_ANY;
    sock_udp_ep_t remote = SOCK_IPV6_EP_ANY;

    dtls_context_t *dtls_context = NULL;
    dtls_remote_peer_t remote_peer;

    remote_peer.sock = &udp_socket;
    remote_peer.remote = &remote;

    /* Prepare (thread) messages reception */
    msg_init_queue(_reader_queue, READER_QUEUE_SIZE);

    local.port = *(uint16_t *)arg;
    if (local.port == 0) {
        local.port = DTLS_DEFAULT_PORT;
    }

    ssize_t res = sock_udp_create(&udp_socket, &local, NULL, 0);
    if (res == -1) {
        LOG_ERROR("ERROR: Unable create sock.");
        return (void *) NULL;
    }

    dtls_context = _server_init_dtls(&remote_peer);

    if (!dtls_context) {
        LOG_ERROR("ERROR: Server unable to load context!");
        return (void *) NULL;
    }

    while (active) {

        msg_try_receive(&msg); /* Check if we got an (thread) message */
        if (msg.type == DTLS_STOP_SERVER_MSG) {
            active = false;
        }
        else {
            /* Listening for any DTLS recodrd */
            dtls_handle_read(dtls_context);
        }
    }

    /* Release resources (strict order) */
    dtls_free_context(dtls_context);    /* This also sends a DTLS Alert record */
    sock_udp_close(&udp_socket);
    msg_reply(&msg, &msg);              /* Basic answer to the main thread */

    return (void *) NULL;
}

void coap_start_server(char *addr_str, uint16_t port)
{
    /* Only one instance of the server */
    if (_dtls_server_pid != KERNEL_PID_UNDEF) {
        puts("Error: server already running");
        return;
    }

    if (!_dtls_init_called) {
        _dtls_init_called = true;
        dtls_init();
    }

    static char thread_name[23];
    snprintf(thread_name, sizeof(thread_name), "coap-dtls server %" PRIu16, port);
    _dtls_server_pid = thread_create(_dtls_server_stack,
                                     sizeof(_dtls_server_stack),
                                     THREAD_PRIORITY_MAIN - 1,
                                     THREAD_CREATE_STACKTEST,
                                     _dtls_server_wrapper, (void *)&port, thread_name);

    /* Uncommon but better be sure */
    if (_dtls_server_pid == EINVAL) {
        puts("ERROR: Thread invalid");
        _dtls_server_pid = KERNEL_PID_UNDEF;
        return;
    }

    if (_dtls_server_pid == EOVERFLOW) {
        puts("ERROR: Thread overflow!");
        _dtls_server_pid = KERNEL_PID_UNDEF;
        return;
    }

    LOG_INFO("[coap-dtls server] started listening on port %" PRIu16 "\n", port);

    /* set the listening address on each interface. This is dirty */
    ipv6_addr_t addr;
    gnrc_netif_t *netif = NULL;
    while ((netif = gnrc_netif_iter(netif)) != NULL) {
        if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
            LOG_ERROR("error: unable to parse IPv6 address.");
            return;
        }

        if (ipv6_addr_is_multicast(&addr)) {
            if (gnrc_netapi_set(netif->pid, NETOPT_IPV6_GROUP, 0, &addr,
                sizeof(addr)) < 0) {
                LOG_ERROR("error: unable to join IPv6 multicast group\n");
            }
            else {
                LOG_INFO("joined IPv6 multicast group %s%%%u\n", addr_str, netif->pid);
            }
        }
        else {
            uint16_t flags = GNRC_NETIF_IPV6_ADDRS_FLAGS_STATE_VALID;
            flags |= (_get_prefix_len(addr_str) << 8U);
            if (gnrc_netapi_set(netif->pid, NETOPT_IPV6_ADDR, flags, &addr,
                sizeof(addr)) < 0) {
                LOG_ERROR("error: unable to add IPv6 address\n");
                return;
            }
        }
    }

    return;
}

void coap_stop_server(void)
{
    /* check if server is running at all */
    if (_dtls_server_pid == KERNEL_PID_UNDEF) {
        puts("Error: DTLS server is not running");
        return;
    }

    /* prepare the stop message */
    msg_t m;
    m.type = DTLS_STOP_SERVER_MSG;

    DEBUG("Stopping server...\n");

    /* send the stop message to thread AND wait for (any) answer */
    msg_send_receive(&m, &m, _dtls_server_pid);

    _dtls_server_pid = KERNEL_PID_UNDEF;
    puts("Success: DTLS server stopped");
}

#endif /* MODULE_TINYDTLS */

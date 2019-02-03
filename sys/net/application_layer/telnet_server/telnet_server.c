/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup net_telnet_server
 * @{
 * @file
 * @brief   telnet server implementation
 * @author  Thomas Stilwell <stilwellt@openlabs.co>
 * @}
 */

/* needed for fdopen */
#ifndef _XOPEN_SOURCE
#define _XOPEN_SOURCE 600
#endif

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include "msg.h"
#include "vfs.h"
#include "shell.h"
#include "pipe.h"
#include "thread.h"
#include "net/gnrc.h"
#include "net/telnet_server.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define SERVER_MSG_QUEUE_SIZE   (8)
#define TCP_SEND_TIMEOUT_US     (10 * US_PER_SEC)

static char server_stack[512 + 512 + 512];
static char sending_stack[512 + 512];
static char shell_thread_stack[1600 + 1024 + 1024];

static msg_t _shell_msg_queue[SERVER_MSG_QUEUE_SIZE];

#define TELNET_MAX_PAYLOAD_SIZE (128) /* 46 max to avoid fragmenting */
static char _stdin_pipe_buf[4];
static char _stdout_pipe_buf[128];
static ringbuffer_t _stdin_ringbuffer;
static ringbuffer_t _stdout_ringbuffer;
static pipe_t _stdin_pipe;
static pipe_t _stdout_pipe;

static mutex_t _server_is_running = MUTEX_INIT;

extern FILE *stdin_real;
extern FILE *stdout_real;
extern FILE *stdin_telnet;
extern FILE *stdout_telnet;
extern kernel_pid_t telnet_shell_pid;
extern shell_command_t *util_all_shell_commands; /* util/shell.c */

static ssize_t _process_outgoing_data(const char *data, size_t data_length);
static void _disconnect_client(telnet_server_t *srv);

static void fprintf_hex(FILE *file, const char *data, int length)
{
    fprintf(file, "<");
    for (int i = 0; i < length; i++) {
        fprintf(file, "%02x ", data[i]);
    }
    fprintf(file, "\b> \"");
    for (int i = 0; i < length; i++) {
        fprintf(file, "%c ", data[i]);
    }
    fprintf(file, "\"");
}

static ssize_t _stdout_write(vfs_file_t *filp, const void *src, size_t nbytes)
{
    filp = filp;
    DEBUG("[telnet] _stdout_write() %u bytes <0x%02x>\n", nbytes, *(char*)src);

    if (0) {
        fprintf(stdout_real, "stdout %u bytes: ", nbytes);
        fprintf_hex(stdout_real, src, nbytes);
        fprintf(stdout_real, "\n");
    }

    return _process_outgoing_data(src, nbytes);
}

static ssize_t _stdin_read(vfs_file_t *filp, void *dest, size_t nbytes)
{
    filp = filp;
    ssize_t bytes_read = pipe_read(&_stdin_pipe, dest, nbytes);

    if (0) {
        fprintf(stdout_real, "in %u bytes: ", bytes_read);
        fprintf_hex(stdout_real, dest, bytes_read);
        fprintf(stdout_real, " \n");
    }

    return bytes_read;
}

static vfs_file_ops_t _stdout_ops = {
    .write = _stdout_write,
};

static vfs_file_ops_t _stdin_ops = {
    .read = _stdin_read,
};

static ssize_t _telnet_send(const void *buf, size_t length)
{
    return pipe_write(&_stdout_pipe, buf, length);
}

static ssize_t _process_outgoing_data(const char *data, size_t data_length)
{
    // check each outgoing character in case it needs modifying for telnet
    for (unsigned i = 0; i < data_length; i++) {
        char c = data[i];

        // not ascii printable
        if (c < 32 || c > 126) {
            if (c == 0x1b) {
                // continue;
            }
            // CR must be followed by either LF or NUL
            else if (c == '\r') {
                _telnet_send("\r", 1);
                _telnet_send("\0", 1);
                continue;
            }
            // replace \n with \r\n
            else if (c == '\n') {
                _telnet_send("\r", 1);
                _telnet_send("\n", 1);
                continue;
            }
            else if (c == '\t') {

            }
            else {
                DEBUG("[telnet] send character <0x%02x> (\\0%o)\n", c, c);
            }
        }
        _telnet_send(&c, 1);
    }
    return data_length;
}

static void _handle_telnet_option(uint8_t command, uint8_t option)
{
    /* suppress go ahead */
    if (command == 253 && option == 3) {
        /* will suppress go ahead */
        char reply[] = {255, 251, 3};
        _telnet_send(reply, sizeof(reply));
    }
    // /* will linemode */
    // else if (command == 251 && option == 34) {
    //     /* do line mode */
    //     char reply[] = {255, 253, 34};
    //     _telnet_send(reply, sizeof(reply));
    // }
    // /* do this */
    // else if (command == 253) {
    //     /* won't do whatever it is */
    //     char reply[] = {255, 252, option};
    //     _telnet_send(reply, sizeof(reply));
    // }
    // /* don't do this */
    // else if (command == 254) {
    //     /* won't do whatever it is */
    //     char reply[] = {255, 252, option};
    //     _telnet_send(reply, sizeof(reply));
    // }
    /* will do this */
    else if (command == 251) {
        /* won't do whatever it is */
        char reply[] = {255, 252, option};
        _telnet_send(reply, sizeof(reply));
    }
    /* do timing mark */
    else if (command == 253 && option == 6) {
        /* won't timing mark */
        char reply[] = {255, 252, 6};
        _telnet_send(reply, sizeof(reply));
    }
    else {
        DEBUG("[telnet] got command %u %u\n", command, option);
    }
}

static void *_shell_thread(void *arg)
{
    telnet_server_t *srv = (telnet_server_t *)arg;
    (void)srv;
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_shell_msg_queue, SERVER_MSG_QUEUE_SIZE);

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(util_all_shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    char cd[] = {'C' - '@', 'D' - '@'};
    pipe_write(&_stdout_pipe, &cd, sizeof(cd));

    fclose(stdin);
    fclose(stdout);

    return NULL;
}

static bool _is_sending = false;

static void *_sending_thread(void *arg)
{
    telnet_server_t *srv = (telnet_server_t *)arg;
    char buffer[TELNET_MAX_PAYLOAD_SIZE];
    while (1) {
        _is_sending = false;
        ssize_t bytes_read = pipe_read(&_stdout_pipe, buffer, sizeof(buffer));
        _is_sending = true;
        DEBUG("[telnet] _sending_thread() %u bytes\n", bytes_read);

        if (bytes_read == sizeof(buffer)) {
        }
        else {
        }

        if (0) {
            fprintf(stdout_real, "out %u bytes: ", bytes_read);
            fprintf_hex(stdout_real, buffer, bytes_read);
            fprintf(stdout_real, "\n");
        }

        /* exit on EOT */
        for(ssize_t i = 0; i < bytes_read; i++) {
            if (buffer[i] == ('D' - '@')) {
                gnrc_tcp_abort(&srv->tcb);
                _is_sending = false;
                mutex_unlock(&_server_is_running);
                return NULL;
            }
        }

        int ret = gnrc_tcp_send(&srv->tcb, buffer, bytes_read, TCP_SEND_TIMEOUT_US);
        DEBUG("[telnet] gnrc_tcp_send() ret: %i\n", ret);

        if (ret < 1) {
            switch (ret) {
                case -ENOTCONN:
                    LOG_WARNING("[telnet] gnrc_tcp_send() : -ENOTCONN\n");
//                     gnrc_tcp_abort(&srv->tcb);
                    break;

                case -ECONNABORTED:
                    LOG_WARNING("[telnet] gnrc_tcp_send() : -ECONNABORTED\n");
//                     gnrc_tcp_abort(&srv->tcb);
                    break;

                case -ETIMEDOUT:
                    LOG_WARNING("[telnet] gnrc_tcp_send() : -ETIMEDOUT %u bytes\n", bytes_read);
                    LOG_WARNING("[telnet] TCP send timed out - closing socket\n");
//                     gnrc_tcp_abort(&srv->tcb);
                    break;

                case -ECONNRESET:
                    LOG_WARNING("[telnet] gnrc_tcp_send() : -ECONNRESET\n");
//                     gnrc_tcp_abort(&srv->tcb);
                    break;

                default:
                    LOG_WARNING("[telnet] gnrc_tcp_send() : %d\n", ret);
            }
            _disconnect_client(srv);
            _is_sending = false;
            gnrc_tcp_abort(&srv->tcb);
            mutex_unlock(&_server_is_running);
            return NULL;
        }
    }
}

uint32_t _keepalive_timer_ms;
#define KEEPALIVE_INTERVAL_MS (10 * MS_PER_SEC)

static int _recv(telnet_server_t *srv, char *c)
{
    int ret;
    while (1) {
        if (_is_sending) {
            xtimer_usleep(100 * US_PER_MS);
            continue;
        }
        ret = gnrc_tcp_recv(&srv->tcb, (void *)c, 1, 20 * US_PER_MS);
        if (ret < 0) {
            switch (ret) {
                case -ETIMEDOUT:
                    if ( xtimer_now_usec64() / 1000 - _keepalive_timer_ms > KEEPALIVE_INTERVAL_MS) {
                        pipe_write(&_stdout_pipe, " \b", 2);
                        _keepalive_timer_ms = xtimer_now_usec64() / 1000;
                    }
//                     LOG_DEBUG("[telnet] gnrc_tcp_rcvd() : -ETIMEDOUT\n");
                    continue;
                case -ENOTCONN:
                    LOG_WARNING("[telnet] gnrc_tcp_rcvd() : -ENOTCONN\n");
                    break;
                case -ECONNABORTED:
                    LOG_WARNING("[telnet] gnrc_tcp_rcvd() : -ECONNABORTED\n");
                    break;
                case -ECONNRESET:
                    LOG_WARNING("[telnet] gnrc_tcp_rcvd() : -ECONNRESET\n");
                    break;
                default:
                    LOG_WARNING("[telnet] gnrc_tcp_rcvd() : %d\n", ret);
                    break;
            }
        }
        else if (ret == 0) {
//             LOG_INFO("[telnet] client disconnected\n");
        }
        break;
    }

    if (ret <= 0) {
//         LOG_INFO("[telnet] client disconnected\n");
    }

    // const char msg[] = "[telnet] received data";
    // DEBUG("%s <0x%02x> (\\0%o) (%c)\n", msg, *c, *c, *c);
    return ret;
}

static void _disconnect_client(telnet_server_t *srv)
{
    (void) srv;
    // gnrc_tcp_close(&srv->tcb);
    char cd[] = {'C' - '@', 'D' - '@'};
    pipe_write(&_stdin_pipe, &cd, sizeof(cd));
}

static int _accept_client(telnet_server_t *srv)
{
    stdin_real = stdin;
    stdout_real = stdout;
    DEBUG("[telnet] stdin_real: 0x%lx\n", (uint32_t)stdin_real);
    DEBUG("[telnet] stdout_real: 0x%lx\n", (uint32_t)stdout_real);

    int in_fd = vfs_bind(VFS_ANY_FD, O_RDONLY, &_stdin_ops, (void *)STDIN_FILENO);
    int out_fd = vfs_bind(VFS_ANY_FD, O_WRONLY, &_stdout_ops, (void *)STDOUT_FILENO);
    stdin_telnet = (FILE *)fdopen(in_fd, "r");
    stdout_telnet = (FILE *)fdopen(out_fd, "w");
    DEBUG("[telnet] in fd: %i\n", in_fd);
    DEBUG("[telnet] out fd: %i\n", out_fd);

    ringbuffer_init(&_stdin_ringbuffer, _stdin_pipe_buf, sizeof(_stdin_pipe_buf));
    ringbuffer_init(&_stdout_ringbuffer, _stdout_pipe_buf, sizeof(_stdout_pipe_buf));
    pipe_init(&_stdin_pipe, &_stdin_ringbuffer, NULL);
    pipe_init(&_stdout_pipe, &_stdout_ringbuffer, NULL);

    /* sending thread */
    if (thread_create(sending_stack, sizeof(sending_stack), THREAD_PRIORITY_MAIN - 1,
                      THREAD_CREATE_STACKTEST, _sending_thread, srv,
                      "telnet server tx") <= KERNEL_PID_UNDEF) {
        LOG_ERROR("[telnet] error initializing thread");
        return 1;
    }

    /* Create thread without yield so that telnet_shell_pid is populated before
     * the first time the thread runs or else it will block on the wrong stdin
     * and not switch to the right stdin until after a character is received.
     */
    telnet_shell_pid = thread_create(shell_thread_stack, sizeof(shell_thread_stack), 13,
                  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD, _shell_thread,
                  srv, "telnet shell");
    if (telnet_shell_pid <= KERNEL_PID_UNDEF) {
        LOG_ERROR("[telnet] error initializing thread");
        return 1;
    }

    return 0;
}

static void *_server_thread(void *arg)
{
    telnet_server_t *srv = (telnet_server_t *)arg;
    int ret;

    if (srv->port == 0) {
        LOG_ERROR("[telnet] Error: invalid port specified");
        return NULL;
    }
    while (1) {
        mutex_lock(&_server_is_running);

        gnrc_tcp_tcb_init(&srv->tcb);

        /* wait for incoming connection */
        ret = gnrc_tcp_open_passive(&srv->tcb, AF_INET6, NULL, srv->port);
        if (ret) {
            LOG_ERROR("[telnet] gnrc_tcp_open_passive() failed: %i\n", ret);
            xtimer_usleep(5 * US_PER_SEC);
            continue;
        }
        char addr_str[IPV6_ADDR_MAX_STR_LEN];
        ipv6_addr_to_str(addr_str, (const ipv6_addr_t *)(srv->tcb.peer_addr), sizeof(addr_str));

        LOG_INFO("[telnet] accepted connection from host %s\n", addr_str);

        if (_accept_client(srv)) {
            gnrc_tcp_abort(&srv->tcb);
            continue;
        }

        // char dont_echo[] = {255, 254, 1};
        // _telnet_send(dont_echo, sizeof(dont_echo));

        /* tell client to disable its local echo */
        char will_echo[] = {255, 251, 1};
        _telnet_send(will_echo, sizeof(will_echo));

        while (1) {
            char c;
            if (_recv(srv, &c) <= 0) {
                break;
            }

            /* IAC character - next character is a command */
            if (c == 255) {
                char command;
                if (_recv(srv, &command) <= 0) {
                    break;
                }

                /* receiving two IAC in a row means we pass one IAC */
                if (command == 255) {
                    c = 255;
                    ret = pipe_write(&_stdin_pipe, &c, 1);
                }
                /* telnet sends ^C as 255,244 and we change it to ^C */
                else if (command == 244) {
                    c = 'C' - '@'; /* ^C */
                    ret = pipe_write(&_stdin_pipe, &c, 1);
                }
                /* telnet option */
                else if (command >= 250) {
                    char option;
                    if (_recv(srv, &option) <= 0) {
                        break;
                    }
                    _handle_telnet_option(command, option);
                }
                else {
                    LOG_DEBUG("[telnet] unhandled command %u\n", command);
                }
                continue;
            }
            /* not ascii printable */
            else if (c < 32 || c > 126) {
                /* never pass null character */
                if (c == 0) {
                    continue;
                }
                /* translate \r to \n */
                /* actually, drop it instead */
                /* actually, do neither */
                else if (c == '\r') {
                    // continue;
                    // c = '\n';
                }
                else if (c == '\n') {

                }
                else if (c == '[' - '@') { /* escape character ^[ */

                }
                else {
                    DEBUG("[telnet] got nonascii character <0x%02x>\n", c);
                }
            }

            /* pass character to stdin if we made it this far */
            ret = pipe_write(&_stdin_pipe, &c, 1);
        }

        _disconnect_client(srv);
        LOG_INFO("[telnet] client disconnected %s\n", addr_str);
    }
    return NULL;
}

int telnet_start_server(telnet_server_t *telnet_server)
{
    static char thread_name[23];
    snprintf(thread_name, sizeof(thread_name), "telnet server %" PRIu16, telnet_server->port);

    if (thread_create(server_stack, sizeof(server_stack), THREAD_PRIORITY_MAIN,
                      THREAD_CREATE_STACKTEST, _server_thread, telnet_server,
                      thread_name) <= KERNEL_PID_UNDEF) {
        LOG_ERROR("[telnet] error initializing thread");
        return 1;
    }
    LOG_INFO("[telnet] started listening on port %" PRIu16 "\n", telnet_server->port);
    return 0;
}

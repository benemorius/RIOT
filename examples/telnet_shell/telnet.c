/* needed for posix usleep */
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
#include "mutex.h"
#include "thread.h"
#include "isrpipe.h"
#include "net/gnrc.h"
#include "net/gnrc/tcp.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

extern const shell_command_t shell_commands_main[];

#define SERVER_MSG_QUEUE_SIZE   (8)
#define TCP_SEND_TIMEOUT_US     (1000*1000*10)

static char server_stack[THREAD_STACKSIZE_DEFAULT];
static char sending_stack[THREAD_STACKSIZE_DEFAULT];
static char receive_stack[THREAD_STACKSIZE_DEFAULT];
static char shell_thread_stack[THREAD_STACKSIZE_LARGE];

static void *shell_thread(void *arg);
static msg_t _shell_msg_queue[SERVER_MSG_QUEUE_SIZE];

static char _stdin_buf_mem[512];
static isrpipe_t stdin_isrpipe = ISRPIPE_INIT(_stdin_buf_mem);

static char _telnet_recv_buf_mem[512];
static isrpipe_t telnet_recv_isrpipe = ISRPIPE_INIT(_telnet_recv_buf_mem);

static char _telnet_send_buf_mem[5120];
static isrpipe_t telnet_send_isrpipe = ISRPIPE_INIT(_telnet_send_buf_mem);
mutex_t telnet_send_isrpipe_lock = MUTEX_INIT;

gnrc_tcp_tcb_t tcb;
// mutex_t tcb_lock = MUTEX_INIT;

FILE *stdin_real;
FILE *stdout_real;
FILE *stdin_telnet;
FILE *stdout_telnet;
kernel_pid_t telnet_shell_pid = 0xffff;

static ssize_t _process_outgoing_data(const char *data, size_t data_length);
static int _handle_incoming_data(const char *data, int data_length);

void fprintf_hex(FILE *file, const char *data, int length)
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
    // fprintf(stdout_real, "stdout %u bytes: ", nbytes);
    // fprintf_hex(stdout_real, src, nbytes);
    // fprintf(stdout_real, " ... ");

    mutex_lock(&telnet_send_isrpipe_lock);
    ssize_t ret = _process_outgoing_data(src, nbytes);
    mutex_unlock(&telnet_send_isrpipe_lock);

    // fprintf(stdout_real, "done\r\n");
    return ret;
}

static ssize_t _stdin_read(vfs_file_t *filp, void *dest, size_t nbytes)
{
    filp = filp;
    ssize_t bytes_read = (ssize_t)isrpipe_read(&stdin_isrpipe, (char *)dest, nbytes);
    // fprintf(stdout_real, "in %u bytes: ", bytes_read);
    // fprintf_hex(stdout_real, dest, bytes_read);
    // fprintf(stdout_real, " \r\n");
    return bytes_read;
}

static vfs_file_ops_t _stdout_ops = {
    .write = _stdout_write,
};

static vfs_file_ops_t _stdin_ops = {
    .read = _stdin_read,
};

static void *_server_thread(void *args)
{
    int tid = 0;
    uint16_t port;

    /* parse port */
    port = *(int *)args;
    if (port == 0) {
        fprintf(stdout_real, "Error: invalid port specified");
        return NULL;
    }
    while (1) {

        /* Initialize TCB struct */
        // mutex_lock(&tcb_lock);
        gnrc_tcp_tcb_init(&tcb);

        /* Connect to peer */
        int ret = gnrc_tcp_open_passive(&tcb, AF_INET6, NULL, port);
        if (ret) {
            printf("gnrc_tcp_open_passive() failed: %i\n", ret);
            return 0;
        }
        printf("client connected to telnet server on port %" PRIu16 "\n", port);

        // char dont_echo[] = {255, 254, 1};
        // ret = isrpipe_write(&telnet_send_isrpipe, dont_echo, sizeof(dont_echo));

        // char will_echo[] = {255, 251, 1};
        // ret = isrpipe_write(&telnet_send_isrpipe, will_echo, sizeof(will_echo));

        while (1) {
            char c;
            ret = gnrc_tcp_recv(&tcb, (void *)&c, 1, 1000*50);
            if (ret <= 0) {
                switch (ret) {
                    case -ENOTCONN:
                        printf("TID=%d : gnrc_tcp_rcvd() : -ENOTCONN\n", tid);
                        break;

                    case -EAGAIN:
                        printf("TID=%d : gnrc_tcp_rcvd() : -EAGAIN : retry after 10sec\n", tid);
                        ret = 0;
                        xtimer_sleep(10);
                        break;

                    case -ECONNABORTED:
                        printf("TID=%d : gnrc_tcp_rcvd() : -ECONNABORTED\n", tid);
                        break;

                    case -ECONNRESET:
                        printf("TID=%d : gnrc_tcp_rcvd() : -ECONNRESET\n", tid);
                        break;

                    case -ETIMEDOUT:
                        // printf("TID=%d : gnrc_tcp_rcvd() : -ETIMEDOUT\n", tid);
                        continue;

                    default:
                        printf("TID=%d : gnrc_tcp_rcvd() : %d\n", tid, ret);
                        break;
                }
                break;
            }
            _handle_incoming_data(&c, 1);
        }

        /* Close connection */
        gnrc_tcp_close(&tcb);

        printf("closed socket\n");
    }

    return 0;
}

static void *_sending_thread(void *args)
{
    args = args;
    int tid = 0;
    static char buffer[800];
    while (1) {
        ssize_t bytes_read = (ssize_t)isrpipe_read(&telnet_send_isrpipe, (char *)buffer, sizeof(buffer));
        // fprintf(stdout_real, "out %u bytes: ", bytes_read);
        // fprintf_hex(stdout_real, buffer, bytes_read);
        // fprintf(stdout_real, "\r\n");

        // mutex_lock(&tcb_lock);
        int ret = gnrc_tcp_send(&tcb, buffer, bytes_read, TCP_SEND_TIMEOUT_US);
        // mutex_unlock(&tcb_lock);
        if (ret < 1) {
            printf("gnrc_tcp_send() failed: %i\n", ret);
            switch (ret) {
                case -ENOTCONN:
                    printf("TID=%d : gnrc_tcp_send() : -ENOTCONN\n", tid);
                    break;

                case -ECONNABORTED:
                    printf("TID=%d : gnrc_tcp_send() : -ECONNABORTED\n", tid);
                    break;

                case -ETIMEDOUT:
                    printf("gnrc_tcp_send() : -ETIMEDOUT %u bytes\n", bytes_read);
                    printf("TCP send timed out - closing socket\n");
                    gnrc_tcp_close(&tcb);
                    break;

                case -ECONNRESET:
                    printf("TID=%d : gnrc_tcp_send() : -ECONNRESET\n", tid);
                    break;

                default:
                    printf("TID=%d : gnrc_tcp_send() : %d\n", tid, ret);
                    return 0;
            }
        }
        // printf("finish %u done\r\n", bytes_read);
        xtimer_usleep(1000*30);
        // char nul[] = {0x0};
        // ret = gnrc_tcp_send(&tcb, nul, sizeof(nul), TCP_SEND_TIMEOUT_US);
    }
}

static void *shell_thread(void *arg)
{
    arg = arg;
    telnet_shell_pid = thread_getpid();
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_shell_msg_queue, SERVER_MSG_QUEUE_SIZE);

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    // shell_run(shell_commands_main, line_buf, SHELL_DEFAULT_BUFSIZE, (void*)fdopen(8, "r"), (void*)fdopen(7, "w"));
    shell_run(shell_commands_main, line_buf, SHELL_DEFAULT_BUFSIZE);
    return NULL;
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
            else if (c == 0xd) {
                int ret = (ssize_t)isrpipe_write_one(&telnet_send_isrpipe, 0xd);
                if (ret < 1) {
                    // buffer full or something, maybe block
                }
                ret = (ssize_t)isrpipe_write_one(&telnet_send_isrpipe, 0x0);
                if (ret < 1) {
                    // buffer full or something, maybe block
                }
                continue;
            }
            // replace \n with \r\n
            else if (c == 0xa) {
                int ret = (ssize_t)isrpipe_write_one(&telnet_send_isrpipe, 0xd);
                if (ret < 1) {
                    // buffer full or something, maybe block
                }
                ret = (ssize_t)isrpipe_write_one(&telnet_send_isrpipe, 0xa);
                if (ret < 1) {
                    // buffer full or something, maybe block
                }
                continue;
            }
            // else if (c == 0x9) {

            // }
            else {
                fprintf(stdout_real, "send character %u 0x%02x\n", c, c);
            }
        }
        int ret = (ssize_t)isrpipe_write_one(&telnet_send_isrpipe, c);
        if (ret < 1) {
            // buffer full or something, maybe block
        }
    }
    return data_length;
}

static void *_telnet_recv_thread(void *args)
{
    args = args;
    int ret;
    while (1) {
        char c;
        isrpipe_read(&telnet_recv_isrpipe, &c, 1);

        // IAC character - next character is a command
        if (c == 255) {
            char command;
            ret = isrpipe_read(&telnet_recv_isrpipe, &command, 1);
            if (command == 255) {
                c = command; // received 255 255, so pass 255
                ret = isrpipe_write_one(&stdin_isrpipe, c);
            }
            else if (command == 244) { // ^C
                c = 0x03; // ^C
                ret = isrpipe_write_one(&stdin_isrpipe, c);
            }
            else if (command >= 250) { // command requires an option character next
                char option;
                ret = isrpipe_read(&telnet_recv_isrpipe, &option, 1);
                if (command == 253 && option == 3) { // suppress go ahead
                    // will suppress go ahead
                    char reply[] = {255, 251, 3};
                    ret = isrpipe_write(&telnet_send_isrpipe, reply, sizeof(reply));
                }
                // else if (command == 251 && option == 34) { // will linemode
                //     // do line mode
                //     char reply[] = {255, 253, 34};
                //     ret = isrpipe_write(&telnet_send_isrpipe, reply, sizeof(reply));
                // }
                // else if (command == 253) { // do this
                //     // won't do whatever it is
                //     char reply[] = {255, 252, option};
                //     ret = isrpipe_write(&telnet_send_isrpipe, reply, sizeof(reply));
                // }
                else if (command == 254) { // don't do this
                    // won't do whatever it is
                    // char reply[] = {255, 252, option};
                    // ret = isrpipe_write(&telnet_send_isrpipe, reply, sizeof(reply));
                }
                else if (command == 251) { // will do this
                    // won't do whatever it is
                    char reply[] = {255, 252, option};
                    ret = isrpipe_write(&telnet_send_isrpipe, reply, sizeof(reply));
                }
                else if (command == 253 && option == 6) { // do timing mark
                    // won't timing mark
                    char reply[] = {255, 252, 6};
                    ret = isrpipe_write(&telnet_send_isrpipe, reply, sizeof(reply));
                }
                else {
                    fprintf(stdout_real, "got command %u %u\n", command, option);
                }
            }
            else {
                fprintf(stdout_real, "got command %u\n", command);
            }
            continue;
        }
        // not ascii printable
        if (c < 32 || c > 126) {
            if (c == 0) {
                continue;
            }
            else if (c == 0xd) {
                continue;
                // c = 0xa;
            }
            else if (c == 0xa) {

            }
            else {
                fprintf(stdout_real, "got character %u 0x%02x\n", c, c);
            }
        }

        // pass character to stdin if we made it this far
        ret = isrpipe_write_one(&stdin_isrpipe, c);
    }
    ret = ret;
    return NULL;
}

static int _handle_incoming_data(const char *data, int data_length)
{
    // printf("received %i bytes\n", data_length);
    int ret = isrpipe_write(&telnet_recv_isrpipe, data, data_length);
    if (ret < 1) {
        // buffer full or something, maybe block
    }
    return ret;
}

int telnet_start_server(int port)
{
    stdin_real = stdin;
    stdout_real = stdout;

    int out_fd = vfs_bind(VFS_ANY_FD, O_WRONLY, &_stdout_ops, (void *)STDOUT_FILENO);
    int in_fd = vfs_bind(VFS_ANY_FD, O_RDONLY, &_stdin_ops, (void *)STDIN_FILENO);
    stdout_telnet = (FILE *)fdopen(out_fd, "w");
    stdin_telnet = (FILE *)fdopen(in_fd, "r");
    DEBUG("out fd: %i\n", out_fd);
    DEBUG("in fd: %i\n", in_fd);

    /* start server */
    if (thread_create(server_stack, sizeof(server_stack), THREAD_PRIORITY_MAIN - 1,
                      THREAD_CREATE_STACKTEST,
                      _server_thread, &port, "telnet server rx") <= KERNEL_PID_UNDEF) {
        fprintf(stdout_real, "error initializing thread");
        return 1;
    }
    /* sending thread */
    if (thread_create(sending_stack, sizeof(sending_stack), THREAD_PRIORITY_MAIN - 1,
                      THREAD_CREATE_STACKTEST,
                      _sending_thread, &port, "telnet server tx") <= KERNEL_PID_UNDEF) {
        fprintf(stdout_real, "error initializing thread");
        return 1;
    }

    if (thread_create(receive_stack, sizeof(receive_stack), THREAD_PRIORITY_MAIN - 1,
                      THREAD_CREATE_STACKTEST, _telnet_recv_thread,
                      &port, "telnet server rx") <= KERNEL_PID_UNDEF) {
        fprintf(stdout_real, "error initializing thread");
        return 1;
    }

    /* start shell */
    thread_create(shell_thread_stack, sizeof(shell_thread_stack), 13,
                  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD, shell_thread,
                  NULL, "telnet shell");

    // fprintf(stdout_real, "telnet server started\n");
    return 0;
}

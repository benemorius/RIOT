/*
 * Copyright (C) 2015 Freie Universität Berlin
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
 * @brief       Example application for demonstrating the RIOT network stack
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "msg.h"
#include "vfs.h"
#include "log.h"
#include "shell.h"
#include "xtimer.h"
#include "stdio_base.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

static char shell_thread_stack [THREAD_STACKSIZE_LARGE];
static void *shell_thread(void *arg);

static char term_thread_stack [THREAD_STACKSIZE_LARGE];
static void *term_thread(void *arg);

extern int telnet_start_server(int port);

FILE *fdopen(int fd, const char *mode);

static ssize_t _uart_write(vfs_file_t *filp, const void *src, size_t nbytes)
{
    filp = filp;
    return stdio_write(src, nbytes);
}

static ssize_t _uart_read(vfs_file_t *filp, void *dest, size_t nbytes)
{
    filp = filp;
    return stdio_read(dest, nbytes);
}

static vfs_file_ops_t _uart_pipe_ops = {
    .write = _uart_write,
    .read = _uart_read,
};

int date(int argc, char **argv)
{
    argc = argc;
    argv = argv;
    return 0;
}

int kill(int argc, char **argv)
{
    argc = argc;
    if (!argv[1]){
        printf("usage: %s <pid>", argv[0]);
        return 1;
    }
    kernel_pid_t pid = atoi(argv[1]);

    /* oh nevermind there's no thread_kill() */
    pid = pid;
    return 1;
}

int dmesg_print(int argc, char **argv)
{
    argc = argc;
#ifdef MODULE_LOG_DMESG
    bool clear = false;
    if (argv[1]) {
        if (strcmp(argv[1], "-m") == 0) { // print free buffer memory
            printf("free %u\n", dmesg_free());
            printf("used %u\n", dmesg_used());
            printf("total %u\n", dmesg_total());
            return 0;
        }
        else if (strcmp(argv[1], "-c") == 0) { // read and clear buffer
            clear = true;
        }
        else if (strcmp(argv[1], "-C") == 0) { // clear buffer
            dmesg_clear();
            return 0;
        }
        else {
            printf("usage: %s [-m] [-c] [-C]\n",
                argv[0]
            );
            return 1;
        }
    }
    char buf[256];
    size_t total_bytes = 0;
    size_t bytes_read = 0;
    while (1) {
        if (clear) {
            bytes_read = dmesg_read(buf, sizeof(buf));
        }
        else {
            bytes_read = dmesg_peek(buf, sizeof(buf), total_bytes);
        }
        if (!bytes_read) {
            break;
        }
        fwrite(buf, 1, bytes_read, stdout);
        total_bytes += bytes_read;
    }
#endif
    return 0;
}

int uptime(int argc, char **argv)
{
    argc = argc;
    argv = argv;
    timex_t now;
    xtimer_now_timex(&now);
    printf("%lu.%06lu\n", now.seconds, now.microseconds);
    return 0;
}

static FILE *uart_out;
static FILE *uart_in;
kernel_pid_t term_pid = 0xffff;

void *term_thread(void *arg)
{
    arg = arg;
    while(1) {
        int c_to_uart = getc(stdin);
        // printf("%c", c_to_uart);
        // fflush(stdout);
        putc(c_to_uart, uart_out);
        fflush(uart_out);
    }
    return NULL;
}

int term(int argc, char **argv)
{
    argc = argc;
    argv = argv;
    printf("term\n");

    int uart_out_fd = vfs_bind(VFS_ANY_FD, O_WRONLY, &_uart_pipe_ops, (void *)STDOUT_FILENO);
    int uart_in_fd = vfs_bind(VFS_ANY_FD, O_RDONLY, &_uart_pipe_ops, (void *)STDIN_FILENO);
    uart_out = (FILE *)fdopen(uart_out_fd, "w");
    uart_in = (FILE *)fdopen(uart_in_fd, "r");
    printf("uart_out fd: %i\n", uart_out_fd);
    printf("uart_in fd: %i\n", uart_in_fd);
    fflush(stdout);

    /* Create thread without yield so that term_pid is populated before the
     * first time the thread runs or else it will block on the wrong stdin
     * and not switch to the right stdin until after a character is received.
     */
    term_pid = thread_create(term_thread_stack, sizeof(term_thread_stack), 13,
                  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD,
                  term_thread, NULL, "term");

    while(1) {
        int c_from_uart = getc(uart_in);
        putc(c_from_uart, stdout);
        if (c_from_uart == '\r') {
            putc('\n', stdout);
        }
        fflush(stdout);
    }

    /* approaches below did not work */


    /* O_NONBLOCK has no effect */

    // fcntl(uart_in_fd, F_SETFL, O_NONBLOCK);
    // fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);
    // while (1) {
    //     int c_to_uart;
    //     int c_from_uart;

    //     int ret = read(fileno(stdin), &c_to_uart, 1);
    //     fprintf(uart_out, "ret %i\n", ret);
    //     fflush(uart_out);

    //     if (read(uart_in_fd, &c_from_uart, 1) > 0) {
    //         putc(c_from_uart, stdout);
    //         fflush(stdout);
    //     }
    //     else if( ret > 0) {
    //         putc(c_to_uart, uart_out);
    //         fflush(uart_out);
    //     }
    //     else {
    //         xtimer_usleep(1000*30);
    //     }
    // }


    /* fd_set etc. not implemented */

    // fd_set fds;
    // while (1) {
    //     FD_ZERO(&fds);
    //     FD_SET(stdin, &fds);
    //     FD_SET(uart_in, &fds);

    //     select(FD_SETSIZE, &fds, NULL, NULL, NULL);

    //     if (FD_ISSET(stdin, &fds)){
    //         int c_to_uart = getc(stdin);
    //         printf("%c", c_to_uart);
    //         fflush(stdout);
    //         putc(c_to_uart, uart_out);
    //         fflush(uart_out);
    //     }
    //     if (FD_ISSET(uart_in, &fds)){
    //         int c_from_uart = getc(uart_in);
    //         putc(c_from_uart, stdout);
    //         fflush(stdout);
    //     }
    // }
}

const shell_command_t shell_commands_main[] = {
    { "dmesg",  "print kernel messages", dmesg_print },
    { "uptime", "print system uptime in seconds",   uptime },
    { "term",   "like picocom for stdio_uart",      term },
    { "kill",   "oops not implemented",             kill },
    { "date",   "print system date",                date },
    { NULL, NULL, NULL }
};

int main(void)
{
    /* kw41z sometimes crashes and resets a few times on reset
       so sleep for a bit and if we're still here after the
       sleep then we're probably done crashing
    */
    xtimer_usleep(1000*100);

    /* Hack to populate stdin and stdio - otherwise they are null until
       the first printf and we need them sooner.
       This causes stdin, stdout, stderr to all become properly initialized.
    */
    stdin = (FILE *)fdopen(0, "r");

    // FILE *in;
    // FILE *out;
    // in = stdin;
    // out = stdout;
    // printf("in 0x%x out 0x%x\n", (unsigned)in, (unsigned)out);
    // in = stdin;
    // out = stdout;
    // printf("in 0x%x out 0x%x\n", (unsigned)in, (unsigned)out);

    // puts("\033[34mRIOT telnet_shell example application\033[0m");
    LOG_INFO("\033[34mRIOT telnet_shell example application\033[0m\n");

    /* start shell */
    thread_create(shell_thread_stack, sizeof(shell_thread_stack), 13,
                  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD, shell_thread,
                  NULL, "shell");

    telnet_start_server(23);

    xtimer_usleep(1000*1000);

    LOG_ERROR("hello error\n");
    LOG_WARNING("hello warning\n");
    LOG_INFO("hello info\n");
    LOG_DEBUG("hello debug\n");

    return 0;
}

void *shell_thread(void *arg)
{
    arg = arg;
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands_main, line_buf, sizeof(line_buf));
    return NULL;
}

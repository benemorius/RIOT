/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
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
 * @brief       Shell command to access uart through terminal
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 * @}
 */

/* needed for fdopen */
#ifndef _XOPEN_SOURCE
#define _XOPEN_SOURCE 600
#endif

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include "stdio_base.h"
#include "thread.h"
#include "vfs.h"

static FILE *uart_out;
static FILE *uart_in;
extern kernel_pid_t pcom_pid;

static char _pcom_thread_stack [512];

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

void *_pcom_thread(void *arg)
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

int _pcom(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    printf("opening uart\n");

    int uart_in_fd = vfs_bind(VFS_ANY_FD, O_RDONLY, &_uart_pipe_ops, (void *)STDIN_FILENO);
    int uart_out_fd = vfs_bind(VFS_ANY_FD, O_WRONLY, &_uart_pipe_ops, (void *)STDOUT_FILENO);
    uart_in = (FILE *)fdopen(uart_in_fd, "r");
    uart_out = (FILE *)fdopen(uart_out_fd, "w");
    printf("uart_in fd: %i\n", uart_in_fd);
    printf("uart_out fd: %i\n", uart_out_fd);
    fflush(stdout);

    /* Create thread without yield so that pcom_pid is populated before the
     * first time the thread runs or else it will block on the wrong stdin
     * and not switch to the right stdin until after a character is received.
     */
    pcom_pid = thread_create(_pcom_thread_stack, sizeof(_pcom_thread_stack), 13,
                  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD,
                  _pcom_thread, NULL, "pcom");

    while(1) {
        int c_from_uart = getc(uart_in);
        putc(c_from_uart, stdout);
        if (c_from_uart == '\r') {
            putc('\n', stdout);
        }
        fflush(stdout);
    }
}

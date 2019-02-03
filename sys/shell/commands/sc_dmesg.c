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
 * @brief       Shell commands for dmesg
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "log_dmesg.h"

int _dmesg(int argc, char **argv)
{
    argc = argc;
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
        else if (strcmp(argv[1], "-w") == 0) { // enable output to stdout
            dmesg_attach(stdout);
            return 0;
        }
        else if (strcmp(argv[1], "-W") == 0) { // disable output to stdout
            dmesg_attach(NULL);
            return 0;
        }
        else if ((strcmp(argv[1], "-F") == 0)) { // set filter string
            if (argc != 3) {
                printf("set filter string to filter out messages by level and base filename\n");
                printf("current filter string: \"%s\"\n", dmesg_get_filter_string());
                printf("usage: %s -F \"4-msg.c 4-xtimer_core.c\"\n", argv[0]);
                return 1;
            }
            dmesg_set_filter_string(argv[2]);
            return 0;
        }
        else {
            printf("usage: %s [-m] [-c] [-C] [-F {filter_string}] [-w] [-W]\n",
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
    return 0;
}

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
 * @brief       Shell command to interact with the PM subsystem
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pm_layered.h"

/* TODO deduplicate this definition with the one in sys/pm_layered/pm.c */
typedef union {
    uint32_t val_u32;
    uint8_t val_u8[PM_NUM_MODES];
} pm_blocker_t;

extern volatile pm_blocker_t pm_blocker; /* sys/pm_layered/pm.c */

static void _print_usage(void) {
    printf("Usage:\n");
    printf("\tpm show: display current blockers for each power mode\n");
    printf("\tpm set <mode>: manually set power mode (lasts until WFI returns)\n");
    printf("\tpm block <mode>: manually block power mode\n");
    printf("\tpm unblock <mode>: manually unblock power mode\n");
}

static int check_mode(int argc, char **argv)
{
    if (argc != 3) {
        printf("Usage: %s %s <power mode>\n", argv[0], argv[1]);
        return -1;
    }

    return 0;
}

static int parse_mode(char *argv)
{
    uint8_t mode = atoi(argv);

    if (mode >= PM_NUM_MODES) {
        printf("Error: power mode not in range 0 - %d.\n", PM_NUM_MODES - 1);
        return -1;
    }

    return mode;
}

static int cmd_block(char *arg)
{
    int mode = parse_mode(arg);
    if (mode < 0) {
        return 1;
    }

    printf("Blocking power mode %d.\n", mode);
    fflush(stdout);

    pm_block(mode);

    return 0;
}

static int cmd_set(char *arg)
{
    int mode = parse_mode(arg);
    if (mode < 0) {
        return 1;
    }

    printf("CPU is entering power mode %d.\n", mode);
    printf("Now waiting for a wakeup event...\n");
    fflush(stdout);

    pm_set(mode);
    /* execution stops here until anything (like shell input) wakes the CPU */

    printf("CPU has returned from power mode %d.\n", mode);

    return 0;
}

static int cmd_unblock(char *arg)
{
    int mode = parse_mode(arg);

    if (mode < 0) {
        return 1;
    }

    if (pm_blocker.val_u8[mode] == 0) {
        printf("Mode %d is already unblocked.\n", mode);
        return 1;
    }

    printf("Unblocking power mode %d.\n", mode);
    fflush(stdout);

    pm_unblock(mode);

    return 0;
}

static int cmd_show(char *arg)
{
    (void)arg;
    uint8_t lowest_allowed_mode = 0;

    for (unsigned i = 0; i < PM_NUM_MODES; i++) {
        printf("mode %u blockers: %u \n", i, pm_blocker.val_u8[i]);
        if (pm_blocker.val_u8[i]) {
            lowest_allowed_mode = i + 1;
        }
    }

    printf("Lowest allowed mode: %u\n", lowest_allowed_mode);
    return 0;
}

int _pm(int argc, char **argv)
{
    if (!strcmp(argv[1], "show")) {
        if (argc != 2) {
            printf("usage: pm show: display current blockers for each power mode\n");
            return 1;
        }

        return cmd_show(argv[1]);
    }

    if (!strcmp(argv[1], "block"))
    {
        if (check_mode(argc, argv) != 0) {
            return 1;
        }

        return cmd_block(argv[2]);
    }

    if (!strcmp(argv[1], "unblock"))
    {
        if (check_mode(argc, argv) != 0) {
            return 1;
        }

        return cmd_unblock(argv[2]);
    }

    if (!strcmp(argv[1], "set"))
    {
        if (check_mode(argc, argv) != 0) {
            return 1;
        }

        return cmd_set(argv[2]);
    }

    _print_usage();
    return 1;
}

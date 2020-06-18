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
 * @author      Bas Stottelaar <basstottelaar@gmail.com>
 * @author      Vincent Dupont <vincent@otakeys.com>
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "periph/pm.h"

#ifdef MODULE_PM_LAYERED
#include "pm_layered.h"
#endif

#ifdef MODULE_XTIMER
#include "xtimer.h"
#endif

static void _print_usage(void) {
    puts("Usage:");
#ifdef MODULE_PM_LAYERED
    puts("\tpm show: display current blockers for each power mode");
    puts("\tpm set <mode>: manually set power mode (lasts until WFI returns)");
    puts("\tpm block <mode>: manually block power mode");
    puts("\tpm unblock <mode>: manually unblock power mode");
#ifdef MODULE_XTIMER
    puts("\tpm xsleep <mode> <msec>: unblock <mode> while sleeping for <msec> using xtimer");
#endif
#endif /* MODULE_PM_LAYERED */
    puts("\tpm off: call pm_off()");
}

#ifdef MODULE_PM_LAYERED
static int get_lowest_allowed_mode(void)
{
    pm_blocker_t pm_blocker = pm_get_blocker();

    uint8_t lowest_allowed_mode = 0;
    for (unsigned i = 0; i < PM_NUM_MODES; i++) {
        if (pm_blocker.val_u8[i]) {
            lowest_allowed_mode = i + 1;
        }
    }
    return lowest_allowed_mode;
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
    puts("Now waiting for a wakeup event...");
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

    pm_blocker_t pm_blocker = pm_get_blocker();
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

    pm_blocker_t pm_blocker = pm_get_blocker();
    for (unsigned i = 0; i < PM_NUM_MODES; i++) {
        printf("mode %u blockers: %u \n", i, pm_blocker.val_u8[i]);
    }

    printf("Lowest allowed mode: %u\n", get_lowest_allowed_mode());
    return 0;
}

#ifdef MODULE_XTIMER
static int cmd_xsleep(int argc, char **argv)
{
    if (argc != 4) {
        printf("Usage: %s %s <power mode> <msec>\n", argv[0], argv[1]);
        return -1;
    }

    unsigned long msec = atol(argv[3]);

    cmd_unblock(argv[2]);

    printf("sleeping with xtimer_usleep() in mode %i for %lu msec\n",
           get_lowest_allowed_mode(), msec);

    /* If xtimer is still running in the mode we've ended up in, then this will
     * return as expected. If xtimer isn't running then this won't return. */
    xtimer_usleep(US_PER_MS * msec);
    puts("CPU has returned from sleep");

    cmd_block(argv[2]);

    return 0;
}
#endif /* MODULE_XTIMER */
#endif /* MODULE_PM_LAYERED */

static int cmd_off(char *arg)
{
    (void)arg;

    pm_off();

    return 0;
}

int _pm_handler(int argc, char **argv)
{
    if (argc < 2) {
        _print_usage();
        return 1;
    }

#ifdef MODULE_PM_LAYERED
    if (!strcmp(argv[1], "show")) {
        if (argc != 2) {
            puts("usage: pm show: display current blockers for each power mode");
            return 1;
        }

        return cmd_show(argv[1]);
    }

    if (!strcmp(argv[1], "block")) {
        if (check_mode(argc, argv) != 0) {
            return 1;
        }

        return cmd_block(argv[2]);
    }

    if (!strcmp(argv[1], "unblock")) {
        if (check_mode(argc, argv) != 0) {
            return 1;
        }

        return cmd_unblock(argv[2]);
    }

    if (!strcmp(argv[1], "set")) {
        if (check_mode(argc, argv) != 0) {
            return 1;
        }

        return cmd_set(argv[2]);
    }

#ifdef MODULE_XTIMER
    if (!strcmp(argv[1], "xsleep")) {
        return cmd_xsleep(argc, argv);
    }
#endif /* MODULE_XTIMER */
#endif /* MODULE_PM_LAYERED */

    if (!strcmp(argv[1], "off")) {
        return cmd_off(NULL);
    }

    _print_usage();
    return 1;
}

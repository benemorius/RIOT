/*
 * Copyright (C) 2020 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup examples
 * @{
 *
 * @file
 * @brief   BLE rshell application
 *
 * @author  Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include "ble_rshell.h"
#include "shell.h"
#include "msg.h"

static const char *device_name = "NimBLE ruart on RIOT-OS";

static shell_command_t shell_commands[] = {
    {NULL, NULL, NULL},
};

int main(void)
{
    int rc;

    puts("NimBLE ruart example");

    ble_rshell_t bleshell;
    bleshell.device_name = device_name;
    bleshell.shell_commands = shell_commands;

    /* run the BLE shell */
    rc = ble_rshell_init(&bleshell);
    assert(!rc);

    return 0;
}

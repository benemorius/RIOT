/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_telnet_server    telnet server
 * @ingroup     net
 *
 * @brief       telnet server
 *
 * @{
 *
 * @file
 * @brief   telnet server definitions
 *
 * @author  Thomas Stilwell <stilwellt@openlabs.co>
 */

#ifndef NET_TELNET_SERVER_H
#define NET_TELNET_SERVER_H

#include <errno.h>
#include <stdint.h>
#include <unistd.h>

#include "net/gnrc/tcp.h"
#include "shell.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief telnet server struct
 */
typedef struct {
    uint16_t port;
    gnrc_tcp_tcb_t tcb;
    const shell_command_t *shell_commands; /* deprecated */
} telnet_server_t;

int telnet_start_server(telnet_server_t * telnet_server);

#ifdef __cplusplus
}
#endif

#endif /* NET_TELNET_SERVER_H */
/** @} */

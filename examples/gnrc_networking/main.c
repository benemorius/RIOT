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

#include "shell.h"
#include "msg.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/ipv6.h"

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

extern int udp_cmd(int argc, char **argv);

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};

int main(void)
{
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT network stack example application");

    /* subscribe to ns multicast */
    ipv6_addr_t m_prefix;
    ipv6_addr_from_str(&m_prefix, "ff02::1:ff00:0");
    ipv6_addr_t m_addr;
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    gnrc_netif_get(ifs);
    ipv6_addr_t *addr = &gnrc_ipv6_netif_get(ifs[0])->addrs[1].addr;
    ipv6_addr_init_prefix(&m_addr, addr, 128);
    ipv6_addr_init_prefix(&m_addr, &m_prefix, 104);
    gnrc_ipv6_netif_add_addr(7, &m_addr, 128, GNRC_IPV6_NETIF_ADDR_FLAGS_NDP_AUTO | GNRC_IPV6_NETIF_ADDR_FLAGS_UNICAST);

//     /* set short and long addresses */
//     uint8_t addr[8];
//     size_t addr_len = gnrc_netif_addr_from_str(addr, sizeof(addr), "1:2:3:4:5:6:7:8");
//     gnrc_netapi_set(7, NETOPT_ADDRESS_LONG, 0, addr, addr_len);
//     addr_len = gnrc_netif_addr_from_str(addr, sizeof(addr), "1:2");
//     gnrc_netapi_set(7, NETOPT_ADDRESS, 0, addr, addr_len);
//
//     /* set pan id */
//     uint16_t pan_id = 0x777;
//     gnrc_netapi_set(7, NETOPT_NID, 0, (uint16_t *)&pan_id, sizeof(uint16_t));

    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}

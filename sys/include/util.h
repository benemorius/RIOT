#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include <stddef.h>

#if defined MODULE_GNRC_SOCK_UDP
#include "net/sock/udp.h"
#endif

#include "net/ipv6/addr.h"

#include "board.h"

#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE (3.3f)
#endif

#ifndef ADC_TEMPERATURE_CHANNEL
#define ADC_TEMPERATURE_CHANNEL (0)
#endif

#if defined MODULE_GNRC_SOCK_UDP || defined DOXYGEN
typedef void (*recv_callback_t)(const char *data, size_t length);

typedef struct {
    sock_udp_t sock;
    ipv6_addr_t address;
    char  *address_str;
    uint16_t port;
    recv_callback_t callback;
    sock_udp_ep_t remote;
    int8_t rssi;
} udp_server_t;

ssize_t udp_send(char *dest_ipv6, uint16_t port, const char *data, size_t data_len);
int udp_start_server(udp_server_t *srv);
#endif

#if defined MODULE_SHELL || defined DOXYGEN
#include "shell.h"
void shell_start(const shell_command_t *shell_commands);
#endif

#if defined MODULE_SNTP || defined DOXYGEN
int ntpdate(void);
#endif

void heartbeat_start(void);
void heartbeat_pause(void);
void heartbeat_resume(void);

int ota_start_server(void);

void coap_start_server(char *addr_str, uint16_t port);
void coap_stop_server(void);
int coap_post(char *addr_str, uint16_t port, char *path, char *data, size_t data_len);

#endif /* UTIL_H */

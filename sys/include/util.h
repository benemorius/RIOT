#include <stdint.h>
#include <stddef.h>
#include "net/ipv6/addr.h"

#include "board.h"

#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE (3.3f)
#endif

#ifndef ADC_TEMPERATURE_CHANNEL
#define ADC_TEMPERATURE_CHANNEL (0)
#endif

typedef void (*recv_callback_t)(const char *data, size_t length);

typedef struct {
    ipv6_addr_t address;
    char  *address_str;
    uint16_t port;
    recv_callback_t callback;
} udp_server_t;


size_t udp_send(const char *dest_ipv6, uint16_t port, const char *data, size_t data_len);
int udp_start_server(udp_server_t *srv);

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

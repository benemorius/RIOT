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
#include "xtimer.h"
#include "periph/rtc.h"
#include "periph/adc.h"
#include "net/gnrc/netreg.h"
#include "net/gnrc/pktbuf.h"
#include "fmt.h"

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

char shell_thread_stack [THREAD_STACKSIZE_MAIN + 256];
void *shell_thread(void *arg);

char heartbeat_thread_stack [THREAD_STACKSIZE_MAIN + 256];
void *heartbeat_thread(void *arg);

extern void send(char *addr_str, char *port_str, char *data, unsigned int num,
                 unsigned int delay);


void heartbeat(int on_time, int off_time, int count)
{
    while(count--) {
        LED0_ON;
        xtimer_usleep(1000*on_time);
        LED0_OFF;
        xtimer_usleep(1000*off_time);
    }
}

void *heartbeat_thread(void *arg)
{
    while(1) {
        wdog_feed();

        heartbeat(80, 50, 2);

        xtimer_usleep(1000*(2000 - 80*2 - 50*2));
    }
}

static const uint8_t adc_channel_map[] = {6, 7, 8, 9, 0};

int32_t _read_temperature(void)
{
    uint32_t sample = adc_sample(0, ADC_RES_16BIT);
    // // printf("sample %lu\n", sample);

    float vtemp = (float)sample / 65535 * 3.3;
    float vtemp25 = 0.716f;
    float m = 0.00174f;
    float temperature_c = 25.0f - ((vtemp - vtemp25) / m);
    int32_t temperature_centi_c = (int32_t)(temperature_c * 100);
    // printf("temperature: %i\n", temperature_centi_c);


    // float temperature_c = 25.0 - (((float)sample/65535 * 3.3) - 0.716) / 0.00174;
    int32_t temperature_deci_c = temperature_c * 10;
    // printf("sample %lu temperature %li\n", sample, temperature_deci_c);
    // printf("%li\n", (int32_t)(temperature_c * 1000));
    return temperature_deci_c;
}

int32_t read_temperature(void)
{
    uint16_t averages = 10000;
    int32_t total = 0;
    for(uint16_t i = 0; i < averages; i++) {
        total += _read_temperature();
    }
    return total / averages;
}

int32_t read_adc(uint8_t channel)
{
    channel = adc_channel_map[channel];
    int32_t raw_value = adc_sample(channel, ADC_RES_16BIT);
    return raw_value;
}

void listen_udp(uint16_t port)
{
    msg_t msg_queue[2];
    msg_init_queue(msg_queue, 2);
    gnrc_netreg_entry_t server = GNRC_NETREG_ENTRY_INIT_PID(port, sched_active_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);

    while(1) {
        msg_t msg;
        msg_receive(&msg);
        gnrc_pktsnip_t *pkt = (gnrc_pktsnip_t *)msg.content.ptr;

        gnrc_pktbuf_release(pkt);
        printf("got packet\n");
        heartbeat(200, 100, 2);
    }
}

int main(void)
{
    puts("RIOT network stack example application");

    // while(1) {
    //    xtimer_usleep(1000*1000*10);
    //     printf("hi\n");
    // }

    // /* set pin high for uart level shifter vdd */
    // gpio_init(IO2_PIN, GPIO_OUT);
    // gpio_set(IO2_PIN);

    thread_create(heartbeat_thread_stack, sizeof(heartbeat_thread_stack), 7,
                  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD, heartbeat_thread,
                  NULL, "heartbeat");

    /* start shell */
    thread_create(shell_thread_stack, sizeof(shell_thread_stack), 7,
                  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD, shell_thread,
                  NULL, "shell");

    adc_init(0);
    adc_init(6);
    adc_init(7);
    adc_init(8);
    adc_init(9);

    heartbeat(15, 150, 5);

    // listen_udp(8888);

    xtimer_usleep(1000*1000*1);


    char address[] = "ff02::2%7";
    // char address[] = "2001:470:4bb0:1540::1";
    // char address[] = "fe80::28ac:4270:8210:beec%7";
    // char address[] = "fe80::d808:78c5:678e:b08d";
    char port[] = "9999";
    char data[128] = "";

    xtimer_ticks32_t last_run = xtimer_now();
    while(1) {
        xtimer_periodic_wakeup(&last_run, 1000 * 1000);

        int32_t temperature = read_temperature();
        // continue;
        // int32_t temperature = 0;

        timex_t time;
        xtimer_now_timex(&time);

        // int32_t a0 = read_adc(4);
        // int32_t a6 = read_adc(0);
        // int32_t a7 = read_adc(1);
        // int32_t a8 = read_adc(2);
        // int32_t a9 = read_adc(3);
        // printf("a0: %li\n", a0);
        // printf("a6: %li\n", a6);
        // printf("a7: %li\n", a7);
        // printf("a8: %li\n", a8);
        // printf("a9: %li\n", a9);

        // char temperature_string[8] = {"7"};
        char temperature_string[8] = {'\0'};
        fmt_float(temperature_string, (float)temperature / 10, 1);

        const char json[128] = "{\"name\": \"%s\", \"temperature_c\": %s, \"uptime\": %lu}\n";
        snprintf(data, 128, json, "em", temperature_string, time.seconds);

        // printf("%s", data);
        send(address, port, data, 1, 0);

        xtimer_usleep(1000*10);
    }

    /* should be never reached */
    return 0;
}

extern int udp_cmd(int argc, char **argv);

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};

void *shell_thread(void *arg)
{
    arg = arg;
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    return NULL;
}

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

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

extern int udp_cmd(int argc, char **argv);
extern void send(char *addr_str, char *port_str, char *data, unsigned int num,
                 unsigned int delay);

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};

int32_t read_temperature(void)
{
    uint32_t sample = adc_sample(0, ADC_RES_16BIT);
    printf("sample %lu\n", sample);
    int32_t temperature_milli_c = sample * 100 / 3108 - 212;
    return temperature_milli_c;
}

int main(void)
{
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT network stack example application");

    adc_init(0);
    adc_init(6);
    adc_init(7);
    adc_init(8);
    adc_init(9);


    // rtc_poweron();

    // struct tm time;
    // rtc_set_time(&time);

    char address[] = "ff02::1";
    // char address[] = "fe80::d808:78c5:678e:b08d";
    char port[] = "9999";
    char data[32] = "";


    while(0) {
        int32_t temperature = read_temperature();

        timex_t time;
        xtimer_now_timex(&time);

        printf("%lu %li\n", time.seconds, temperature);

        snprintf(data, 32, "%lu %li %s\n", time.seconds, temperature, "omg!!");

        send(address, port, data, 1, 0);
        LED0_ON;
        xtimer_usleep(1000*100);
        LED0_OFF;
        xtimer_usleep(1000*100);
        LED0_ON;
        xtimer_usleep(1000*100);
        LED0_OFF;
        xtimer_usleep(1000*700);
    }

    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}

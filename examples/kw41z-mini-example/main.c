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

char shell_thread_stack [THREAD_STACKSIZE_MAIN + 256];
void *shell_thread(void *arg);

extern int udp_cmd(int argc, char **argv);
extern void send(char *addr_str, char *port_str, char *data, unsigned int num,
                 unsigned int delay);

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};

static const uint8_t adc_channel_map[] = {6, 7, 8, 9};

int32_t read_temperature(void)
{
    uint32_t sample = adc_sample(0, ADC_RES_16BIT);
    // printf("sample %lu\n", sample);
    int32_t temperature_milli_c = sample * 100 / 3108 - 212;
    return temperature_milli_c;
}

int32_t read_adc(uint8_t channel)
{
    channel = adc_channel_map[channel];
    int32_t raw_value = adc_sample(channel, ADC_RES_16BIT);
    return raw_value;
}

int main(void)
{
    puts("RIOT network stack example application");

    // /* set pin high for uart level shifter vdd */
    // gpio_init(IO2_PIN, GPIO_OUT);
    // gpio_set(IO2_PIN);

    /* start shell */
    thread_create(shell_thread_stack, sizeof(shell_thread_stack), 13,
                  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD, shell_thread,
                  NULL, "shell");

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
    char data[90] = "";

    xtimer_ticks32_t last_run = xtimer_now();
    while(1) {
        xtimer_periodic_wakeup(&last_run, 1000 * 500);
        LED0_ON;

        int32_t temperature = read_temperature();
        // int32_t temperature = 0;

        timex_t time;
        xtimer_now_timex(&time);

        int32_t a0 = read_adc(0);

        int32_t a0_microvolts = (a0 - 32768) * (3.3 / 5 * 1000 / 65535 * 1000 * 1000) / 1000;
        int32_t a0_millivolts = a0_microvolts / 1000;

        // snprintf(data, 90, "%lu %li A0:%liuV A0:%limV\n", time.seconds, temperature, a0_microvolts, a0_millivolts);
        snprintf(data, 90, "%lu %li A0:%limV (%limV)\n", time.seconds, temperature, a0_millivolts, a0_millivolts / 10);

        printf("%s", data);
        send(address, port, data, 1, 0);

        // xtimer_usleep(1000*10);

        LED0_OFF;
    }

    /* should be never reached */
    return 0;
}

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

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

char sensor_thread_stack [THREAD_STACKSIZE_MAIN + 256];
void *sensor_thread(void *arg);

extern int udp_cmd(int argc, char **argv);
extern void send(char *addr_str, char *port_str, char *data, unsigned int num,
                 unsigned int delay);

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};

static const uint8_t adc_channel_map[] = {6, 7, 8, 9};

static int32_t average_adc_value;
static float average_temperature;
static double average_voltage_raw;
static double average_voltage2_raw;

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
    return _read_temperature();
    uint16_t averages = 100;
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
    raw_value = adc_sample(channel, ADC_RES_16BIT);
    raw_value = adc_sample(channel, ADC_RES_16BIT);
    raw_value = adc_sample(channel, ADC_RES_16BIT);
    raw_value = adc_sample(channel, ADC_RES_16BIT);
    return raw_value;
}

void *sensor_thread(void *arg)
{
    average_voltage_raw = read_adc(2);
    average_voltage2_raw = read_adc(0);
    average_temperature = read_temperature();
    uint16_t averages = 200;

    adc_init(0);
    adc_init(6);
    // adc_init(7);
    adc_init(8);
    // adc_init(9);

    xtimer_ticks32_t last_run = xtimer_now();
    while(1) {
        xtimer_periodic_wakeup(&last_run, 1000 * 10);

        int32_t voltage_raw = read_adc(2);
        average_voltage_raw = ((float)voltage_raw + (average_voltage_raw * ((float)averages - 1))) / (float)averages;

        int32_t voltage2_raw = read_adc(0);
        average_voltage2_raw = ((float)voltage2_raw + (average_voltage2_raw * ((float)averages - 1))) / (float)averages;

        int32_t temperature_raw = read_temperature();
        average_temperature = ((float)temperature_raw + (average_temperature * ((float)averages - 1))) / (float)averages;

    }


    return NULL;
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

    thread_create(sensor_thread_stack, sizeof(sensor_thread_stack), 6,
                  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD, sensor_thread,
                  NULL, "sensor");


    // rtc_poweron();

    // struct tm time;
    // rtc_set_time(&time);

    char address[] = "ff02::2%7";
    // char address[] = "fe80::d808:78c5:678e:b08d";
    char port[] = "9999";
    char data[128] = "";

    xtimer_ticks32_t last_run = xtimer_now();
    while(1) {
        xtimer_periodic_wakeup(&last_run, 1000 * 1000);
        LED0_ON;

        int32_t temperature = average_temperature;
        // int32_t temperature = 0;

        timex_t time;
        xtimer_now_timex(&time);

        int32_t battery_millivolts = average_voltage_raw * (3.3f / 65536.0f / (1200.0f/(1200.0f+10000.0f)) * 1000);
        battery_millivolts = (float)battery_millivolts * 1.012;

        int32_t battery2_millivolts = average_voltage2_raw * (3.3f / 65536.0f / (2000.0f/(2000.0f+10000.0f)) * 1000);
        battery2_millivolts = (float)battery2_millivolts * 1.0;
        battery2_millivolts -= 56;

        int32_t difference = battery_millivolts - battery2_millivolts;


        int32_t a0 = average_adc_value;
        int32_t a0_microvolts = (a0 - 32768) * (3.3 / 5 * 1000 / 65536 * 1000 * 1000) / 1000;
        a0_microvolts = (float)a0_microvolts * 1.04;
        a0_microvolts = difference * 1000;

        int32_t a0_millivolts = a0_microvolts / 1000;
        // int32_t milliamps = a0_millivolts * 24;
        int32_t milliamps = a0_microvolts * 100 / 1000;
        // milliamps = (float)milliamps * 1.11;

        int32_t milliwatts = milliamps * battery_millivolts / 1000;

        char sign[2] = " ";
        if(milliamps < 0) sign[0] = '-';

        char temperature_sign[2] = " ";
        if(temperature < 0) temperature_sign[0] = '-';

        // snprintf(data, 90, "%lu %li A0:%liuV A0:%limV\n", time.seconds, temperature, a0_microvolts, a0_millivolts);
        snprintf(data, 90, "%lu %s%i.%01i C  %s%i.%01i A  %li.%03li V  %li mV  %li watts\n", time.seconds, temperature_sign, abs(temperature / 10), abs(temperature % 10), sign, abs(milliamps / 1000), abs(milliamps / 100 % 10), battery_millivolts / 1000, battery_millivolts % 1000, difference, milliwatts / 1000);
        // snprintf(data, 90, "%lu %li A0: %limV (%limV)\n", time.seconds, temperature, a0_millivolts, a0_millivolts / 10);
        printf("%s", data);

        const char json[128] = "{\"name\": \"%s\", \"temperature_c\": %s%i.%01i, \"uptime\": %lu, \"voltage\": %li.%03li}\n";
        snprintf(data, 128, json, "bm", temperature_sign, abs(temperature / 10), abs(temperature % 10), time.seconds, battery_millivolts / 1000, battery_millivolts % 1000);

        printf("%s", data);
        send(address, port, data, 1, 0);

        // xtimer_usleep(1000*10);

        wdog_feed();

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

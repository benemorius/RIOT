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

char heartbeat_thread_stack [THREAD_STACKSIZE_MAIN + 256];
void *heartbeat_thread(void *arg);

extern int udp_cmd(int argc, char **argv);
extern void send(char *addr_str, char *port_str, char *data, unsigned int num,
                 unsigned int delay);

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};

void *heartbeat_thread(void *arg)
{
    while(1) {
        wdog_feed();

        LED0_ON;
        xtimer_usleep(1000*80);
        LED0_OFF;
        xtimer_usleep(1000*50);

        LED0_ON;
        xtimer_usleep(1000*80);
        LED0_OFF;
        xtimer_usleep(1000*50);

        xtimer_usleep(1000*1740);
    }
}

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

void send_packet(void)
{
    char address[] = "ff02::1";
    // char address[] = "fe80::b413:88a6:ba1:2b39";
    // char port[] = "55555";
    char port[] = "55556";
    char data[50] = "";

    // snprintf(data, 50, "{\"door\":\"13\", \"badge\":\"7718923\"}\n");
    snprintf(data, 50, "{\"relay\":\"13\", \"seconds\":\"1\"}\n");
    send(address, port, data, 1, 0);
    thread_yield();
    printf("%s", data);
}

void send_tag_packet(uint16_t door_number, uint32_t tag_number)
{
    char address[] = "ff02::1";
    // char address[] = "fd01:beef::1";
    // char address[] = "2001:470:4bb0:803::2";
    char port[] = "45555";
    char data[50] = "";

    snprintf(data, 50, "{\"door\":\"%u\", \"badge\":\"%lu\"}\n", door_number, tag_number);
    send(address, port, data, 1, 0);
    thread_yield();
    // printf("%s", data);
}

typedef enum WiegandBit {
    bit_0,
    bit_1
} WiegandBit;

typedef struct {
    gpio_t pin;
    bool last_state;
    uint32_t low_start_time;

} WiegandLine;

typedef struct {
    uint8_t door;
    WiegandLine *pin_0;
    WiegandLine *pin_1;
    uint64_t last_activity;
    uint64_t first_activity;
    uint8_t current_bit;
    uint32_t current_value;
    uint32_t bit_lengths[26];
    uint64_t packet_time;
} WiegandChannel;
WiegandChannel wiegand_channels[4];

void finish_wiegand_packet(WiegandChannel *channel) {

}

void handle_wiegand_bit(WiegandChannel *channel, WiegandBit bit, uint32_t microseconds)
{
    uint64_t usec_since_last_activity = xtimer_now_usec64() - channel->last_activity;
    if(usec_since_last_activity > 100000) {
        channel->current_bit = 0;
        channel->current_value = 0;
        channel->first_activity = xtimer_now_usec64();
    }

    channel->bit_lengths[channel->current_bit] = microseconds;

    if(bit == bit_1) {
        channel->current_value |= 1 << (25 - channel->current_bit);
    }
    channel->current_bit++;
    channel->last_activity = xtimer_now_usec64();

    // printf("%u bit %u: %lu\n", (uint32_t)channel & 0xff, bit, microseconds);

    if(channel->current_bit == 26) {
        uint32_t tag_number = (channel->current_value >> 1) & 0xffffff;
        send_tag_packet(channel->door, tag_number);

        printf("\n");

        for(uint8_t i=0; i<26; i++) {
            printf("  %u ", (channel->current_value >> (25 - i)) & 1);
        }
        printf(" value\n");

        for(uint8_t i=0; i<26; i++) {
            printf("% 3lu ", channel->bit_lengths[i]);
        }
        printf(" us\n");

        for(uint8_t i=0; i<26; i++) {
            printf("% 3u ", i);
        }
        printf(" bit\n");

        channel->current_value = (channel->current_value >> 1) & 0xffffff;
        printf("tag number: %lu\n", channel->current_value);

        channel->packet_time = channel->last_activity - channel->first_activity;
        channel->packet_time += channel->bit_lengths[0];
        uint32_t packet_time = channel->packet_time;
        printf("transmission length: %lu us\n", packet_time);

        printf("door %u\n", channel->door);
    }
}

uint32_t wiegand_low_start_times[8];
static void wiegand_interrupt(const void *arg)
{
    gpio_t gpio_pin = *(gpio_t*)arg;
    bool pin_state = gpio_read(gpio_pin);

    WiegandChannel *channel = 0;
    WiegandLine *line = 0;
    WiegandBit bit;
    for(uint8_t i = 0; i < 4; i++) {
        if(wiegand_channels[i].pin_0->pin == gpio_pin) {
            channel = &wiegand_channels[i];
            line = channel->pin_0;
            bit = bit_0;
        }
        else if(wiegand_channels[i].pin_1->pin == gpio_pin) {
            channel = &wiegand_channels[i];
            line = channel->pin_1;
            bit = bit_1;
        }
    }
    if(channel == 0) {
        printf("error\n");
    }

    if(pin_state == false && line->last_state == true) {
        line->low_start_time = xtimer_now_usec();
    }

    if(pin_state == true && line->last_state == false) {
        uint32_t low_time = xtimer_now_usec() - line->low_start_time;
        handle_wiegand_bit(channel, bit, low_time);
    }

    line->last_state = pin_state;
}

void register_wiegand_channel(uint8_t channel, uint8_t door, WiegandLine *pin_0, WiegandLine *pin_1)
{
    wiegand_channels[channel].door = door;
    wiegand_channels[channel].pin_0 = pin_0;
    wiegand_channels[channel].pin_1 = pin_1;
    wiegand_channels[channel].last_activity = 0;
    wiegand_channels[channel].current_bit = 0;
    wiegand_channels[channel].current_value = 0;
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

    WiegandLine channel_0_pin_0 = {IO6_PIN, true, 0};
    WiegandLine channel_0_pin_1 = {IO7_PIN, true, 0};

    WiegandLine channel_1_pin_0 = {IO9_PIN, true, 0};
    WiegandLine channel_1_pin_1 = {IO8_PIN, true, 0};

    WiegandLine channel_2_pin_0 = {IO10_PIN, true, 0};
    WiegandLine channel_2_pin_1 = {IO12_PIN, true, 0};

    WiegandLine channel_3_pin_0 = {IO11_PIN, true, 0};
    WiegandLine channel_3_pin_1 = {IO13_PIN, true, 0};
    register_wiegand_channel(0, 13, &channel_0_pin_0, &channel_0_pin_1);
    register_wiegand_channel(1, 11, &channel_1_pin_0, &channel_1_pin_1);
    register_wiegand_channel(2, 14, &channel_2_pin_0, &channel_2_pin_1);
    register_wiegand_channel(3, 12, &channel_3_pin_0, &channel_3_pin_1);


    // /* setup wiegand inputs */
    // int ret;
    // ret = gpio_init_int(IO6_PIN, GPIO_IN, GPIO_FALLING, wiegand_interrupt, 0);
    // printf("got %i\n", ret);
    // ret = gpio_init_int(IO7_PIN, GPIO_IN, GPIO_FALLING, wiegand_interrupt, 1);
    // printf("got %i\n", ret);

    // gpio_irq_enable(IO6_PIN);
    // gpio_irq_enable(IO7_PIN);

    while(0) {
        if(!gpio_read(IO6_PIN)) {
            send_packet();
            uint32_t start = xtimer_now_usec();
            while(!gpio_read(IO6_PIN));
            uint32_t ticks = xtimer_now_usec() - start;
            printf("0 %lu\n", ticks);

            xtimer_usleep(1000*100);
        }
        if(!gpio_read(IO7_PIN)) {
            uint32_t start = xtimer_now_usec();
            while(!gpio_read(IO7_PIN));
            uint32_t ticks = xtimer_now_usec() - start;
            printf("1 %lu\n", ticks);
        }
        thread_yield();
    }

    gpio_t wiegand_pins[8] = {IO6_PIN, IO7_PIN, IO8_PIN, IO9_PIN, IO10_PIN, IO12_PIN, IO11_PIN, IO13_PIN};
    uint8_t pin_states[8];
    for(uint8_t i = 0; i < 8; i++) {
        gpio_init(wiegand_pins[i], GPIO_IN_PD);
        uint8_t state = gpio_read(wiegand_pins[i]);
        pin_states[i] = state;
        printf("pin %u gpio %u state %u\n", i, wiegand_pins[i], state);
    }
    while(1) {
        for(uint8_t i = 0; i < 8; i++) {
            uint8_t pin_state = gpio_read(wiegand_pins[i]);
            if(pin_states[i] != pin_state) {
                wiegand_interrupt(&wiegand_pins[i]);
            }
            pin_states[i] = pin_state;
        }
        thread_yield();
    }

    uint32_t start_packet_time = 0;
    uint32_t end_packet_time = 0;
    uint32_t bit_periods[32];
    uint8_t bit = 0;
    uint32_t tag_number = 0;
    while(1) {
        if(!gpio_read(IO9_PIN)) {
            uint32_t start = xtimer_now_usec();
            if(!start_packet_time)
                start_packet_time = start;
            while(!gpio_read(IO9_PIN));
            uint32_t now = xtimer_now_usec();
            uint32_t ticks = now - start;
            bit_periods[bit] = ticks;
            tag_number <<= 1;
            bit++;
            end_packet_time = now;
            if(bit == 1) {
                printf("\n");
            }
            printf("  0 ");
        }
        else if(!gpio_read(IO8_PIN)) {
            uint32_t start = xtimer_now_usec();
            if(!start_packet_time)
                start_packet_time = start;
            while(!gpio_read(IO8_PIN));
            uint32_t now = xtimer_now_usec();
            uint32_t ticks = now - start;
            bit_periods[bit] = ticks;
            tag_number <<= 1;
            tag_number |= 1;
            bit++;
            end_packet_time = now;
            if(bit == 1) {
                printf("\n");
            }
            printf("  1 ");
        }
        else if(start_packet_time) {
            if(start_packet_time + 100000 < xtimer_now_usec()) {
                printf(" value\n");
                uint8_t bits = bit;
                for(uint8_t i=0; i<bits; i++) {
                    printf("% 3lu ", bit_periods[i]);
                }
                printf(" us\n");
                for(uint8_t i=0; i<bits; i++) {
                    printf("% 3u ", i);
                }
                printf(" bit\n");
                tag_number >>= 1;
                tag_number &= 0xffffff;
                send_tag_packet(113, tag_number);
                printf("tag number: %lu\n", tag_number);
                uint32_t packet_time = end_packet_time - start_packet_time;
                printf("transmission length: %lu us\n", packet_time);
                start_packet_time = 0;
                end_packet_time = 0;
                bit = 0;
                tag_number = 0;
            }
        }
        thread_yield();
    }


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

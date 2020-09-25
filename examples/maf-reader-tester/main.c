/*
 * Copyright (C) 2020 Thomas Stilwell <stilwellt@openlabs.co>
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
 * @brief       maf application
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include <stdio.h>

#include "msg.h"
#include "util.h"
#include "luid.h"
#include "xtimer.h"
#include "random.h"
#include "periph/pm.h"
#include "periph/adc.h"
#include "periph/gpio.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define NUM_CHANNELS (16)
static uint32_t last_us[NUM_CHANNELS];
static uint32_t period_ns[NUM_CHANNELS];
static bool on_state[NUM_CHANNELS];
static char device_id[17];

void pin_interrupt_cb(void *arg)
{
    uint8_t channel = (uintptr_t)arg;

    uint32_t now_us = xtimer_now_usec();

    uint32_t this_period_us = now_us - last_us[channel];
    period_ns[channel] = (period_ns[channel] * 999 + this_period_us * NS_PER_US) / 1000;

    last_us[channel] = now_us;
}

int main(void)
{
    msg_t msg_queue[4];
    msg_init_queue(msg_queue, 4);

    /* get the microcontroller's unique ID to use as devie_id */
    eui64_t eui;
    luid_get_eui64(&eui);
    snprintf(device_id, sizeof(device_id), "%lx",
             byteorder_ntohl(eui.uint32[1]));

    /* initialize all the input channels */
    gpio_init_int(GPIO_PIN(0, 16), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)0);
    gpio_init_int(GPIO_PIN(0, 17), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)1);
    gpio_init_int(GPIO_PIN(0, 19), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)2);
    gpio_init_int(GPIO_PIN(2, 4), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)3);
    gpio_init_int(GPIO_PIN(2, 3), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)4);
    gpio_init_int(GPIO_PIN(2, 2), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)5);
    gpio_init_int(GPIO_PIN(2, 1), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)6);
    gpio_init_int(GPIO_PIN(2, 19), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)7);
    gpio_init_int(GPIO_PIN(2, 5), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)8);
    gpio_init_int(GPIO_PIN(2, 17), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)9);
    gpio_init_int(GPIO_PIN(2, 18), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)10);
    gpio_init_int(GPIO_PIN(2, 16), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)11);
    gpio_init_int(GPIO_PIN(1, 1), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)12);
    gpio_init_int(GPIO_PIN(1, 2), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)13);
    gpio_init_int(GPIO_PIN(1, 3), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)14);
    gpio_init_int(GPIO_PIN(1, 18), GPIO_IN_PU, GPIO_FALLING, pin_interrupt_cb, (void *)15);

    /* set period for all channels to 1000 us to default to 0% flow */
    for (uint8_t channel = 0; channel < NUM_CHANNELS; channel++) {
        period_ns[channel] = 1000000;
    }

    /* wait for readings to stabilize */
    xtimer_sleep(1);

    while(1) {
        xtimer_usleep(5 * US_PER_SEC);

        printf("%s, ", device_id);

        for (uint8_t channel = 0; channel < NUM_CHANNELS; channel++) {
            uint32_t period = period_ns[channel];

            int32_t perthou = ((float)period - 220000) / 80000 * 1000;
            perthou = 1000 - perthou;
            if (perthou < 0) {
                perthou = 0;
            }
            if (perthou > 1000) {
                perthou = 1000;
            }

            if (channel) {
                printf(", ");
            }
            printf("%3" PRIi32 ".%" PRIu32, perthou / 10, perthou % 10);
        }

        printf("\n");

        for (uint8_t channel = 0; channel < NUM_CHANNELS; channel++) {
            uint32_t period = period_ns[channel];
            int32_t perthou = ((float)period - 220000) / 80000 * 1000;
            perthou = 1000 - perthou;

            if (!on_state[channel] && perthou > 500) {
                on_state[channel] = true;
                printf("air flow %s %u has turned ON\n", device_id, channel);
            }
            else if (on_state[channel] && perthou < 500) {
                on_state[channel] = false;
                printf("air flow %s %u has turned OFF\n", device_id,  channel);
            }
        }
    }

    return 0;
}

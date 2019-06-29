/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_openlabs-6lowpan-led-driver
 * @{
 *
 * @file
 * @brief       Board specific initialization for openlabs-6lowpan-led-driver
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include "board.h"
#include "periph/rtc.h"
#include "periph/gpio.h"

void board_init(void)
{
    /* initialize the CPU core */
    cpu_init();

    /* initialize and turn off LEDs */
    gpio_clear(LED0_PIN);
    gpio_init(LED0_PIN, GPIO_OUT);

    gpio_clear(LED_R_PIN);
    gpio_clear(LED_G_PIN);
    gpio_clear(LED_B_PIN);
    gpio_clear(LED_W_PIN);
    gpio_clear(LED_WW_PIN);

    gpio_init(LED_R_PIN, GPIO_OUT);
    gpio_init(LED_G_PIN, GPIO_OUT);
    gpio_init(LED_B_PIN, GPIO_OUT);
    gpio_init(LED_W_PIN, GPIO_OUT);
    gpio_init(LED_WW_PIN, GPIO_OUT);

#ifdef MODULE_RTC
    rtc_set_compensation(-10, 124);
#endif
}

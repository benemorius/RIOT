/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_openlabs-kw41z-slip
 * @{
 *
 * @file
 * @brief       Board specific initialization for openlabs-kw41z-slip
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
    LED0_OFF;
    gpio_init(LED0_PIN, GPIO_OUT);

    LED1_OFF;
    gpio_init(LED1_PIN, GPIO_OUT);

#ifdef MODULE_RTC
    rtc_set_compensation(-10, 124);
#endif
}

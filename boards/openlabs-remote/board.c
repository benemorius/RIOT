/*
 * Copyright (C) 2015 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_openlabs-remote
 * @{
 *
 * @file
 * @brief       Board specific implementations for openlabs-remote
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include "board.h"
#include "periph/gpio.h"

void board_init(void)
{
    cpu_init();

    /* initialize epaper gpios */
    gpio_set(LCD_POWER); /* active low */
    gpio_clear(LCD_CS);
    gpio_clear(LCD_RST);
    gpio_clear(LCD_DISCHARGE);

    gpio_init(LCD_CS, GPIO_OUT);
    gpio_init(LCD_RST, GPIO_OUT);
    gpio_init(LCD_POWER, GPIO_OUT);
    gpio_init(LCD_DISCHARGE, GPIO_OUT);
    gpio_init(LCD_BUSY, GPIO_IN);
}

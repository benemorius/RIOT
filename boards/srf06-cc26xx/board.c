/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_efm32gg-stk3700
 * @{
 *
 * @file
 * @brief       Board specific implementations for the STM32F3Discovery evaluation board
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author		Ryan Kurte <ryankurte@gmail.com>
 *
 * @}
 */

#include "board.h"
#include "cpu.h"

#include "stdio.h"

static void leds_init(void);

void board_init(void)
{
// 	SystemInit();

    /* initialize the CPU */
    cpu_init();

    /* initialize the boards LEDs */
    leds_init();


}

/**
 * @brief Initialize the boards on-board LEDs
 *
 */
static void leds_init(void)
{

}



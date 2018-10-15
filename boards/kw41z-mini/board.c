/*
 * Copyright (C) 2017 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_kw41z-mini
 * @{
 *
 * @file
 * @brief       Board specific initialization for the kw41z-mini
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include "board.h"
#include "periph/gpio.h"
#include "periph/rtt.h"

static inline void set_lpuart_clock_source(void)
{
	/* Use MCGIRCLK (internal reference 4 MHz clock) */
	SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_LPUART0SRC_MASK) | SIM_SOPT2_LPUART0SRC(3);
}

void board_init(void)
{
	/* initialize the CPU core */
	cpu_init();

	/* configure hsosc trim */
	MCG->C4 = MCG->C4 & ~MCG_C4_FCTRIM_MASK | MCG_C4_FCTRIM(0xc);
	MCG->C2 = MCG->C2 & ~MCG_C2_FCFTRIM_MASK | MCG_C2_FCFTRIM(0x1);

	set_lpuart_clock_source();

	/* initialize and turn off LEDs */
	gpio_init(LED0_PIN, GPIO_OUT);
	LED0_OFF;
}

/*
 * Copyright (C) 2017 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_openlabs-kw41z-mini
 * @{
 *
 * @file
 * @brief       Board specific initialization for openlabs-kw41z-mini
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

#ifdef OPENLABS_KW41Z_MINI_BMX280
    /* GND pin for mini-bmx280 */
    gpio_clear(IO_9_PIN);
    gpio_init(IO_9_PIN, GPIO_OUT);
    /* VDD pin for mini-bmx280 */
    gpio_set(IO_6_PIN);
    gpio_init(IO_6_PIN, GPIO_OUT);
#endif

#ifdef OPENLABS_KW41Z_MINI_EEPROM
    /* GND pin for mini-eeprom */
    gpio_clear(IO_9_PIN);
    gpio_init(IO_9_PIN, GPIO_OUT);
    /* VDD pin for mini-eeprom */
    gpio_set(IO_6_PIN);
    gpio_init(IO_6_PIN, GPIO_OUT);
#endif

    /* enable OSCERCLK output on PTB3 */
#if PTB3_OUTPUT_OSCERCLK
    SIM->SOPT2 |= SIM_SOPT2_CLKOUTSEL(0b110);
    gpio_init_port(GPIO_PIN(PORT_B,  3), PORT_PCR_MUX(4));

    /* enable 32KHz oscillator output on PTB3 */
#elif PTB3_OUTPUT_OSC32K
    SIM->SOPT1 |= SIM_SOPT1_OSC32KOUT_MASK;
#endif

#ifdef MODULE_RTC
    rtc_set_compensation(-10, 124);
#endif
}

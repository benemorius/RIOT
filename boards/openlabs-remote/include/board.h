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
 * @brief       Board specific definitions for openlabs-remote
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Bas Stottelaar <basstottelaar@gmail.com>
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 */
#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"
#include "periph_conf.h"
#include "periph/gpio.h"
#include "periph/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Xtimer configuration
 *
 * The timer runs at 250 KHz to increase accuracy.
 * @{
 */
#define XTIMER_HZ           (32768UL)
#define XTIMER_WIDTH        (16)
/** @} */

/**
 * @name    Board controller configuration
 *
 * Define the GPIO pin to enable the BC, to allow serial communication
 * via the USB port.
 * @{
 */
#define BC_PIN              GPIO_PIN(PF, 7) /* not connected */
/** @} */

/**
 * @name    Push button pin definitions
 * @{
 */
#if 0
#define PB1_PIN             GPIO_PIN(PF, 3) /* conflicts with at86_irq */
#else
#define PB1_PIN             GPIO_PIN(PE, 8) /* bodge wire to mem_wp */
#endif
#define PB2_PIN             GPIO_PIN(PC, 9)
#define PB3_PIN             GPIO_PIN(PC, 15)
#define PB4_PIN             GPIO_PIN(PC, 10)
#define PB5_PIN             GPIO_PIN(PC, 14)
#define PB6_PIN             GPIO_PIN(PC, 11)
#define PB7_PIN             GPIO_PIN(PC, 13)
#define PB8_PIN             GPIO_PIN(PC, 12)
/** @} */

/**
 * @name    LED pin definitions
 * @{
 */
#if 0
#define LED0_PIN            GPIO_PIN(PE, 15)
#define LED1_PIN            GPIO_PIN(PE, 14)
#define LED2_PIN            GPIO_PIN(PE, 13)
#else
/* steal pins 14 and 15 for lpuart */
#define LED0_PIN            GPIO_PIN(PE, 13)
#endif

/** @} */

/**
 * @name    Macros for controlling the on-board LEDs
 * @{
 */
#define LED0_ON             gpio_set(LED0_PIN)
#define LED0_OFF            gpio_clear(LED0_PIN)
#define LED0_TOGGLE         gpio_toggle(LED0_PIN)
// #define LED1_ON             gpio_set(LED1_PIN)
// #define LED1_OFF            gpio_clear(LED1_PIN)
// #define LED1_TOGGLE         gpio_toggle(LED1_PIN)
// #define LED2_ON             gpio_set(LED2_PIN)
// #define LED2_OFF            gpio_clear(LED2_PIN)
// #define LED2_TOGGLE         gpio_toggle(LED2_PIN)
/** @} */


/**
 * @brief   Initialize the board (GPIO, sensors, clocks).
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */

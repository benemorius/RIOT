/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_openlabs-6lowpan-led-driver
 * @{
 *
 * @file
 * @brief       Board specific definitions for openlabs-6lowpan-led-driver
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * NMI shares a pin with DAC output and ADC input. Holding the pin low during
 * reset will cause a hang unless NMI is disabled. It can be enabled in
 * applications where the pin is not held low during reset.
 */
#define KINETIS_FOPT 0xFB /* disable NMI (0xFF to enable) */

/**
 * @name   LED pin definitions and handlers
 * @{
 */
#define LED0_PIN           GPIO_PIN(PORT_C,  16)
#define LED0_MASK          (1 << 16)
#define LED0_ON            (GPIOC->PSOR = LED0_MASK)
#define LED0_OFF           (GPIOC->PCOR = LED0_MASK)
#define LED0_TOGGLE        (GPIOC->PTOR = LED0_MASK)
/** @} */

/**
 * @name   GPIO pin definitions
 * @{
 */
#define LED_R_PIN             GPIO_PIN(PORT_B,  2)
#define LED_G_PIN             GPIO_PIN(PORT_B,  1)
#define LED_B_PIN             GPIO_PIN(PORT_B,  0)
#define LED_W_PIN             GPIO_PIN(PORT_A,  18)
#define LED_WW_PIN            GPIO_PIN(PORT_A,  16)
/** @} */

/**
 * @name    xtimer configuration
 * @{
 */
#if KINETIS_XTIMER_SOURCE_PIT
/* PIT xtimer configuration */
#define XTIMER_DEV                  (TIMER_PIT_DEV(0))
#define XTIMER_CHAN                 (0)
/* Default xtimer settings should work on the PIT */
#else
/* LPTMR xtimer configuration */
#define XTIMER_DEV                  (TIMER_LPTMR_DEV(0))
#define XTIMER_CHAN                 (0)
/* LPTMR is 16 bits wide and runs at 32768 Hz (clocked by the RTC) */
#define XTIMER_WIDTH                (16)
#define XTIMER_BACKOFF              (16)
#define XTIMER_ISR_BACKOFF          (5)
#define XTIMER_HZ                   (32768ul)
#endif
/** @} */

/**
 * @brief Initialize board-specific hardware, including clock, LEDs, and stdio
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /** BOARD_H */
/** @} */

/*
 * Copyright (C) 2017 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_openlabs-kw41z-mini
 * @{
 *
 * @file
 * @brief       Board specific definitions for openlabs-kw41z-mini
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
#define LED0_PIN           GPIO_PIN(PORT_B,  0)
#define LED0_MASK          (1 << 0)
#define LED0_ON            (GPIOB->PCOR = LED0_MASK)
#define LED0_OFF           (GPIOB->PSOR = LED0_MASK)
#define LED0_TOGGLE        (GPIOB->PTOR = LED0_MASK)
/** @} */

/**
 * @name   GPIO pin definitions
 * @{
 */
#define IO_0_PIN           GPIO_PIN(PORT_C,  7)
#define IO_1_PIN           GPIO_PIN(PORT_C,  6)
#define IO_2_PIN           GPIO_PIN(PORT_A,  16)
#define IO_3_PIN           GPIO_PIN(PORT_A,  17)
#define IO_4_PIN           GPIO_PIN(PORT_A,  18)
#define IO_5_PIN           GPIO_PIN(PORT_A,  19)
#define IO_6_PIN           GPIO_PIN(PORT_C,  4)
#define IO_7_PIN           GPIO_PIN(PORT_C,  3)
#define IO_8_PIN           GPIO_PIN(PORT_C,  2)
#define IO_9_PIN           GPIO_PIN(PORT_C,  1)
#define IO_10_PIN          GPIO_PIN(PORT_C,  19)
#define IO_11_PIN          GPIO_PIN(PORT_C,  17)
#define IO_12_PIN          GPIO_PIN(PORT_C,  18)
#define IO_13_PIN          GPIO_PIN(PORT_C,  16)

#define IO_SDA_PIN         IO_7_PIN
#define IO_SCL_PIN         IO_8_PIN

#define IO_TXO_PIN         IO_0_PIN
#define IO_RXI_PIN         IO_1_PIN

#define IO_MOSI_PIN        IO_11_PIN
#define IO_MISO_PIN        IO_12_PIN
#define IO_SCK_PIN         IO_13_PIN

#define IO_A0_PIN          GPIO_PIN(PORT_B,  1)
#define IO_A1_PIN          GPIO_PIN(PORT_B,  2)
#define IO_A2_PIN          GPIO_PIN(PORT_B,  3)
#define IO_A3_PIN          GPIO_PIN(PORT_B,  18)
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

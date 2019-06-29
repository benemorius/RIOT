/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_openlabs-kw41z-slip
 * @{
 *
 * @file
 * @brief       Board specific definitions for openlabs-kw41z-slip
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
#define LED0_PIN           GPIO_PIN(PORT_A,  18)
#define LED0_MASK          (1 << 18)
#define LED0_ON            (GPIOA->PSOR = LED0_MASK)
#define LED0_OFF           (GPIOA->PCOR = LED0_MASK)
#define LED0_TOGGLE        (GPIOA->PTOR = LED0_MASK)

#define LED1_PIN           GPIO_PIN(PORT_C,  4)
#define LED1_MASK          (1 << 4)
#define LED1_ON            (GPIOC->PSOR = LED1_MASK)
#define LED1_OFF           (GPIOC->PCOR = LED1_MASK)
#define LED1_TOGGLE        (GPIOC->PTOR = LED1_MASK)

/* kw41zrf by default uses
 * KW41ZRF_LED_RX       LED0
 * KW41ZRF_LED_TX       LED1
 * KW41ZRF_LED_NDSM     LED2
 * KW41ZRF_LED_IRQ      LED3
 */
#ifndef KW41ZRF_ENABLE_LEDS
#define KW41ZRF_ENABLE_LEDS         (1)
#endif
#ifndef KW41Z_SLIP_LED_RX_AS_IRQ
#define KW41Z_SLIP_LED_RX_AS_IRQ    (0)
#endif
#ifndef KW41Z_SLIP_LED_RX_AS_NDSM
#define KW41Z_SLIP_LED_RX_AS_NDSM   (0)
#endif

#if KW41Z_SLIP_LED_RX_AS_IRQ
/* use LED0 as KW41ZRF_LED_IRQ instead of as KW41ZRF_LED_RX */
#define KW41ZRF_LED_RX_ON
#define KW41ZRF_LED_RX_OFF
#define KW41ZRF_LED_IRQ_ON     LED0_ON
#define KW41ZRF_LED_IRQ_OFF    LED0_OFF
#endif /* KW41Z_SLIP_LED_RX_AS_IRQ */

#if KW41Z_SLIP_LED_RX_AS_NDSM
/* use LED0 as KW41ZRF_LED_NDSM instead of as KW41ZRF_LED_RX */
#define KW41ZRF_LED_RX_ON
#define KW41ZRF_LED_RX_OFF
#define KW41ZRF_LED_NDSM_ON     LED0_ON
#define KW41ZRF_LED_NDSM_OFF    LED0_OFF
#endif /* KW41Z_SLIP_LED_RX_AS_NDSM */
/** @} */

/**
 * @name   GPIO pin definitions
 * @{
 */
#define UART_TXO_PIN         GPIO_PIN(PORT_C,  7)
#define UART_RXI_PIN         GPIO_PIN(PORT_C,  6)
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

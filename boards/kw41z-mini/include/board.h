/*
 * Copyright (C) 2017 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    board_kw41z-mini openlabs.co kw41z-mini board
 * @ingroup     boards
 * @brief       Support for the kw41z-mini board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the kw41z-mini
 *
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

/**
 * @brief   LED pin definitions and handlers
 * @{
 */
#define LED0_PIN           GPIO_PIN(PORT_B,  0)
#define LED0_MASK          (1 << 0)
#define LED0_ON            (GPIOB->PCOR = LED0_MASK)
#define LED0_OFF           (GPIOB->PSOR = LED0_MASK)
#define LED0_TOGGLE        (GPIOB->PTOR = LED0_MASK)
/** @} */

/**
 * @brief   GPIO pin definitions
 * @{
 */
#define IO2_PIN            GPIO_PIN(PORT_A,  16)
#define IO3_PIN            GPIO_PIN(PORT_A,  17)
#define IO4_PIN            GPIO_PIN(PORT_A,  18)
#define IO5_PIN            GPIO_PIN(PORT_A,  19)
#define IO6_PIN            GPIO_PIN(PORT_C,  4)
#define IO7_PIN            GPIO_PIN(PORT_C,  3)
#define IO8_PIN            GPIO_PIN(PORT_C,  2)
#define IO9_PIN            GPIO_PIN(PORT_C,  1)
#define IO10_PIN           GPIO_PIN(PORT_C,  19)
#define IO11_PIN           GPIO_PIN(PORT_C,  17)
#define IO12_PIN           GPIO_PIN(PORT_C,  18)
#define IO13_PIN           GPIO_PIN(PORT_C,  16)

#define IO_SDA_PIN         IO7_PIN
#define IO_SCL_PIN         IO8_PIN

#define IO_TXO_PIN         GPIO_PIN(PORT_C,  7)
#define IO_RXI_PIN         GPIO_PIN(PORT_C,  6)

#define IO_MOSI_PIN        IO11_PIN
#define IO_MISO_PIN        IO12_PIN
#define IO_SCK_PIN         IO13_PIN

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
#define XTIMER_BACKOFF              (5)
#define XTIMER_ISR_BACKOFF          (5)
#define XTIMER_OVERHEAD             (4)
#define XTIMER_HZ                   (32768ul)
#endif
/** @} */

/**
 * @name    NOR flash hardware configuration
 * @{
 */
#define FRDM_NOR_SPI_DEV               SPI_DEV(0)
#define FRDM_NOR_SPI_CLK               SPI_CLK_5MHZ
#define FRDM_NOR_SPI_CS                SPI_HWCS(0) /**< Flash CS pin */
/** @} */

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /** BOARD_H */
/** @} */

/*
 * Copyright (C) 2015 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_openlabs-vent
 * @{
 *
 * @file
 * @brief       Board specific definitions for openlabs-vent
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
#define XTIMER_HZ                   (32768UL)
#define XTIMER_WIDTH                (16)
#define XTIMER_WIDTH                (16)
#define XTIMER_BACKOFF              (16)
#define XTIMER_ISR_BACKOFF          (5)
#define XTIMER_OVERHEAD             (0)
/** @} */

/**
 * @name    Push button pin definitions
 * @{
 */
#define BL_PIN              GPIO_PIN(PE, 15)
/** @} */

/**
 * @name    LED pin definitions
 * @{
 */
#define LED0_PIN            GPIO_PIN(PA, 8)
#define LED1_PIN            GPIO_PIN(PA, 9)
#define LED2_PIN            GPIO_PIN(PA, 10)
/** @} */

/**
 * @name    Macros for controlling the on-board LEDs
 * @{
 */
#define LED0_ON             gpio_set(LED0_PIN)
#define LED0_OFF            gpio_clear(LED0_PIN)
#define LED0_TOGGLE         gpio_toggle(LED0_PIN)
#define LED1_ON             gpio_set(LED1_PIN)
#define LED1_OFF            gpio_clear(LED1_PIN)
#define LED1_TOGGLE         gpio_toggle(LED1_PIN)
#define LED2_ON             gpio_set(LED2_PIN)
#define LED2_OFF            gpio_clear(LED2_PIN)
#define LED2_TOGGLE         gpio_toggle(LED2_PIN)
/** @} */

/**
 * @brief   Initialize the board (GPIO, sensors, clocks).
 */
void board_init(void);


// radio
#define AT86RF231_CS        GPIO_4
#define GPIO_4_PORT         gpioPortD
#define GPIO_4_PIN          3

#define AT86RF231_INT       GPIO_5
#define GPIO_5_PORT         gpioPortC
#define GPIO_5_PIN          9

#define AT86RF231_SLEEP     GPIO_6
#define GPIO_6_PORT         gpioPortC
#define GPIO_6_PIN          10

#define AT86RF231_RESET     GPIO_7
#define GPIO_7_PORT         gpioPortC
#define GPIO_7_PIN          11


// rotary encoder
#define ROTARYENCODER_A       GPIO_8
#define GPIO_8_PORT           gpioPortE
#define GPIO_8_PIN            14

#define ROTARYENCODER_B       GPIO_9
#define GPIO_9_PORT           gpioPortE
#define GPIO_9_PIN            15

#define ROTARYENCODER_BUTTON  GPIO_10
#define GPIO_10_PORT          gpioPortE
#define GPIO_10_PIN           13


// relay output
#define RELAY_W             GPIO_11
#define GPIO_11_PORT        gpioPortD
#define GPIO_11_PIN         6

#define RELAY_W2            GPIO_12
#define GPIO_12_PORT        gpioPortD
#define GPIO_12_PIN         7

#define RELAY_G             GPIO_13
#define GPIO_13_PORT        gpioPortD
#define GPIO_13_PIN         8

#define RELAY_OB            GPIO_14
#define GPIO_14_PORT        gpioPortE
#define GPIO_14_PIN         8

#define RELAY_Y             GPIO_15
#define GPIO_15_PORT        gpioPortF
#define GPIO_15_PIN         5

#define RELAY_Y2            GPIO_16
#define GPIO_16_PORT        gpioPortF
#define GPIO_16_PIN         12


// wire input
#define DETECT_W            GPIO_17
#define GPIO_17_PORT        gpioPortB
#define GPIO_17_PIN         11

#define DETECT_W2           GPIO_18
#define GPIO_18_PORT        gpioPortE
#define GPIO_18_PIN         9

#define DETECT_G            GPIO_19
#define GPIO_19_PORT        gpioPortE
#define GPIO_19_PIN         12

#define DETECT_OB           GPIO_20
#define GPIO_20_PORT        gpioPortA
#define GPIO_20_PIN         3

#define DETECT_Y            GPIO_21
#define GPIO_21_PORT        gpioPortA
#define GPIO_21_PIN         4

#define DETECT_Y2           GPIO_22
#define GPIO_22_PORT        gpioPortA
#define GPIO_22_PIN         2


// EEPROM
#define EEPROM_WP           GPIO_23
#define GPIO_23_PORT        gpioPortC
#define GPIO_23_PIN         1


// piezo
#define PIEZO               GPIO_24
#define GPIO_24_PORT        gpioPortC
#define GPIO_24_PIN         8


// wifi
#define WIFI_RST            GPIO_25
#define GPIO_25_PORT        gpioPortA
#define GPIO_25_PIN         8

#define WIFI_WPS            GPIO_26
#define GPIO_26_PORT        gpioPort9
#define GPIO_26_PIN         9


#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */

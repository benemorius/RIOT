/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_efm32gg-st3700
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the STM32F0discovery board
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author		Ryan Kurte <ryankurte@gmail.com>
 */

#ifndef __PERIPH_CONF_H
#define __PERIPH_CONF_H

#include "hw_memmap.h"

/******     From cortex-m3_common/cpu.h for RIOT compatibility      ******/
/**
 * @brief Macro has to be called in the beginning of each ISR
 */
#define ISR_ENTER()     asm("push {LR}")

/**
 * @brief Macro has to be called on each exit of an ISR
 */
#define ISR_EXIT()      asm("pop {r0} \n bx r0")
/******     End     ******/



/**
 * @name Clock system configuration
 * @{
 */
#define CLOCK_CORECLOCK     (48000000U)

/**
 * @name Timer configuration
 * @{
 */
#define TIMER_NUMOF         (3U)
#define TIMER_0_EN          1
#define TIMER_1_EN          1
#define TIMER_2_EN          1

/* Timer 0 configuration */
#define TIMER_0_DEV         TIMER0
#define TIMER_0_CHANNELS    3
#define TIMER_0_PRESCALER   (timerPrescale64)
#define TIMER_0_MAX_VALUE   (0xffff)
#define TIMER_0_CLKEN()     CMU_ClockEnable(cmuClock_TIMER0, true)
#define TIMER_0_ISR         TIMER0_IRQHandler
//#define TIMER_0_IRQ_CHAN    TIM2_IRQn
#define TIMER_0_IRQ_PRIO    1

/* Timer 1 configuration */
#define TIMER_1_DEV         TIMER1
#define TIMER_1_CHANNELS    3
#define TIMER_1_PRESCALER   (timerPrescale1)
#define TIMER_1_MAX_VALUE   (0xffff)
#define TIMER_1_CLKEN()     CMU_ClockEnable(cmuClock_TIMER1, true)
#define TIMER_1_ISR         TIMER1_IRQHandler
//#define TIMER_0_IRQ_CHAN    TIM2_IRQn
#define TIMER_1_IRQ_PRIO    1

/* Low Energy Timer 0 configuration */
#define TIMER_2_DEV         LETIMER0
#define TIMER_2_CHANNELS    2
#define TIMER_2_PRESCALER   (timerPrescale1024)
#define TIMER_2_MAX_VALUE   (0xffff)
#define TIMER_2_CLKEN()     CMU_ClockEnable(cmuClock_LETIMER0, true);CMU_ClockEnable(cmuClock_CORELE, true)
#define TIMER_2_ISR         LETIMER0_IRQHandler
//#define TIMER_2_IRQ_CHAN    LETIMER0_IRQn
#define TIMER_2_IRQ_PRIO    1

#define WTIMER TIMER_2
#define WTIMER_CHAN 0
#define WTIMER_US_PER_TICK 31
#define WTIMER_BACKOFF 3
#define WTIMER_OVERHEAD 1
#define WTIMER_ISR_BACKOFF 1
#define WTIMER_USLEEP_UNTIL_OVERHEAD 1
#define WTIMER_MASK 0xFFFF0000


/**
 * @name UART configuration
 * @{
 */
#define UART_NUMOF          (1U)
#define UART_0_EN           1
#define UART_1_EN           0
#define UART_2_EN           0
#define UART_IRQ_PRIO       1

#define UART_0_DEV          (uint32_t*)(UART0_BASE)
#define UART_0_CLKEN()      CMU_ClockEnable(cmuClock_USART0, true)
#define UART_0_RX_IRQ       USART0_RX_IRQn
#define UART_0_TX_IRQ       USART0_TX_IRQn
#define UART_0_RX_ISR       USART0_RX_IRQHandler
#define UART_0_TX_ISR       USART0_TX_IRQHandler
#define UART_0_PORT         gpioPortE
#define UART_0_PORT_CLKEN() CMU_ClockEnable(cmuClock_GPIO, true)
#define UART_0_TX_PIN       10
#define UART_0_RX_PIN       11
#define UART_0_ROUTE        USART_ROUTE_LOCATION_LOC0

#define UART_1_DEV          LEUART0
#define UART_1_CLKEN()      CMU_ClockEnable(cmuClock_LEUART0, true)
#define UART_1_RX_IRQ       LEUART0_IRQn
#define UART_1_TX_IRQ       LEUART0_IRQn
#define UART_1_ISR          LEUART0_IRQHandler
#define UART_1_PORT         gpioPortD
#define UART_1_PORT_CLKEN() CMU_ClockEnable(cmuClock_GPIO, true)
#define UART_1_TX_PIN       4
#define UART_1_RX_PIN       5
#define UART_1_ROUTE        USART_ROUTE_LOCATION_LOC0

#define UART_2_DEV          LEUART1
#define UART_2_CLKEN()      CMU_ClockEnable(cmuClock_LEUART1, true)
#define UART_2_RX_IRQ       LEUART1_IRQn
#define UART_2_TX_IRQ       LEUART1_IRQn
#define UART_2_ISR          LEUART1_IRQHandler
#define UART_2_PORT         gpioPortC
#define UART_2_PORT_CLKEN() CMU_ClockEnable(cmuClock_GPIO, true)
#define UART_2_TX_PIN       6
#define UART_2_RX_PIN       7
#define UART_2_ROUTE        USART_ROUTE_LOCATION_LOC0


/**
 * @name ADC configuration
 * @{
 */
#define ADC_NUMOF           (1U)
#define ADC_0_EN            1
#define ADC_MAX_CHANNELS    1

/* ADC 0 configuration */
#define ADC_0_DEV           ADC0
#define ADC_0_CHANNELS      1
#define ADC_0_CLKEN()       CMU_ClockEnable(cmuClock_ADC0, true);
#define ADC_0_CLKDIS()      CMU_ClockEnable(cmuClock_ADC0, false);
/* ADC 0 channel 0 pin config */
#define ADC_0_CH0           1
#define ADC_0_CH0_PIN       1


/**
 * @name PWM configuration
 * @{
 */
#define PWM_NUMOF           (0U)
#define PWM_0_EN            0
#define PWM_1_EN            0

/**
 * @name SPI configuration
 * @{
 */
#define SPI_NUMOF           (2U)
#define SPI_0_EN            1
#define SPI_1_EN            1

/* SPI 0 device configuration */
#define SPI_0_DEV           USART1
#define SPI_0_CLKEN()       CMU_ClockEnable(cmuClock_USART1, true);
#define SPI_0_PORT          gpioPortD
#define SPI_0_PORT_CLKEN()  CMU_ClockEnable(cmuClock_GPIO, true)
#define SPI_0_MOSI_PIN      0
#define SPI_0_MISO_PIN      1
#define SPI_0_CLK_PIN       2
#define SPI_0_CS_PIN        3
#define SPI_0_ROUTE         USART_ROUTE_LOCATION_LOC1

/* SPI 1 device configuration */
#define SPI_1_DEV           USART2
#define SPI_1_CLKEN()       CMU_ClockEnable(cmuClock_USART2, true);
#define SPI_1_PORT          gpioPortC
#define SPI_1_PORT_CLKEN()  CMU_ClockEnable(cmuClock_GPIO, true)
#define SPI_1_MOSI_PIN      2
#define SPI_1_MISO_PIN      3
#define SPI_1_CLK_PIN       4
#define SPI_1_CS_PIN        5
#define SPI_1_ROUTE         USART_ROUTE_LOCATION_LOC0


/**
 * @name I2C configuration
 * @{
 */
#define I2C_NUMOF (1U)

#define I2C_0_EN (1)
#define I2C_0_DEV (I2C0)
#define I2C_0_ROUTE (0)
#define I2C_0_SDA_PORT (gpioPortA)
#define I2C_0_SDA_PIN (0)
#define I2C_0_SCL_PORT (gpioPortA)
#define I2C_0_SCL_PIN (1)


/**
 * @name GPIO configuration
 * @{
 */
// #define GPIO_NUMOF          (27U)
#define GPIO_0_EN           1
#define GPIO_1_EN           1
#define GPIO_2_EN           1
#define GPIO_3_EN           1
#define GPIO_4_EN           1
#define GPIO_5_EN           1
#define GPIO_6_EN           1
#define GPIO_7_EN           1
#define GPIO_8_EN           0
#define GPIO_9_EN           0
#define GPIO_10_EN          0
#define GPIO_11_EN          1
#define GPIO_12_EN          1
#define GPIO_13_EN          1
#define GPIO_14_EN          1
#define GPIO_15_EN          1
#define GPIO_16_EN          1
#define GPIO_17_EN          0
#define GPIO_18_EN          0
#define GPIO_19_EN          0
#define GPIO_20_EN          1
#define GPIO_21_EN          1
#define GPIO_22_EN          1
#define GPIO_23_EN          1
#define GPIO_24_EN          1
#define GPIO_25_EN          0
#define GPIO_26_EN          0
#define GPIO_IRQ_PRIO       1

#define GPIO_IRQ_0          GPIO_0_PIN
#define GPIO_IRQ_1          GPIO_1_PIN
#define GPIO_IRQ_2          GPIO_2_PIN
#define GPIO_IRQ_3          GPIO_3_PIN
#define GPIO_IRQ_4          GPIO_4_PIN
#define GPIO_IRQ_5          GPIO_5_PIN
#define GPIO_IRQ_6          GPIO_6_PIN
#define GPIO_IRQ_7          GPIO_7_PIN
#define GPIO_IRQ_8          GPIO_8_PIN
#define GPIO_IRQ_9          GPIO_9_PIN
#define GPIO_IRQ_10         GPIO_10_PIN
#define GPIO_IRQ_11         GPIO_11_PIN
#define GPIO_IRQ_12         GPIO_12_PIN
#define GPIO_IRQ_13         GPIO_13_PIN
#define GPIO_IRQ_14         GPIO_14_PIN
#define GPIO_IRQ_15         GPIO_15_PIN
#define GPIO_IRQ_16         GPIO_16_PIN
#define GPIO_IRQ_17         GPIO_17_PIN
#define GPIO_IRQ_18         GPIO_18_PIN
#define GPIO_IRQ_19         GPIO_19_PIN
#define GPIO_IRQ_20         GPIO_20_PIN
#define GPIO_IRQ_21         GPIO_21_PIN
#define GPIO_IRQ_22         GPIO_22_PIN
#define GPIO_IRQ_23         GPIO_23_PIN
#define GPIO_IRQ_24         GPIO_24_PIN
#define GPIO_IRQ_25         GPIO_25_PIN
#define GPIO_IRQ_26         GPIO_26_PIN


// radio
#define AT86RF231_CS        GPIO_4
#define GPIO_4_PORT         gpioPortA
#define GPIO_4_PIN          8

#define AT86RF231_INT       GPIO_5
#define GPIO_5_PORT         gpioPortB
#define GPIO_5_PIN          11

#define AT86RF231_SLEEP     GPIO_6
#define GPIO_6_PORT         gpioPortA
#define GPIO_6_PIN          9

#define AT86RF231_RESET     GPIO_7
#define GPIO_7_PORT         gpioPortA
#define GPIO_7_PIN          10


// buttons
#define BUTTON				GPIO_11
#define GPIO_11_PORT		gpioPortC
#define GPIO_11_PIN			1

#define BTN_1               GPIO_12
#define GPIO_12_PORT		gpioPortC
#define GPIO_12_PIN			10

#define BTN_2               GPIO_13
#define GPIO_13_PORT		gpioPortC
#define GPIO_13_PIN			12

#define BTN_X               GPIO_14
#define GPIO_14_PORT        gpioPortC
#define GPIO_14_PIN         11

#define BTN_3               BTN_X
#define BTN_4               BTN_X
#define BTN_5               BTN_X
#define BTN_6               BTN_X
#define BTN_7               BTN_X
#define BTN_8               BTN_X

#define REMOTE_ROUND


// rotary encoder
#define ROTARYENCODER_A     GPIO_15
#define GPIO_15_PORT        gpioPortC
#define GPIO_15_PIN         8

#define ROTARYENCODER_B     GPIO_16
#define GPIO_16_PORT        gpioPortC
#define GPIO_16_PIN         9


// epaper display
#define LCD_CS              GPIO_20
#define GPIO_20_PORT        gpioPortC
#define GPIO_20_PIN         13

#define LCD_RST             GPIO_21
#define GPIO_21_PORT        gpioPortD
#define GPIO_21_PIN         4

#define LCD_BUSY            GPIO_22
#define GPIO_22_PORT        gpioPortC
#define GPIO_22_PIN         14

#define LCD_POWER           GPIO_23
#define GPIO_23_PORT        gpioPortC
#define GPIO_23_PIN         15

#define LCD_DISCHARGE       GPIO_24
#define GPIO_24_PORT        gpioPortF
#define GPIO_24_PIN         4

#define LCD_SPI SPI_0


// flash memory
#define MEM_RST             GPIO_1
#define GPIO_1_PORT         gpioPortA
#define GPIO_1_PIN          3

#define MEM_CS              GPIO_2
#define GPIO_2_PORT         gpioPortA
#define GPIO_2_PIN          2

#define MEM_WP              GPIO_3
#define GPIO_3_PORT         gpioPortA
#define GPIO_3_PIN          4


// hacked memory display
#define DISPLAY_DISP        LCD_RST
#define DISPLAY_CS          LCD_CS

#define DISPLAY_EXTCOM      GPIO_0
#define GPIO_0_PORT         gpioPortD
#define GPIO_0_PIN          6


#endif /* __PERIPH_CONF_H */

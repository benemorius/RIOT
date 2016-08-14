/*
 *    Copyright (c) 2016 Thomas Stilwell <stilwellt@openlabs.co>
 *
 *    Permission is hereby granted, free of charge, to any person
 *    obtaining a copy of this software and associated documentation
 *    files (the "Software"), to deal in the Software without
 *    restriction, including without limitation the rights to use,
 *    copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the
 *    Software is furnished to do so, subject to the following
 *    conditions:
 *
 *    The above copyright notice and this permission notice shall be
 *    included in all copies or substantial portions of the Software.
 *
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *    OTHER DEALINGS IN THE SOFTWARE.
 */


/**
 * @ingroup     board_srf06-cc26xx
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the Sensortag CC2650 board
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 */

#ifndef PERIPH_CONF_H_
#define PERIPH_CONF_H_

#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Clock system configuration
 * @{
 */
#define CLOCK_CORECLOCK     (48000000U)

/**
 * @brief   Timer configuration
 * @{
 */
static const timer_conf_t timer_config[] = {
    {
        .dev  = GPT0,
        .num  = 0
    },
    {
        .dev  = GPT1,
        .num  = 1
    }
};

#define TIMER_0_ISR         isr_timer0_chan0
#define TIMER_1_ISR         isr_timer1_chan0

#define TIMER_NUMOF         (sizeof(timer_config) / sizeof(timer_config[0]))
/** @} */

/**
 * @brief   UART configuration
 *
 * The used CC26x0 CPU only supports a single UART device, so all we need to
 * configure are the RX and TX pins.
 *
 * Optionally we can enable hardware flow control, by setting UART_HW_FLOW_CTRL
 * to 1 and defining pins for UART_CTS_PIN and UART_RTS_PIN.
 * @{
 */
#define UART_NUMOF          (1)
#define UART_RX_PIN         (2)
#define UART_TX_PIN         (3)
/** @} */

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
#define SPI_NUMOF           (1U)
#define SPI_0_EN            1
// static const ssi_conf_t spi_config[] = {
//     {
//         .dev        = SSI0_BASE,
//         .prcmp      = PRCM_PERIPH_SSI0,
//         .bits       = 8,
//         .irqn       = INT_SSI0,
//         .gpio_mosi  = GPIO_PIN_9,
//         .gpio_miso  = GPIO_PIN_8,
//         .gpio_clk   = GPIO_PIN_10,
//         .gpio_cs    = GPIO_PIN_11,
//         .ioid_mosi  = IOID_9,
//         .ioid_miso  = IOID_8,
//         .ioid_clk   = IOID_10,
//         .ioid_cs    = IOID_11,
//     },
// };

/**
 * @name I2C configuration
 * @{
 */
#define I2C_NUMOF       (1U)
#define I2C_0_EN        (1)
static const i2c_conf_t i2c_config[] = {
    {
        .scl_pin   = 26,
        .sda_pin   = 25,
    },
};

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

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H_ */

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
 * @brief       Configuration of CPU peripherals for openlabs-remote
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Bas Stottelaar <basstottelaar@gmail.com>
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#include "cpu.h"
#include "periph_cpu.h"
#include "em_cmu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Internal macro to calculate *_NUMOF based on config.
 */
#define PERIPH_NUMOF(config)    (sizeof(config) / sizeof(config[0]))

/**
 * @name    Clock configuration
 * @{
 */
#ifndef CLOCK_HF
#define CLOCK_HF            cmuSelect_HFXO
#endif
#ifndef CLOCK_CORE_DIV
#define CLOCK_CORE_DIV      cmuClkDiv_1
#endif
#ifndef CLOCK_LFA
#define CLOCK_LFA           cmuSelect_LFXO
#endif
#ifndef CLOCK_LFB
#define CLOCK_LFB           cmuSelect_LFXO
#endif
/** @} */

/**
 * @name    ADC configuration
 * @{
 */
static const adc_conf_t adc_config[] = {
    {
        .dev = ADC0,
        .cmu = cmuClock_ADC0,
    }
};

static const adc_chan_conf_t adc_channel_config[] = {
    {
        .dev = 0,
        .input = adcSingleInputTemp,
        .reference = adcRef1V25,
        .acq_time = adcAcqTime8
    },
    {
        .dev = 0,
        .input = adcSingleInputVDDDiv3,
        .reference = adcRef1V25,
        .acq_time = adcAcqTime8
    }
};

#define ADC_DEV_NUMOF       PERIPH_NUMOF(adc_config)
#define ADC_NUMOF           PERIPH_NUMOF(adc_channel_config)
/** @} */

/**
 * @name    DAC configuration
 * @{
 */
static const dac_conf_t dac_config[] = {
    {
        .dev = DAC0,
        .cmu = cmuClock_DAC0,
    }
};

static const dac_chan_conf_t dac_channel_config[] = {
    {
        .dev = 0,
        .index = 1,
        .ref = dacRefVDD,
    }
};

#define DAC_DEV_NUMOF       PERIPH_NUMOF(dac_config)
#define DAC_NUMOF           PERIPH_NUMOF(dac_channel_config)
/** @} */

/**
 * @name    I2C configuration
 * @{
 */
static const i2c_conf_t i2c_config[] = {
    {
        .dev = I2C0,
        .sda_pin = GPIO_PIN(PA, 0),
        .scl_pin = GPIO_PIN(PA, 1),
        .loc = I2C_ROUTE_LOCATION_LOC0,
        .cmu = cmuClock_I2C0,
        .irq = I2C0_IRQn,
        .speed = I2C_SPEED_NORMAL
    },
    {
        .dev = I2C1,
        .sda_pin = GPIO_PIN(PC, 4),
        .scl_pin = GPIO_PIN(PC, 5),
        .loc = I2C_ROUTE_LOCATION_LOC0,
        .cmu = cmuClock_I2C1,
        .irq = I2C1_IRQn,
        .speed = I2C_SPEED_NORMAL
    }
};

#define I2C_NUMOF           PERIPH_NUMOF(i2c_config)
#define I2C_0_ISR           isr_i2c0
#define I2C_1_ISR           isr_i2c1
/** @} */

/**
 * @name    PWM configuration
 * @{
 */
static const pwm_chan_conf_t pwm_channel_config[] = {
    {
        .index = 2,
        .pin = GPIO_PIN(PE, 2),
        .loc = TIMER_ROUTE_LOCATION_LOC1
    }
};

static const pwm_conf_t pwm_config[] = {
    {
        .dev = TIMER3,
        .cmu = cmuClock_TIMER3,
        .irq = TIMER3_IRQn,
        .channels = 1,
        .channel = pwm_channel_config
    }
};

#define PWM_DEV_NUMOF       PERIPH_NUMOF(pwm_config)
#define PWM_NUMOF           PERIPH_NUMOF(pwm_channel_config)
/** @} */

/**
 * @name    RTC configuration
 * @{
 */
#define RTC_NUMOF           (1U)
/** @} */

/**
 * @name    RTT configuration
 * @{
 */
#define RTT_NUMOF           (1U)

#define RTT_MAX_VALUE       (0xFFFFFF)
#define RTT_FREQUENCY       (1U)
/** @} */

/**
 * @name    SPI configuration
 * @{
 */
static const spi_dev_t spi_config[] = {
    {
        .dev = USART1,
        .mosi_pin = GPIO_PIN(PD, 0),
        .miso_pin = GPIO_PIN(PD, 1),
        .clk_pin = GPIO_PIN(PD, 2),
        .loc = USART_ROUTE_LOCATION_LOC1,
        .cmu = cmuClock_USART1,
        .irq = USART1_RX_IRQn
    },
    {
        .dev = USART2,
        .mosi_pin = GPIO_PIN(PC, 2),
        .miso_pin = GPIO_PIN(PC, 3),
        .clk_pin = GPIO_PIN(PC, 4),
        .loc = USART_ROUTE_LOCATION_LOC0,
        .cmu = cmuClock_USART2,
        .irq = USART2_RX_IRQn
    }
};

#define SPI_NUMOF           PERIPH_NUMOF(spi_config)
/** @} */

/**
 * @name    Timer configuration
 *
 * The implementation can use one 32kHz low-energy timer
 * or two regular timers in cascade mode.
 * @{
 */
#ifndef EFM32_USE_LETIMER
#define EFM32_USE_LETIMER   1
#endif

#if EFM32_USE_LETIMER
static const timer_conf_t timer_config[] = {
    {
        .timer = {
            .dev = LETIMER0,
            .cmu = cmuClock_LETIMER0
        },
        .irq = LETIMER0_IRQn
    }
};
#define TIMER_0_ISR         isr_letimer0

#else
static const timer_conf_t timer_config[] = {
    {
        .prescaler = {
            .dev = TIMER0,
            .cmu = cmuClock_TIMER0
        },
        .timer = {
            .dev = TIMER1,
            .cmu = cmuClock_TIMER1
        },
        .irq = TIMER1_IRQn
    }
};
#define TIMER_0_ISR         isr_timer1

#endif /* EFM32_USE_LETIMER */

#define TIMER_NUMOF         PERIPH_NUMOF(timer_config)
/** @} */

/**
 * @name    UART configuration
 * @{
 */
/* TODO move GPIO_IN_PU from uart.c to here */
static const uart_conf_t uart_config[] = {
//     {
//         .dev = USART0,
//         .rx_pin = GPIO_PIN(PE, 11),
//         .tx_pin = GPIO_PIN(PE, 10),
//         .loc = USART_ROUTE_LOCATION_LOC0,
// #if EFM32_UART_MODES
//         .mode = UART_MODE_8N1,
// #endif
//         .cmu = cmuClock_USART0,
//         .irq = USART0_RX_IRQn
//     },
    {
        .dev = LEUART0,
        .rx_pin = GPIO_PIN(PE, 15),
        .tx_pin = GPIO_PIN(PE, 14),
        .loc = LEUART_ROUTE_LOCATION_LOC2,
#if EFM32_UART_MODES
        .mode = UART_MODE_8N1,
#endif
        .cmu = cmuClock_LEUART0,
        .irq = LEUART0_IRQn
    },


//     {
//         .dev = LEUART0,
//         .rx_pin = GPIO_PIN(PD, 5),
//         .tx_pin = GPIO_PIN(PD, 4),
//         .loc = LEUART_ROUTE_LOCATION_LOC0,
// #if EFM32_UART_MODES
//         .mode = UART_MODE_8N1,
// #endif
//         .cmu = cmuClock_LEUART0,
//         .irq = LEUART0_IRQn
//     },
//     {
//         .dev = LEUART1,
//         .rx_pin = GPIO_PIN(PC, 7),
//         .tx_pin = GPIO_PIN(PC, 6),
//         .loc = LEUART_ROUTE_LOCATION_LOC0,
// #if EFM32_UART_MODES
//         .mode = UART_MODE_8N1,
// #endif
//         .cmu = cmuClock_LEUART1,
//         .irq = LEUART1_IRQn
//     },
};

#define UART_NUMOF          PERIPH_NUMOF(uart_config)
#define UART_0_ISR_RX       isr_leuart0
// #define UART_0_ISR_RX       isr_usart0_rx
// #define UART_1_ISR_RX       isr_leuart0
// #define UART_2_ISR_RX       isr_leuart1
/** @} */


#define AT86RF2XX_PARAMS            { .spi       = SPI_DEV(1),      \
                                      .spi_clk   = SPI_CLK_5MHZ,    \
                                      .cs_pin    = GPIO_PIN(PA, 4), \
                                      .int_pin   = GPIO_PIN(PA, 3), \
                                      .sleep_pin = GPIO_PIN(PA, 2), \
                                      .reset_pin = GPIO_PIN(PA, 5) }
#define AT86RF2XX_SMART_IDLE_LISTENING     (0)


#define LCD_SPI             SPI_DEV(0)
#define LCD_CS              GPIO_PIN(PA, 9)
#define LCD_RST             GPIO_PIN(PD, 4)
#define LCD_BUSY            GPIO_PIN(PA, 8)
#define LCD_POWER           GPIO_PIN(PC, 6)
#define LCD_DISCHARGE       GPIO_PIN(PC, 7)


#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */

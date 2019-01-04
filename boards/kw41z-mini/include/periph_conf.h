/*
 * Copyright (C) 2017 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_kw41z-mini
 * @{
 *
 * @file
 * @name        Peripheral MCU configuration for the kw41z-mini
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#include "periph_cpu.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*! @name CR - OSC Control Register */
#define OSC_CR_SC16P_MASK                        (0x1U)
#define OSC_CR_SC16P_SHIFT                       (0U)
#define OSC_CR_SC16P(x)                          (((uint8_t)(((uint8_t)(x)) << OSC_CR_SC16P_SHIFT)) & OSC_CR_SC16P_MASK)
#define OSC_CR_SC8P_MASK                         (0x2U)
#define OSC_CR_SC8P_SHIFT                        (1U)
#define OSC_CR_SC8P(x)                           (((uint8_t)(((uint8_t)(x)) << OSC_CR_SC8P_SHIFT)) & OSC_CR_SC8P_MASK)
#define OSC_CR_SC4P_MASK                         (0x4U)
#define OSC_CR_SC4P_SHIFT                        (2U)
#define OSC_CR_SC4P(x)                           (((uint8_t)(((uint8_t)(x)) << OSC_CR_SC4P_SHIFT)) & OSC_CR_SC4P_MASK)
#define OSC_CR_SC2P_MASK                         (0x8U)
#define OSC_CR_SC2P_SHIFT                        (3U)
#define OSC_CR_SC2P(x)                           (((uint8_t)(((uint8_t)(x)) << OSC_CR_SC2P_SHIFT)) & OSC_CR_SC2P_MASK)
#define OSC_CR_EREFSTEN_MASK                     (0x20U)
#define OSC_CR_EREFSTEN_SHIFT                    (5U)
#define OSC_CR_EREFSTEN(x)                       (((uint8_t)(((uint8_t)(x)) << OSC_CR_EREFSTEN_SHIFT)) & OSC_CR_EREFSTEN_MASK)
#define OSC_CR_ERCLKEN_MASK                      (0x80U)
#define OSC_CR_ERCLKEN_SHIFT                     (7U)
#define OSC_CR_ERCLKEN(x)                        (((uint8_t)(((uint8_t)(x)) << OSC_CR_ERCLKEN_SHIFT)) & OSC_CR_ERCLKEN_MASK)
#define RTC_CR_SC16P_MASK                        (0x400U)
#define RTC_CR_SC16P_SHIFT                       (10U)
#define RTC_CR_SC16P(x)                          (((uint32_t)(((uint32_t)(x)) << RTC_CR_SC16P_SHIFT)) & RTC_CR_SC16P_MASK)
#define RTC_CR_SC8P_MASK                         (0x800U)
#define RTC_CR_SC8P_SHIFT                        (11U)
#define RTC_CR_SC8P(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_CR_SC8P_SHIFT)) & RTC_CR_SC8P_MASK)
#define RTC_CR_SC4P_MASK                         (0x1000U)
#define RTC_CR_SC4P_SHIFT                        (12U)
#define RTC_CR_SC4P(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_CR_SC4P_SHIFT)) & RTC_CR_SC4P_MASK)
#define RTC_CR_SC2P_MASK                         (0x2000U)
#define RTC_CR_SC2P_SHIFT                        (13U)
#define RTC_CR_SC2P(x)                           (((uint32_t)(((uint32_t)(x)) << RTC_CR_SC2P_SHIFT)) & RTC_CR_SC2P_MASK)

/**
 * @name Clock system configuration
 * @{
 */
static const clock_config_t clock_config = {
    /*
     * This configuration results in the system running with the internal clock
     * with the following clock frequencies:
     * Core:  48 MHz
     * Bus:   24 MHz
     * Flash: 24 MHz
     */
    .clkdiv1            = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV4(1),
    /* unsure if this RTC load cap configuration (0) is correct, but it matches the
     * settings used by the example code in the NXP provided SDK */
    // .rtc_clc            = 0,
    .rtc_clc            = RTC_CR_SC16P_MASK | RTC_CR_SC8P_MASK | RTC_CR_SC4P_MASK | RTC_CR_SC2P_MASK,
    /* Use the 32 kHz oscillator as ERCLK32K. Note that the values here have a
     * different mapping for the KW41Z than the values used in the Kinetis K series */
    .osc32ksel          = SIM_SOPT1_OSC32KSEL(0),
    .clock_flags =
        KINETIS_CLOCK_OSC0_EN | /* Enable RSIM oscillator */
        KINETIS_CLOCK_RTCOSC_EN |
        KINETIS_CLOCK_USE_FAST_IRC |
        KINETIS_CLOCK_MCGIRCLK_EN | /* Used for LPUART clocking */
        KINETIS_CLOCK_MCGIRCLK_STOP_EN |
        0,
    /* Select BLPE to use the 32 MHz crystal clock signal without the FLL */
    // .default_mode = KINETIS_MCG_MODE_BLPE,
    /* Using FEI mode by default, the external crystal settings below are only
     * used if mode is changed to an external mode (PEE, FBE, or FEE) */
    .default_mode       = KINETIS_MCG_MODE_FEI,
    /* The crystal connected to RSIM OSC is 32 MHz */
    .erc_range          = KINETIS_MCG_ERC_RANGE_VERY_HIGH,
    // .osc_clc            = 0,
    .osc_clc            = OSC_CR_SC16P_MASK | OSC_CR_SC8P_MASK | OSC_CR_SC4P_MASK | OSC_CR_SC2P_MASK,
    .oscsel             = MCG_C7_OSCSEL(0), /* Use RSIM for external clock */
    .fcrdiv             = MCG_SC_FCRDIV(0), /* Fast IRC divide by 1 => 4 MHz */
    .fll_frdiv          = MCG_C1_FRDIV(0b101), /* Divide by 1024 */
    .fll_factor_fei     = KINETIS_MCG_FLL_FACTOR_1464, /* FEI FLL freq = 48 MHz */
    .fll_factor_fee     = KINETIS_MCG_FLL_FACTOR_1280, /* FEE FLL freq = 40 MHz */
};
/* Radio xtal frequency, either 32 MHz or 26 MHz */
#define CLOCK_RADIOXTAL              (32000000ul)
/* CPU core clock, the MCG clock output frequency */
#define CLOCK_CORECLOCK              (48000000ul)
#define CLOCK_BUSCLOCK               (CLOCK_CORECLOCK / 2)
#define CLOCK_MCGIRCLK               (4000000ul)
/** @} */

/**
 * @name Timer configuration
 * @{
 */
#define PIT_NUMOF               (1U)
#define PIT_CONFIG {                 \
        {                            \
            .prescaler_ch = 0,       \
            .count_ch = 1,           \
        },                           \
    }
#define LPTMR_NUMOF             (1U)
#define LPTMR_CONFIG { \
        { \
            .dev = LPTMR0, \
            .irqn = LPTMR0_IRQn, \
            .src = 2, \
            .base_freq = 32768u, \
        } \
    }
#define TIMER_NUMOF             ((PIT_NUMOF) + (LPTMR_NUMOF))
#define PIT_BASECLOCK           (CLOCK_BUSCLOCK)
#define LPTMR_ISR_0             isr_lptmr0
/** @} */

/**
 * @name UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .dev    = LPUART0,
        .freq   = CLOCK_MCGIRCLK,
        .pin_rx = GPIO_PIN(PORT_C,  6),
        .pin_tx = GPIO_PIN(PORT_C,  7),
        .pcr_rx = PORT_PCR_MUX(4),
        .pcr_tx = PORT_PCR_MUX(4),
        .irqn   = LPUART0_IRQn,
        .scgc_addr = &SIM->SCGC5,
        .scgc_bit = SIM_SCGC5_LPUART0_SHIFT,
        .mode   = UART_MODE_8N1,
        .type   = KINETIS_LPUART,
    },
};
#define UART_NUMOF          (sizeof(uart_config) / sizeof(uart_config[0]))
#define LPUART_0_ISR        isr_lpuart0
/* Use MCGIRCLK (internal reference 4 MHz clock) */
#define LPUART_0_SRC        3
/** @} */

/**
 * @name ADC configuration
 * @{
 */
static const adc_conf_t adc_config[] = {
    [ 0] = { .dev = ADC0, .pin = GPIO_UNDEF, .chan = 26, .avg = ADC_AVG_NONE },       /* internal: temperature sensor */
    [ 1] = { .dev = ADC0, .pin = GPIO_UNDEF, .chan = 27, .avg = ADC_AVG_MAX },       /* internal: band gap */
    [ 2] = { .dev = ADC0, .pin = GPIO_UNDEF, .chan = 29, .avg = ADC_AVG_MAX },       /* internal: V_REFH */
    [ 3] = { .dev = ADC0, .pin = GPIO_UNDEF, .chan = 30, .avg = ADC_AVG_MAX },       /* internal: V_REFL */
    [ 4] = { .dev = ADC0, .pin = GPIO_UNDEF, .chan = 23, .avg = ADC_AVG_MAX },       /* internal: DCDC divided battery level */
    [ 5] = { .dev = ADC0, .pin = GPIO_UNDEF, .chan = 0 | ADC_SC1_DIFF_MASK, .avg = ADC_AVG_MAX }, /* ADC0_DP/ADC0_DM differential */

    [ 6] = { .dev = ADC0, .pin = GPIO_PIN(PORT_B,  1),  .chan = 1, .avg = ADC_AVG_MAX }, /* ADC0_SE1 A0 */
    [ 7] = { .dev = ADC0, .pin = GPIO_PIN(PORT_B,  2),  .chan = 3, .avg = ADC_AVG_MAX }, /* ADC0_SE2 A1 */
    [ 8] = { .dev = ADC0, .pin = GPIO_PIN(PORT_B,  3),  .chan = 2, .avg = ADC_AVG_MAX }, /* ADC0_SE3 A2 */
    [ 9] = { .dev = ADC0, .pin = GPIO_PIN(PORT_B,  18), .chan = 4, .avg = ADC_AVG_MAX }, /* ADC0_SE4 A3 */
    [10] = { .dev = ADC0, .pin = GPIO_PIN(PORT_A,  19), .chan = 5, .avg = ADC_AVG_MAX }, /* ADC0_SE5 5 */
};

#define ADC_NUMOF           (sizeof(adc_config) / sizeof(adc_config[0]))
#define ADC_REF_SETTING     1
/** @} */

/**
 * @name   SPI configuration
 *
 * Clock configuration values based on the configured 16Mhz module clock.
 *
 * Auto-generated by:
 * cpu/kinetis/dist/calc_spi_scalers/calc_spi_scalers.c
 *
* @{
*/
static const uint32_t spi_clk_config[] = {
    (
        SPI_CTAR_PBR(2) | SPI_CTAR_BR(5) |          /* -> 100000Hz */
        SPI_CTAR_PCSSCK(2) | SPI_CTAR_CSSCK(4) |
        SPI_CTAR_PASC(2) | SPI_CTAR_ASC(4) |
        SPI_CTAR_PDT(2) | SPI_CTAR_DT(4)
    ),
    (
        SPI_CTAR_PBR(2) | SPI_CTAR_BR(3) |          /* -> 400000Hz */
        SPI_CTAR_PCSSCK(2) | SPI_CTAR_CSSCK(2) |
        SPI_CTAR_PASC(2) | SPI_CTAR_ASC(2) |
        SPI_CTAR_PDT(2) | SPI_CTAR_DT(2)
    ),
    (
        SPI_CTAR_PBR(0) | SPI_CTAR_BR(3) |          /* -> 1000000Hz */
        SPI_CTAR_PCSSCK(0) | SPI_CTAR_CSSCK(3) |
        SPI_CTAR_PASC(0) | SPI_CTAR_ASC(3) |
        SPI_CTAR_PDT(0) | SPI_CTAR_DT(3)
    ),
    (
        SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) |          /* -> 4000000Hz */
        SPI_CTAR_PCSSCK(0) | SPI_CTAR_CSSCK(1) |
        SPI_CTAR_PASC(0) | SPI_CTAR_ASC(1) |
        SPI_CTAR_PDT(0) | SPI_CTAR_DT(1)
    ),
    (
        SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) |          /* -> 4000000Hz */
        SPI_CTAR_PCSSCK(0) | SPI_CTAR_CSSCK(0) |
        SPI_CTAR_PASC(0) | SPI_CTAR_ASC(0) |
        SPI_CTAR_PDT(0) | SPI_CTAR_DT(0)
    )
};

static const spi_conf_t spi_config[] = {
    {
        .dev      = SPI0,
        .pin_miso = GPIO_PIN(PORT_C, 18),
        .pin_mosi = GPIO_PIN(PORT_C, 17),
        .pin_clk  = GPIO_PIN(PORT_C, 16),
        .pin_cs   = {
            GPIO_PIN(PORT_C, 19),
            GPIO_UNDEF,
            GPIO_UNDEF,
            GPIO_UNDEF,
            GPIO_UNDEF
        },
        .pcr      = GPIO_AF_2,
        .simmask  = SIM_SCGC6_SPI0_MASK
    },
};

#define SPI_NUMOF           (sizeof(spi_config) / sizeof(spi_config[0]))
/** @} */


/**
* @name I2C configuration
* @{
*/
/* This CPU has I2C0 clocked by the bus clock and I2C1 clocked by the system
 * clock. This causes trouble with the current implementation in kinetis
 * which only supports one set of frequency dividers at a time */
/* The current configuration sets the dividers so that the I2C0 bus will run at
 * half the requested speed, to avoid exceeding the requested speed on I2C1 with
 * the same configuration */
#define I2C_NUMOF                    (1U)
#define I2C_0_EN                     0
/* Disabled while waiting for a rewritten i2c driver which supports different
 * clock sources for each i2c module */
#define I2C_1_EN                     1
/* Low (10 kHz): MUL = 2, SCL divider = 1792, total: 3584 */
#define KINETIS_I2C_F_ICR_LOW        (0x3A)
#define KINETIS_I2C_F_MULT_LOW       (1)
/* Normal (100 kHz): MUL = 1, SCL divider = 320, total: 320 */
#define KINETIS_I2C_F_ICR_NORMAL     (0x25)
#define KINETIS_I2C_F_MULT_NORMAL    (0)
/* Fast (400 kHz): MUL = 1, SCL divider = 80, total: 80 */
#define KINETIS_I2C_F_ICR_FAST       (0x14)
#define KINETIS_I2C_F_MULT_FAST      (0)
/* Fast plus (1000 kHz): MUL = 1, SCL divider = 32, total: 32 */
#define KINETIS_I2C_F_ICR_FAST_PLUS  (0x09)
#define KINETIS_I2C_F_MULT_FAST_PLUS (0)

/* I2C 1 device configuration */
#define I2C_1_DEV                    I2C1
#define I2C_1_CLKEN()                (bit_set32(&SIM->SCGC4, SIM_SCGC4_I2C1_SHIFT))
#define I2C_1_CLKDIS()               (bit_clear32(&SIM->SCGC4, SIM_SCGC4_I2C1_SHIFT))
#define I2C_1_IRQ                    I2C1_IRQn
#define I2C_1_IRQ_HANDLER            isr_i2c1
/* I2C 1 pin configuration */
#define I2C_1_PORT                   PORTC
#define I2C_1_PORT_CLKEN()           (bit_set32(&SIM->SCGC5, SIM_SCGC5_PORTC_SHIFT))
#define I2C_1_PIN_AF                 3
#define I2C_1_SDA_PIN                3
#define I2C_1_SCL_PIN                2
#define I2C_1_PORT_CFG               (PORT_PCR_MUX(I2C_1_PIN_AF))
/** @} */

/**
* @name RTT and RTC configuration
* @{
*/
#define RTT_NUMOF                    (1U)
#define RTC_NUMOF                    (1U)
#define RTT_DEV                      RTC
#define RTT_IRQ                      RTC_IRQn
#define RTT_IRQ_PRIO                 10
#define RTT_UNLOCK()                 (bit_set32(&SIM->SCGC6, SIM_SCGC6_RTC_SHIFT))
#define RTT_ISR                      isr_rtc
#define RTT_FREQUENCY                (1)
#define RTT_MAX_VALUE                (0xffffffff)
/** @} */

/**
 * @name Random Number Generator configuration
 * @{
 */
#define KINETIS_TRNG                TRNG
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */

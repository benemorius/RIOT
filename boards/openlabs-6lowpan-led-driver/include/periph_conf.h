/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_openlabs-6lowpan-led-driver
 * @{
 *
 * @file
 * @name        Peripheral MCU configuration for openlabs-6lowpan-led-driver
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
    .rtc_clc            = RTC_CR_SC16P_MASK | RTC_CR_SC4P_MASK | RTC_CR_SC2P_MASK,
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
    /* Using FEI mode by default, the external crystal settings below are only
     * used if mode is changed to an external mode (PEE, FBE, or FEE) */
    .default_mode       = KINETIS_MCG_MODE_FEI,
    /* The crystal connected to RSIM OSC is 32 MHz */
    .erc_range          = KINETIS_MCG_ERC_RANGE_VERY_HIGH,
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
#define CLOCK_MCGFLLCLK              (CLOCK_CORECLOCK)
#define CLOCK_OSCERCLK               (CLOCK_RADIOXTAL)
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
            .base_freq = 32768u, \
            .src = 2, \
            .irqn = LPTMR0_IRQn, \
        }, \
    }
#define TIMER_NUMOF             ((PIT_NUMOF) + (LPTMR_NUMOF))
#define PIT_BASECLOCK           (CLOCK_BUSCLOCK)
#define LPTMR_ISR_0             isr_lptmr0
/** @} */

/**
 * @name UART configuration
 * @{
 */
#ifndef LPUART_0_SRC
#define LPUART_0_SRC                1
#endif

#if LPUART_0_SRC ==                 3
/* Use MCGIRCLK (4 MHz internal reference - not available in KINETIS_PM_LLS) */
#define LPUART_0_CLOCK                  CLOCK_MCGIRCLK
#define UART_CLOCK_PM_BLOCKER           KINETIS_PM_LLS
#define UART_MAX_UNCLOCKED_BAUDRATE     19200ul
#elif LPUART_0_SRC ==               2
#define LPUART_0_CLOCK                  CLOCK_OSCERCLK
#elif LPUART_0_SRC ==               1
/* Use CLOCK_MCGFLLCLK (48 MHz FLL output - not available in KINETIS_PM_STOP) */
#define LPUART_0_CLOCK                  CLOCK_MCGFLLCLK
#define UART_CLOCK_PM_BLOCKER           KINETIS_PM_STOP
#define UART_MAX_UNCLOCKED_BAUDRATE     57600ul
#endif

static const uart_conf_t uart_config[] = {
    {
        .dev    = LPUART0,
        .freq   = LPUART_0_CLOCK,
        .pin_rx = GPIO_PIN(PORT_C,  17),
        .pin_tx = GPIO_PIN(PORT_C,  18),
        .pcr_rx = PORT_PCR_MUX(4) | GPIO_IN_PU,
        .pcr_tx = PORT_PCR_MUX(4),
        .irqn   = LPUART0_IRQn,
        .scgc_addr = &SIM->SCGC5,
        .scgc_bit = SIM_SCGC5_LPUART0_SHIFT,
        .mode   = UART_MODE_8N1,
        .type   = KINETIS_LPUART,
#ifdef MODULE_PERIPH_LLWU /* remove ifdef after #7897 is merged */
        .llwu_rx = LLWU_WAKEUP_PIN_PTC17,
#endif
    },
};
#define UART_NUMOF          (sizeof(uart_config) / sizeof(uart_config[0]))
#define LPUART_0_ISR        isr_lpuart0
/** @} */

/**
 * @name ADC configuration
 * @{
 */
static const adc_conf_t adc_config[] = {
    /* internal: temperature sensor */
    [0] = { .dev = ADC0, .pin = GPIO_UNDEF, .chan = 26, .avg = ADC_AVG_NONE },
    /* internal: band gap */
    [1] = { .dev = ADC0, .pin = GPIO_UNDEF, .chan = 27, .avg = ADC_AVG_MAX },
    /* internal: DCDC divided battery level */
    [2] = { .dev = ADC0, .pin = GPIO_UNDEF, .chan = 23, .avg = ADC_AVG_MAX },
    /* supply voltage */
    [3] = { .dev = ADC0, .pin = GPIO_PIN(PORT_B, 3), .chan = 2, .avg = ADC_AVG_MAX },
    /* white led current */
    [4] = { .dev = ADC0, .pin = GPIO_PIN(PORT_A, 19), .chan = 5, .avg = ADC_AVG_MAX },
    /* not connected */
    [5] = { .dev = ADC0, .pin = GPIO_PIN(PORT_B, 18), .chan = 4, .avg = ADC_AVG_MAX },
};

#define ADC_NUMOF           (sizeof(adc_config) / sizeof(adc_config[0]))
/*
 * KW41Z ADC reference settings:
 * 0: VREFH external pin or VREF_OUT 1.2 V signal (if VREF module is enabled)
 * 1: VDDA (analog supply input voltage)
 * 2-3: reserved
 */
#define ADC_REF_SETTING     0
#if ADC_REF_SETTING
#define ADC_REF_VOLTAGE     (3.3f)
#else
#define ADC_REF_VOLTAGE     (1.2f)
#endif

#define ADC_TEMPERATURE_CHANNEL     (4)
/** @} */

/**
 * @name   DAC configuration
 * @{
 */
static const dac_conf_t dac_config[] = {
    {
        .dev       = DAC0,
        .scgc_addr = &SIM->SCGC6,
        .scgc_bit  = SIM_SCGC6_DAC0_SHIFT,
    },
};

#define DAC_NUMOF           (sizeof(dac_config) / sizeof(dac_config[0]))
/** @} */

/**
 * @name   PWM mode configuration
 * @{
 */
#define HAVE_PWM_MODE_T
typedef enum {
    PWM_LEFT   = (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK),  /**< left aligned */
    PWM_RIGHT  = (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK),  /**< right aligned */
    PWM_CENTER = (TPM_CnSC_MSB_MASK)                        /**< center aligned */
} pwm_mode_t;

/**
 * @brief   PWM configuration structure
 */
#define PWM_CHAN_MAX        (6U)
typedef struct {
    TPM_Type* tpm;          /**< used TPM */
    struct {
        gpio_t pin;         /**< GPIO pin used, set to GPIO_UNDEF */
        uint8_t af;         /**< alternate function mapping */
        uint8_t ftm_chan;   /**< the actual FTM channel used */
    } chan[PWM_CHAN_MAX];   /**< logical channel configuration */
    uint8_t chan_numof;     /**< number of actually configured channels */
    uint8_t tpm_num;        /**< FTM number used */
} pwm_conf_t;

/**
 * @name    PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
{
    .tpm        = TPM0,
    .chan       = {
        /* green */
        { .pin = GPIO_PIN(PORT_B, 1), .af = 5, .ftm_chan = 2 },
        /* blue */
        { .pin = GPIO_PIN(PORT_B, 0), .af = 5, .ftm_chan = 1 },
        /* warmer white */
        { .pin = GPIO_PIN(PORT_A, 16), .af = 5, .ftm_chan = 0 },
    },
    .chan_numof = 3,
    .tpm_num    = 0,
},
{
    .tpm        = TPM1,
    .chan       = {
        /* red */
        { .pin = GPIO_PIN(PORT_B, 2), .af = 5, .ftm_chan = 0 },
    },
    .chan_numof = 1,
    .tpm_num    = 1,
    },
{
    .tpm        = TPM2,
    .chan       = {
        /* white */
        { .pin = GPIO_PIN(PORT_A, 18), .af = 5, .ftm_chan = 0 },
    },
    .chan_numof = 1,
    .tpm_num    = 2,
    },
};

#define PWM_NUMOF           (sizeof(pwm_config) / sizeof(pwm_config[0]))
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

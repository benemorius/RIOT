/*
 * Copyright (C) 2017 Eistec AB
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2015 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_kw41z
 * @{
 *
 * @file
 * @brief       Interrupt vector definition for MKW41Zxxxxxxx MCUs
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 *
 * @}
 */

#include "vectors_kinetis.h"

/* interrupt vector table */
ISR_VECTORS const isr_func_t interrupt_vector[] = {
    /* Stack pointer */
    (isr_func_t)(&_estack), /* pointer to the top of the empty stack */
    /* Cortex-M4 handlers */
    reset_handler_default,  /* entry point of the program */
    nmi_default,            /* non maskable interrupt handler */
    hard_fault_default,     /* hard fault exception */
    dummy_handler,          /* (not supported) memory manage exception */
    dummy_handler,          /* (not supported) bus fault exception */
    dummy_handler,          /* (not supported) usage fault exception */
    dummy_handler,          /* Reserved */
    dummy_handler,          /* Reserved */
    dummy_handler,          /* Reserved */
    dummy_handler,          /* Reserved */
    isr_svc,                /* system call interrupt, in RIOT used for
                             * switching into thread context on boot */
    dummy_handler,          /* debug monitor exception */
    dummy_handler,          /* Reserved */
    isr_pendsv,             /* pendSV interrupt, in RIOT the actual
                             * context switching is happening here */
    isr_systick,            /* SysTick interrupt, not used in RIOT */
    /* KW41Z specific peripheral handlers */
    isr_dma0,              /* DMA channel 0 transfer complete or error */
    isr_dma1,              /* DMA channel 1 transfer complete or error */
    isr_dma2,              /* DMA channel 2 transfer complete or error */
    isr_dma3,              /* DMA channel 3 transfer complete or error */
    dummy_handler,         /* Reserved for future MCM */
    isr_ftfa,              /* FTFA command complete and read collision */
    isr_pmc,               /* PMC: Low-voltage detect, low-voltage warning,
                            * DCDC: PSWITCH interrupt */
    isr_llwu,              /* Low leakage wakeup */
    isr_i2c0,              /* I2C bus 0 interrupt */
    isr_i2c1,              /* I2C bus 1 interrupt */
    isr_spi0,              /* SPI bus 0 interrupt */
    isr_tsi,               /* TSI interrupt */
    isr_lpuart0,           /* LPUART0 interrupt */
    isr_trng0an,           /* TRNG0AN interrupt */
    isr_cmt,               /* Carrier modulator transmitter status or error */
    isr_adc0,              /* Analog-to-digital converter 0 */
    isr_cmp0,              /* Comparator 0 */
    isr_tpm0,              /* Timer/PWM module 0 */
    isr_tpm1,              /* Timer/PWM module 1 */
    isr_tpm2,              /* Timer/PWM module 2 */
    isr_rtc,               /* Real time clock alarm interrupt */
    isr_rtc_seconds,       /* Real time clock seconds seconds interrupt */
    isr_pit,               /* Periodic interrupt timer single vector for all channels */
    isr_ltc,               /* LTC(AESA) */
    isr_radio_int0,        /* Radio transceiver INT0 */
    isr_dac0,              /* Digital-to-analog converter 0 */
    isr_radio_int1,        /* Radio transceiver INT1 */
    isr_mcg,               /* Multipurpose clock generator */
    isr_lptmr0,            /* Low power timer interrupt */
    isr_spi1,              /* SPI bus 1 interrupt */
    isr_porta,             /* Port A pin detect interrupt */
    isr_portbc,            /* Port B and C pin detect interrupt */
    dummy_handler,         /* reserved 48 */
    dummy_handler,         /* reserved 49 */
    dummy_handler,         /* reserved 50 */
    dummy_handler,         /* reserved 51 */
    dummy_handler,         /* reserved 52 */
    dummy_handler,         /* reserved 53 */
    dummy_handler,         /* reserved 54 */
    dummy_handler,         /* reserved 55 */
    dummy_handler,         /* reserved 56 */
    dummy_handler,         /* reserved 57 */
    dummy_handler,         /* reserved 58 */
    dummy_handler,         /* reserved 59 */
    dummy_handler,         /* reserved 60 */
    dummy_handler,         /* reserved 61 */
    dummy_handler,         /* reserved 62 */
    dummy_handler,         /* reserved 63 */
    dummy_handler,         /* reserved 64 */
    dummy_handler,         /* reserved 65 */
    dummy_handler,         /* reserved 66 */
    dummy_handler,         /* reserved 67 */
    dummy_handler,         /* reserved 68 */
    dummy_handler,         /* reserved 69 */
    dummy_handler,         /* reserved 70 */
    dummy_handler,         /* reserved 71 */
    dummy_handler,         /* reserved 72 */
    dummy_handler,         /* reserved 73 */
    dummy_handler,         /* reserved 74 */
    dummy_handler,         /* reserved 75 */
    dummy_handler,         /* reserved 76 */
    dummy_handler,         /* reserved 77 */
    dummy_handler,         /* reserved 78 */
    dummy_handler,         /* reserved 79 */
    dummy_handler,         /* reserved 80 */
    dummy_handler,         /* reserved 81 */
    dummy_handler,         /* reserved 82 */
    dummy_handler,         /* reserved 83 */
    dummy_handler,         /* reserved 84 */
    dummy_handler,         /* reserved 85 */
    dummy_handler,         /* reserved 86 */
    dummy_handler,         /* reserved 87 */
    dummy_handler,         /* reserved 88 */
    dummy_handler,         /* reserved 89 */
    dummy_handler,         /* reserved 90 */
    dummy_handler,         /* reserved 91 */
    dummy_handler,         /* reserved 92 */
    dummy_handler,         /* reserved 93 */
    dummy_handler,         /* reserved 94 */
    dummy_handler,         /* reserved 95 */
    dummy_handler,         /* reserved 96 */
    dummy_handler,         /* reserved 97 */
    dummy_handler,         /* reserved 98 */
    dummy_handler,         /* reserved 99 */
    dummy_handler,         /* reserved 100 */
    dummy_handler,         /* reserved 101 */
    dummy_handler,         /* reserved 102 */
    dummy_handler,         /* reserved 103 */
    dummy_handler,         /* reserved 104 */
    dummy_handler,         /* reserved 105 */
    dummy_handler,         /* reserved 106 */
    dummy_handler,         /* reserved 107 */
    dummy_handler,         /* reserved 108 */
    dummy_handler,         /* reserved 109 */
    dummy_handler,         /* reserved 110 */
    dummy_handler,         /* reserved 111 */
    dummy_handler,         /* reserved 112 */
    dummy_handler,         /* reserved 113 */
    dummy_handler,         /* reserved 114 */
    dummy_handler,         /* reserved 115 */
    dummy_handler,         /* reserved 116 */
    dummy_handler,         /* reserved 117 */
    dummy_handler,         /* reserved 118 */
    dummy_handler,         /* reserved 119 */
    dummy_handler,         /* reserved 120 */
    dummy_handler,         /* reserved 121 */
    dummy_handler,         /* reserved 122 */
    dummy_handler,         /* reserved 123 */
    dummy_handler,         /* reserved 124 */
    dummy_handler,         /* reserved 125 */
    dummy_handler,         /* reserved 126 */
    dummy_handler,         /* reserved 127 */
    dummy_handler,         /* reserved 128 */
    dummy_handler,         /* reserved 129 */
    dummy_handler,         /* reserved 130 */
    dummy_handler,         /* reserved 131 */
    dummy_handler,         /* reserved 132 */
    dummy_handler,         /* reserved 133 */
    dummy_handler,         /* reserved 134 */
    dummy_handler,         /* reserved 135 */
    dummy_handler,         /* reserved 136 */
    dummy_handler,         /* reserved 137 */
    dummy_handler,         /* reserved 138 */
    dummy_handler,         /* reserved 139 */
    dummy_handler,         /* reserved 140 */
    dummy_handler,         /* reserved 141 */
    dummy_handler,         /* reserved 142 */
    dummy_handler,         /* reserved 143 */
    dummy_handler,         /* reserved 144 */
    dummy_handler,         /* reserved 145 */
    dummy_handler,         /* reserved 146 */
    dummy_handler,         /* reserved 147 */
    dummy_handler,         /* reserved 148 */
    dummy_handler,         /* reserved 149 */
    dummy_handler,         /* reserved 150 */
    dummy_handler,         /* reserved 151 */
    dummy_handler,         /* reserved 152 */
    dummy_handler,         /* reserved 153 */
    dummy_handler,         /* reserved 154 */
    dummy_handler,         /* reserved 155 */
    dummy_handler,         /* reserved 156 */
    dummy_handler,         /* reserved 157 */
    dummy_handler,         /* reserved 158 */
    dummy_handler,         /* reserved 159 */
    dummy_handler,         /* reserved 160 */
    dummy_handler,         /* reserved 161 */
    dummy_handler,         /* reserved 162 */
    dummy_handler,         /* reserved 163 */
    dummy_handler,         /* reserved 164 */
    dummy_handler,         /* reserved 165 */
    dummy_handler,         /* reserved 166 */
    dummy_handler,         /* reserved 167 */
    dummy_handler,         /* reserved 168 */
    dummy_handler,         /* reserved 169 */
    dummy_handler,         /* reserved 170 */
    dummy_handler,         /* reserved 171 */
    dummy_handler,         /* reserved 172 */
    dummy_handler,         /* reserved 173 */
    dummy_handler,         /* reserved 174 */
    dummy_handler,         /* reserved 175 */
    dummy_handler,         /* reserved 176 */
    dummy_handler,         /* reserved 177 */
    dummy_handler,         /* reserved 178 */
    dummy_handler,         /* reserved 179 */
    dummy_handler,         /* reserved 180 */
    dummy_handler,         /* reserved 181 */
    dummy_handler,         /* reserved 182 */
    dummy_handler,         /* reserved 183 */
    dummy_handler,         /* reserved 184 */
    dummy_handler,         /* reserved 185 */
    dummy_handler,         /* reserved 186 */
    dummy_handler,         /* reserved 187 */
    dummy_handler,         /* reserved 188 */
    dummy_handler,         /* reserved 189 */
    dummy_handler,         /* reserved 190 */
    dummy_handler,         /* reserved 191 */
    dummy_handler,         /* reserved 192 */
    dummy_handler,         /* reserved 193 */
    dummy_handler,         /* reserved 194 */
    dummy_handler,         /* reserved 195 */
    dummy_handler,         /* reserved 196 */
    dummy_handler,         /* reserved 197 */
    dummy_handler,         /* reserved 198 */
    dummy_handler,         /* reserved 199 */
    dummy_handler,         /* reserved 200 */
    dummy_handler,         /* reserved 201 */
    dummy_handler,         /* reserved 202 */
    dummy_handler,         /* reserved 203 */
    dummy_handler,         /* reserved 204 */
    dummy_handler,         /* reserved 205 */
    dummy_handler,         /* reserved 206 */
    dummy_handler,         /* reserved 207 */
    dummy_handler,         /* reserved 208 */
    dummy_handler,         /* reserved 209 */
    dummy_handler,         /* reserved 210 */
    dummy_handler,         /* reserved 211 */
    dummy_handler,         /* reserved 212 */
    dummy_handler,         /* reserved 213 */
    dummy_handler,         /* reserved 214 */
    dummy_handler,         /* reserved 215 */
    dummy_handler,         /* reserved 216 */
    dummy_handler,         /* reserved 217 */
    dummy_handler,         /* reserved 218 */
    dummy_handler,         /* reserved 219 */
    dummy_handler,         /* reserved 220 */
    dummy_handler,         /* reserved 221 */
    dummy_handler,         /* reserved 222 */
    dummy_handler,         /* reserved 223 */
    dummy_handler,         /* reserved 224 */
    dummy_handler,         /* reserved 225 */
    dummy_handler,         /* reserved 226 */
    dummy_handler,         /* reserved 227 */
    dummy_handler,         /* reserved 228 */
    dummy_handler,         /* reserved 229 */
    dummy_handler,         /* reserved 230 */
    dummy_handler,         /* reserved 231 */
    dummy_handler,         /* reserved 232 */
    dummy_handler,         /* reserved 233 */
    dummy_handler,         /* reserved 234 */
    dummy_handler,         /* reserved 235 */
    dummy_handler,         /* reserved 236 */
    dummy_handler,         /* reserved 237 */
    dummy_handler,         /* reserved 238 */
    dummy_handler,         /* reserved 239 */
    dummy_handler,         /* reserved 240 */
    dummy_handler,         /* reserved 241 */
    dummy_handler,         /* reserved 242 */
    dummy_handler,         /* reserved 243 */
    dummy_handler,         /* reserved 244 */
    dummy_handler,         /* reserved 245 */
    dummy_handler,         /* reserved 246 */
    dummy_handler,         /* reserved 247 */
    dummy_handler,         /* reserved 248 */
    dummy_handler,         /* reserved 249 */
    dummy_handler,         /* reserved 250 */
    dummy_handler,         /* reserved 251 */
    dummy_handler,         /* reserved 252 */
    dummy_handler,         /* reserved 253 */
    dummy_handler,         /* reserved 254 */
    dummy_handler,         /* reserved 255 */
};

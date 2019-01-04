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
 * @brief       Board specific initialization for the kw41z-mini
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include "board.h"
#include "periph/gpio.h"
// #include "periph/rtt.h"

// static inline void set_lpuart_clock_source(void)
// {
// 	 Use MCGIRCLK (internal reference 4 MHz clock)
// 	SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_LPUART0SRC_MASK) | SIM_SOPT2_LPUART0SRC(3);
// }

void board_init(void)
{
	// bit_set32(&SIM->SCGC5, SIM_SCGC5_DCDC_SHIFT);

 //    /* We don't have a status bit for checking if the DCDC is in bypass mode
 //     * so we check if the PSWITCH bit is set, assuming that the power switch
 //     * has not been released since the DCDC was started. This is run early in
 //     * the boot process so it should be fine. */
 //    if (DCDC->REG0 & DCDC_REG0_PSWITCH_STATUS_MASK) {
 //        /* DCDC is on */
 //        /* Spin until DCDC output is stable */
 //        while (!(DCDC->REG0 & DCDC_REG0_DCDC_STS_DC_OK_MASK)) {}
 //        /* Set up DCDC for pulsed mode in VLPS, LLS */
 //        DCDC->REG0 = (DCDC->REG0 & ~(DCDC_REG0_DCDC_VBAT_DIV_CTRL_MASK | DCDC_REG0_VLPS_CONFIG_DCDC_HP_MASK)) |
 //            DCDC_REG0_DCDC_VBAT_DIV_CTRL(0b10) | /* provide VBAT/2 to ADC input to measure battery level */
 //            DCDC_REG0_DCDC_LP_DF_CMP_ENABLE_MASK;
 //        DCDC->REG1 = DCDC->REG1 |
 //            DCDC_REG1_DCDC_LOOPCTRL_EN_CM_HYST_MASK |
 //            DCDC_REG1_DCDC_LOOPCTRL_EN_DF_HYST_MASK;
 //        DCDC->REG2 = DCDC->REG2 |
 //            DCDC_REG2_DCDC_LOOPCTRL_HYST_SIGN_MASK;
 //        DCDC->REG3 = (DCDC->REG3 &
 //            ~(DCDC_REG3_DCDC_MINPWR_HALF_FETS_MASK |
 //              DCDC_REG3_DCDC_MINPWR_DOUBLE_FETS_MASK |
 //              DCDC_REG3_DCDC_MINPWR_DOUBLE_FETS_PULSED_MASK)) |
 //            DCDC_REG3_DCDC_MINPWR_HALF_FETS_PULSED_MASK;

 //        /* Spin until DCDC is stable */
 //        while (!(DCDC->REG0 & DCDC_REG0_DCDC_STS_DC_OK_MASK)) {}

 //        /* DCDC has stabilized, halt stepping
 //         * This must be set before entering low power modes */
 //        DCDC->REG3 |= DCDC_REG3_DCDC_VDD1P5CTRL_DISABLE_STEP_MASK |
 //            DCDC_REG3_DCDC_VDD1P8CTRL_DISABLE_STEP_MASK;

 //        /* Spin until DCDC is stable */
 //        while (!(DCDC->REG0 & DCDC_REG0_DCDC_STS_DC_OK_MASK)) {}
 //    }

	/* initialize the CPU core */
	cpu_init();

	/* configure hsosc trim */
	// MCG->C4 = MCG->C4 & ~MCG_C4_FCTRIM_MASK | MCG_C4_FCTRIM(0xc);
	// MCG->C2 = MCG->C2 & ~MCG_C2_FCFTRIM_MASK | MCG_C2_FCFTRIM(0x1);

	// set_lpuart_clock_source();

	/* initialize and turn off LEDs */
	gpio_init(LED0_PIN, GPIO_OUT);
	LED0_OFF;
}

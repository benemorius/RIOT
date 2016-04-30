/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_efm32
 * @{
 *
 * @file
 * @brief       Implementation of the kernels power management interface
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Ryan Kurte <ryankurte@gmail.com>
 *
 * @}
 */

#include "arch/lpm_arch.h"
#include "cpu.h"

#define NON_WIC_INT_MASK_0    (~(0xff020e63U))
#define NON_WIC_INT_MASK_1    (~(0x00000046U))
uint32_t nonWicIntEn[2];

static enum lpm_mode current_mode = LPM_UNKNOWN;
// static uint16_t cmuStatus;

void lpm_arch_init(void)
{
	current_mode = LPM_ON;
}

enum lpm_mode lpm_arch_set(enum lpm_mode target)
{
	enum lpm_mode last_mode = current_mode;

//     switch (target) {
// 		case LPM_ON:
// 			current_mode = LPM_ON;
// 			break;
//
// 		case LPM_IDLE:
// 			current_mode = LPM_IDLE;
// 			SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
// 			__WFI();
// // 			BSP_TraceSwoSetup();
// 			break;
//
// 		case LPM_SLEEP:
// 			current_mode = LPM_SLEEP;
//
// 			//wait for uart tx to complete
// 			while (!(USART0->STATUS & USART_STATUS_TXC));
//
// 			cmuStatus = (uint16_t)(CMU->STATUS);
// 			SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
//
// 			//errata EMU_E107
// 			nonWicIntEn[0] = NVIC->ISER[0] & NON_WIC_INT_MASK_0;
// 			NVIC->ICER[0] = nonWicIntEn[0];
//
// #if (NON_WIC_INT_MASK_1 != (~(0x0U)))
// 			nonWicIntEn[1] = NVIC->ISER[1] & NON_WIC_INT_MASK_1;
// 			NVIC->ICER[1] = nonWicIntEn[1];
// #endif
//
// 			__WFI();
//
//
//
// 			break;
//
// 		case LPM_POWERDOWN:
// 			break;
//
// 		case LPM_OFF:
// 			break;
//
// 		case LPM_UNKNOWN:
// 			break;
//     }

    return last_mode;
}

enum lpm_mode inline lpm_arch_get(void)
{
    return current_mode;
}

void lpm_arch_awake(void)
{
// 	//errata EMU_E107
// 	NVIC->ISER[0] = nonWicIntEn[0];
// #if (NON_WIC_INT_MASK_1 != (~(0x0U)))
//     NVIC->ISER[1] = nonWicIntEn[1];
// #endif
//
//
// 	/*  Enable high speed crystal */
// 	CMU->OSCENCMD |= CMU_OSCENCMD_HFXOEN;
//
// 	/*  Wait for high speed crystal to stabilise */
// 	while (!(CMU->STATUS & CMU_STATUS_HFXORDY)) {
// 		;
// 	}
//
// 	/*  Set high speed crystal as core clock with divisor of 1 */
// 	CMU->CMD |= CMU_CMD_HFCLKSEL_HFXO;
// // 	CMU->CTRL |= (1 << 14);
//
// 	/*  Wait for clock switch */
// 	while ((CMU->STATUS & CMU_STATUS_HFRCOSEL)) {
// 		;
// 	}
//
// 	/*  Disable high speed oscillator */
// 	CMU->OSCENCMD |= CMU_OSCENCMD_HFRCODIS;
//
// 	/*  Enable low energy interface (for watchdog) */
// 	CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_LE;
//
//
//
//
// 	/* AUXHFRCO was automatically disabled (except if using debugger). */
// 	/* HFXO was automatically disabled. */
// 	/* LFRCO/LFXO were possibly disabled by SW in EM3. */
// 	/* Restore according to status prior to entering EM. */
// 	CMU->OSCENCMD = cmuStatus & (CMU_STATUS_AUXHFRCOENS |
// 	CMU_STATUS_HFXOENS |
// 	CMU_STATUS_LFRCOENS |
// 	CMU_STATUS_LFXOENS);
//
// 	/* Restore oscillator used for clocking core */
// 	switch (cmuStatus & (CMU_STATUS_HFXOSEL | CMU_STATUS_HFRCOSEL |
// 		CMU_STATUS_LFXOSEL | CMU_STATUS_LFRCOSEL))
// 	{
// 		case CMU_STATUS_LFRCOSEL:
// 			/* Wait for LFRCO to stabilize */
// 			while (!(CMU->STATUS & CMU_STATUS_LFRCORDY))
// 				;
// 			CMU->CMD = CMU_CMD_HFCLKSEL_LFRCO;
// 			break;
//
// 		case CMU_STATUS_LFXOSEL:
// 			/* Wait for LFXO to stabilize */
// 			while (!(CMU->STATUS & CMU_STATUS_LFXORDY))
// 				;
// 			CMU->CMD = CMU_CMD_HFCLKSEL_LFXO;
// 			break;
//
// 		case CMU_STATUS_HFXOSEL:
// 			/* Wait for HFXO to stabilize */
// 			while (!(CMU->STATUS & CMU_STATUS_HFXORDY))
// 				;
// 			CMU->CMD = CMU_CMD_HFCLKSEL_HFXO;
// 			break;
//
// 		default: /* CMU_STATUS_HFRCOSEL */
// 			/* If core clock was HFRCO core clock, it is automatically restored to */
// 			/* state prior to entering energy mode. No need for further action. */
// 			break;
// 	}
//
// 	/* If HFRCO was disabled before entering Energy Mode, turn it off again */
// 	/* as it is automatically enabled by wake up */
// 	if ( ! (cmuStatus & CMU_STATUS_HFRCOENS) )
// 	{
// 		CMU->OSCENCMD = CMU_OSCENCMD_HFRCODIS;
// 	}
}

void lpm_arch_begin_awake(void)
{
    /* TODO */
}

void lpm_arch_end_awake(void)
{
    /* TODO */
}

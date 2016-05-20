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
 * @ingroup     cpu_cc26xx-cc13xx
 * @{
 *
 * @file
 * @brief       Implementation of the kernel's power management interface
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include "cpu.h"
#include "arch/lpm_arch.h"

#include "driverlib/uart.h"
#include "driverlib/sys_ctrl.h"

static enum lpm_mode current_mode = LPM_UNKNOWN;

void lpm_arch_init(void)
{
    current_mode = LPM_ON;
}

enum lpm_mode lpm_arch_set(enum lpm_mode target)
{
	enum lpm_mode last_mode = current_mode;

    switch (target) {
		case LPM_ON:
			current_mode = LPM_ON;
			break;

		case LPM_IDLE:
			current_mode = LPM_IDLE;
            SCB->SCR &= ~(SCB_SCR_SLEEPDEEP_Msk);

            //wait for uart tx to complete
            while (UARTBusy(UART0_BASE));

            __WFI();
			break;

		case LPM_SLEEP:
			current_mode = LPM_SLEEP;
            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

			//wait for uart tx to complete
            while (UARTBusy(UART0_BASE));

			__WFI();

			break;

		case LPM_POWERDOWN:
			break;

		case LPM_OFF:
			break;

		case LPM_UNKNOWN:
			break;
    }
    return last_mode;
}

// void lpm_arch_awake(void)
// {
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
// }

enum lpm_mode lpm_arch_get(void)
{
    return current_mode;
}

void lpm_arch_awake(void)
{
    if (current_mode == LPM_SLEEP) {

        /* Sync so that we get the latest values before adjusting recharge settings */
        SysCtrlAonSync();

        /* Adjust recharge settings */
        SysCtrlAdjustRechargeAfterPowerDown();

        /*
         * Release the request to the uLDO
         * This is likely not required, since the switch to GLDO/DCDC is automatic
         * when coming back from deep sleep
         */
        PRCMMcuUldoConfigure(false);

        /* Turn on cache again */
        VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);
        PRCMCacheRetentionEnable();

        AONIOCFreezeDisable();
        SysCtrlAonSync();

        /* Check operating conditions, optimally choose DCDC versus GLDO */
        SysCtrl_DCDC_VoltageConditionalControl();


        /* After stop mode, the clock system needs to be reconfigured */
        cpu_init();
    }
    current_mode = LPM_ON;
}

/** Not provided */
inline void lpm_arch_begin_awake(void) { }

/** Not provided */
inline void lpm_arch_end_awake(void) { }

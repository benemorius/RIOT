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
#include "driverlib/pwr_ctrl.h"

void osc_switch_to_hf_rc(void);

static enum lpm_mode current_mode = LPM_UNKNOWN;

void lpm_arch_init(void)
{
    current_mode = LPM_ON;
}

enum lpm_mode lpm_arch_set(enum lpm_mode target)
{
	enum lpm_mode last_mode = current_mode;

    if(target == LPM_IDLE)
        target = LPM_SLEEP;

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

			/* wait for uart tx to complete */
            while (UARTBusy(UART0_BASE));

            AONIOCFreezeEnable();

//             PRCMPowerDomainOff(0);

            osc_switch_to_hf_rc();

            /* Configure clock sources for MCU and AUX: No clock */
            AONWUCMcuPowerDownConfig(AONWUC_NO_CLOCK);
            AONWUCAuxPowerDownConfig(AONWUC_NO_CLOCK);

            /* Full RAM retention. */
            AONWUCMcuSRamConfig(MCU_RAM0_RETENTION | MCU_RAM1_RETENTION |
            MCU_RAM2_RETENTION | MCU_RAM3_RETENTION);

            /* Disable retention of AUX RAM */
            AONWUCAuxSRamConfig(false);

            /*
             * Always turn off RFCORE, CPU, SYSBUS and VIMS. RFCORE should be off
             * already
             */
            PRCMPowerDomainOff(PRCM_DOMAIN_RFCORE | PRCM_DOMAIN_CPU |
            PRCM_DOMAIN_VIMS | PRCM_DOMAIN_SYSBUS |
            PRCM_DOMAIN_PERIPH);

            /* Request JTAG domain power off */
            AONWUCJtagPowerOff();

            /* Turn off AUX */
//             AUXWUCPowerCtrl(AUX_WUC_POWER_OFF);
            AONWUCDomainPowerDownEnable();
            while(AONWUCPowerStatusGet() & AONWUC_AUX_POWER_ON);

            /* Configure the recharge controller */
            SysCtrlSetRechargeBeforePowerDown(XOSC_IN_HIGH_POWER_MODE);

            //
            // Request the uLDO for standby power consumption.
            //
            PowerCtrlSourceSet(PWRCTRL_PWRSRC_ULDO);

            /* Sync the AON interface to ensure all writes have gone through. */
            SysCtrlAonSync();

            /*
             * Explicitly turn off VIMS cache, CRAM and TRAM. Needed because of
             * retention mismatch between VIMS logic and cache. We wait to do this
             * until right before deep sleep to be able to use the cache for as long
             * as possible.
             */
            PRCMCacheRetentionDisable();
            VIMSModeSet(VIMS_BASE, VIMS_MODE_OFF);

            /* Deep Sleep */
            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
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

enum lpm_mode lpm_arch_get(void)
{
    return current_mode;
}

static void uart_init(void) {
    /* Enable peripheral power domain */
    if(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON) {
        PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
        while(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON);
    }

    /* Enable serial power domain */
    if(PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON) {
        PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL);
        while(PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON);
    }

    /* Enable UART0 peripheral */
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
//     PRCMPeripheralRunEnable(uart_config[uart].prcmp);

    /* Apply settings and wait for them to take effect */
    PRCMLoadSet();
    while(!PRCMLoadGet());
}

static void gpio_init(void) {
    /* Enable peripheral power domain */
    if(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON) {
        PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
        while(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON);
    }

    /* Enable GPIO peripheral */
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);

    /* Apply settings and wait for them to take effect */
    PRCMLoadSet();
    while(!PRCMLoadGet());
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
        gpio_init();
        uart_init();
    }
    current_mode = LPM_ON;
}

/** Not provided */
inline void lpm_arch_begin_awake(void) { }

/** Not provided */
inline void lpm_arch_end_awake(void) { }

void osc_switch_to_hf_rc(void)
{
    /* Enable the Osc interface and remember the state of the SMPH clock */
    OSCInterfaceEnable();

    /* Set all clock sources to the HF RC Osc */
    OSCClockSourceSet(OSC_SRC_CLK_MF | OSC_SRC_CLK_HF, OSC_RCOSC_HF);

    /* Check to not enable HF RC oscillator if already enabled */
    if(OSCClockSourceGet(OSC_SRC_CLK_HF) != OSC_RCOSC_HF) {
        /* Switch the HF clock source (cc26xxware executes this from ROM) */
        OSCHfSourceSwitch();
    }

    /* Restore the SMPH clock and disable the OSC interface */
    OSCInterfaceDisable();
}

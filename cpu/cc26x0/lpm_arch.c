/*
 * Copyright (C) 2016 Leon George
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_cc26x0
 * @{
 *
 * @file
 * @brief       implementation of the kernels power management interface
 *
 * @}
 */

#include "arch/lpm_arch.h"
#include "cpu.h"
#include "sys_ctrl.h"
#include "hw_aon_ioc.h"
#include "hw_ddi_0_osc.h"
#include "hw_aon_wuc.h"
#include "hw_prcm.h"
#include "hw_aon_rtc.h"
#include "hw_vims.h"
#include "hw_aux_wuc.h"

/* ROM HAPI HFSourceSafeSwitch function */
#define ROM_HAPI_HFSOURCESAFESWITCH_ADDR_P (0x10000048 + (14*4))
#define ROM_HAPI_HFSOURCESAFESWITCH_ADDR (*(reg32_t*)ROM_HAPI_HFSOURCESAFESWITCH_ADDR_P)
#define ROM_HAPI_HFSOURCESAFESWITCH() (((void(*)(void))ROM_HAPI_HFSOURCESAFESWITCH_ADDR)())

static enum lpm_mode current_mode = LPM_UNKNOWN;

void lpm_arch_init(void)
{
}

enum lpm_mode lpm_arch_set(enum lpm_mode target)
{
    enum lpm_mode last_mode = current_mode;

//     target = LPM_ON;

    switch (target) {
        case LPM_ON:
        case LPM_UNKNOWN:
            current_mode = LPM_ON;
            break;

        case LPM_IDLE:
        case LPM_POWERDOWN:
        case LPM_OFF:
            current_mode = LPM_IDLE;
            SCB->SCR &= ~(SCB_SCR_SLEEPDEEP_Msk);

            //wait for uart tx to complete
            while (UART->FR & UART_FR_BUSY) {}

            __WFI();
            break;

        case LPM_SLEEP:
            current_mode = LPM_SLEEP;
            /* wait for uart tx to complete */
            while (UART->FR & UART_FR_BUSY) {}

            /* freeze gpio output states */
            AON->IOCLATCH = 0x0;

            /* Enable the Osc interface and remember the state of the SMPH clock */
            // Force power on AUX to ensure CPU has access
            AON_WUC->AUXCTL |= AON_WUC_AUXCTL_AUX_FORCE_ON;
            while(!(AON_WUC->PWRSTAT & AONWUC_AUX_POWER_ON)) {}

            // Enable the AUX domain OSC clock and wait for it to be ready
            AUX_WUC->MODCLKEN0 |= MODCLKEN0_AUX_DDI0_OSC_EN;
            while ((AUX_WUC->MODCLKEN0 & MODCLKEN0_AUX_DDI0_OSC_EN) == 0) {}
            AUX_WUC->MODCLKEN0 |= MODCLKEN0_SMPH_EN;
            while ((AUX_WUC->MODCLKEN0 & MODCLKEN0_SMPH_EN) == 0) {}

            /* Set HF and MF clock sources to the HF RC Osc */
            DDI_0_OSC->CTL0 &= ~(DDI_0_OSC_CTL0_SCLK_HF_SRC_SEL_M | DDI_0_OSC_CTL0_SCLK_MF_SRC_SEL_M);

            /* Check to not enable HF RC oscillator if already enabled */
            if ((DDI_0_OSC->STAT0 & DDI_0_OSC_STAT0_SCLK_HF_SRC_M) != DDI_0_OSC_STAT0_SCLK_HF_SRC_RCOSC) {
                /* Switch the HF clock source (cc26xxware executes this from ROM) */
                ROM_HAPI_HFSOURCESAFESWITCH();
            }

            /* Restore the SMPH clock and disable the OSC interface */
            AUX_WUC->MODCLKEN0 &= ~AUX_WUC_MODCLKEN0_AUX_DDI0_OSC_EN;
            AON_WUC->AUXCTL &= ~AON_WUC_AUXCTL_AUX_FORCE_ON;
            AUX_WUC->MODCLKEN0 &= ~AUX_WUC_MODCLKEN0_SMPH_EN;

            /* Configure clock sources for MCU and AUX: No clock */
            AON_WUC->MCUCLK &= ~AON_WUC_MCUCLK_PWR_DWN_SRC_M;
            AON_WUC->AUXCLK &= ~AON_WUC_AUXCLK_PWR_DWN_SRC_M;


            /* Full RAM retention. */
            AON_WUC->MCUCFG |= MCU_RAM0_RETENTION | MCU_RAM1_RETENTION | MCU_RAM2_RETENTION | MCU_RAM3_RETENTION;

            /* Disable retention of AUX RAM */
            AON_WUC->AUXCFG &= ~(1 << AON_WUC_AUXCFG_RAM_RET_EN_BITN);

            /*
             * Always turn off RFCORE, CPU, and VIMS. RFCORE should be off
             * already
             */
//             PRCM->PDCTL0 &= ~(PDCTL0_RFC_ON | PDCTL0_PERIPH_ON | PDCTL0_SERIAL_ON);
            PRCM->PDCTL0 &= ~(PDCTL0_RFC_ON | PDCTL0_PERIPH_ON);
            PRCM->PDCTL1 &= ~(PDCTL1_CPU_ON | PDCTL1_VIMS_ON);

            /* Request JTAG domain power off */
            /* otherwise MCU domain can't be turned off */
            AON_WUC->JTAGCFG = 0;

//             /* stop aux osc clock before powering down aux */
//             AUX_WUC->MODCLKEN0 &= ~MODCLKEN0_AUX_DDI0_OSC_EN;
//             AON_WUC->AUXCTL &= ~AUXCTL_AUX_FORCE_ON;
//
//             /* Turn off AUX */
//             AUX_WUC->PWROFFREQ = AUX_WUC_PWROFFREQ_REQ;
//             AUX_WUC->MCUBUSCTL = AUX_WUC_MCUBUSCTL_DISCONNECT_REQ;

            // set AUX and MCU domain power down mode
            AON_WUC->CTL0 &= ~(1 << AON_WUC_CTL0_PWR_DWN_DIS_BITN);
            while(AON_WUC->PWRSTAT & AONWUC_AUX_POWER_ON) {}

            /* Configure the recharge controller */
            SysCtrlSetRechargeBeforePowerDown(XOSC_IN_HIGH_POWER_MODE);

            //
            // Request the uLDO for standby power consumption.
            //
//             PRCM->VDCTL |= (1 << PRCM_VDCTL_ULDO_BITN);

            /* Sync the AON interface to ensure all writes have gone through. */
            *(reg32_t*)(AON_RTC_BASE + AON_RTC_O_SYNC);

            /* disable flash power */
            PRCM->PDCTL1 &= ~PRCM_O_PDCTL1VIMS;

            /*
             * Explicitly turn off VIMS cache, CRAM and TRAM. Needed because of
             * retention mismatch between VIMS logic and cache. We wait to do this
             * until right before deep sleep to be able to use the cache for as long
             * as possible.
             */
            PRCM->RAMRETEN &= ~PRCM_RAMRETEN_VIMS_M;
            VIMS->CTL |= (VIMS_MODE_OFF);

            /* Deep Sleep */
            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
            __WFI();
            break;
    }
    return last_mode;
}

enum lpm_mode lpm_arch_get(void)
{
    return current_mode;
}

void lpm_arch_awake(void)
{
    if (current_mode == LPM_SLEEP) {

//         AON_WUC->AUXCTL |= AUXCTL_AUX_FORCE_ON; /* power on AUX_PD */
//         while(!(AON_WUC->PWRSTAT & PWRSTAT_AUX_PD_ON)); /* wait for AUX_PD to be powered on */
//         AUX_WUC->MODCLKEN0 |= MODCLKEN0_AUX_DDI0_OSC_EN; /* turn on oscillator interface clock */

        /* Sync so that we get the latest values before adjusting recharge settings */
        *(reg32_t*)(AON_RTC_BASE + AON_RTC_O_SYNC);

        /* Adjust recharge settings */
        SysCtrlAdjustRechargeAfterPowerDown();

        /*
         * Release the request to the uLDO
         * This is likely not required, since the switch to GLDO/DCDC is automatic
         * when coming back from deep sleep
         */
        PRCM->VDCTL &= ~(1 << PRCM_VDCTL_ULDO_BITN);

        /* Turn on cache again */
        VIMS->CTL = (VIMS->CTL & ~(VIMS_CTL_MODE_M)) | (VIMS_MODE_ENABLED);
//         PRCM->RAMRETEN |= PRCM_RAMRETEN_VIMS_M;

        /* unfreeze gpio outputs */
        AON->IOCLATCH = AON_IOC_IOCLATCH_EN;
        *(reg32_t*)(AON_RTC_BASE + AON_RTC_O_SYNC);

        /* Check operating conditions, optimally choose DCDC versus GLDO */
        SysCtrl_DCDC_VoltageConditionalControl();

        /* After stop mode, the clock system needs to be reconfigured */
        cpu_init();

        /* init gpio */
        PRCM->PDCTL0 |= PDCTL0_PERIPH_ON;
        while(!(PRCM->PDSTAT0 & PDSTAT0_PERIPH_ON)) ;
        PRCM->GPIOCLKGR |= 1;
        PRCM->CLKLOADCTL |= CLKLOADCTL_LOAD;
        while (!(PRCM->CLKLOADCTL & CLKLOADCTL_LOADDONE)) {}

        /* init uart */
        PRCM->PDCTL0SERIAL = 1;
        while (!(PRCM->PDSTAT0 & PDSTAT0_SERIAL_ON)) ;
        PRCM->UARTCLKGR = 1;
        PRCM->CLKLOADCTL = CLKLOADCTL_LOAD;
        while (!(PRCM->CLKLOADCTL & CLKLOADCTL_LOADDONE)) {}
    }
    current_mode = LPM_ON;
}

void lpm_arch_begin_awake(void)
{
}

void lpm_arch_end_awake(void)
{
}

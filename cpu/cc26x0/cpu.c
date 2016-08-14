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
 * @brief       implementation of the CPU initialization
 *
 * @author      Leon M. George <leon@georgemail.eu>
 * @}
 */

#include "cpu.h"
#include "periph_conf.h"
#include "trim.h"

/* ROM HAPI HFSourceSafeSwitch function */
#define ROM_HAPI_HFSOURCESAFESWITCH_ADDR_P (0x10000048 + (14*4))
#define ROM_HAPI_HFSOURCESAFESWITCH_ADDR (*(uint32_t*)ROM_HAPI_HFSOURCESAFESWITCH_ADDR_P)
#define ROM_HAPI_HFSOURCESAFESWITCH() (((void(*)(void))ROM_HAPI_HFSOURCESAFESWITCH_ADDR)())

/**
 *@brief Configure the MCU system clock
 */
static void cpu_clock_init(void);
static void setup(void);

/**
 * @brief Initialize the CPU, set IRQ priorities
 */
void cpu_init(void)
{
    /* initialize the Cortex-M core */
    cortexm_init();

    /* undocumented setup */
    setup();

    /* initialize the system clock */
    cpu_clock_init();
}

static void setup(void)
{
    //
    // Clock must always be enabled for the semaphore module (due to ADI/DDI HW workaround)
    //
    *(uint32_t*)(AUX_WUC_BASE + AUX_WUC_O_MODCLKEN1) = AUX_WUC_MODCLKEN1_SMPH;

    //
    // Enable the clocks for AUX_DDI0_OSC and AUX_ADI4
    //
    *(uint32_t*)(AUX_WUC_BASE + AUX_WUC_O_MODCLKEN0) = AUX_WUC_MODCLKEN0_AUX_DDI0_OSC |
                                                       AUX_WUC_MODCLKEN0_AUX_ADI4;

    //
    // Following sequence is required for using XOSCHF, if not included
    // devices crashes when trying to switch to XOSCHF.
    //

    // Get layout revision of the factory configuration area
    uint32_t chip_revision = *(uint32_t*)(FCFG1_BASE + FCFG1_O_FCFG1_REVISION);
    if (chip_revision == 0xFFFFFFFF) {
        chip_revision = 0;
    }
    // Trim AMPCOMP settings required before switch to XOSCHF
    uint32_t trim;
    trim = GetTrimForAmpcompTh2();
    *(uint32_t*)(AUX_DDI0_OSC_BASE + DDI_0_OSC_O_AMPCOMPTH2) = trim;
    trim = GetTrimForAmpcompTh1();
    *(uint32_t*)(AUX_DDI0_OSC_BASE + DDI_0_OSC_O_AMPCOMPTH1) = trim;
    trim = GetTrimForAmpcompCtrl(chip_revision);
    *(uint32_t*)(AUX_DDI0_OSC_BASE + DDI_0_OSC_O_AMPCOMPCTL) = trim;
}

static void cpu_clock_init(void)
{
    AON_WUC->AUXCTL |= AUXCTL_AUX_FORCE_ON; /* power on AUX_PD */
    while(!(AON_WUC->PWRSTAT & PWRSTAT_AUX_PD_ON)); /* wait for AUX_PD to be powered on */
    AUX_WUC->MODCLKEN0 |= MODCLKEN0_AUX_DDI0_OSC_EN; /* turn on oscillator interface clock */

    /* start hf_xosc and configure xtal frequency */
    DDI_0_OSC->CTL0 = DDI_0_OSC_CTL0_SCLK_HF_SRC_SEL_XOSC
                    | DDI_0_OSC_CTL0_SCLK_MF_SRC_SEL
                    | DDI_0_OSC_CTL0_XTAL_IS_24M;

    /* wait for hf_xosc to start */
    while ((DDI_0_OSC->STAT0 & 0x1) == 0) { }

    /* switch to hf_xosc */
    ROM_HAPI_HFSOURCESAFESWITCH();
}

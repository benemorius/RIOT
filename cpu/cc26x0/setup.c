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

#include "setup.h"
#include "cpu.h"
#include "periph_conf.h"

#include "hw_fcfg1.h"
#include "hw_ccfg.h"
#include "hw_ddi_0_osc.h"
#include "hw_flash.h"
#include "hw_aux_wuc.h"
#include "hw_aon_ioc.h"
#include "hw_aon_sysctl.h"
#include "hw_vims.h"
#include "hw_aon_wuc.h"
#include "hw_prcm.h"
#include "hw_aon_rtc.h"
#include "hw_ddi.h"
#include "hw_adi_3_refsys.h"
#include "hw_adi.h"
#include "hw_adi_4_aux.h"

#ifndef HF_CLOCK_SOURCE
#define HF_CLOCK_SOURCE DDI_0_OSC_CTL0_SCLK_HF_SRC_SEL_XOSC /* set 48MHz XOSC */
#endif
#ifndef LF_CLOCK_SOURCE
#define LF_CLOCK_SOURCE DDI_0_OSC_CTL0_SCLK_LF_SRC_SEL_RCOSCHFDLF /* set 31.25kHz derived from 48MHz RCOSC */
#endif

//*****************************************************************************
// Need to know the CCFG:MODE_CONF.VDDR_TRIM_SLEEP_DELTA field width in order
// to sign extend correctly but this is however not defined in the hardware
// description fields and is therefore defined separately here.
//*****************************************************************************
#define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_WIDTH    4

/**
 *@brief Configure the MCU system clock
 */
static void SetupCacheModeAccordingToCcfgSetting(void);
static void HapiTrimDeviceShutDown(uint32_t ui32Fcfg1Revision);
static uint32_t GetTrimForAnabypassValue1(uint32_t ccfg_ModeConfReg);
static uint32_t GetTrimForRcOscLfRtuneCtuneTrim(void);
static uint32_t GetTrimForXoscHfIbiastherm(void);
static uint32_t GetTrimForAmpcompTh2(void);
static uint32_t GetTrimForAmpcompTh1(void);
static uint32_t GetTrimForAmpcompCtrl(uint32_t ui32Fcfg1Revision);
static void aux_write(uint32_t address, uint32_t data);
static uint32_t GetTrimForXoscHfFastStart(void);
static uint32_t GetTrimForRadcExtCfg(uint32_t ui32Fcfg1Revision);
static uint32_t GetTrimForXoscHfCtl(uint32_t ui32Fcfg1Revision);
static uint32_t GetTrimForAdcShModeEn(uint32_t ui32Fcfg1Revision);
static int32_t  SignExtendVddrTrimValue( uint32_t ui32VddrTrimVal );
static uint32_t GetTrimForAdcShVbufEn( uint32_t ui32Fcfg1Revision );
static uint32_t GetTrimForDblrLoopFilterResetVoltage( uint32_t ui32Fcfg1Revision );
static uint32_t GetTrimForRcOscLfIBiasTrim( uint32_t ui32Fcfg1Revision );
static uint32_t GetTrimForXoscLfRegulatorAndCmirrwrRatio( uint32_t ui32Fcfg1Revision );

#undef FLASH_BASE

#define FCFG1_BASE (0x50001000)
#define FLASH_BASE (0x40030000)
// #define FCFG1_O_FCFG1_REVISION (0x31c)
// #define FLASH_O_CFG (0x24)
// #define FLASH_CFG_DIS_STANDBY_BITN (1)
// #define AUX_WUC_O_MODCLKEN1 (0x5c)
// #define AUX_WUC_MODCLKEN1_SMPH (1)
// #define PRCM_O_WARMRESET (0x110)
// #define PRCM_WARMRESET_WR_TO_PINRESET_BITN (2)
// #define AON_IOC_O_IOCLATCH (0xc)
// #define AON_IOC_IOCLATCH_EN_BITN (0)
// #define AON_SYSCTL_O_SLEEPCTL (0x8)
// #define AON_SYSCTL_SLEEPCTL_IO_PAD_SLEEP_DIS_BITN (0)
// #define PRCM_O_PDCTL1VIMS (0x18c)
// #define FLASH_O_FPAC1 (0x2048)
// #define FLASH_FPAC1_PSLEEPTDIS_M (0x0fff0000)
// #define FLASH_FPAC1_PSLEEPTDIS_S (16)
// #define AON_SYSCTL_O_RESETCTL (0x4)
// #define AON_SYSCTL_RESETCTL_BOOT_DET_1_M (0x2000)
// #define AON_SYSCTL_RESETCTL_BOOT_DET_0_M (0x1000)
// #define AON_SYSCTL_RESETCTL_BOOT_DET_0_S (12)
// #define AON_SYSCTL_RESETCTL_BOOT_DET_1_CLR_M (0x02000000)
// #define AON_SYSCTL_RESETCTL_BOOT_DET_0_CLR_M (0x01000000)
// #define AON_SYSCTL_RESETCTL_BOOT_DET_1_SET_M (0x00020000)
// #define AON_SYSCTL_RESETCTL_BOOT_DET_0_SET_M (0x00010000)
// #define VIMS_O_STAT (0x0)
// #define VIMS_STAT_MODE_CHANGING_BITN (3)

void cc26x0_setup(void)
{
    uint32_t ui32Fcfg1Revision;
    uint32_t ui32AonSysResetctl;

    //
    // Get layout revision of the factory configuration area
    // (Handle undefined revision as revision = 0)
    //
    ui32Fcfg1Revision = *(uint32_t*)(FCFG1_BASE + FCFG1_O_FCFG1_REVISION);
    if ( ui32Fcfg1Revision == 0xFFFFFFFF ) {
        ui32Fcfg1Revision = 0;
    }

    //
    // This driverlib version and setup file is for CC26xx PG2.2 and later
    // Halt if violated
    //
//     ThisCodeIsBuiltForCC26xxHwRev22AndLater_HaltIfViolated();

    //
    // Enable standby in flash bank
    //
    *(uint32_t*)(FLASH_BASE + FLASH_O_CFG) &= ~(1 << FLASH_CFG_DIS_STANDBY_BITN);

    //
    // Clock must always be enabled for the semaphore module (due to ADI/DDI HW workaround)
    //
    *(uint32_t*)(AUX_WUC_BASE + AUX_WUC_O_MODCLKEN1) = AUX_WUC_MODCLKEN1_SMPH;

    //
    // Warm resets on CC26XX complicates software design as much of our software
    // expect that initialization is done from a full system reset.
    // This includes RTC setup, oscillator configuration and AUX setup.
    // To ensure a full reset of the device is done when customers get e.g. a Watchdog
    // reset, the following is set here:
    //
    *(uint32_t*)(PRCM_BASE + PRCM_O_WARMRESET) |=  (1 << PRCM_WARMRESET_WR_TO_PINRESET_BITN);

    //
    // Select correct CACHE mode and set correct CACHE configuration
    //
    SetupCacheModeAccordingToCcfgSetting();

    // 1. Check for powerdown
    // 2. Check for shutdown
    // 3. Assume cold reset if none of the above.
    //
    // It is always assumed that the application will freeze the latches in
    // AON_IOC when going to powerdown in order to retain the values on the IOs.
    //
    // NB. If this bit is not cleared before proceeding to powerdown, the IOs
    //     will all default to the reset configuration when restarting.
    if (!(*(uint32_t*)(AON_IOC_BASE + AON_IOC_O_IOCLATCH) & (1 << AON_IOC_IOCLATCH_EN_BITN)))
    {
        //
        // NB. This should be calling a ROM implementation of required trim and
        // compensation
        // e.g. HapiTrimDevicePowerDown()
//         HapiTrimDevicePowerDown();
    }
    // Check for shutdown
    //
    // When device is going to shutdown the hardware will automatically clear
    // the SLEEPDIS bit in the SLEEP register in the AON_SYSCTRL12 module.
    // It is left for the application to assert this bit when waking back up,
    // but not before the desired IO configuration has been re-established.
    else if (!(*(uint32_t*)(AON_SYSCTL_BASE + AON_SYSCTL_O_SLEEPCTL) & (1 << AON_SYSCTL_SLEEPCTL_IO_PAD_SLEEP_DIS_BITN)))
    {
        //
        // NB. This should be calling a ROM implementation of required trim and
        // compensation
        // e.g. HapiTrimDeviceShutDown()    -->
        //      HapiTrimDevicePowerDown();
        HapiTrimDeviceShutDown(ui32Fcfg1Revision);
//         HapiTrimDevicePowerDown();
    }
    else
    {
        // Consider adding a check for soft reset to allow debugging to skip
        // this section!!!
        //
        // NB. This should be calling a ROM implementation of required trim and
        // compensation
        // e.g. HapiTrimDeviceColdReset()   -->
        //      HapiTrimDeviceShutDown()    -->
        //      HapiTrimDevicePowerDown()
//         HapiTrimDeviceColdReset();
        HapiTrimDeviceShutDown(ui32Fcfg1Revision);
//         HapiTrimDevicePowerDown();

    }

    //
    // Set VIMS power domain control.
    // PDCTL1VIMS = 0 ==> VIMS power domain is only powered when CPU power domain is powered
    //
    *(uint32_t*)(PRCM_BASE + PRCM_O_PDCTL1VIMS) = 0;

    //
    // Configure optimal wait time for flash FSM in cases where flash pump
    // wakes up from sleep
    //
    *(uint32_t*)(FLASH_BASE + FLASH_O_FPAC1) = (*(uint32_t*)(FLASH_BASE + FLASH_O_FPAC1) &
                                                ~FLASH_FPAC1_PSLEEPTDIS_M) |
                                                (0x139<<FLASH_FPAC1_PSLEEPTDIS_S);

    //
    // And finally at the end of the flash boot process:
    // SET BOOT_DET bits in AON_SYSCTL to 3 if already found to be 1
    // Note: The BOOT_DET_x_CLR/SET bits must be manually cleared
    //
    if (((*(uint32_t*)(AON_SYSCTL_BASE + AON_SYSCTL_O_RESETCTL) &
        (AON_SYSCTL_RESETCTL_BOOT_DET_1_M | AON_SYSCTL_RESETCTL_BOOT_DET_0_M)) >>
        AON_SYSCTL_RESETCTL_BOOT_DET_0_S) == 1)
    {
        ui32AonSysResetctl = (*(uint32_t*)(AON_SYSCTL_BASE + AON_SYSCTL_O_RESETCTL) &
        ~(AON_SYSCTL_RESETCTL_BOOT_DET_1_CLR_M | AON_SYSCTL_RESETCTL_BOOT_DET_0_CLR_M |
        AON_SYSCTL_RESETCTL_BOOT_DET_1_SET_M | AON_SYSCTL_RESETCTL_BOOT_DET_0_SET_M));
        *(uint32_t*)(AON_SYSCTL_BASE + AON_SYSCTL_O_RESETCTL) = ui32AonSysResetctl | AON_SYSCTL_RESETCTL_BOOT_DET_1_SET_M;
        *(uint32_t*)(AON_SYSCTL_BASE + AON_SYSCTL_O_RESETCTL) = ui32AonSysResetctl;
    }

    //
    // Make sure there are no ongoing VIMS mode change when leaving trimDevice()
    // (There should typically be no wait time here, but need to be sure)
    //
    while ( *(uint32_t*)(VIMS_BASE + VIMS_O_STAT) & (1 << VIMS_STAT_MODE_CHANGING_BITN)) {
        // Do nothing - wait for an eventual ongoing mode change to complete.
    }
}

// #define VIMS_O_CTL (0x4)
// #define VIMS_CTL_MODE_M (0x3)
// #define VIMS_CTL_DYN_CG_EN_M (0x20000000)
// #define VIMS_CTL_PREF_EN_M (0x4)
// #define CCFG_O_SIZE_AND_DIS_FLAGS (0xfb0)
// #define VIMS_STAT_MODE_M (0x3)
// #define VIMS_STAT_MODE_GPRAM (0x0)
// #define VIMS_STAT_MODE_OFF (0x3)

static void SetupCacheModeAccordingToCcfgSetting(void)
{
    //
    // - Make sure to enable aggressive VIMS clock gating for power optimization
    //   Only for PG2 devices.
    // - Enable cache prefetch enable as default setting
    //   (Slightly higher power consumption, but higher CPU performance)
    // - IF ( CCFG_..._DIS_GPRAM == 1 )
    //   then: Enable cache (set cache mode = 1), even if set by ROM boot code
    //         (This is done because it's not set by boot code when running inside
    //         a debugger supporting the Halt In Boot (HIB) functionality).
    //   else: Set MODE_GPRAM if not already set (see inline comments as well)
    //
    uint32_t vimsCtlMode0 ;

    while (*(uint32_t*)(VIMS_BASE + VIMS_O_STAT) & (1 << VIMS_STAT_MODE_CHANGING_BITN)) {
        // Do nothing - wait for an eventual ongoing mode change to complete.
        // (There should typically be no wait time here, but need to be sure)
    }

    //
    // Note that Mode=0 is equal to MODE_GPRAM
    //
    vimsCtlMode0 = ((*(uint32_t*)(VIMS_BASE + VIMS_O_CTL) & ~VIMS_CTL_MODE_M) | VIMS_CTL_DYN_CG_EN_M | VIMS_CTL_PREF_EN_M);


    if (*(uint32_t*)(CCFG_BASE + CCFG_O_SIZE_AND_DIS_FLAGS) & CCFG_SIZE_AND_DIS_FLAGS_DIS_GPRAM) {
        // Enable cache (and hence disable GPRAM)
        *(uint32_t*)(VIMS_BASE + VIMS_O_CTL) = (vimsCtlMode0 | VIMS_CTL_MODE_CACHE);
    } else if ((*(uint32_t*)(VIMS_BASE + VIMS_O_STAT) & VIMS_STAT_MODE_M) != VIMS_STAT_MODE_GPRAM) {
        //
        // GPRAM is enabled in CCFG but not selected
        // Note: It is recommended to go via MODE_OFF when switching to MODE_GPRAM
        //
        *(uint32_t*)(VIMS_BASE + VIMS_O_CTL) = (vimsCtlMode0 | VIMS_CTL_MODE_OFF);
        while ((*(uint32_t*)(VIMS_BASE + VIMS_O_STAT) & VIMS_STAT_MODE_M) != VIMS_STAT_MODE_OFF) {
            // Do nothing - wait for an eventual mode change to complete (This goes fast).
        }
        *(uint32_t*)(VIMS_BASE + VIMS_O_CTL) = vimsCtlMode0;
    } else {
        // Correct mode, but make sure PREF_EN and DYN_CG_EN always are set
        *(uint32_t*)(VIMS_BASE + VIMS_O_CTL) = vimsCtlMode0;
    }
}

#define AUX_DDI0_OSC_BASE (0x400ca000)
#define ADI3_BASE (0x40086200)
#define AUX_ADI4_BASE (0x400cb000)
// #define AON_WUC_O_AUXCTL (0x10)
// #define AON_WUC_AUXCTL_AUX_FORCE_ON (1)
// #define AON_WUC_O_PWRSTAT (0x14)
// #define AON_WUC_PWRSTAT_AUX_PD_ON_BITN (5)
// #define AUX_WUC_O_MODCLKEN0 (0x0)
// #define AUX_WUC_MODCLKEN0_AUX_DDI0_OSC (0x40)
// #define AUX_WUC_MODCLKEN0_AUX_ADI4 (0x80)
// #define CCFG_O_MODE_CONF (0xfac)
// #define DDI_0_OSC_O_ANABYPASSVAL1 (0x18)

// #define CCFG_SIZE_AND_DIS_FLAGS_DIS_ALT_DCDC_SETTING (0)
// #define ADI3_BASE (0)
// #define ADI_O_MASK4B (0)
// #define ADI_3_REFSYS_O_DCDCCTL5 (0)
// #define CCFG_O_MODE_CONF_1 (0)
// #define CCFG_MODE_CONF_1_ALT_DCDC_IPEAK_S (0)
// #define FCFG1_O_LDO_TRIM (0)
// #define FCFG1_LDO_TRIM_VDDR_TRIM_SLEEP_M (0)
// #define FCFG1_LDO_TRIM_VDDR_TRIM_SLEEP_S (0)
// #define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_WIDTH (0)
// #define CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_S (0)
// #define ADI_O_MASK8B (0)
// #define ADI_3_REFSYS_O_DCDCCTL1 (0)
// #define ADI_3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_M (0)
// #define ADI_3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_S (0)
// #define AON_SYSCTL_O_PWRCTL (0)
// #define AON_SYSCTL_PWRCTL_DCDC_EN_BITN (0)
// #define CCFG_MODE_CONF_DCDC_RECHARGE_S (0)
// #define AON_SYSCTL_PWRCTL_DCDC_ACTIVE_BITN (0)
// #define CCFG_MODE_CONF_DCDC_ACTIVE_S (0)
// #define DDI_0_OSC_O_LFOSCCTL (0)
// #define DDI_0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_M (0)
// #define DDI_0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_M (0)
// #define DDI_0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_S (0)
// #define DDI_0_OSC_O_ANABYPASSVAL2 (0)
// #define DDI_0_OSC_ANABYPASSVAL2_XOSC_HF_IBIASTHERM_S (0)
// #define DDI_0_OSC_O_AMPCOMPTH2 (0)
// #define DDI_0_OSC_O_AMPCOMPTH1 (0)
// #define DDI_0_OSC_O_AMPCOMPCTL (0)
// #define DDI_O_MASK4B (0)
// #define DDI_0_OSC_O_ADCDOUBLERNANOAMPCTL (0)
// #define DDI_0_OSC_O_XOSCHFCTL (0)
// #define DDI_O_MASK8B (0)
// #define DDI_0_OSC_O_RADCEXTCFG (0)
// #define DDI_0_OSC_O_CTL0 (0)
// #define CCFG_MODE_CONF_SCLK_LF_OPTION_M (0)
// #define CCFG_MODE_CONF_SCLK_LF_OPTION_S (0)
// #define OSC_SRC_CLK_LF (0)
// #define OSC_XOSC_HF (0)
// #define OSC_SRC_CLK_HF (0)
// #define CCFG_O_EXT_LF_CLK (0)
// #define CCFG_EXT_LF_CLK_RTC_INCREMENT_M (0)
// #define CCFG_EXT_LF_CLK_RTC_INCREMENT_S (0)
// #define CCFG_EXT_LF_CLK_DIO_M (0)
// #define CCFG_EXT_LF_CLK_DIO_S (0)
// #define IOC_PORT_AON_CLK32K (0)
// #define DDI_O_SET (0)
// #define OSC_XOSC_LF (0)
// #define OSC_RCOSC_LF (0)
// #define AUX_ADI4_BASE (0)
// #define ADI_4_AUX_O_ADCREF1 (0)
// #define FCFG1_O_SOC_ADC_REF_TRIM_AND_OFFSET_EXT (0)
// #define FCFG1_SOC_ADC_REF_TRIM_AND_OFFSET_EXT_SOC_ADC_REF_VOLTAGE_TRIM_TEMP1_S (0)
// #define ADI_4_AUX_ADCREF1_VTRIM_S (0)
// #define ADI_4_AUX_ADCREF1_VTRIM_M (0)
// #define ADI_4_AUX_O_ADC0 (0)
// #define ADI_4_AUX_ADC0_SMPL_CYCLE_EXP_M (0)
// #define ADI_4_AUX_ADC0_SMPL_CYCLE_EXP_S (0)
// #define AUX_WUC_POWER_DOWN (0)
// #define FLASH_CFG_DIS_EFUSECLK_BITN (0)

static void HapiTrimDeviceShutDown(uint32_t ui32Fcfg1Revision)
{
    uint32_t   ui32Trim          ;
    uint32_t   ccfg_ModeConfReg  ;
    uint32_t   currentHfClock    ;
    uint32_t   ccfgExtLfClk      ;
    int32_t    i32VddrSleepTrim  ;
    int32_t    i32VddrSleepDelta ;

    //
    // Force AUX on and enable clocks
    //
    // No need to save the current status of the power/clock registers.
    // At this point both AUX and AON should have been reset to 0x0.
    //
    *(reg32_t*)(AON_WUC_BASE + AON_WUC_O_AUXCTL) = AON_WUC_AUXCTL_AUX_FORCE_ON;

    //
    // Wait for power on on the AUX domain
    //
    while( ! ( *(reg32_t*)( AON_WUC_BASE + AON_WUC_O_PWRSTAT) & (1 << AON_WUC_PWRSTAT_AUX_PD_ON_BITN )));

    //
    // Enable the clocks for AUX_DDI0_OSC and AUX_ADI4
    //
    *(reg32_t*)(AUX_WUC_BASE + AUX_WUC_O_MODCLKEN0) = AUX_WUC_MODCLKEN0_AUX_DDI0_OSC | AUX_WUC_MODCLKEN0_AUX_ADI4;

    //
    // It's found to be optimal to override the FCFG1..DCDC_IPEAK setting as follows:
    // if ( alternative DCDC setting in CCFG is enabled )  ADI3..IPEAK = CCFG..DCDC_IPEAK
    // else                                                ADI3..IPEAK = 2
    //
    if (( *(reg32_t*)( CCFG_BASE + CCFG_O_SIZE_AND_DIS_FLAGS ) & CCFG_SIZE_AND_DIS_FLAGS_DIS_ALT_DCDC_SETTING ) == 0 ) {
        //
        // ADI_3_REFSYS:DCDCCTL5[3]  (=DITHER_EN) = CCFG_MODE_CONF_1[19]   (=ALT_DCDC_DITHER_EN)
        // ADI_3_REFSYS:DCDCCTL5[2:0](=IPEAK    ) = CCFG_MODE_CONF_1[18:16](=ALT_DCDC_IPEAK    )
        // Using a single 4-bit masked write since layout is equal for both source and destination
        //
//         HWREGB( ADI3_BASE + ADI_O_MASK4B + ( ADI_3_REFSYS_O_DCDCCTL5 * 2 )) = ( 0xF0 | ( *(reg32_t*)( CCFG_BASE + CCFG_O_MODE_CONF_1 ) >> CCFG_MODE_CONF_1_ALT_DCDC_IPEAK_S ));

    }

    //
    // Enable for JTAG to be powered down (will still be powered on if debugger is connected)
    //
    *(reg32_t*)(AON_WUC_BASE + AON_WUC_O_JTAGCFG) = 0;

    //
    // read the MODE_CONF register in CCFG
    //
    ccfg_ModeConfReg = *(reg32_t*)( CCFG_BASE + CCFG_O_MODE_CONF );

    //
    // Adjust the VDDR_TRIM_SLEEP value with value adjustable by customer (CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA)
    //
    i32VddrSleepTrim = SignExtendVddrTrimValue(( *(reg32_t*)( FCFG1_BASE + FCFG1_O_LDO_TRIM )
                     & FCFG1_LDO_TRIM_VDDR_TRIM_SLEEP_M ) >> FCFG1_LDO_TRIM_VDDR_TRIM_SLEEP_S );
    // Read and sign extend VddrSleepDelta (in range -8 to +7)
    i32VddrSleepDelta = ((((int32_t)ccfg_ModeConfReg )
                     << ( 32 - CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_WIDTH - CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_S ))
                     >> ( 32 - CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_WIDTH ));
    // Calculate new VDDR sleep trim
    i32VddrSleepTrim = ( i32VddrSleepTrim + i32VddrSleepDelta + 1 );
    if ( i32VddrSleepTrim < -10 ) {
        i32VddrSleepTrim = -10;
    }
    // Write adjusted value using MASKED write (MASK8)
//     HWREGH( ADI3_BASE + ADI_O_MASK8B + ( ADI_3_REFSYS_O_DCDCCTL1 * 2 )) = (( ADI_3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_M << 8 ) | (( i32VddrSleepTrim << ADI_3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_S ) & ADI_3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_M ));

    //
    // set the RECHARGE source based upon CCFG:MODE_CONF:DCDC_RECHARGE
    // Note: Inverse polarity
    //
    *(reg32_t*)( AON_SYSCTL_BASE + AON_SYSCTL_O_PWRCTL) = (*(reg32_t*)( AON_SYSCTL_BASE + AON_SYSCTL_O_PWRCTL) & ~(AON_SYSCTL_PWRCTL_DCDC_EN_M)) | ((( ccfg_ModeConfReg >> CCFG_MODE_CONF_DCDC_RECHARGE_S ) & 1 ) ^ 1 ) << AON_SYSCTL_PWRCTL_DCDC_EN_S;

    //
    // set the ACTIVE source based upon CCFG:MODE_CONF:DCDC_ACTIVE
    // Note: Inverse polarity
    //
    *(reg32_t*)(AON_SYSCTL_BASE + AON_SYSCTL_O_PWRCTL) = (*(reg32_t*)(AON_SYSCTL_BASE + AON_SYSCTL_O_PWRCTL) & ~(AON_SYSCTL_PWRCTL_DCDC_ACTIVE_M)) | ((( ccfg_ModeConfReg >> CCFG_MODE_CONF_DCDC_ACTIVE_S ) & 1 ) ^ 1 ) << AON_SYSCTL_PWRCTL_DCDC_ACTIVE_S;

    //
    // Following sequence is required for using XOSCHF, if not included
    // devices crashes when trying to switch to XOSCHF.
    //
    // Trim CAP settings. Get and set trim value for the ANABYPASS_VALUE1
    // register
    ui32Trim = GetTrimForAnabypassValue1( ccfg_ModeConfReg );
    aux_write(AUX_DDI0_OSC_BASE + DDI_0_OSC_O_ANABYPASSVAL1, ui32Trim);

    // Trim RCOSC_LF. Get and set trim values for the RCOSCLF_RTUNE_TRIM and
    // RCOSCLF_CTUNE_TRIM fields in the XOSCLF_RCOSCLF_CTRL register.
    ui32Trim = GetTrimForRcOscLfRtuneCtuneTrim();
    DDI_0_OSC->LFOSCCTL = (DDI_0_OSC->LFOSCCTL & ~(DDI_0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_M | DDI_0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_S)) | (ui32Trim << DDI_0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_S);

    // Trim XOSCHF IBIAS THERM. Get and set trim value for the
    // XOSCHF IBIAS THERM bit field in the ANABYPASS_VALUE2 register. Other
    // register bit fields are set to 0.
    ui32Trim = GetTrimForXoscHfIbiastherm();
    aux_write(AUX_DDI0_OSC_BASE + DDI_0_OSC_O_ANABYPASSVAL2,
                  ui32Trim<<DDI_0_OSC_ANABYPASSVAL2_XOSC_HF_IBIASTHERM_S);

    // Trim AMPCOMP settings required before switch to XOSCHF
    ui32Trim = GetTrimForAmpcompTh2();
    aux_write(AUX_DDI0_OSC_BASE + DDI_0_OSC_O_AMPCOMPTH2, ui32Trim);
    ui32Trim = GetTrimForAmpcompTh1();
    aux_write(AUX_DDI0_OSC_BASE + DDI_0_OSC_O_AMPCOMPTH1, ui32Trim);
    ui32Trim = GetTrimForAmpcompCtrl( ui32Fcfg1Revision );
    aux_write(AUX_DDI0_OSC_BASE + DDI_0_OSC_O_AMPCOMPCTL, ui32Trim);

    //
    // Set trim for DDI_0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_MODE_EN in accordance to FCFG1 setting
    // This is bit[5] in the DDI_0_OSC_O_ADCDOUBLERNANOAMPCTL register
    // Using MASK4 write + 1 => writing to bits[7:4]
    //
    ui32Trim = GetTrimForAdcShModeEn( ui32Fcfg1Revision );
    if (ui32Trim) {
        ui32Trim = 1;
    }
    DDI_0_OSC->ADCDOUBLERNANOAMPCTL = (DDI_0_OSC->ADCDOUBLERNANOAMPCTL & ~(DDI_0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_MODE_EN_M)) | (ui32Trim << DDI_0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_MODE_EN_S);

    //
    // Set trim for DDI_0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_VBUF_EN in accordance to FCFG1 setting
    // This is bit[4] in the DDI_0_OSC_O_ADCDOUBLERNANOAMPCTL register
    // Using MASK4 write + 1 => writing to bits[7:4]
    //
    ui32Trim = GetTrimForAdcShVbufEn( ui32Fcfg1Revision );
    if (ui32Trim) {
        ui32Trim = 1;
    }
    DDI_0_OSC->ADCDOUBLERNANOAMPCTL = (DDI_0_OSC->ADCDOUBLERNANOAMPCTL & ~(DDI_0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_VBUF_EN_M)) | (ui32Trim << DDI_0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_VBUF_EN_S);

    //
    // Set trim for the PEAK_DET_ITRIM, HP_BUF_ITRIM and LP_BUF_ITRIM bit fields
    // in the DDI0_OSC_O_XOSCHFCTL register in accordance to FCFG1 setting.
    // Remaining register bit fields are set to their reset values of 0.
    //
    ui32Trim = GetTrimForXoscHfCtl(ui32Fcfg1Revision);
    *(reg32_t*)(AUX_DDI0_OSC_BASE + DDI_0_OSC_O_XOSCHFCTL) = ui32Trim;

    //
    // Set trim for DBLR_LOOP_FILTER_RESET_VOLTAGE in accordance to FCFG1 setting
    // (This is bits [18:17] in DDI_0_OSC_O_ADCDOUBLERNANOAMPCTL)
    // (Using MASK4 write + 4 => writing to bits[19:16] => (4*4))
    // (Assuming: DDI_0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_LOOP_FILTER_RESET_VOLTAGE_S = 17 and
    //  that DDI_0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_LOOP_FILTER_RESET_VOLTAGE_M = 0x00060000)
    //
    ui32Trim = GetTrimForDblrLoopFilterResetVoltage( ui32Fcfg1Revision );
//     HWREGB( AUX_DDI0_OSC_BASE + DDI_O_MASK4B + ( DDI_0_OSC_O_ADCDOUBLERNANOAMPCTL * 2 ) + 4 ) = ( 0x60 | ( ui32Trim << 1 ));

    //
    // Update DDI_0_OSC_ATESTCTL_ATESTLF_RCOSCLF_IBIAS_TRIM with data from
    // FCFG1_OSC_CONF_ATESTLF_RCOSCLF_IBIAS_TRIM
    // This is DDI_0_OSC_O_ATESTCTL bit[7]
    // ( DDI_0_OSC_O_ATESTCTL is currently hidden (but=0x00000020))
    // Using MASK4 write + 1 => writing to bits[7:4]
    //
    ui32Trim = GetTrimForRcOscLfIBiasTrim( ui32Fcfg1Revision );
//     HWREGB( AUX_DDI0_OSC_BASE + DDI_O_MASK4B + ( 0x00000020 * 2 ) + 1 ) = ( 0x80 | ( ui32Trim << 3 ));

    //
    // Update DDI_0_OSC_LFOSCCTL_XOSCLF_REGULATOR_TRIM and
    //        DDI_0_OSC_LFOSCCTL_XOSCLF_CMIRRWR_RATIO in one write
    // This can be simplified since the registers are packed together in the same
    // order both in FCFG1 and in the HW register.
    // This spans DDI_0_OSC_O_LFOSCCTL bits[23:18]
    // Using MASK8 write + 4 => writing to bits[23:16]
    //
    ui32Trim = GetTrimForXoscLfRegulatorAndCmirrwrRatio( ui32Fcfg1Revision );
//     HWREGH( AUX_DDI0_OSC_BASE + DDI_O_MASK8B + ( DDI_0_OSC_O_LFOSCCTL * 2 ) + 4 ) = ( 0xFC00 | ( ui32Trim << 2 ));

    //
    // Set trim the HPM_IBIAS_WAIT_CNT, LPM_IBIAS_WAIT_CNT and IDAC_STEP bit
    // fields in the DDI0_OSC_O_RADCEXTCFG register in accordance to FCFG1 setting.
    // Remaining register bit fields are set to their reset values of 0.
    //
    ui32Trim = GetTrimForRadcExtCfg(ui32Fcfg1Revision);
    *(reg32_t*)(AUX_DDI0_OSC_BASE + DDI_0_OSC_O_RADCEXTCFG) = ui32Trim;

    // Setting FORCE_KICKSTART_EN (ref. CC26_V1_BUG00261). Should also be done for PG2
    // (This is bit 22 in DDI_0_OSC_O_CTL0)
    DDI_0_OSC->CTL0 |= DDI_0_OSC_CTL0_FORCE_KICKSTART_EN;

    // XOSC source is a 24 MHz xtal (default)
    // Set bit DDI_0_OSC_CTL0_XTAL_IS_24M (this is bit 31 in DDI_0_OSC_O_CTL0)
    DDI_0_OSC->CTL0 |= DDI_0_OSC_CTL0_XTAL_IS_24M_24M;

    // Setting DDI_0_OSC_CTL1_XOSC_HF_FAST_START according to value found in FCFG1
    ui32Trim = GetTrimForXoscHfFastStart();
    DDI_0_OSC->CTL1 = (DDI_0_OSC->CTL1 & ~(DDI_0_OSC_CTL1_XOSC_HF_FAST_START_M)) | (ui32Trim << DDI_0_OSC_CTL1_XOSC_HF_FAST_START_S);

    //
    // setup the LF clock based upon CCFG:MODE_CONF:SCLK_LF_OPTION
    //
    switch (( ccfg_ModeConfReg & CCFG_MODE_CONF_SCLK_LF_OPTION_M ) >> CCFG_MODE_CONF_SCLK_LF_OPTION_S ) {
        case 0 : // XOSC_HF_DLF (XOSCHF/1536) -> SCLK_LF (=31250Hz)
            DDI_0_OSC->CTL0 |= DDI_0_OSC_CTL0_SCLK_LF_SRC_SEL_XOSCHFDLF;

            /* set rtc for some frequency other than 32768 using this magic number */
            *(reg32_t*)(AON_RTC_BASE + AON_RTC_O_SUBSECINC) = 0x8637BD;

            break;
        case 1 : // EXTERNAL signal -> SCLK_LF (frequency=2^38/CCFG_EXT_LF_CLK_RTC_INCREMENT)
            // Set SCLK_LF to use the same source as SCLK_HF
            // Can be simplified a bit since possible return values for HF matches LF settings
            currentHfClock = (*(reg32_t*)DDI_0_OSC->STAT0 & DDI_0_OSC_STAT0_SCLK_HF_SRC_M) >> DDI_0_OSC_STAT0_SCLK_HF_SRC_S;
            DDI_0_OSC->CTL0 &= ~(DDI_0_OSC_CTL0_SCLK_LF_SRC_SEL_M) | (currentHfClock << DDI_0_OSC_CTL0_SCLK_LF_SRC_SEL_S);
            while ((*(reg32_t*)DDI_0_OSC->STAT0 & DDI_0_OSC_STAT0_SCLK_HF_SRC_M) >> DDI_0_OSC_STAT0_SCLK_HF_SRC_S != currentHfClock ) {
                // Wait until switched
            }
            ccfgExtLfClk = *(reg32_t*)( CCFG_BASE + CCFG_O_EXT_LF_CLK );
            *(reg32_t*)(AON_RTC_BASE + AON_RTC_O_SUBSECINC) = (( ccfgExtLfClk & CCFG_EXT_LF_CLK_RTC_INCREMENT_M ) >> CCFG_EXT_LF_CLK_RTC_INCREMENT_S );
//             IOCPortConfigureSet(( ccfgExtLfClk & CCFG_EXT_LF_CLK_DIO_M ) >> CCFG_EXT_LF_CLK_DIO_S,
//                                 IOC_PORT_AON_CLK32K,
//                                 IOC_STD_INPUT | IOC_HYST_ENABLE );   // Route external clock to AON IOC w/hysteresis
            // Set XOSC_LF in bypass mode to allow external 32k clock
            *(reg32_t*)( AUX_DDI0_OSC_BASE + DDI_O_SET + DDI_0_OSC_O_CTL0 ) = DDI_0_OSC_CTL0_XOSC_LF_DIG_BYPASS;
            // Fall through to set XOSC_LF as SCLK_LF source
        case 2 : // XOSC_LF -> SLCK_LF (32768 Hz)
            DDI_0_OSC->CTL0 = (DDI_0_OSC->CTL0 & ~(DDI_0_OSC_CTL0_SCLK_LF_SRC_SEL_M)) | DDI_0_OSC_CTL0_SCLK_LF_SRC_SEL_XOSCLF;
            break;
        default : // (=3) RCOSC_LF
            DDI_0_OSC->CTL0 = (DDI_0_OSC->CTL0 & ~(DDI_0_OSC_CTL0_SCLK_LF_SRC_SEL_M)) | DDI_0_OSC_CTL0_SCLK_LF_SRC_SEL_RCOSCLF;
            break;
    }

    //
    // Update ADI_4_AUX_ADCREF1_VTRIM with value from FCFG1
    //
    *(reg8_t*)( AUX_ADI4_BASE + ADI_4_AUX_O_ADCREF1 ) =
        ((( *(reg32_t*)( FCFG1_BASE + FCFG1_O_SOC_ADC_REF_TRIM_AND_OFFSET_EXT )
        >> FCFG1_SOC_ADC_REF_TRIM_AND_OFFSET_EXT_SOC_ADC_REF_VOLTAGE_TRIM_TEMP1_S )
        << ADI_4_AUX_ADCREF1_VTRIM_S )
        & ADI_4_AUX_ADCREF1_VTRIM_M );

    //
    // Set ADI_4_AUX:ADC0.SMPL_CYCLE_EXP to it's default minimum value (=3)
    // (Note: Using MASK8B requires that the bits to be modified must be within the same
    //        byte boundary which is the case for the ADI_4_AUX_ADC0_SMPL_CYCLE_EXP field)
    //
    *(reg32_t*)(AUX_ADI4_BASE + ADI_4_AUX_O_ADC0) = (*(reg32_t*)(AUX_ADI4_BASE + ADI_4_AUX_O_ADC0) & ~(ADI_4_AUX_ADC0_SMPL_CYCLE_EXP_M)) | ADI_4_AUX_ADC0_SMPL_CYCLE_EXP_2P7_US;

    //
    // Sync with AON
    //
    *(reg32_t*)(AON_RTC_BASE + AON_RTC_O_SYNC);

    //
    // Allow AUX to power down
    //
    *(reg32_t*)(AUX_WUC_BASE + AUX_WUC_O_PWROFFREQ) = 0x0;
    *(reg32_t*)(AUX_WUC_BASE + AUX_WUC_O_PWRDWNREQ) = AUX_WUC_PWRDWNREQ_REQ;
    *(reg32_t*)(AUX_WUC_BASE + AUX_WUC_O_MCUBUSCTL) = AUX_WUC_MCUBUSCTL_DISCONNECT_REQ;

    //
    // Leaving on AUX and clock for AUX_DDI0_OSC on but turn off clock for AUX_ADI4
    //
    *(reg32_t*)( AUX_WUC_BASE + AUX_WUC_O_MODCLKEN0 ) = AUX_WUC_MODCLKEN0_AUX_DDI0_OSC;

    // Disable EFUSE clock
    *(reg32_t*)( FLASH_BASE + FLASH_O_CFG) |= FLASH_CFG_DIS_EFUSECLK;
}

//*****************************************************************************
//
//! \brief Sign extend the VDDR_TRIM setting (special format ranging from -10 to +21)
//!
//! \return
//
//*****************************************************************************
static int32_t SignExtendVddrTrimValue(uint32_t ui32VddrTrimVal)
{
    //
    // The VDDR trim value is 5 bits representing the range from -10 to +21
    // (where -10=0x16, -1=0x1F, 0=0x00, 1=0x01 and +21=0x15)
    //
    int32_t i32SignedVddrVal = ui32VddrTrimVal;
    if ( i32SignedVddrVal > 0x15 ) {
        i32SignedVddrVal -= 0x20;
    }
    return ( i32SignedVddrVal );
}

// #define FCFG1_O_CONFIG_OSC_TOP (0x350)
// #define FCFG1_CONFIG_OSC_TOP_XOSC_HF_ROW_Q12_M (0x3c000000)
// #define FCFG1_CONFIG_OSC_TOP_XOSC_HF_ROW_Q12_S (26)
// #define FCFG1_CONFIG_OSC_TOP_XOSC_HF_COLUMN_Q12_M (0x03fffc00)
// #define FCFG1_CONFIG_OSC_TOP_XOSC_HF_COLUMN_Q12_S (10)
// #define CCFG_MODE_CONF_XOSC_CAP_MOD (0x00020000)
// #define DDI_0_OSC_ANABYPASSVAL1_XOSC_HF_ROW_Q12_S (16)
// #define DDI_0_OSC_ANABYPASSVAL1_XOSC_HF_COLUMN_Q12_S (0)


static uint32_t GetTrimForAnabypassValue1( uint32_t ccfg_ModeConfReg )
{
    uint32_t ui32Fcfg1Value            ;
    uint32_t ui32XoscHfRow             ;
    uint32_t ui32XoscHfCol             ;
    int32_t  i32CustomerDeltaAdjust    ;
    uint32_t ui32TrimValue             ;

    // Use device specific trim values located in factory configuration
    // area for the XOSC_HF_COLUMN_Q12 and XOSC_HF_ROW_Q12 bit fields in
    // the ANABYPASS_VALUE1 register. Value for the other bit fields
    // are set to 0.

    ui32Fcfg1Value = *(reg32_t*)(FCFG1_BASE + FCFG1_O_CONFIG_OSC_TOP);
    ui32XoscHfRow = (( ui32Fcfg1Value &
        FCFG1_CONFIG_OSC_TOP_XOSC_HF_ROW_Q12_M ) >>
        FCFG1_CONFIG_OSC_TOP_XOSC_HF_ROW_Q12_S );
    ui32XoscHfCol = (( ui32Fcfg1Value &
        FCFG1_CONFIG_OSC_TOP_XOSC_HF_COLUMN_Q12_M ) >>
        FCFG1_CONFIG_OSC_TOP_XOSC_HF_COLUMN_Q12_S );

    i32CustomerDeltaAdjust = 0;
    if (( ccfg_ModeConfReg & CCFG_MODE_CONF_XOSC_CAP_MOD ) == 0 ) {
        // XOSC_CAP_MOD = 0 means: CAP_ARRAY_DELTA is in use -> Apply compensation
        // XOSC_CAPARRAY_DELTA is located in bit[15:8] of ccfg_ModeConfReg
        // Note: HW_REV_DEPENDENT_IMPLEMENTATION. Field width is not given by
        // a define and sign extension must therefore be hardcoded.
        // ( A small test program is created verifying the code lines below:
        //   Ref.: ..\test\small_standalone_test_programs\CapArrayDeltaAdjust_test.c)
        i32CustomerDeltaAdjust = ((int32_t)ccfg_ModeConfReg << 16 ) >> 24;

        while ( i32CustomerDeltaAdjust < 0 ) {
            ui32XoscHfCol >>= 1;                              // COL 1 step down
            if ( ui32XoscHfCol == 0 ) {                       // if COL below minimum
                ui32XoscHfCol = 0xFFFF;                       //   Set COL to maximum
                ui32XoscHfRow >>= 1;                          //   ROW 1 step down
                if ( ui32XoscHfRow == 0 ) {                   // if ROW below minimum
                   ui32XoscHfRow = 1;                         //   Set both ROW and COL
                   ui32XoscHfCol = 1;                         //   to minimum
                }
            }
            i32CustomerDeltaAdjust++;
        }
        while ( i32CustomerDeltaAdjust > 0 ) {
            ui32XoscHfCol = ( ui32XoscHfCol << 1 ) | 1;       // COL 1 step up
            if ( ui32XoscHfCol > 0xFFFF ) {                   // if COL above maximum
                ui32XoscHfCol = 1;                            //   Set COL to minimum
                ui32XoscHfRow = ( ui32XoscHfRow << 1 ) | 1;   //   ROW 1 step up
                if ( ui32XoscHfRow > 0xF ) {                  // if ROW above maximum
                   ui32XoscHfRow = 0xF;                       //   Set both ROW and COL
                   ui32XoscHfCol = 0xFFFF;                    //   to maximum
                }
            }
            i32CustomerDeltaAdjust--;
        }
    }

    ui32TrimValue = (( ui32XoscHfRow << DDI_0_OSC_ANABYPASSVAL1_XOSC_HF_ROW_Q12_S    ) |
                     ( ui32XoscHfCol << DDI_0_OSC_ANABYPASSVAL1_XOSC_HF_COLUMN_Q12_S )   );

    return (ui32TrimValue);
}

// #define FCFG1_CONFIG_OSC_TOP_RCOSCLF_CTUNE_TRIM_M (0x3fc)
// #define FCFG1_CONFIG_OSC_TOP_RCOSCLF_CTUNE_TRIM_S (2)
// #define FCFG1_CONFIG_OSC_TOP_RCOSCLF_RTUNE_TRIM_M (0x3)
// #define FCFG1_CONFIG_OSC_TOP_RCOSCLF_RTUNE_TRIM_S (0)
// #define DDI_0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_S (8)

//*****************************************************************************
//
//! \brief Returns the trim value to be used for the RCOSCLF_RTUNE_TRIM and the
//! RCOSCLF_CTUNE_TRIM bit fields in the XOSCLF_RCOSCLF_CTRL register in OSC_DIG.
//
//*****************************************************************************
static uint32_t GetTrimForRcOscLfRtuneCtuneTrim(void)
{
    uint32_t ui32TrimValue;

    // Use device specific trim values located in factory configuration
    // area
    ui32TrimValue =
        ((*(reg32_t*)(FCFG1_BASE + FCFG1_O_CONFIG_OSC_TOP) &
          FCFG1_CONFIG_OSC_TOP_RCOSCLF_CTUNE_TRIM_M)>>
          FCFG1_CONFIG_OSC_TOP_RCOSCLF_CTUNE_TRIM_S)<<
            DDI_0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_S;

    ui32TrimValue |=
        ((*(reg32_t*)(FCFG1_BASE + FCFG1_O_CONFIG_OSC_TOP) &
          FCFG1_CONFIG_OSC_TOP_RCOSCLF_RTUNE_TRIM_M)>>
          FCFG1_CONFIG_OSC_TOP_RCOSCLF_RTUNE_TRIM_S)<<
            DDI_0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_S;

    return(ui32TrimValue);
}

// #define FCFG1_O_ANABYPASS_VALUE2 (0x37c)
// #define FCFG1_ANABYPASS_VALUE2_XOSC_HF_IBIASTHERM_M (0x3fff)
// #define FCFG1_ANABYPASS_VALUE2_XOSC_HF_IBIASTHERM_S (0)

//*****************************************************************************
//
//! \brief Returns the trim value to be used for the XOSC_HF_IBIASTHERM bit field in
//! the ANABYPASS_VALUE2 register in OSC_DIG.
//
//*****************************************************************************
static uint32_t GetTrimForXoscHfIbiastherm(void)
{
    uint32_t ui32TrimValue;

    // Use device specific trim value located in factory configuration
    // area
    ui32TrimValue =
        (*(reg32_t*)(FCFG1_BASE + FCFG1_O_ANABYPASS_VALUE2) &
         FCFG1_ANABYPASS_VALUE2_XOSC_HF_IBIASTHERM_M)>>
         FCFG1_ANABYPASS_VALUE2_XOSC_HF_IBIASTHERM_S;

    return(ui32TrimValue);
}

// #define FCFG1_O_AMPCOMP_TH2 (0x374)
// #define FCFG1_AMPCOMP_TH2_LPMUPDATE_LTH_M (0xfc000000)
// #define FCFG1_AMPCOMP_TH2_LPMUPDATE_LTH_S (26)
// #define DDI_0_OSC_AMPCOMPTH2_LPMUPDATE_LTH_S (26)
// #define FCFG1_AMPCOMP_TH2_LPMUPDATE_HTM_M (0x00fc0000)
// #define FCFG1_AMPCOMP_TH2_LPMUPDATE_HTM_S (18)
// #define DDI_0_OSC_AMPCOMPTH2_LPMUPDATE_HTH_S (18)
// #define FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_LPM_M (0x0000fc00)
// #define FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_LPM_S (10)
// #define DDI_0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_LPM_S (10)
// #define FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_HPM_M (0x000000fc)
// #define FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_HPM_S (2)
// #define DDI_0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_HPM_S (2)

//*****************************************************************************
//
//! \brief Returns the trim value to be used for the AMPCOMP_TH2 register in OSC_DIG.
//
//*****************************************************************************
static uint32_t GetTrimForAmpcompTh2(void)
{
    uint32_t ui32TrimValue;
    uint32_t ui32Fcfg1Value;

    // Use device specific trim value located in factory configuration
    // area. All defined register bit fields have corresponding trim
    // value in the factory configuration area
    ui32Fcfg1Value = *(reg32_t*)(FCFG1_BASE + FCFG1_O_AMPCOMP_TH2);
    ui32TrimValue = ((ui32Fcfg1Value &
                      FCFG1_AMPCOMP_TH2_LPMUPDATE_LTH_M)>>
                      FCFG1_AMPCOMP_TH2_LPMUPDATE_LTH_S)<<
                   DDI_0_OSC_AMPCOMPTH2_LPMUPDATE_LTH_S;
    ui32TrimValue |= (((ui32Fcfg1Value &
                        FCFG1_AMPCOMP_TH2_LPMUPDATE_HTM_M)>>
                        FCFG1_AMPCOMP_TH2_LPMUPDATE_HTM_S)<<
                     DDI_0_OSC_AMPCOMPTH2_LPMUPDATE_HTH_S);
    ui32TrimValue |= (((ui32Fcfg1Value &
                        FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_LPM_M)>>
                        FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_LPM_S)<<
                     DDI_0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_LPM_S);
    ui32TrimValue |= (((ui32Fcfg1Value &
                        FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_HPM_M)>>
                        FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_HPM_S)<<
                     DDI_0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_HPM_S);

    return(ui32TrimValue);
}

// #define FCFG1_O_AMPCOMP_TH1 (0x370)
// #define FCFG1_AMPCOMP_TH1_HPMRAMP3_LTH_M (0x00fc0000)
// #define FCFG1_AMPCOMP_TH1_HPMRAMP3_LTH_S (18)
// #define DDI_0_OSC_AMPCOMPTH1_HPMRAMP3_LTH_S (18)
// #define FCFG1_AMPCOMP_TH1_HPMRAMP3_HTH_M (0x0000fc00)
// #define FCFG1_AMPCOMP_TH1_HPMRAMP3_HTH_S (10)
// #define DDI_0_OSC_AMPCOMPTH1_HPMRAMP3_HTH_S (10)
// #define FCFG1_AMPCOMP_TH1_IBIASCAP_LPTOHP_OL_CNT_M (0x3c0)
// #define FCFG1_AMPCOMP_TH1_IBIASCAP_LPTOHP_OL_CNT_S (6)
// #define DDI_0_OSC_AMPCOMPTH1_IBIASCAP_LPTOHP_OL_CNT_S (6)
// #define FCFG1_AMPCOMP_TH1_HPMRAMP1_TH_M (0x3f)
// #define FCFG1_AMPCOMP_TH1_HPMRAMP1_TH_S (0)
// #define DDI_0_OSC_AMPCOMPTH1_HPMRAMP1_TH_S (0)

//*****************************************************************************
//
//! \brief Returns the trim value to be used for the AMPCOMP_TH1 register in OSC_DIG.
//
//*****************************************************************************
static uint32_t GetTrimForAmpcompTh1(void)
{
    uint32_t ui32TrimValue;
    uint32_t ui32Fcfg1Value;

    // Use device specific trim values located in factory configuration
    // area. All defined register bit fields have a corresponding trim
    // value in the factory configuration area
    ui32Fcfg1Value = *(reg32_t*)(FCFG1_BASE + FCFG1_O_AMPCOMP_TH1);
    ui32TrimValue = (((ui32Fcfg1Value &
                        FCFG1_AMPCOMP_TH1_HPMRAMP3_LTH_M)>>
                        FCFG1_AMPCOMP_TH1_HPMRAMP3_LTH_S)<<
                     DDI_0_OSC_AMPCOMPTH1_HPMRAMP3_LTH_S);
    ui32TrimValue |= (((ui32Fcfg1Value &
                        FCFG1_AMPCOMP_TH1_HPMRAMP3_HTH_M)>>
                        FCFG1_AMPCOMP_TH1_HPMRAMP3_HTH_S)<<
                     DDI_0_OSC_AMPCOMPTH1_HPMRAMP3_HTH_S);
    ui32TrimValue |= (((ui32Fcfg1Value &
                        FCFG1_AMPCOMP_TH1_IBIASCAP_LPTOHP_OL_CNT_M)>>
                        FCFG1_AMPCOMP_TH1_IBIASCAP_LPTOHP_OL_CNT_S)<<
                     DDI_0_OSC_AMPCOMPTH1_IBIASCAP_LPTOHP_OL_CNT_S);
    ui32TrimValue |= (((ui32Fcfg1Value &
                        FCFG1_AMPCOMP_TH1_HPMRAMP1_TH_M)>>
                        FCFG1_AMPCOMP_TH1_HPMRAMP1_TH_S)<<
                     DDI_0_OSC_AMPCOMPTH1_HPMRAMP1_TH_S);

    return(ui32TrimValue);
}

// #define FCFG1_O_AMPCOMP_CTRL1 (0x378)
// #define FCFG1_AMPCOMP_CTRL1_IBIAS_OFFSET_M (0x00f00000)
// #define FCFG1_AMPCOMP_CTRL1_IBIAS_OFFSET_S (20)
// #define FCFG1_AMPCOMP_CTRL1_IBIAS_INIT_M (0x000f0000)
// #define FCFG1_AMPCOMP_CTRL1_IBIAS_INIT_S (16)
// #define CCFG_BASE (0x50003000)
// #define CCFG_O_SIZE_AND_DIS_FLAGS (0xfb0)
// #define CCFG_SIZE_AND_DIS_FLAGS_DIS_XOSC_OVR_M (0x1)
// #define CCFG_O_MODE_CONF_1 (0xfac)
// #define CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET_S (8)
// #define DDI_0_OSC_AMPCOMPCTL_IBIAS_OFFSET_M (0x00f00000)
// #define DDI_0_OSC_AMPCOMPCTL_IBIAS_OFFSET_S (20)
// #define CCFG_MODE_CONF_1_DELTA_IBIAS_INIT_S (12)
// #define DDI_0_OSC_AMPCOMPCTL_IBIAS_INIT_M (0x000f0000)
// #define DDI_0_OSC_AMPCOMPCTL_IBIAS_INIT_S (16)
// #define FCFG1_AMPCOMP_CTRL1_LPM_IBIAS_WAIT_CNT_FINAL_M (0x0000ff00)
// #define FCFG1_AMPCOMP_CTRL1_LPM_IBIAS_WAIT_CNT_FINAL_S (8)
// #define DDI_0_OSC_AMPCOMPCTL_LPM_IBIAS_WAIT_CNT_FINAL_S (8)
// #define FCFG1_AMPCOMP_CTRL1_CAP_STEP_M (0xf0)
// #define FCFG1_AMPCOMP_CTRL1_CAP_STEP_S (4)
// #define DDI_0_OSC_AMPCOMPCTL_CAP_STEP_S (4)
// #define FCFG1_AMPCOMP_CTRL1_IBIASCAP_HPTOLP_OL_CNT_M (0xf)
// #define FCFG1_AMPCOMP_CTRL1_IBIASCAP_HPTOLP_OL_CNT_S (0)
// #define DDI_0_OSC_AMPCOMPCTL_IBIASCAP_HPTOLP_OL_CNT_S (0)
// #define FCFG1_AMPCOMP_CTRL1_AMPCOMP_REQ_MODE_M (0x40000000)
// #define FCFG1_AMPCOMP_CTRL1_AMPCOMP_REQ_MODE_S (30)
// #define DDI_0_OSC_AMPCOMPCTL_AMPCOMP_REQ_MODE_S (30)

//*****************************************************************************
//
//! \brief Returns the trim value to be used for the AMPCOMP_CTRL register in OSC_DIG.
//
//*****************************************************************************
static uint32_t GetTrimForAmpcompCtrl(uint32_t ui32Fcfg1Revision)
{
    uint32_t ui32TrimValue    ;
    uint32_t ui32Fcfg1Value   ;
    uint32_t ibiasOffset      ;
    uint32_t ibiasInit        ;
    uint32_t modeConf1        ;
    int32_t  deltaAdjust      ;

    // Use device specific trim values located in factory configuration
    // area. Register bit fields without trim values in the factory
    // configuration area will be set to the value of 0.
    ui32Fcfg1Value = *(reg32_t*)( FCFG1_BASE + FCFG1_O_AMPCOMP_CTRL1 );

    ibiasOffset    = ( ui32Fcfg1Value &
                       FCFG1_AMPCOMP_CTRL1_IBIAS_OFFSET_M ) >>
                       FCFG1_AMPCOMP_CTRL1_IBIAS_OFFSET_S ;
    ibiasInit      = ( ui32Fcfg1Value &
                       FCFG1_AMPCOMP_CTRL1_IBIAS_INIT_M ) >>
                       FCFG1_AMPCOMP_CTRL1_IBIAS_INIT_S ;

    if (( *(reg32_t*)( CCFG_BASE + CCFG_O_SIZE_AND_DIS_FLAGS ) & CCFG_SIZE_AND_DIS_FLAGS_DIS_XOSC_OVR_M ) == 0 ) {
        // Adjust with DELTA_IBIAS_OFFSET and DELTA_IBIAS_INIT from CCFG
        modeConf1   = *(reg32_t*)( CCFG_BASE + CCFG_O_MODE_CONF_1 );

        // Both fields are signed 4-bit values. This is an assumption when doing the sign extension.
        deltaAdjust = ((int32_t)modeConf1 << ( 32 - CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET_S - 4 )) >> 28;
        deltaAdjust += (int32_t)ibiasOffset;
        if ( deltaAdjust < 0 ) {
           deltaAdjust = 0;
        }
        if ( deltaAdjust > ( DDI_0_OSC_AMPCOMPCTL_IBIAS_OFFSET_M >> DDI_0_OSC_AMPCOMPCTL_IBIAS_OFFSET_S )) {
            deltaAdjust  = ( DDI_0_OSC_AMPCOMPCTL_IBIAS_OFFSET_M >> DDI_0_OSC_AMPCOMPCTL_IBIAS_OFFSET_S );
        }
        ibiasOffset = (uint32_t)deltaAdjust;

        deltaAdjust = ((int32_t)modeConf1 << ( 32 - CCFG_MODE_CONF_1_DELTA_IBIAS_INIT_S - 4 )) >> 28;
        deltaAdjust += (int32_t)ibiasInit;
        if ( deltaAdjust < 0 ) {
           deltaAdjust = 0;
        }
        if ( deltaAdjust > ( DDI_0_OSC_AMPCOMPCTL_IBIAS_INIT_M >> DDI_0_OSC_AMPCOMPCTL_IBIAS_INIT_S )) {
            deltaAdjust  = ( DDI_0_OSC_AMPCOMPCTL_IBIAS_INIT_M >> DDI_0_OSC_AMPCOMPCTL_IBIAS_INIT_S );
        }
        ibiasInit = (uint32_t)deltaAdjust;
    }
    ui32TrimValue = ( ibiasOffset << DDI_0_OSC_AMPCOMPCTL_IBIAS_OFFSET_S ) |
                    ( ibiasInit   << DDI_0_OSC_AMPCOMPCTL_IBIAS_INIT_S   ) ;

    ui32TrimValue |= (((ui32Fcfg1Value &
                        FCFG1_AMPCOMP_CTRL1_LPM_IBIAS_WAIT_CNT_FINAL_M)>>
                        FCFG1_AMPCOMP_CTRL1_LPM_IBIAS_WAIT_CNT_FINAL_S)<<
                       DDI_0_OSC_AMPCOMPCTL_LPM_IBIAS_WAIT_CNT_FINAL_S);
    ui32TrimValue |= (((ui32Fcfg1Value &
                        FCFG1_AMPCOMP_CTRL1_CAP_STEP_M)>>
                        FCFG1_AMPCOMP_CTRL1_CAP_STEP_S)<<
                       DDI_0_OSC_AMPCOMPCTL_CAP_STEP_S);
    ui32TrimValue |= (((ui32Fcfg1Value &
                        FCFG1_AMPCOMP_CTRL1_IBIASCAP_HPTOLP_OL_CNT_M)>>
                        FCFG1_AMPCOMP_CTRL1_IBIASCAP_HPTOLP_OL_CNT_S)<<
                       DDI_0_OSC_AMPCOMPCTL_IBIASCAP_HPTOLP_OL_CNT_S);

    if ( ui32Fcfg1Revision >= 0x00000022 ) {
        ui32TrimValue |= ((( ui32Fcfg1Value &
            FCFG1_AMPCOMP_CTRL1_AMPCOMP_REQ_MODE_M ) >>
            FCFG1_AMPCOMP_CTRL1_AMPCOMP_REQ_MODE_S ) <<
           DDI_0_OSC_AMPCOMPCTL_AMPCOMP_REQ_MODE_S );
    }

    return(ui32TrimValue);
}

#define AUX_SMPH_O_SMPH0 (0)

static void aux_write(uint32_t address, uint32_t data)
{
    while (!*(reg32_t*)(AUX_SMPH_BASE + AUX_SMPH_O_SMPH0));
    *(reg32_t*)address = data;
    *(reg32_t*)(AUX_SMPH_BASE + AUX_SMPH_O_SMPH0) = 1;
}

//*****************************************************************************
//
//! \brief Returns the trim value from FCFG1 to be used as DBLR_LOOP_FILTER_RESET_VOLTAGE setting.
//
//*****************************************************************************
static uint32_t GetTrimForDblrLoopFilterResetVoltage( uint32_t ui32Fcfg1Revision )
{
   uint32_t dblrLoopFilterResetVoltageValue = 0; // Reset value

   if ( ui32Fcfg1Revision >= 0x00000020 ) {
      dblrLoopFilterResetVoltageValue = ( *(reg32_t*)( FCFG1_BASE + FCFG1_O_MISC_OTP_DATA_1 ) &
         FCFG1_MISC_OTP_DATA_1_DBLR_LOOP_FILTER_RESET_VOLTAGE_M ) >>
         FCFG1_MISC_OTP_DATA_1_DBLR_LOOP_FILTER_RESET_VOLTAGE_S;
   }

   return ( dblrLoopFilterResetVoltageValue );
}

//*****************************************************************************
//
//! \brief Returns the trim value from FCFG1 to be used as ADC_SH_MODE_EN setting.
//
//*****************************************************************************
static uint32_t GetTrimForAdcShModeEn(uint32_t ui32Fcfg1Revision)
{
   uint32_t getTrimForAdcShModeEnValue = 1; // Recommended default setting

   if ( ui32Fcfg1Revision >= 0x00000022 ) {
      getTrimForAdcShModeEnValue = ( *(reg32_t*)( FCFG1_BASE + FCFG1_O_OSC_CONF ) &
         FCFG1_OSC_CONF_ADC_SH_MODE_EN_M ) >>
         FCFG1_OSC_CONF_ADC_SH_MODE_EN_S;
   }

   return ( getTrimForAdcShModeEnValue );
}

//*****************************************************************************
//
//! \brief Returns the trim value from FCFG1 to be used as ADC_SH_VBUF_EN setting.
//
//*****************************************************************************
static uint32_t GetTrimForAdcShVbufEn( uint32_t ui32Fcfg1Revision )
{
   uint32_t getTrimForAdcShVbufEnValue = 1; // Recommended default setting

   if ( ui32Fcfg1Revision >= 0x00000022 ) {
      getTrimForAdcShVbufEnValue = ( *(reg32_t*)( FCFG1_BASE + FCFG1_O_OSC_CONF ) &
         FCFG1_OSC_CONF_ADC_SH_VBUF_EN_M ) >>
         FCFG1_OSC_CONF_ADC_SH_VBUF_EN_S;
   }

   return ( getTrimForAdcShVbufEnValue );
}

//*****************************************************************************
//
//! \brief Returns the trim value to be used for the XOSCHFCTL register in OSC_DIG.
//
//*****************************************************************************
static uint32_t GetTrimForXoscHfCtl(uint32_t ui32Fcfg1Revision)
{
   uint32_t getTrimForXoschfCtlValue = 0; // Recommended default setting
   uint32_t fcfg1Data;

   if ( ui32Fcfg1Revision >= 0x00000020 ) {
      fcfg1Data = *(reg32_t*)( FCFG1_BASE + FCFG1_O_MISC_OTP_DATA_1 );
      getTrimForXoschfCtlValue =
         ( ( ( fcfg1Data & FCFG1_MISC_OTP_DATA_1_PEAK_DET_ITRIM_M ) >>
             FCFG1_MISC_OTP_DATA_1_PEAK_DET_ITRIM_S ) <<
           DDI_0_OSC_XOSCHFCTL_PEAK_DET_ITRIM_S);

      getTrimForXoschfCtlValue |=
         ( ( ( fcfg1Data & FCFG1_MISC_OTP_DATA_1_HP_BUF_ITRIM_M ) >>
             FCFG1_MISC_OTP_DATA_1_HP_BUF_ITRIM_S ) <<
           DDI_0_OSC_XOSCHFCTL_HP_BUF_ITRIM_S);

      getTrimForXoschfCtlValue |=
         ( ( ( fcfg1Data & FCFG1_MISC_OTP_DATA_1_LP_BUF_ITRIM_M ) >>
             FCFG1_MISC_OTP_DATA_1_LP_BUF_ITRIM_S ) <<
           DDI_0_OSC_XOSCHFCTL_LP_BUF_ITRIM_S);
   }

   return ( getTrimForXoschfCtlValue );
}

//*****************************************************************************
//
//! \brief Returns the trim value to be used as OSC_DIG:CTL1.XOSC_HF_FAST_START.
//
//*****************************************************************************
static uint32_t GetTrimForXoscHfFastStart(void)
{
   uint32_t ui32XoscHfFastStartValue   ;

   // Get value from FCFG1
   ui32XoscHfFastStartValue = ( *(reg32_t*)( FCFG1_BASE + FCFG1_O_OSC_CONF ) &
      FCFG1_OSC_CONF_XOSC_HF_FAST_START_M ) >>
      FCFG1_OSC_CONF_XOSC_HF_FAST_START_S;

   return ( ui32XoscHfFastStartValue );
}

//*****************************************************************************
//
//! \brief Returns the trim value to be used for the RADCEXTCFG register in OSC_DIG.
//
//*****************************************************************************
static uint32_t GetTrimForRadcExtCfg(uint32_t ui32Fcfg1Revision)
{
   uint32_t getTrimForRadcExtCfgValue = 0x403F8000; // Recommended default setting
   uint32_t fcfg1Data;

   if ( ui32Fcfg1Revision >= 0x00000020 ) {
      fcfg1Data = *(reg32_t*)( FCFG1_BASE + FCFG1_O_MISC_OTP_DATA_1 );
      getTrimForRadcExtCfgValue =
         ( ( ( fcfg1Data & FCFG1_MISC_OTP_DATA_1_HPM_IBIAS_WAIT_CNT_M ) >>
             FCFG1_MISC_OTP_DATA_1_HPM_IBIAS_WAIT_CNT_S ) <<
           DDI_0_OSC_RADCEXTCFG_HPM_IBIAS_WAIT_CNT_S);

      getTrimForRadcExtCfgValue |=
         ( ( ( fcfg1Data & FCFG1_MISC_OTP_DATA_1_LPM_IBIAS_WAIT_CNT_M ) >>
             FCFG1_MISC_OTP_DATA_1_LPM_IBIAS_WAIT_CNT_S ) <<
           DDI_0_OSC_RADCEXTCFG_LPM_IBIAS_WAIT_CNT_S);

      getTrimForRadcExtCfgValue |=
         ( ( ( fcfg1Data & FCFG1_MISC_OTP_DATA_1_IDAC_STEP_M ) >>
             FCFG1_MISC_OTP_DATA_1_IDAC_STEP_S ) <<
           DDI_0_OSC_RADCEXTCFG_IDAC_STEP_S);
   }

   return ( getTrimForRadcExtCfgValue );
}

//*****************************************************************************
//
//! \brief Returns the FCFG1_OSC_CONF_ATESTLF_RCOSCLF_IBIAS_TRIM.
//
//*****************************************************************************
static uint32_t GetTrimForRcOscLfIBiasTrim( uint32_t ui32Fcfg1Revision )
{
   uint32_t trimForRcOscLfIBiasTrimValue = 0; // Default value

   if ( ui32Fcfg1Revision >= 0x00000022 ) {
      trimForRcOscLfIBiasTrimValue = ( *(reg32_t*)( FCFG1_BASE + FCFG1_O_OSC_CONF ) &
         FCFG1_OSC_CONF_ATESTLF_RCOSCLF_IBIAS_TRIM_M ) >>
         FCFG1_OSC_CONF_ATESTLF_RCOSCLF_IBIAS_TRIM_S ;
   }

   return ( trimForRcOscLfIBiasTrimValue );
}

//*****************************************************************************
//
//! \brief Returns XOSCLF_REGULATOR_TRIM and XOSCLF_CMIRRWR_RATIO as one packet
//! spanning bits [5:0] in the returned value.
//
//*****************************************************************************
static uint32_t GetTrimForXoscLfRegulatorAndCmirrwrRatio( uint32_t ui32Fcfg1Revision )
{
   uint32_t trimForXoscLfRegulatorAndCmirrwrRatioValue = 0; // Default value for both fields

   if ( ui32Fcfg1Revision >= 0x00000022 ) {
      trimForXoscLfRegulatorAndCmirrwrRatioValue = ( *(reg32_t*)( FCFG1_BASE + FCFG1_O_OSC_CONF ) &
         ( FCFG1_OSC_CONF_XOSCLF_REGULATOR_TRIM_M |
           FCFG1_OSC_CONF_XOSCLF_CMIRRWR_RATIO_M  )) >>
           FCFG1_OSC_CONF_XOSCLF_CMIRRWR_RATIO_S  ;
   }

   return ( trimForXoscLfRegulatorAndCmirrwrRatioValue );
}

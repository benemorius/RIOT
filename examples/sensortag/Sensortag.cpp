/*
    Copyright (c) 2016 Thomas Stilwell <stilwellt@openlabs.co>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
*/

#include "Sensortag.h"

#include "board.h"
#include "periph_conf.h"
#include "periph/cpuid.h"
#include "periph/gpio.h"
#include "thread.h"
#include "xtimer.h"
#include "periph/spi.h"
#include "at45.h"
#include "si70xx.h"

#include <stdio.h>

#include "flash.h"

#ifndef GIT_VERSION
#define GIT_VERSION "undefined"
#endif

#define GPIO_MEM_PWR GPIO_PIN(22)
#define GPIO_MEM_RST GPIO_PIN(21)
#define GPIO_MEM_CS GPIO_PIN(20)
#define GPIO_MEM_WP GPIO_PIN(19)
#define GPIO_LED GPIO_PIN(18)

Sensortag* sensortagS;

void debug_printf(const char *format, ...) {
//     tm time;
//     wtimer_get_localtime(&time);
//
//     va_list args;
//     va_start(args, format);
//     char new_format[256];
//     snprintf(new_format, sizeof(new_format), "%04i-%02i-%02i %02i:%02i:%02i %s",
//              time.tm_year, time.tm_mon, time.tm_mday, time.tm_hour,
//              time.tm_min, time.tm_sec, format);
// //     snprintf(new_format, 256, "%s", format);
//     vprintf(new_format, args);
//     va_end(args);
}

// #define DEBUG(...) debug_printf(__VA_ARGS__)
#define DEBUG(...) printf(__VA_ARGS__)

void flash_led(gpio_t led, uint flashes) {
    for (uint flash = 0; flash < flashes; flash++) {
        gpio_set(led);
        xtimer_usleep(200*1000);
        gpio_clear(led);
        xtimer_usleep(150*1000);
    }
}

Sensortag::Sensortag() :
main_pid(thread_getpid())
{
    sensortagS = this;

    gpio_init(GPIO_LED, GPIO_OUT);
    flash_led(GPIO_LED, 3);

	DEBUG("firmware version: %s\r\n", GIT_VERSION);

// 	cpuid_get(cpuid);
	DEBUG("cpuid: %02x %02x %02x %02x %02x %02x %02x %02x\n",
		   cpuid[0], cpuid[1], cpuid[2], cpuid[3], cpuid[4], cpuid[5], cpuid[6], cpuid[7]
	);
	cpuid[0] ^= cpuid[4]; // first byte is not sufficiently unique
	uint8_t radio_id = cpuid[0];
    DEBUG("radio_id: %02x\n", radio_id);

// 	setup_network(cpuid[0], false, 1111, &handle_packet, &handle_transmit);

//     gpio_init_int(BUTTON, GPIO_PULLDOWN, GPIO_RISING, button_handler, NULL);
//     gpio_init_int(BTN_1, GPIO_PULLDOWN, GPIO_RISING, button_handler, NULL);
//     gpio_init_int(BTN_2, GPIO_PULLDOWN, GPIO_RISING, button_handler, NULL);


    at45_t flash;
    at45_init(&flash, SPI_DEV(0), GPIO_MEM_CS, GPIO_MEM_RST, GPIO_MEM_WP, GPIO_MEM_PWR);

    uint8_t id[4];
    id[0] = 0x55;
    id[1] = 0x55;
    id[2] = 0x55;
    id[3] = 0x55;
    at45_Read_DF_ID(&flash, id);
    printf("flash id: %02x %02x %02x %02x\n", id[0], id[1], id[2], id[3]);



//     uint8_t buf[128];
//     at45_read(&flash, 0, buf, 128);
//
//     for(int i = 0; i < 128; i++) {
//         printf("%02x ", buf[i]);
//         if ((i+1) % 16 == 0)
//             printf("\n");
//     }
//     at45_power(&flash, false);



}

void Sensortag::mainloop()
{
	thread_yield();

	DEBUG("starting mainloop...\r\n");

    printf("do si70xx_test\n");
    si70xx_t si;
    si70xx_init(&si, I2C_DEV(0), 0x40);
    int ret = si70xx_test(&si);
    printf("si70xx_test returned %i\n", ret);

    xtimer_t t;
    int i =0;
    int16_t temperature;
    uint16_t humidity;

    while(1) {
        si70xx_get_both(&si, &humidity, &temperature);

        printf("%i %lu %u ", i++, xtimer_now() / 1000, timer_read(TIMER_DEV(0)));
        printf("%i.%02uC %u.%02u%%\n",
               temperature / 100, temperature % 100,
               humidity / 100, humidity % 100);

        xtimer_set_wakeup(&t, 2000*1000, thread_getpid());
        thread_sleep();

        flash_led(GPIO_PIN(18), 2);
    }
}

#include <stdint.h>
#include <inc/hw_types.h>
#include <inc/hw_ccfg.h>
#include <inc/hw_ccfg_simple_struct.h>

//*****************************************************************************
//
// Introduction
//
// This file contains fields used by Boot ROM, startup code, and SW radio
// stacks to configure chip behavior.
//
// Fields are documented in more details in hw_ccfg.h and CCFG.html in
// DriverLib documentation (doc_overview.html -> CPU Domain Memory Map -> CCFG).
//
//*****************************************************************************

//*****************************************************************************
//
// Set the values of the individual bit fields.
//
//*****************************************************************************

//#####################################
// Alternate DC/DC settings
//#####################################

#define SET_CCFG_SIZE_AND_DIS_FLAGS_DIS_ALT_DCDC_SETTING    0x0    // Alternate DC/DC setting enabled
// #define SET_CCFG_SIZE_AND_DIS_FLAGS_DIS_ALT_DCDC_SETTING 0x1    // Alternate DC/DC setting disabled

#define SET_CCFG_MODE_CONF_1_ALT_DCDC_VMIN              0x8        // 2.25V

#define SET_CCFG_MODE_CONF_1_ALT_DCDC_DITHER_EN         0x0        // Disable
// #define SET_CCFG_MODE_CONF_1_ALT_DCDC_DITHER_EN      0x1        // Enable

#define SET_CCFG_MODE_CONF_1_ALT_DCDC_IPEAK             0x2        // 39mA

//#####################################
// XOSC override settings
//#####################################

// #define SET_CCFG_SIZE_AND_DIS_FLAGS_DIS_XOSC_OVR     0x0        // Enable override
#define SET_CCFG_SIZE_AND_DIS_FLAGS_DIS_XOSC_OVR        0x1        // Disable override

#define SET_CCFG_MODE_CONF_1_DELTA_IBIAS_INIT           0x0        // Delta = 0

#define SET_CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET         0x0        // Delta = 0

#define SET_CCFG_MODE_CONF_1_XOSC_MAX_START             0x10       // 1600us

//#####################################
// Power settings
//#####################################

#define SET_CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA        0xF        // Signed delta value to apply to the VDDR_TRIM_SLEEP target, minus one

#define SET_CCFG_MODE_CONF_DCDC_RECHARGE                0x0        // Use the DC/DC during recharge in powerdown
// #define SET_CCFG_MODE_CONF_DCDC_RECHARGE             0x1        // Do not use the DC/DC during recharge in powerdown

#define SET_CCFG_MODE_CONF_DCDC_ACTIVE                  0x0        // Use the DC/DC during active mode
// #define SET_CCFG_MODE_CONF_DCDC_ACTIVE               0x1        // Do not use the DC/DC during active mode

// #define SET_CCFG_MODE_CONF_VDDS_BOD_LEVEL            0x0        // VDDS BOD level is 2.0V
#define SET_CCFG_MODE_CONF_VDDS_BOD_LEVEL               0x1        // VDDS BOD level is 1.8V (or 1.65V for external regulator mode)

#define SET_CCFG_MODE_CONF_VDDR_CAP                     0x3A       // Unsigned 8-bit integer representing the min. decoupling capacitance on VDDR in units of 100nF

//#####################################
// Clock settings
//#####################################

// #define SET_CCFG_MODE_CONF_SCLK_LF_OPTION            0x0        // LF clock derived from High Frequency XOSC
// #define SET_CCFG_MODE_CONF_SCLK_LF_OPTION            0x1        // External LF clock
#define SET_CCFG_MODE_CONF_SCLK_LF_OPTION               0x2        // LF XOSC
// #define SET_CCFG_MODE_CONF_SCLK_LF_OPTION            0x3        // LF RCOSC

// #define SET_CCFG_MODE_CONF_XOSC_CAP_MOD              0x0        // Apply cap-array delta
#define SET_CCFG_MODE_CONF_XOSC_CAP_MOD                 0x1        // Don't apply cap-array delta

#define SET_CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA          0xFF       // Signed 8-bit value, directly modifying trimmed XOSC cap-array value

#define SET_CCFG_EXT_LF_CLK_DIO                         0x01       // DIO number if using external LF clock

#define SET_CCFG_EXT_LF_CLK_RTC_INCREMENT               0x800000   // RTC increment representing the external LF clock frequency

//#####################################
// Special HF clock source setting
//#####################################
// #define SET_CCFG_MODE_CONF_XOSC_FREQ                 0x1        // Use BAW oscillator as HF source (if executed on a BAW chip, otherwise uing default (=3))
// #define SET_CCFG_MODE_CONF_XOSC_FREQ                 0x2        // HF source is a 48 MHz xtal
#define SET_CCFG_MODE_CONF_XOSC_FREQ                    0x3        // HF source is a 24 MHz xtal (default)

//#####################################
// Bootloader settings
//#####################################

// #define SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE         0x00       // Disable ROM boot loader
#define SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE            0xC5       // Enable ROM boot loader

// #define SET_CCFG_BL_CONFIG_BL_LEVEL                  0x0        // Active low to open boot loader backdoor
#define SET_CCFG_BL_CONFIG_BL_LEVEL                     0x1        // Active high to open boot loader backdoor

#define SET_CCFG_BL_CONFIG_BL_PIN_NUMBER                0xFF       // DIO number for boot loader backdoor

// #define SET_CCFG_BL_CONFIG_BL_ENABLE                 0xC5       // Enabled boot loader backdoor
#define SET_CCFG_BL_CONFIG_BL_ENABLE                    0xFF       // Disabled boot loader backdoor

//#####################################
// Debug access settings
//#####################################

// #define SET_CCFG_CCFG_TI_OPTIONS_TI_FA_ENABLE        0x00       // Disable unlocking of TI FA option.
#define SET_CCFG_CCFG_TI_OPTIONS_TI_FA_ENABLE           0xC5       // Enable unlocking of TI FA option with the unlock code

// #define SET_CCFG_CCFG_TAP_DAP_0_CPU_DAP_ENABLE       0x00       // Access disabled
#define SET_CCFG_CCFG_TAP_DAP_0_CPU_DAP_ENABLE          0xC5       // Access enabled if also enabled in FCFG

// #define SET_CCFG_CCFG_TAP_DAP_0_PRCM_TAP_ENABLE      0x00       // Access disabled
#define SET_CCFG_CCFG_TAP_DAP_0_PRCM_TAP_ENABLE         0xC5       // Access enabled if also enabled in FCFG

// #define SET_CCFG_CCFG_TAP_DAP_0_TEST_TAP_ENABLE      0x00       // Access disabled
#define SET_CCFG_CCFG_TAP_DAP_0_TEST_TAP_ENABLE         0xC5       // Access enabled if also enabled in FCFG

// #define SET_CCFG_CCFG_TAP_DAP_1_PBIST2_TAP_ENABLE    0x00       // Access disabled
#define SET_CCFG_CCFG_TAP_DAP_1_PBIST2_TAP_ENABLE       0xC5       // Access enabled if also enabled in FCFG

// #define SET_CCFG_CCFG_TAP_DAP_1_PBIST1_TAP_ENABLE    0x00       // Access disabled
#define SET_CCFG_CCFG_TAP_DAP_1_PBIST1_TAP_ENABLE       0xC5       // Access enabled if also enabled in FCFG

// #define SET_CCFG_CCFG_TAP_DAP_1_WUC_TAP_ENABLE       0x00       // Access disabled
#define SET_CCFG_CCFG_TAP_DAP_1_WUC_TAP_ENABLE          0xC5       // Access enabled if also enabled in FCFG

//#####################################
// Alternative IEEE 802.15.4 MAC address
//#####################################
#define SET_CCFG_IEEE_MAC_0                             0xFFFFFFFF // Bits [31:0]
#define SET_CCFG_IEEE_MAC_1                             0xFFFFFFFF // Bits [63:32]

//#####################################
// Alternative BLE address
//#####################################
#define SET_CCFG_IEEE_BLE_0                             0xFFFFFFFF // Bits [31:0]
#define SET_CCFG_IEEE_BLE_1                             0xFFFFFFFF // Bits [63:32]

//#####################################
// Flash erase settings
//#####################################

// #define SET_CCFG_ERASE_CONF_CHIP_ERASE_DIS_N         0x0        // Any chip erase request detected during boot will be ignored
#define SET_CCFG_ERASE_CONF_CHIP_ERASE_DIS_N            0x1        // Any chip erase request detected during boot will be performed by the boot FW

// #define SET_CCFG_ERASE_CONF_BANK_ERASE_DIS_N         0x0        // Disable the boot loader bank erase function
#define SET_CCFG_ERASE_CONF_BANK_ERASE_DIS_N            0x1        // Enable the boot loader bank erase function

//#####################################
// Flash image valid
//#####################################
#define SET_CCFG_IMAGE_VALID_CONF_IMAGE_VALID           0x00000000 // Flash image is valid
// #define SET_CCFG_IMAGE_VALID_CONF_IMAGE_VALID        <non-zero> // Flash image is invalid, call bootloader

//#####################################
// Flash sector write protection
//#####################################
#define SET_CCFG_CCFG_PROT_31_0                         0xFFFFFFFF
#define SET_CCFG_CCFG_PROT_63_32                        0xFFFFFFFF
#define SET_CCFG_CCFG_PROT_95_64                        0xFFFFFFFF
#define SET_CCFG_CCFG_PROT_127_96                       0xFFFFFFFF

//#####################################
// Select between cache or GPRAM
//#####################################
// #define SET_CCFG_SIZE_AND_DIS_FLAGS_DIS_GPRAM        0x0        // Cache is disabled and GPRAM is available at 0x11000000-0x11001FFF
#define SET_CCFG_SIZE_AND_DIS_FLAGS_DIS_GPRAM           0x1        // Cache is enabled and GPRAM is disabled (unavailable)

//*****************************************************************************
//
// CCFG values that should not be modified.
//
//*****************************************************************************
#define SET_CCFG_SIZE_AND_DIS_FLAGS_SIZE_OF_CCFG        0x0058
#define SET_CCFG_SIZE_AND_DIS_FLAGS_DISABLE_FLAGS       0x3FFF

#define SET_CCFG_MODE_CONF_VDDR_EXT_LOAD                0x1
#define SET_CCFG_MODE_CONF_RTC_COMP                     0x1
#define SET_CCFG_MODE_CONF_HF_COMP                      0x1

#define SET_CCFG_VOLT_LOAD_0_VDDR_EXT_TP45              0xFF
#define SET_CCFG_VOLT_LOAD_0_VDDR_EXT_TP25              0xFF
#define SET_CCFG_VOLT_LOAD_0_VDDR_EXT_TP5               0xFF
#define SET_CCFG_VOLT_LOAD_0_VDDR_EXT_TM15              0xFF

#define SET_CCFG_VOLT_LOAD_1_VDDR_EXT_TP125             0xFF
#define SET_CCFG_VOLT_LOAD_1_VDDR_EXT_TP105             0xFF
#define SET_CCFG_VOLT_LOAD_1_VDDR_EXT_TP85              0xFF
#define SET_CCFG_VOLT_LOAD_1_VDDR_EXT_TP65              0xFF

#define SET_CCFG_RTC_OFFSET_RTC_COMP_P0                 0xFFFF
#define SET_CCFG_RTC_OFFSET_RTC_COMP_P1                 0xFF
#define SET_CCFG_RTC_OFFSET_RTC_COMP_P2                 0xFF

#define SET_CCFG_FREQ_OFFSET_HF_COMP_P0                 0xFFFF
#define SET_CCFG_FREQ_OFFSET_HF_COMP_P1                 0xFF
#define SET_CCFG_FREQ_OFFSET_HF_COMP_P2                 0xFF

//*****************************************************************************
//
// Concatenate bit fields to words.
// DO NOT EDIT!
//
//*****************************************************************************
#define DEFAULT_CCFG_O_EXT_LF_CLK        ( \
( ((uint32_t)( SET_CCFG_EXT_LF_CLK_DIO           << CCFG_EXT_LF_CLK_DIO_S           )) | ~CCFG_EXT_LF_CLK_DIO_M           ) & \
( ((uint32_t)( SET_CCFG_EXT_LF_CLK_RTC_INCREMENT << CCFG_EXT_LF_CLK_RTC_INCREMENT_S )) | ~CCFG_EXT_LF_CLK_RTC_INCREMENT_M ) )

#define DEFAULT_CCFG_MODE_CONF_1         ( \
( ((uint32_t)( SET_CCFG_MODE_CONF_1_ALT_DCDC_VMIN      << CCFG_MODE_CONF_1_ALT_DCDC_VMIN_S      )) | ~CCFG_MODE_CONF_1_ALT_DCDC_VMIN_M      ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_1_ALT_DCDC_DITHER_EN << CCFG_MODE_CONF_1_ALT_DCDC_DITHER_EN_S )) | ~CCFG_MODE_CONF_1_ALT_DCDC_DITHER_EN_M ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_1_ALT_DCDC_IPEAK     << CCFG_MODE_CONF_1_ALT_DCDC_IPEAK_S     )) | ~CCFG_MODE_CONF_1_ALT_DCDC_IPEAK_M     ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_1_DELTA_IBIAS_INIT   << CCFG_MODE_CONF_1_DELTA_IBIAS_INIT_S   )) | ~CCFG_MODE_CONF_1_DELTA_IBIAS_INIT_M   ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET << CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET_S )) | ~CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET_M ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_1_XOSC_MAX_START     << CCFG_MODE_CONF_1_XOSC_MAX_START_S     )) | ~CCFG_MODE_CONF_1_XOSC_MAX_START_M     ) )

#define DEFAULT_CCFG_SIZE_AND_DIS_FLAGS  ( \
( ((uint32_t)( SET_CCFG_SIZE_AND_DIS_FLAGS_SIZE_OF_CCFG         << CCFG_SIZE_AND_DIS_FLAGS_SIZE_OF_CCFG_S         )) | ~CCFG_SIZE_AND_DIS_FLAGS_SIZE_OF_CCFG_M         ) & \
( ((uint32_t)( SET_CCFG_SIZE_AND_DIS_FLAGS_DISABLE_FLAGS        << CCFG_SIZE_AND_DIS_FLAGS_DISABLE_FLAGS_S        )) | ~CCFG_SIZE_AND_DIS_FLAGS_DISABLE_FLAGS_M        ) & \
( ((uint32_t)( SET_CCFG_SIZE_AND_DIS_FLAGS_DIS_GPRAM            << CCFG_SIZE_AND_DIS_FLAGS_DIS_GPRAM_S            )) | ~CCFG_SIZE_AND_DIS_FLAGS_DIS_GPRAM_M            ) & \
( ((uint32_t)( SET_CCFG_SIZE_AND_DIS_FLAGS_DIS_ALT_DCDC_SETTING << CCFG_SIZE_AND_DIS_FLAGS_DIS_ALT_DCDC_SETTING_S )) | ~CCFG_SIZE_AND_DIS_FLAGS_DIS_ALT_DCDC_SETTING_M ) & \
( ((uint32_t)( SET_CCFG_SIZE_AND_DIS_FLAGS_DIS_XOSC_OVR         << CCFG_SIZE_AND_DIS_FLAGS_DIS_XOSC_OVR_S         )) | ~CCFG_SIZE_AND_DIS_FLAGS_DIS_XOSC_OVR_M         ) )

#define DEFAULT_CCFG_MODE_CONF           ( \
( ((uint32_t)( SET_CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA << CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_S )) | ~CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_M ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_DCDC_RECHARGE         << CCFG_MODE_CONF_DCDC_RECHARGE_S         )) | ~CCFG_MODE_CONF_DCDC_RECHARGE_M         ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_DCDC_ACTIVE           << CCFG_MODE_CONF_DCDC_ACTIVE_S           )) | ~CCFG_MODE_CONF_DCDC_ACTIVE_M           ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_VDDR_EXT_LOAD         << CCFG_MODE_CONF_VDDR_EXT_LOAD_S         )) | ~CCFG_MODE_CONF_VDDR_EXT_LOAD_M         ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_VDDS_BOD_LEVEL        << CCFG_MODE_CONF_VDDS_BOD_LEVEL_S        )) | ~CCFG_MODE_CONF_VDDS_BOD_LEVEL_M        ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_SCLK_LF_OPTION        << CCFG_MODE_CONF_SCLK_LF_OPTION_S        )) | ~CCFG_MODE_CONF_SCLK_LF_OPTION_M        ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_RTC_COMP              << CCFG_MODE_CONF_RTC_COMP_S              )) | ~CCFG_MODE_CONF_RTC_COMP_M              ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_XOSC_FREQ             << CCFG_MODE_CONF_XOSC_FREQ_S             )) | ~CCFG_MODE_CONF_XOSC_FREQ_M             ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_XOSC_CAP_MOD          << CCFG_MODE_CONF_XOSC_CAP_MOD_S          )) | ~CCFG_MODE_CONF_XOSC_CAP_MOD_M          ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_HF_COMP               << CCFG_MODE_CONF_HF_COMP_S               )) | ~CCFG_MODE_CONF_HF_COMP_M               ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA   << CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA_S   )) | ~CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA_M   ) & \
( ((uint32_t)( SET_CCFG_MODE_CONF_VDDR_CAP              << CCFG_MODE_CONF_VDDR_CAP_S              )) | ~CCFG_MODE_CONF_VDDR_CAP_M              ) )

#define DEFAULT_CCFG_VOLT_LOAD_0         ( \
( ((uint32_t)( SET_CCFG_VOLT_LOAD_0_VDDR_EXT_TP45 << CCFG_VOLT_LOAD_0_VDDR_EXT_TP45_S )) | ~CCFG_VOLT_LOAD_0_VDDR_EXT_TP45_M ) & \
( ((uint32_t)( SET_CCFG_VOLT_LOAD_0_VDDR_EXT_TP25 << CCFG_VOLT_LOAD_0_VDDR_EXT_TP25_S )) | ~CCFG_VOLT_LOAD_0_VDDR_EXT_TP25_M ) & \
( ((uint32_t)( SET_CCFG_VOLT_LOAD_0_VDDR_EXT_TP5  << CCFG_VOLT_LOAD_0_VDDR_EXT_TP5_S  )) | ~CCFG_VOLT_LOAD_0_VDDR_EXT_TP5_M  ) & \
( ((uint32_t)( SET_CCFG_VOLT_LOAD_0_VDDR_EXT_TM15 << CCFG_VOLT_LOAD_0_VDDR_EXT_TM15_S )) | ~CCFG_VOLT_LOAD_0_VDDR_EXT_TM15_M ) )

#define DEFAULT_CCFG_VOLT_LOAD_1         ( \
( ((uint32_t)( SET_CCFG_VOLT_LOAD_1_VDDR_EXT_TP125 << CCFG_VOLT_LOAD_1_VDDR_EXT_TP125_S )) | ~CCFG_VOLT_LOAD_1_VDDR_EXT_TP125_M ) & \
( ((uint32_t)( SET_CCFG_VOLT_LOAD_1_VDDR_EXT_TP105 << CCFG_VOLT_LOAD_1_VDDR_EXT_TP105_S )) | ~CCFG_VOLT_LOAD_1_VDDR_EXT_TP105_M ) & \
( ((uint32_t)( SET_CCFG_VOLT_LOAD_1_VDDR_EXT_TP85  << CCFG_VOLT_LOAD_1_VDDR_EXT_TP85_S  )) | ~CCFG_VOLT_LOAD_1_VDDR_EXT_TP85_M  ) & \
( ((uint32_t)( SET_CCFG_VOLT_LOAD_1_VDDR_EXT_TP65  << CCFG_VOLT_LOAD_1_VDDR_EXT_TP65_S  )) | ~CCFG_VOLT_LOAD_1_VDDR_EXT_TP65_M  ) )

#define DEFAULT_CCFG_RTC_OFFSET          ( \
( ((uint32_t)( SET_CCFG_RTC_OFFSET_RTC_COMP_P0 << CCFG_RTC_OFFSET_RTC_COMP_P0_S )) | ~CCFG_RTC_OFFSET_RTC_COMP_P0_M ) & \
( ((uint32_t)( SET_CCFG_RTC_OFFSET_RTC_COMP_P1 << CCFG_RTC_OFFSET_RTC_COMP_P1_S )) | ~CCFG_RTC_OFFSET_RTC_COMP_P1_M ) & \
( ((uint32_t)( SET_CCFG_RTC_OFFSET_RTC_COMP_P2 << CCFG_RTC_OFFSET_RTC_COMP_P2_S )) | ~CCFG_RTC_OFFSET_RTC_COMP_P2_M ) )

#define DEFAULT_CCFG_FREQ_OFFSET         ( \
( ((uint32_t)( SET_CCFG_FREQ_OFFSET_HF_COMP_P0 << CCFG_FREQ_OFFSET_HF_COMP_P0_S )) | ~CCFG_FREQ_OFFSET_HF_COMP_P0_M ) & \
( ((uint32_t)( SET_CCFG_FREQ_OFFSET_HF_COMP_P1 << CCFG_FREQ_OFFSET_HF_COMP_P1_S )) | ~CCFG_FREQ_OFFSET_HF_COMP_P1_M ) & \
( ((uint32_t)( SET_CCFG_FREQ_OFFSET_HF_COMP_P2 << CCFG_FREQ_OFFSET_HF_COMP_P2_S )) | ~CCFG_FREQ_OFFSET_HF_COMP_P2_M ) )

#define DEFAULT_CCFG_IEEE_MAC_0          SET_CCFG_IEEE_MAC_0
#define DEFAULT_CCFG_IEEE_MAC_1          SET_CCFG_IEEE_MAC_1
#define DEFAULT_CCFG_IEEE_BLE_0          SET_CCFG_IEEE_BLE_0
#define DEFAULT_CCFG_IEEE_BLE_1          SET_CCFG_IEEE_BLE_1

#define DEFAULT_CCFG_BL_CONFIG           ( \
( ((uint32_t)( SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE << CCFG_BL_CONFIG_BOOTLOADER_ENABLE_S )) | ~CCFG_BL_CONFIG_BOOTLOADER_ENABLE_M ) & \
( ((uint32_t)( SET_CCFG_BL_CONFIG_BL_LEVEL          << CCFG_BL_CONFIG_BL_LEVEL_S          )) | ~CCFG_BL_CONFIG_BL_LEVEL_M          ) & \
( ((uint32_t)( SET_CCFG_BL_CONFIG_BL_PIN_NUMBER     << CCFG_BL_CONFIG_BL_PIN_NUMBER_S     )) | ~CCFG_BL_CONFIG_BL_PIN_NUMBER_M     ) & \
( ((uint32_t)( SET_CCFG_BL_CONFIG_BL_ENABLE         << CCFG_BL_CONFIG_BL_ENABLE_S         )) | ~CCFG_BL_CONFIG_BL_ENABLE_M         ) )

#define DEFAULT_CCFG_ERASE_CONF          ( \
( ((uint32_t)( SET_CCFG_ERASE_CONF_CHIP_ERASE_DIS_N << CCFG_ERASE_CONF_CHIP_ERASE_DIS_N_S )) | ~CCFG_ERASE_CONF_CHIP_ERASE_DIS_N_M ) & \
( ((uint32_t)( SET_CCFG_ERASE_CONF_BANK_ERASE_DIS_N << CCFG_ERASE_CONF_BANK_ERASE_DIS_N_S )) | ~CCFG_ERASE_CONF_BANK_ERASE_DIS_N_M ) )

#define DEFAULT_CCFG_CCFG_TI_OPTIONS     ( \
( ((uint32_t)( SET_CCFG_CCFG_TI_OPTIONS_TI_FA_ENABLE << CCFG_CCFG_TI_OPTIONS_TI_FA_ENABLE_S )) | ~CCFG_CCFG_TI_OPTIONS_TI_FA_ENABLE_M ) )

#define DEFAULT_CCFG_CCFG_TAP_DAP_0      ( \
( ((uint32_t)( SET_CCFG_CCFG_TAP_DAP_0_CPU_DAP_ENABLE  << CCFG_CCFG_TAP_DAP_0_CPU_DAP_ENABLE_S  )) | ~CCFG_CCFG_TAP_DAP_0_CPU_DAP_ENABLE_M  ) & \
( ((uint32_t)( SET_CCFG_CCFG_TAP_DAP_0_PRCM_TAP_ENABLE << CCFG_CCFG_TAP_DAP_0_PRCM_TAP_ENABLE_S )) | ~CCFG_CCFG_TAP_DAP_0_PRCM_TAP_ENABLE_M ) & \
( ((uint32_t)( SET_CCFG_CCFG_TAP_DAP_0_TEST_TAP_ENABLE << CCFG_CCFG_TAP_DAP_0_TEST_TAP_ENABLE_S )) | ~CCFG_CCFG_TAP_DAP_0_TEST_TAP_ENABLE_M ) )

#define DEFAULT_CCFG_CCFG_TAP_DAP_1      ( \
( ((uint32_t)( SET_CCFG_CCFG_TAP_DAP_1_PBIST2_TAP_ENABLE << CCFG_CCFG_TAP_DAP_1_PBIST2_TAP_ENABLE_S )) | ~CCFG_CCFG_TAP_DAP_1_PBIST2_TAP_ENABLE_M ) & \
( ((uint32_t)( SET_CCFG_CCFG_TAP_DAP_1_PBIST1_TAP_ENABLE << CCFG_CCFG_TAP_DAP_1_PBIST1_TAP_ENABLE_S )) | ~CCFG_CCFG_TAP_DAP_1_PBIST1_TAP_ENABLE_M ) & \
( ((uint32_t)( SET_CCFG_CCFG_TAP_DAP_1_WUC_TAP_ENABLE    << CCFG_CCFG_TAP_DAP_1_WUC_TAP_ENABLE_S    )) | ~CCFG_CCFG_TAP_DAP_1_WUC_TAP_ENABLE_M    ) )

#define DEFAULT_CCFG_IMAGE_VALID_CONF    ( \
( ((uint32_t)( SET_CCFG_IMAGE_VALID_CONF_IMAGE_VALID << CCFG_IMAGE_VALID_CONF_IMAGE_VALID_S )) | ~CCFG_IMAGE_VALID_CONF_IMAGE_VALID_M ) )

#define DEFAULT_CCFG_CCFG_PROT_31_0      SET_CCFG_CCFG_PROT_31_0
#define DEFAULT_CCFG_CCFG_PROT_63_32     SET_CCFG_CCFG_PROT_63_32
#define DEFAULT_CCFG_CCFG_PROT_95_64     SET_CCFG_CCFG_PROT_95_64
#define DEFAULT_CCFG_CCFG_PROT_127_96    SET_CCFG_CCFG_PROT_127_96

const ccfg_t __ccfg __attribute__((used,section(".ccfg"))) =
{                                     // Mapped to address
    DEFAULT_CCFG_O_EXT_LF_CLK       , // 0x50003FA8 (0x50003xxx maps to last
    DEFAULT_CCFG_MODE_CONF_1        , // 0x50003FAC  sector in FLASH.
    DEFAULT_CCFG_SIZE_AND_DIS_FLAGS , // 0x50003FB0  Independent of FLASH size)
    DEFAULT_CCFG_MODE_CONF          , // 0x50003FB4
    DEFAULT_CCFG_VOLT_LOAD_0        , // 0x50003FB8
    DEFAULT_CCFG_VOLT_LOAD_1        , // 0x50003FBC
    DEFAULT_CCFG_RTC_OFFSET         , // 0x50003FC0
    DEFAULT_CCFG_FREQ_OFFSET        , // 0x50003FC4
    DEFAULT_CCFG_IEEE_MAC_0         , // 0x50003FC8
    DEFAULT_CCFG_IEEE_MAC_1         , // 0x50003FCC
    DEFAULT_CCFG_IEEE_BLE_0         , // 0x50003FD0
    DEFAULT_CCFG_IEEE_BLE_1         , // 0x50003FD4
    DEFAULT_CCFG_BL_CONFIG          , // 0x50003FD8
    DEFAULT_CCFG_ERASE_CONF         , // 0x50003FDC
    DEFAULT_CCFG_CCFG_TI_OPTIONS    , // 0x50003FE0
    DEFAULT_CCFG_CCFG_TAP_DAP_0     , // 0x50003FE4
    DEFAULT_CCFG_CCFG_TAP_DAP_1     , // 0x50003FE8
    DEFAULT_CCFG_IMAGE_VALID_CONF   , // 0x50003FEC
    DEFAULT_CCFG_CCFG_PROT_31_0     , // 0x50003FF0
    DEFAULT_CCFG_CCFG_PROT_63_32    , // 0x50003FF4
    DEFAULT_CCFG_CCFG_PROT_95_64    , // 0x50003FF8
    DEFAULT_CCFG_CCFG_PROT_127_96   , // 0x50003FFC
};

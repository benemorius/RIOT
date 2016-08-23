/******************************************************************************
*  Filename:       sys_ctrl.h
*  Revised:        2015-07-16 12:12:04 +0200 (Thu, 16 Jul 2015)
*  Revision:       44151
*
*  Description:    Defines and prototypes for the System Controller.
*
*  Copyright (c) 2015, Texas Instruments Incorporated
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  1) Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*
*  2) Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*
*  3) Neither the name of the ORGANIZATION nor the names of its contributors may
*     be used to endorse or promote products derived from this software without
*     specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
*  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
*  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
*  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

//*****************************************************************************
//
//! \addtogroup system_control_group
//! @{
//! \addtogroup sysctrl_api
//! @{
//
//*****************************************************************************

#ifndef __SYSCTRL_H__
#define __SYSCTRL_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include "cpu.h"

// #include <stdbool.h>
// #include <stdint.h>
// #include <inc/hw_types.h>
// #include <inc/hw_memmap.h>
// #include <inc/hw_ints.h>
// #include <inc/hw_sysctl.h>
// #include <inc/hw_prcm.h>
// #include <inc/hw_nvic.h>
// #include <inc/hw_aon_wuc.h>
// #include <inc/hw_aux_wuc.h>
// #include <inc/hw_aon_ioc.h>
// #include <inc/hw_ddi_0_osc.h>
// #include <inc/hw_rfc_pwr.h>
// #include <inc/hw_prcm.h>
// #include <inc/hw_adi_3_refsys.h>
// #include <inc/hw_aon_sysctl.h>
// #include <inc/hw_aon_rtc.h>
// #include <inc/hw_fcfg1.h>

//*****************************************************************************
//
// Defines for the settings of the main XOSC
//
//*****************************************************************************
#define SYSCTRL_SYSBUS_ON       0x00000001
#define SYSCTRL_SYSBUS_OFF      0x00000000

//*****************************************************************************
//
// Defines for the different power modes of the System CPU
//
//*****************************************************************************
#define CPU_RUN                 0x00000000
#define CPU_SLEEP               0x00000001
#define CPU_DEEP_SLEEP          0x00000002

//*****************************************************************************
//
// Defines for SysCtrlSetRechargeBeforePowerDown
//
//*****************************************************************************
#define XOSC_IN_HIGH_POWER_MODE 0 // When xosc_hf is in HIGH_POWER_XOSC
#define XOSC_IN_LOW_POWER_MODE  1 // When xosc_hf is in LOW_POWER_XOSC

//
// Keeping backward compatibility until major revision number is incremented
//
#define XoscInHighPowerMode      ( XOSC_IN_HIGH_POWER_MODE     )
#define XoscInLowPowerMode       ( XOSC_IN_LOW_POWER_MODE      )

//*****************************************************************************
//
//! \brief Set Recharge values before entering Power Down.
//!
//! This function shall be called just before entering Power Down.
//! It calculates an optimal and safe recharge setting of the adaptive recharge
//! controller. The results of previous setting are also taken into account.
//!
//! \note In order to make sure that the register writes are completed, \ref SysCtrlAonSync()
//! must be called before entering standby/power down. This is not done internally
//! in this function due to two reasons:
//! - 1) There might be other register writes that must be synchronized as well.
//! - 2) It is possible to save some time by doing other things before calling
//! \ref SysCtrlAonSync() since this call will not return before there are no
//! outstanding write requests between MCU and AON.
//!
//! \param xoscPowerMode (typically running in XOSC_IN_HIGH_POWER_MODE all the time).
//! - \ref XOSC_IN_HIGH_POWER_MODE : When xosc_hf is in HIGH_POWER_XOSC.
//! - \ref XOSC_IN_LOW_POWER_MODE  : When xosc_hf is in LOW_POWER_XOSC.
//!
//! \return None
//
//*****************************************************************************
void
SysCtrlSetRechargeBeforePowerDown( uint32_t xoscPowerMode );


//*****************************************************************************
//
//! \brief Adjust Recharge calculations to be used next.
//!
//! This function shall be called just after returning from Power Down.
//!
//! Reads the results from the adaptive recharge controller and current chip
//! temperature. This is used as additional information when calculating
//! optimal recharge controller settings next time (When
//! \ref SysCtrlSetRechargeBeforePowerDown() is called next time).
//!
//! \note
//! Special care must be taken to make sure that the AON registers read are
//! updated after the wakeup. Writing to an AON register and then calling
//! \ref SysCtrlAonSync() will handle this. Typically this must be done anyway,
//! for example by calling \ref AONWUCAuxWakeupEvent() and then later on calling
//! \ref SysCtrlAonSync() just before calling \ref SysCtrlSetRechargeBeforePowerDown().
//!
//! \return None
//
//*****************************************************************************
void
SysCtrlAdjustRechargeAfterPowerDown( void );


//*****************************************************************************
//
//! \brief Turns DCDC on or off depending of what is considered to be optimal usage.
//!
//! This function controls the DCDC only if both the following CCFG settings are \c true:
//! - DCDC is configured to be used.
//! - Alternative DCDC settings are defined and enabled.
//!
//! The DCDC is configured in accordance to the CCFG settings when turned on.
//!
//! This function should be called periodically.
//!
//! \return None
//
//*****************************************************************************
void
SysCtrl_DCDC_VoltageConditionalControl( void );


//*****************************************************************************
// \name Return values from calling SysCtrlResetSourceGet()
//@{
//*****************************************************************************
#define RSTSRC_PWR_ON               (( AON_SYSCTL_RESETCTL_RESET_SRC_PWR_ON    >> AON_SYSCTL_RESETCTL_RESET_SRC_S ))
#define RSTSRC_PIN_RESET            (( AON_SYSCTL_RESETCTL_RESET_SRC_PIN_RESET >> AON_SYSCTL_RESETCTL_RESET_SRC_S ))
#define RSTSRC_VDDS_LOSS            (( AON_SYSCTL_RESETCTL_RESET_SRC_VDDS_LOSS >> AON_SYSCTL_RESETCTL_RESET_SRC_S ))
#define RSTSRC_VDD_LOSS             (( AON_SYSCTL_RESETCTL_RESET_SRC_VDD_LOSS  >> AON_SYSCTL_RESETCTL_RESET_SRC_S ))
#define RSTSRC_VDDR_LOSS            (( AON_SYSCTL_RESETCTL_RESET_SRC_VDDR_LOSS >> AON_SYSCTL_RESETCTL_RESET_SRC_S ))
#define RSTSRC_CLK_LOSS             (( AON_SYSCTL_RESETCTL_RESET_SRC_CLK_LOSS  >> AON_SYSCTL_RESETCTL_RESET_SRC_S ))
#define RSTSRC_SYSRESET             (( AON_SYSCTL_RESETCTL_RESET_SRC_SYSRESET  >> AON_SYSCTL_RESETCTL_RESET_SRC_S ))
#define RSTSRC_WARMRESET            (( AON_SYSCTL_RESETCTL_RESET_SRC_WARMRESET >> AON_SYSCTL_RESETCTL_RESET_SRC_S ))
#define RSTSRC_WAKEUP_FROM_SHUTDOWN (( AON_SYSCTL_RESETCTL_RESET_SRC_M         >> AON_SYSCTL_RESETCTL_RESET_SRC_S ) + 1 )
//@}

//*****************************************************************************
//
//! \brief Returns last reset source (including "wakeup from shutdown").
//!
//! \return Returns one of the RSTSRC_ defines.
//
//*****************************************************************************
extern uint32_t SysCtrlResetSourceGet( void );

#ifdef __cplusplus
}
#endif

#endif //  __SYSCTRL_H__

//*****************************************************************************
//
//! Close the Doxygen group.
//! @}
//! @}
//
//*****************************************************************************

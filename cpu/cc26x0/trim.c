/******************************************************************************
*  Filename:       setup.c
*  Revised:        2015-07-03 16:22:15 +0200 (Fri, 03 Jul 2015)
*  Revision:       44130
*
*  Description:    Setup file for CC13xx/CC26xx devices.
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

#include "trim.h"

#define FCFG1_BASE (0x50001000)
#define FCFG1_O_AMPCOMP_TH2 (0x374)
#define FCFG1_AMPCOMP_TH2_LPMUPDATE_LTH_M (0xfc000000)
#define FCFG1_AMPCOMP_TH2_LPMUPDATE_LTH_S (26)
#define DDI_0_OSC_AMPCOMPTH2_LPMUPDATE_LTH_S (26)
#define FCFG1_AMPCOMP_TH2_LPMUPDATE_HTM_M (0x00fc0000)
#define FCFG1_AMPCOMP_TH2_LPMUPDATE_HTM_S (18)
#define DDI_0_OSC_AMPCOMPTH2_LPMUPDATE_HTH_S (18)
#define FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_LPM_M (0x0000fc00)
#define FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_LPM_S (10)
#define DDI_0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_LPM_S (10)
#define FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_HPM_M (0x000000fc)
#define FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_HPM_S (2)
#define DDI_0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_HPM_S (2)

//*****************************************************************************
//
//! \brief Returns the trim value to be used for the AMPCOMP_TH2 register in OSC_DIG.
//
//*****************************************************************************
uint32_t GetTrimForAmpcompTh2(void)
{
    uint32_t ui32TrimValue;
    uint32_t ui32Fcfg1Value;

    // Use device specific trim value located in factory configuration
    // area. All defined register bit fields have corresponding trim
    // value in the factory configuration area
    ui32Fcfg1Value = *(uint32_t*)(FCFG1_BASE + FCFG1_O_AMPCOMP_TH2);
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

#define FCFG1_O_AMPCOMP_TH1 (0x370)
#define FCFG1_AMPCOMP_TH1_HPMRAMP3_LTH_M (0x00fc0000)
#define FCFG1_AMPCOMP_TH1_HPMRAMP3_LTH_S (18)
#define DDI_0_OSC_AMPCOMPTH1_HPMRAMP3_LTH_S (18)
#define FCFG1_AMPCOMP_TH1_HPMRAMP3_HTH_M (0x0000fc00)
#define FCFG1_AMPCOMP_TH1_HPMRAMP3_HTH_S (10)
#define DDI_0_OSC_AMPCOMPTH1_HPMRAMP3_HTH_S (10)
#define FCFG1_AMPCOMP_TH1_IBIASCAP_LPTOHP_OL_CNT_M (0x3c0)
#define FCFG1_AMPCOMP_TH1_IBIASCAP_LPTOHP_OL_CNT_S (6)
#define DDI_0_OSC_AMPCOMPTH1_IBIASCAP_LPTOHP_OL_CNT_S (6)
#define FCFG1_AMPCOMP_TH1_HPMRAMP1_TH_M (0x3f)
#define FCFG1_AMPCOMP_TH1_HPMRAMP1_TH_S (0)
#define DDI_0_OSC_AMPCOMPTH1_HPMRAMP1_TH_S (0)

//*****************************************************************************
//
//! \brief Returns the trim value to be used for the AMPCOMP_TH1 register in OSC_DIG.
//
//*****************************************************************************
uint32_t GetTrimForAmpcompTh1(void)
{
    uint32_t ui32TrimValue;
    uint32_t ui32Fcfg1Value;

    // Use device specific trim values located in factory configuration
    // area. All defined register bit fields have a corresponding trim
    // value in the factory configuration area
    ui32Fcfg1Value = *(uint32_t*)(FCFG1_BASE + FCFG1_O_AMPCOMP_TH1);
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

#define FCFG1_O_AMPCOMP_CTRL1 (0x378)
#define FCFG1_AMPCOMP_CTRL1_IBIAS_OFFSET_M (0x00f00000)
#define FCFG1_AMPCOMP_CTRL1_IBIAS_OFFSET_S (20)
#define FCFG1_AMPCOMP_CTRL1_IBIAS_INIT_M (0x000f0000)
#define FCFG1_AMPCOMP_CTRL1_IBIAS_INIT_S (16)
#define CCFG_BASE (0x50003000)
#define CCFG_O_SIZE_AND_DIS_FLAGS (0xfb0)
#define CCFG_SIZE_AND_DIS_FLAGS_DIS_XOSC_OVR_M (0x1)
#define CCFG_O_MODE_CONF_1 (0xfac)
#define CCFG_MODE_CONF_1_DELTA_IBIAS_OFFSET_S (8)
#define DDI_0_OSC_AMPCOMPCTL_IBIAS_OFFSET_M (0x00f00000)
#define DDI_0_OSC_AMPCOMPCTL_IBIAS_OFFSET_S (20)
#define CCFG_MODE_CONF_1_DELTA_IBIAS_INIT_S (12)
#define DDI_0_OSC_AMPCOMPCTL_IBIAS_INIT_M (0x000f0000)
#define DDI_0_OSC_AMPCOMPCTL_IBIAS_INIT_S (16)
#define FCFG1_AMPCOMP_CTRL1_LPM_IBIAS_WAIT_CNT_FINAL_M (0x0000ff00)
#define FCFG1_AMPCOMP_CTRL1_LPM_IBIAS_WAIT_CNT_FINAL_S (8)
#define DDI_0_OSC_AMPCOMPCTL_LPM_IBIAS_WAIT_CNT_FINAL_S (8)
#define FCFG1_AMPCOMP_CTRL1_CAP_STEP_M (0xf0)
#define FCFG1_AMPCOMP_CTRL1_CAP_STEP_S (4)
#define DDI_0_OSC_AMPCOMPCTL_CAP_STEP_S (4)
#define FCFG1_AMPCOMP_CTRL1_IBIASCAP_HPTOLP_OL_CNT_M (0xf)
#define FCFG1_AMPCOMP_CTRL1_IBIASCAP_HPTOLP_OL_CNT_S (0)
#define DDI_0_OSC_AMPCOMPCTL_IBIASCAP_HPTOLP_OL_CNT_S (0)
#define FCFG1_AMPCOMP_CTRL1_AMPCOMP_REQ_MODE_M (0x40000000)
#define FCFG1_AMPCOMP_CTRL1_AMPCOMP_REQ_MODE_S (30)
#define DDI_0_OSC_AMPCOMPCTL_AMPCOMP_REQ_MODE_S (30)

//*****************************************************************************
//
//! \brief Returns the trim value to be used for the AMPCOMP_CTRL register in OSC_DIG.
//
//*****************************************************************************
uint32_t GetTrimForAmpcompCtrl(uint32_t ui32Fcfg1Revision)
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
    ui32Fcfg1Value = *(uint32_t*)( FCFG1_BASE + FCFG1_O_AMPCOMP_CTRL1 );

    ibiasOffset    = ( ui32Fcfg1Value &
    FCFG1_AMPCOMP_CTRL1_IBIAS_OFFSET_M ) >>
    FCFG1_AMPCOMP_CTRL1_IBIAS_OFFSET_S ;
    ibiasInit      = ( ui32Fcfg1Value &
    FCFG1_AMPCOMP_CTRL1_IBIAS_INIT_M ) >>
    FCFG1_AMPCOMP_CTRL1_IBIAS_INIT_S ;

    if (( *(uint32_t*)( CCFG_BASE + CCFG_O_SIZE_AND_DIS_FLAGS ) & CCFG_SIZE_AND_DIS_FLAGS_DIS_XOSC_OVR_M ) == 0 ) {
        // Adjust with DELTA_IBIAS_OFFSET and DELTA_IBIAS_INIT from CCFG
        modeConf1   = *(uint32_t*)( CCFG_BASE + CCFG_O_MODE_CONF_1 );

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

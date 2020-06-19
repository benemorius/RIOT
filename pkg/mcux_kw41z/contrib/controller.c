/*! *********************************************************************************
 * \defgroup CONTROLLER
 * @{
 ********************************************************************************** */
/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
* All rights reserved.
*
* file
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "EmbeddedTypes.h"
#include "Messaging.h"
#include "fsl_os_abstraction.h"
#include "fsl_device_registers.h"
#include "ble_controller_task_config.h"
#include "Panic.h"
#include "controller_interface.h"
#include "Flash_Adapter.h"
#include "SecLib.h"
#include "mcux_board.h"

#include "MWS.h"
#include "fsl_xcvr.h"
/************************************************************************************
*************************************************************************************
* Public constants & macros
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
osaTaskId_t  gControllerTaskId;
osaEventId_t mControllerTaskEvent;

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
static void ControllerTask(osaTaskParam_t argument);

extern void Controller_TaskHandler(void * args);
extern void Controller_InterruptHandler(void);

#if gMWS_Enabled_d
extern uint32_t MWS_BLE_Callback ( mwsEvents_t event );
#endif

#if gMWS_UseCoexistence_d
extern uint32_t MWS_COEX_BLE_Callback ( mwsEvents_t event );
#endif
/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
OSA_TASK_DEFINE(ControllerTask, gControllerTaskPriority_c, 1, gControllerTaskStackSize_c, FALSE);

/* Radio system clock selection. */
#if (RF_OSC_26MHZ == 1)
const uint8_t gRfSysClk26MHz_c = 1;  /* 26MHz radio clock. */
#else
const uint8_t gRfSysClk26MHz_c = 0;  /* 32MHz radio clock. */
#endif

/* Organizationally Unique Identifier used in BD_ADDR. */
const uint8_t gBD_ADDR_OUI_c[3] = {BD_ADDR_OUI};
/* BD_ADDR referenced in the controller */
uint8_t gBD_ADDR[6];
/* Time between the beginning of two consecutive advertising PDU's */
const uint8_t gAdvertisingPacketInterval_c = mcAdvertisingPacketInterval_c;
/* Advertising channels that are enabled for scanning operation. */
const uint8_t gScanChannelMap_c = mcScanChannelMap_c;
/* Advertising channels that are enabled for initiator scanning operation. */
const uint8_t gInitiatorChannelMap_c = mcInitiatorChannelMap_c;
/* Offset to the first instant register */
const uint16_t gOffsetToFirstInstant_c = mcOffsetToFirstInstant_c;
/* Scan FIFO lockup detection interval in milliseconds. */
uint32_t gScanFifoLockupCheckIntervalMilliSeconds = mScanFifoLockupCheckIntervalMilliSeconds_c;
/* Default value for the DTM 2 wire serial connection. Can be changed also by using Controller_SetDTMBaudrate defined in "controller_interface.h". */
const dtmBaudrate_t gDefaultDTMBaudrate = gDTM_BaudRate_115200_c;

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

static void ControllerTask(osaTaskParam_t argument)
{
    Controller_TaskHandler((void *) NULL);
}

static void ControllerSetBD_ADDR(void)
{
    sha256Context_t mCtx;
    uint8_t uid[16] = {0};
    uint8_t len = 0;
    uint8_t i=0;

    NV_ReadHWParameters(&gHardwareParameters);
    while( (i<6) && (gHardwareParameters.bluetooth_address[i] == 0xFF) ){ i++; }
    if( i == 6 )
    {
        BOARD_GetMCUUid(uid, &len);
        SHA256_Hash(&mCtx, uid, len);
        FLib_MemCpy(gHardwareParameters.bluetooth_address, (uint8_t *)(&mCtx.hash), 3);
        FLib_MemCpy(&gHardwareParameters.bluetooth_address[3], (uint8_t *)gBD_ADDR_OUI_c, 3);
        NV_WriteHWParameters(&gHardwareParameters);
    }
    FLib_MemCpy(gBD_ADDR, gHardwareParameters.bluetooth_address, 6);

//     uint8_t addr[] = {0x00, 0x60, 0x37, 0x68, 0x6b, 0xee};
//     uint8_t a[6];
//     a[0] = addr[5];
//     a[1] = addr[4];
//     a[2] = addr[3];
//     a[3] = addr[2];
//     a[4] = addr[1];
//     a[5] = addr[0];
//     FLib_MemCpy(gBD_ADDR, a, 6);
}

#include <stdio.h>
#include "thread.h"

#if gMWS_UseCoexistence_d
uint32_t mws_callback(mwsEvents_t event)
{
    uint32_t res = MWS_BLE_Callback(event);
//     printf("<%u> event %u returned %lu\n", thread_getpid(), event, res);
    return res;
}
#endif

#if gMWS_UseCoexistence_d
uint32_t mws_co_callback(mwsEvents_t event)
{
    uint32_t res = MWS_COEX_BLE_Callback(event);
    printf("<%u> event %u returned %lu\n", thread_getpid(), event, res);
    return res;
}
#endif

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
/**
 * \fn 		Controller_TaskInit
 * \brief   This Function
 * \return  osa_status_t
 */
osaStatus_t Controller_TaskInit(void)
{

    /* Initialization of task related objects */
    if(gControllerTaskId)
    {
      return osaStatus_Error;
    }

    mControllerTaskEvent = OSA_EventCreate(TRUE);

    if(NULL == mControllerTaskEvent)
    {
        return osaStatus_Error;
    }

    /* Task creation */
    gControllerTaskId = OSA_TaskCreate(OSA_TASK(ControllerTask), NULL);
    if( NULL == gControllerTaskId )
    {
        panic(0,0,0,0);
        return osaStatus_Error;
    }

#if (defined(CPU_MKW20Z160VHT4) || defined(CPU_MKW30Z160VHM4) || defined(CPU_MKW40Z160VHT4))
    OSA_InstallIntHandler(BTLL_RSIM_IRQn, &Controller_InterruptHandler);

    NVIC_ClearPendingIRQ(BTLL_RSIM_IRQn);
    NVIC_EnableIRQ(BTLL_RSIM_IRQn);
    NVIC_SetPriority(BTLL_RSIM_IRQn, 0x80 >> (8 - __NVIC_PRIO_BITS));

#elif (defined(CPU_MKW21Z256VHT4) || defined(CPU_MKW21Z512VHT4) || defined(CPU_MKW31Z256CAx4) || \
    defined(CPU_MKW31Z256VHT4) || defined(CPU_MKW31Z512CAx4) || defined(CPU_MKW31Z512VHT4) || \
    defined(CPU_MKW41Z256VHT4) || defined(CPU_MKW41Z512VHT4))

    /* Select BLE protocol on RADIO0_IRQ */
    XCVR_MISC->XCVR_CTRL = (uint32_t)((XCVR_MISC->XCVR_CTRL & (uint32_t)~(uint32_t)(
                               XCVR_CTRL_XCVR_CTRL_RADIO0_IRQ_SEL_MASK
                              )) | (uint32_t)(
                               (0 << XCVR_CTRL_XCVR_CTRL_RADIO0_IRQ_SEL_SHIFT)
                              ));

    OSA_InstallIntHandler(Radio_0_IRQn, &Controller_InterruptHandler);

    NVIC_ClearPendingIRQ(Radio_0_IRQn);
    NVIC_EnableIRQ(Radio_0_IRQn);
    NVIC_SetPriority(Radio_0_IRQn, 0x80 >> (8 - __NVIC_PRIO_BITS));

#elif (defined(CPU_MKW35A512VFP4) || defined(CPU_MKW35Z512VHT4) || defined(CPU_MKW36A512VFP4) || \
    defined(CPU_MKW36A512VHT4) || defined(CPU_MKW36Z512VFP4) || defined(CPU_MKW36Z512VHT4))

    /* Select BLE protocol on RADIO0_IRQ */
    XCVR_MISC->XCVR_CTRL = (uint32_t)((XCVR_MISC->XCVR_CTRL & (uint32_t)~(uint32_t)(
                               XCVR_CTRL_XCVR_CTRL_RADIO0_IRQ_SEL_MASK
                              )) | (uint32_t)(
                               (0 << XCVR_CTRL_XCVR_CTRL_RADIO0_IRQ_SEL_SHIFT)
                              ));

    OSA_InstallIntHandler(Radio_0_IRQn, &Controller_InterruptHandler);

    NVIC_ClearPendingIRQ(Radio_0_IRQn);
    NVIC_EnableIRQ(Radio_0_IRQn);
    NVIC_SetPriority(Radio_0_IRQn, 0x80 >> (8 - __NVIC_PRIO_BITS));

#else
    #error "No valid CPU defined!"
#endif

    /* Set Default Tx Power Level */
    Controller_SetTxPowerLevel(mAdvertisingDefaultTxPower_c, gAdvTxChannel_c);
    Controller_SetTxPowerLevel(mConnectionDefaultTxPower_c, gConnTxChannel_c);


#if gMWS_UseCoexistence_d
    XCVR_CoexistenceInit();
    MWS_CoexistenceInit(NULL, NULL, NULL);
    MWS_CoexistenceRegister(gMWS_BLE_c, mws_co_callback);
    MWS_CoexistenceEnable();
#elif gMWS_Enabled_d
    MWS_Register(gMWS_BLE_c, mws_callback);
#endif

    /* Configure BD_ADDR before calling Controller_Init */
    ControllerSetBD_ADDR();

    return osaStatus_Success;
}

/*! *********************************************************************************
* @}
********************************************************************************** */


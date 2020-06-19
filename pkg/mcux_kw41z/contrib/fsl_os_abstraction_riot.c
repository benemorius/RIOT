/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
* All rights reserved.
*
* \file
*
* This is the source file for the OS Abstraction layer for freertos.
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include <string.h>
#include <stdlib.h>

#include "FunctionLib.h"
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"
#include "fsl_os_abstraction_riot.h"
#include "GenericList.h"
#include "fsl_common.h"
#include "Panic.h"

/* Fwk */
#include "fsl_os_abstraction.h"
#include "MemManager.h"
#include "TimersManager.h"
#include "RNG_Interface.h"
#include "Messaging.h"
#include "Flash_Adapter.h"
#include "SecLib.h"
#include "Panic.h"

/* KSDK */
#include "mcux_board.h"

/* Bluetooth Low Energy */
#include "gatt_interface.h"
#include "gatt_server_interface.h"
#include "gatt_client_interface.h"
#include "gap_interface.h"
#include "ble_init.h"
#include "ble_config.h"
#include "l2ca_cb_interface.h"

#include "ble_general.h"
#include "hci_transport.h"
#include "controller_interface.h"
#include "fsl_xcvr.h"
#include "ble_host_task_config.h"
#include "ble_controller_task_config.h"

#include "xtimer.h"
#include "event.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

extern void (*pfBLE_SignalFromISR)(void);
extern void BleApp_GenericCallback (gapGenericEvent_t* pGenericEvent);

/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#define millisecToTicks(millisec) ((millisec * configTICK_RATE_HZ + 999)/1000)

#ifdef DEBUG_ASSERT
#define OS_ASSERT(condition) if(!(condition))while(1);
#else
#define OS_ASSERT(condition) (void)(condition);
#endif

/* Application Events */
#define gAppEvtMsgFromHostStack_c       (1 << 0)
#define gAppEvtAppCallback_c            (1 << 1)

#ifndef cMCU_SleepDuringBleEvents
#define cMCU_SleepDuringBleEvents    0
#endif

#if (osNumberOfEvents)
#define osObjectAlloc_c 1
#else
#define osObjectAlloc_c 0
#endif

/*! @brief Converts milliseconds to ticks*/
#define MSEC_TO_TICK(msec)  (((uint32_t)(msec)+500uL/(uint32_t)configTICK_RATE_HZ) \
                             *(uint32_t)configTICK_RATE_HZ/1000uL)
#define TICKS_TO_MSEC(tick) ((tick)*1000uL/(uint32_t)configTICK_RATE_HZ)
/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

typedef struct osEventStruct_tag
{
    uint32_t inUse;
    eventt_t event;
}osEventStruct_t;

typedef struct osObjStruct_tag
{
    uint32_t inUse;
    uint32_t osObj;
}osObjStruct_t;

typedef struct osObjectInfo_tag
{
    void* pHeap;
    uint32_t objectStructSize;
    uint32_t objNo;
} osObjectInfo_t;


/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
#if osObjectAlloc_c
static void* osObjectAlloc(const osObjectInfo_t* pOsObjectInfo);
static bool_t osObjectIsAllocated(const osObjectInfo_t* pOsObjectInfo, void* pObjectStruct);
// static void osObjectFree(const osObjectInfo_t* pOsObjectInfo, void* pObjectStruct);
#endif

void startup_task(void* argument);


/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
const uint8_t gUseRtos_c = USE_RTOS;  // USE_RTOS = 0 for BareMetal and 1 for OS
// static uint32_t g_base_priority_array[OSA_MAX_ISR_CRITICAL_SECTION_DEPTH];
// static int32_t  g_base_priority_top = 0;

/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */

#if osNumberOfEvents
osEventStruct_t osEventHeap[osNumberOfEvents];
const osObjectInfo_t osEventInfo = {osEventHeap, sizeof(osEventStruct_t),osNumberOfEvents};
#endif

static osaEventId_t  mAppEvent;

/* Application input queues */
static anchor_t mHostAppInputQueue;
static anchor_t mAppCbInputQueue;

/************************************************************************************
 * ************************************************************************************
 * Private type definitions
 *************************************************************************************
 ************************************************************************************/
/* Host to Application Messages Types */
typedef enum {
    gAppGapGenericMsg_c = 0,
    gAppGapConnectionMsg_c,
    gAppGapAdvertisementMsg_c,
    gAppGapScanMsg_c,
    gAppGattServerMsg_c,
    gAppGattClientProcedureMsg_c,
    gAppGattClientNotificationMsg_c,
    gAppGattClientIndicationMsg_c,
    gAppL2caLeDataMsg_c,
    gAppL2caLeControlMsg_c,
}appHostMsgType_tag;

typedef uint8_t appHostMsgType_t;

/* Host to Application Connection Message */
typedef struct connectionMsg_tag{
    deviceId_t              deviceId;
    gapConnectionEvent_t    connEvent;
}connectionMsg_t;

/* Host to Application GATT Server Message */
typedef struct gattServerMsg_tag{
    deviceId_t          deviceId;
    gattServerEvent_t   serverEvent;
}gattServerMsg_t;

/* Host to Application GATT Client Procedure Message */
typedef struct gattClientProcMsg_tag{
    deviceId_t              deviceId;
    gattProcedureType_t     procedureType;
    gattProcedureResult_t   procedureResult;
    bleResult_t             error;
}gattClientProcMsg_t;

/* Host to Application GATT Client Notification/Indication Message */
typedef struct gattClientNotifIndMsg_tag{
    uint8_t*    aValue;
    uint16_t    characteristicValueHandle;
    uint16_t    valueLength;
    deviceId_t  deviceId;
}gattClientNotifIndMsg_t;

/* L2ca to Application Data Message */
typedef struct l2caLeCbDataMsg_tag{
    deviceId_t  deviceId;
    uint16_t    lePsm;
    uint16_t    packetLength;
    uint8_t     aPacket[0];
}l2caLeCbDataMsg_t;

/* L2ca to Application Control Message */
typedef struct l2caLeCbControlMsg_tag{
    l2capControlMessageType_t   messageType;
    uint16_t                    padding;
    uint8_t                     aMessage[0];
}l2caLeCbControlMsg_t;

typedef struct appMsgFromHost_tag{
    appHostMsgType_t    msgType;
    union {
        gapGenericEvent_t       genericMsg;
        gapAdvertisingEvent_t   advMsg;
        connectionMsg_t         connMsg;
        gapScanningEvent_t      scanMsg;
        gattServerMsg_t         gattServerMsg;
        gattClientProcMsg_t     gattClientProcMsg;
        gattClientNotifIndMsg_t gattClientNotifIndMsg;
        l2caLeCbDataMsg_t       l2caLeCbDataMsg;
        l2caLeCbControlMsg_t    l2caLeCbControlMsg;
    } msgData;
}appMsgFromHost_t;

// typedef PACKED_STRUCT hardwareParameters_tag
// {
//     uint8_t  identificationWord[10];   /* valid data present */
//     uint8_t  reserved[16];             /* for backward compatibillity */
//     uint8_t  zbInstallCode[16];        /* Install code used for joining to a Zigbee network as specified in the Base Device Behavior spec.
//                                           A value of all FFs means that a value defined in the application will be used. */
//     uint8_t  ieee_802_15_4_address[8]; /* IEEE 802.15.4 MAC address */
//     uint8_t  bluetooth_address[6];     /* Bluetooth address */
//     uint32_t xtalTrim;                 /* KW4x only */
//     uint32_t edCalibrationOffset;      /* KW01 ED offset */
//     uint32_t pllFStepOffset;           /* KW01 fine tune pll */
//     uint32_t gInternalStorageAddr;     /* The start address of the internal storage used for OTA update.
//                                           A value of 0xFFFFFFFF means that the External storage is used.
//                                           Warning: The offset to this field in respect to the start address of the structure
//                                           must not be changed.*/
//     /* For forward compatibility additional fields may be added here
//        Existing data in flash will not be compatible after modifying the hardwareParameters_t typedef*/
//     uint16_t hardwareParamsCrc;        /* crc for data between start of reserved area and start of hardwareParamsCrc field (not included). */
// }hardwareParameters_t;

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

// hardwareParameters_t gHardwareParameters;

extern bool_t gMCUSleepDuringBleEvents;

inline void panic(uint32_t id, uint32_t location, uint32_t extra1, uint32_t extra2)
{
    (void)id;
    (void)location;
    (void)extra1;
    (void)extra2;

    assert(0);
}

bleResult_t Ble_Initialize(gapGenericCallback_t gapGenericCallback)
{
    /* BLE Radio Init */
    XCVR_Init(BLE_MODE, DR_1MBPS);
    XCVR_SetXtalTrim( (uint8_t)gHardwareParameters.xtalTrim );

    if (osaStatus_Success != Controller_TaskInit())
    {
        return gBleOsError_c;
    }

    gMCUSleepDuringBleEvents = cMCU_SleepDuringBleEvents;

    /* BLE Controller Init */
    if (osaStatus_Success != Controller_Init(Hcit_SendPacket))
    {
        return gBleOsError_c;
    }

    return 0;
}

/* Called by BLE when a connect is received */
static void BLE_SignalFromISRCallback(void)
{
//     printf("ISR\n");
//     PWR_DisallowDeviceToSleep();
}

void main_task(void *param)
{
    uint8_t pseudoRNGSeed[20] = {0};

    /* Framework init */
    MEM_Init();
    TMR_Init();
//     LED_Init();
    SecLib_Init();

    RNG_Init();
    RNG_GetRandomNo((uint32_t*)(&(pseudoRNGSeed[0])));
    RNG_GetRandomNo((uint32_t*)(&(pseudoRNGSeed[4])));
    RNG_GetRandomNo((uint32_t*)(&(pseudoRNGSeed[8])));
    RNG_GetRandomNo((uint32_t*)(&(pseudoRNGSeed[12])));
    RNG_GetRandomNo((uint32_t*)(&(pseudoRNGSeed[16])));
    RNG_SetPseudoRandomNoSeed(pseudoRNGSeed);

#if gAppUseNvm_d
    /* Initialize NV module */
    NvModuleInit();
#endif

#if !gUseHciTransportDownward_d
    pfBLE_SignalFromISR = BLE_SignalFromISRCallback;
#endif /* !gUseHciTransportDownward_d */

#if (cPWR_UsePowerDownMode || gAppUseNvm_d)
#if (!mAppIdleHook_c)
//     AppIdle_TaskInit();
#endif
#endif

#if (cPWR_UsePowerDownMode)
//     PWR_Init();
#endif

    /* Initialize peripheral drivers specific to the application */
//     BleApp_Init();

    /* Create application event */
    mAppEvent = OSA_EventCreate(TRUE);
    if( NULL == mAppEvent )
    {
        panic(0,0,0,0);
        return;
    }

    /* Prepare application input queue.*/
    MSG_InitQueue(&mHostAppInputQueue);

    /* Prepare callback input queue.*/
    MSG_InitQueue(&mAppCbInputQueue);

    /* BLE Host Stack Init */
    if (Ble_Initialize(NULL) != gBleSuccess_c)
    {
        panic(0,0,0,0);
        return;
    }

    /* Call application task */
//     App_Thread( param );
}

/*FUNCTION**********************************************************************
 *
 * Function Name : startup_task
 * Description   : Wrapper over main_task..
 *
 *END**************************************************************************/
void startup_task(void* argument)
{
    /* Enable RSIM oscillator in RUN and WAIT modes, in order to be able to
     * access the XCVR and ZLL registers when using the internal reference clock
     * for the CPU core */
    RSIM->CONTROL |= RSIM_CONTROL_RF_OSC_EN(1);

    /* Wait for oscillator ready signal */
    while ((RSIM->CONTROL & RSIM_CONTROL_RF_OSC_READY_MASK) == 0) {}

    main_task(argument);

    /* shouldn't get here */
//     assert(0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskGetId
 * Description   : This function is used to get current active task's handler.
 *
 *END**************************************************************************/
osaTaskId_t OSA_TaskGetId(void)
{
    return (osaTaskId_t)thread_get(thread_getpid());
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskYield
 * Description   : When a task calls this function, it will give up CPU and put
 * itself to the tail of ready list.
 *
 *END**************************************************************************/
osaStatus_t OSA_TaskYield(void)
{
    thread_yield();
    return osaStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskGetPriority
 * Description   : This function returns task's priority by task handler.
 *
 *END**************************************************************************/
osaTaskPriority_t OSA_TaskGetPriority(osaTaskId_t taskId)
{
  return thread_get(thread_getpid())->priority;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskSetPriority
 * Description   : This function sets task's priority by task handler.
 *
 *END**************************************************************************/
osaStatus_t OSA_TaskSetPriority(osaTaskId_t taskId, osaTaskPriority_t taskPriority)
{
    ((thread_t *)taskId)->priority = taskPriority;
    return osaStatus_Success;
}

/*FUNCTION**********************************************************************
*
* Function Name : OSA_TaskCreate
* Description   : This function is used to create a task and make it ready.
* Param[in]     :  threadDef  - Definition of the thread.
*                  task_param - Parameter to pass to the new thread.
* Return Thread handle of the new thread, or NULL if failed.
*
*END**************************************************************************/
osaTaskId_t OSA_TaskCreate(osaThreadDef_t *thread_def,osaTaskParam_t task_param)
{

    osaTaskId_t taskId = NULL;

    char *p = malloc(thread_def->stacksize * 2);
    assert(p);
//     thread_def->tstack = (uint32_t *)p;

    kernel_pid_t pid = thread_create(p,
                            thread_def->stacksize * 2,
                            PRIORITY_OSA_TO_RTOS(thread_def->tpriority),
                            THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD,
                            (thread_task_func_t)thread_def->pthread, (void *)task_param,
                            (char const*)thread_def->tname);
    taskId = (osaTaskId_t)thread_get(pid);

    DEBUG("taskcreate %s stack: %lu pid: %u 0x%lx\n", thread_def->tname, thread_def->stacksize, pid, (uint32_t)thread_def->pthread);

    return taskId;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskDestroy
 * Description   : This function destroy a task.
 * Param[in]     :taskId - Thread handle.
 * Return osaStatus_Success if the task is destroied, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_TaskDestroy(osaTaskId_t taskId)
{
  osaStatus_t status;
  uint16_t oldPriority;
  /*Change priority to avoid context switches*/
  oldPriority = OSA_TaskGetPriority(OSA_TaskGetId());
  (void)OSA_TaskSetPriority(OSA_TaskGetId(), OSA_PRIORITY_REAL_TIME);
#if INCLUDE_vTaskDelete /* vTaskDelete() enabled */
  vTaskDelete((task_handler_t)taskId);
  status = osaStatus_Success;
#else
  status = osaStatus_Error; /* vTaskDelete() not available */
#endif
  (void)OSA_TaskSetPriority(OSA_TaskGetId(), oldPriority);

  return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TimeDelay
 * Description   : This function is used to suspend the active thread for the given number of milliseconds.
 *
 *END**************************************************************************/
void OSA_TimeDelay(uint32_t millisec)
{
    xtimer_usleep(millisec * US_PER_MS);
}
/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TimeGetMsec
 * Description   : This function gets current time in milliseconds.
 *
 *END**************************************************************************/
uint32_t OSA_TimeGetMsec(void)
{
    return xtimer_now_usec() / US_PER_MS;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_SemaphoreCreate
 * Description   : This function is used to create a semaphore.
 * Return         : Semaphore handle of the new semaphore, or NULL if failed.
  *
 *END**************************************************************************/
osaSemaphoreId_t OSA_SemaphoreCreate(uint32_t initValue)
{
    DEBUG("sema create\n");
#if osNumberOfSemaphores
    static semaphore_t sem;
    sema_create(&sem, initValue);
    return (osaSemaphoreId_t)&sem;
#else
    (void)initValue;
    return NULL;
#endif

}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_SemaphoreDestroy
 * Description   : This function is used to destroy a semaphore.
 * Return        : osaStatus_Success if the semaphore is destroyed successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_SemaphoreDestroy(osaSemaphoreId_t semId)
{
  DEBUG("sema destroy\n");
#if osNumberOfSemaphores
  semaphore_t *sem = (semaphore_t *)semId;
  if(sem == NULL)
  {
    return osaStatus_Error;
  }
  sema_destroy(sem);
  return osaStatus_Success;
#else
  (void)semId;
  return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_SemaphoreWait
 * Description   : This function checks the semaphore's counting value, if it is
 * positive, decreases it and returns osaStatus_Success, otherwise, timeout
 * will be used for wait. The parameter timeout indicates how long should wait
 * in milliseconds. Pass osaWaitForever_c to wait indefinitely, pass 0 will
 * return osaStatus_Timeout immediately if semaphore is not positive.
 * This function returns osaStatus_Success if the semaphore is received, returns
 * osaStatus_Timeout if the semaphore is not received within the specified
 * 'timeout', returns osaStatus_Error if any errors occur during waiting.
 *
 *END**************************************************************************/
osaStatus_t OSA_SemaphoreWait(osaSemaphoreId_t semId, uint32_t millisec)
{
  DEBUG("sema wait\n");
#if osNumberOfSemaphores
  if(semId == NULL)
  {
    return osaStatus_Error;
  }
  semaphore_t *sem = (semaphore_t *)semId;
  int res = 0;

  /* Convert timeout from millisecond to tick. */
  if (millisec == osaWaitForever_c)
  {
    res = sema_wait(sem);
  }
  else
  {
    res = sema_wait_timed(sem, millisec * US_PER_MS);
  }

  if (res)
  {
    return osaStatus_Timeout; /* timeout */
  }
  else
  {
    return osaStatus_Success; /* semaphore taken */
  }

#else
  (void)semId;
  (void)millisec;
  return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_SemaphorePost
 * Description   : This function is used to wake up one task that wating on the
 * semaphore. If no task is waiting, increase the semaphore. The function returns
 * osaStatus_Success if the semaphre is post successfully, otherwise returns
 * osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_SemaphorePost(osaSemaphoreId_t semId)
{
    DEBUG("sema post\n");
#if osNumberOfSemaphores
    if(!semId)
    {
        return osaStatus_Error;
    }

    return sema_post((semaphore_t *)semId);

#else
    (void)semId;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MutexCreate
 * Description   : This function is used to create a mutex.
 * Return        : Mutex handle of the new mutex, or NULL if failed.
 *
 *END**************************************************************************/
osaMutexId_t OSA_MutexCreate(void)
{
    DEBUG("mutex create\n");
#if osNumberOfMutexes
    static mutex_t mutex;
    mutex_init(&mutex);
    return (osaMutexId_t)&mutex;
#else
    return NULL;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MutexLock
 * Description   : This function checks the mutex's status, if it is unlocked,
 * lock it and returns osaStatus_Success, otherwise, wait for the mutex.
 * This function returns osaStatus_Success if the mutex is obtained, returns
 * osaStatus_Error if any errors occur during waiting. If the mutex has been
 * locked, pass 0 as timeout will return osaStatus_Timeout immediately.
 *
 *END**************************************************************************/
osaStatus_t OSA_MutexLock(osaMutexId_t mutexId, uint32_t millisec)
{
    DEBUG("mutex lock\n");
#if osNumberOfMutexes
    mutex_t *mutex = (mutex_t *)mutexId;
    if(mutexId == NULL)
    {
     return osaStatus_Error;
    }

    int res = 0;

    /* Convert timeout from millisecond to tick. */
    if (millisec == osaWaitForever_c)
    {
        res = mutex_trylock(mutex);
    }
    else
    {
        res = mutex_trylock(mutex);
    }

    if (!res)
    {
        return osaStatus_Timeout; /* timeout */
    }
    else
    {
        return osaStatus_Success; /* semaphore taken */
    }
#else
    (void)mutexId;
    (void)millisec;
    return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MutexUnlock
 * Description   : This function is used to unlock a mutex.
 *
 *END**************************************************************************/
osaStatus_t OSA_MutexUnlock(osaMutexId_t mutexId)
{
    DEBUG("mutex unlock\n");
#if osNumberOfMutexes
  mutex_t *mutex = (mutex_t *)mutexId;
  if(mutexId == NULL)
  {
    return osaStatus_Error;
  }

  mutex_unlock(mutex);

  return osaStatus_Success;

#else
  (void)mutexId;
  return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MutexDestroy
 * Description   : This function is used to destroy a mutex.
 * Return        : osaStatus_Success if the lock object is destroyed successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_MutexDestroy(osaMutexId_t mutexId)
{
    DEBUG("mutex destroy\n");
#if osNumberOfMutexes
  if(mutexId == NULL)
  {
    return osaStatus_Error;
  }

  return osaStatus_Success;
#else
  (void)mutexId;
  return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventCreate
 * Description   : This function is used to create a event object.
 * Return        : Event handle of the new event, or NULL if failed.
 *
 *END**************************************************************************/
osaEventId_t OSA_EventCreate(bool_t autoClear)
{
    DEBUG("event create\n");
#if osNumberOfEvents
    osaEventId_t eventId;
    osEventStruct_t* pEventStruct;
    OSA_InterruptDisable();
    eventId = pEventStruct = osObjectAlloc(&osEventInfo);
    OSA_InterruptEnable();
    if(eventId == NULL)
    {
        return NULL;
    }

    pEventStruct->event.flags = 0;
    pEventStruct->event.autoClear = autoClear;

    DEBUG("create 0x%lx autoclear: %i\n", (uint32_t)eventId, autoClear);

    return eventId;
#else
  (void)autoClear;
  return NULL;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventSet
 * Description   : Set one or more event flags of an event object.
 * Return        : osaStatus_Success if set successfully, osaStatus_Error if failed.
 *
 *END**************************************************************************/
osaStatus_t OSA_EventSet(osaEventId_t eventId, osaEventFlags_t flagsToSet)
{
//     DEBUG("set 0x%lx flags: 0x%08lx\n", (uint32_t)eventId, flagsToSet);

#if osNumberOfEvents

    /* thread_flags_set() is limited to 16 bits */
    assert(!(flagsToSet & 0xffff0000));

    osEventStruct_t *pEventStruct = (osEventStruct_t*)eventId;

    unsigned irq = irq_disable();

    pEventStruct->event.flags |= flagsToSet;

    for (int i = 0; i < 31; i++) {
        if (pEventStruct->event.threads & (1 << i)) {
//             DEBUG("thread %u\n", i);
            thread_flags_set((thread_t *)thread_get(i), flagsToSet);
        }
    }

    irq_restore(irq);

    return osaStatus_Success;

#else
  (void)eventId;
  (void)flagsToSet;
  return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventClear
 * Description   : Clear one or more event flags of an event object.
 * Return        :osaStatus_Success if clear successfully, osaStatus_Error if failed.
 *
 *END**************************************************************************/
osaStatus_t OSA_EventClear(osaEventId_t eventId, osaEventFlags_t flagsToClear)
{
#if osNumberOfEvents
  return osaStatus_Success;
#else
  (void)eventId;
  (void)flagsToClear;
  return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventWait
 * Description   : This function checks the event's status, if it meets the wait
 * condition, return osaStatus_Success, otherwise, timeout will be used for
 * wait. The parameter timeout indicates how long should wait in milliseconds.
 * Pass osaWaitForever_c to wait indefinitely, pass 0 will return the value
 * osaStatus_Timeout immediately if wait condition is not met. The event flags
 * will be cleared if the event is auto clear mode. Flags that wakeup waiting
 * task could be obtained from the parameter setFlags.
 * This function returns osaStatus_Success if wait condition is met, returns
 * osaStatus_Timeout if wait condition is not met within the specified
 * 'timeout', returns osaStatus_Error if any errors occur during waiting.
 *
 *END**************************************************************************/
osaStatus_t OSA_EventWait(osaEventId_t eventId, osaEventFlags_t flagsToWait, bool_t waitAll, uint32_t millisec, osaEventFlags_t *pSetFlags)
{
//     DEBUG("wait 0x%lx pid: %u flags: 0x%lx msec: 0x%lx\n", (uint32_t)eventId,
//            thread_getpid(), flagsToWait, millisec);

#if osNumberOfEvents
    osEventStruct_t* pEventStruct;
    uint32_t flagsSave;
    if(osObjectIsAllocated(&osEventInfo, eventId) == false)
    {
        return osaStatus_Error;
    }

    /* Clean FreeRTOS cotrol flags */
    flagsToWait = flagsToWait & 0xFFFF;

    pEventStruct = (osEventStruct_t*)eventId;

    unsigned irq = irq_disable();
    pEventStruct->event.threads |= 1 << thread_getpid();

    if ((pEventStruct->event.flags & flagsToWait) == 0)
    {
        irq_restore(irq);
        xtimer_t timeout;
        if (millisec != osaWaitForever_c)
        {
//             DEBUG("wake after %lu ms\n", millisec);
            xtimer_set_timeout_flag(&timeout, millisec * US_PER_MS);
        }
//         thread_flags_clear(0xffffffff);
        uint32_t f = thread_flags_wait_any(flagsToWait);
        (void)f;
//         DEBUG("woke %u flags 0x%lx\n", thread_getpid(), f);
        xtimer_remove(&timeout);
        irq = irq_disable();
    } else {
//         DEBUG("no sleep %i\n", thread_getpid());
    }


    flagsSave = pEventStruct->event.flags & flagsToWait;

//     if (flagsSave == 0) {
//         flagsSave = 0x1;
//     }

    if (flagsSave == 0) {
        irq_restore(irq);
        return osaStatus_Timeout;
    }

    pEventStruct->event.threads &= ~(1UL << thread_getpid());

//     DEBUG("flags 0x%lx\n", flagsSave);

    if (pEventStruct->event.autoClear) {
        pEventStruct->event.flags &= ~(flagsSave);
        thread_flags_clear(flagsSave);
    }

    irq_restore(irq);

    if(pSetFlags) {
        *pSetFlags = flagsSave;
    }

    if (flagsSave) {
        return osaStatus_Success;
    } else {
        return osaStatus_Timeout;
    }

#else
  (void)eventId;
  (void)flagsToWait;
  (void)waitAll;
  (void)millisec;
  (void)pSetFlags;
  return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventDestroy
 * Description   : This function is used to destroy a event object. Return
 * osaStatus_Success if the event object is destroyed successfully, otherwise
 * return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_EventDestroy(osaEventId_t eventId)
{
    DEBUG("event destroy\n");
#if osNumberOfEvents
  return osaStatus_Success;
#else
  (void)eventId;
  return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MsgQCreate
 * Description   : This function is used to create a message queue.
 * Return        : the handle to the message queue if create successfully, otherwise
 * return NULL.
 *
 *END**************************************************************************/
osaMsgQId_t OSA_MsgQCreate( uint32_t  msgNo )
{
#if osNumberOfMessageQs
    msg_queue_handler_t msg_queue_handler;

    /* Create the message queue where each element is a pointer to the message item. */
    msg_queue_handler = xQueueCreate(msgNo,sizeof(osaMsg_t));
    return (osaMsgQId_t)msg_queue_handler;
#else
    (void)msgNo;
    return NULL;
#endif
}


/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MsgQPut
 * Description   : This function is used to put a message to a message queue.
* Return         : osaStatus_Success if the message is put successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_MsgQPut(osaMsgQId_t msgQId, void* pMessage)
{
#if osNumberOfMessageQs
  msg_queue_handler_t handler;
  osaStatus_t osaStatus;
  if(msgQId == NULL)
  {
    return osaStatus_Error;
  }
  handler = (msg_queue_handler_t)msgQId;
  {
    if (__get_IPSR())
    {
      portBASE_TYPE taskToWake = pdFALSE;

      if (pdTRUE == xQueueSendToBackFromISR(handler, pMessage, &taskToWake))
      {
        if (pdTRUE == taskToWake)
        {
          portYIELD_FROM_ISR(taskToWake);
        }
        osaStatus = osaStatus_Success;
      }
      else
      {
        osaStatus =  osaStatus_Error;
      }

    }
    else
    {
      osaStatus = (xQueueSendToBack(handler, pMessage, 0)== pdPASS)?(osaStatus_Success):(osaStatus_Error);
    }
  }
  return osaStatus;
#else
  (void)msgQId;
  (void)pMessage;
  return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MsgQGet
 * Description   : This function checks the queue's status, if it is not empty,
 * get message from it and return osaStatus_Success, otherwise, timeout will
 * be used for wait. The parameter timeout indicates how long should wait in
 * milliseconds. Pass osaWaitForever_c to wait indefinitely, pass 0 will return
 * osaStatus_Timeout immediately if queue is empty.
 * This function returns osaStatus_Success if message is got successfully,
 * returns osaStatus_Timeout if message queue is empty within the specified
 * 'timeout', returns osaStatus_Error if any errors occur during waiting.
 *
 *END**************************************************************************/
osaStatus_t OSA_MsgQGet(osaMsgQId_t msgQId, void *pMessage, uint32_t millisec)
{
#if osNumberOfMessageQs
  osaStatus_t osaStatus;
  msg_queue_handler_t handler;
  uint32_t timeoutTicks;
  if( msgQId == NULL )
  {
    return osaStatus_Error;
  }
  handler = (msg_queue_handler_t)msgQId;
  if (millisec == osaWaitForever_c)
  {
    timeoutTicks = portMAX_DELAY;
  }
  else
  {
    timeoutTicks = MSEC_TO_TICK(millisec);
  }
  if (xQueueReceive(handler, pMessage, timeoutTicks)!=pdPASS)
  {
    osaStatus =  osaStatus_Timeout; /* not able to send it to the queue? */
  }
  else
  {
    osaStatus = osaStatus_Success;
  }
  return osaStatus;
#else
  (void)msgQId;
  (void)pMessage;
  (void)millisec;
  return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MsgQDestroy
 * Description   : This function is used to destroy the message queue.
 * Return        : osaStatus_Success if the message queue is destroyed successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_MsgQDestroy(osaMsgQId_t msgQId)
{
#if osNumberOfMessageQs
  msg_queue_handler_t handler;
  if(msgQId == NULL )
  {
    return osaStatus_Error;
  }
  handler = (msg_queue_handler_t)msgQId;
  vQueueDelete(handler);
  return osaStatus_Success;
#else
  (void)msgQId;
  return osaStatus_Error;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_InterruptEnable
 * Description   : self explanatory.
 *
 *END**************************************************************************/
void OSA_InterruptEnable(void)
{
    irq_enable();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_InterruptDisable
 * Description   : self explanatory.
 *
 *END**************************************************************************/
void OSA_InterruptDisable(void)
{
    irq_disable();
}

uint32_t gInterruptDisableCount = 0;
/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EnableIRQGlobal
 * Description   : enable interrupts using PRIMASK register.
 *
 *END**************************************************************************/
void OSA_EnableIRQGlobal(void)
{
    if (gInterruptDisableCount > 0)
    {
        gInterruptDisableCount--;

        if (gInterruptDisableCount == 0)
        {
            __enable_irq();
        }
        /* call core API to enable the global interrupt*/
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_DisableIRQGlobal
 * Description   : disable interrupts using PRIMASK register.
 *
 *END**************************************************************************/
void OSA_DisableIRQGlobal(void)
{
    /* call core API to disable the global interrupt*/
    __disable_irq();

    /* update counter*/
    gInterruptDisableCount++;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_InstallIntHandler
 * Description   : This function is used to install interrupt handler.
 *
 *END**************************************************************************/
void OSA_InstallIntHandler(uint32_t IRQNumber, void (*handler)(void))
{

#if defined ( __IAR_SYSTEMS_ICC__ )
    _Pragma ("diag_suppress = Pm138")
#endif
    InstallIRQHandler((IRQn_Type)IRQNumber, (uint32_t)handler);
#if defined ( __IAR_SYSTEMS_ICC__ )
    _Pragma ("diag_remark = PM138")
#endif
}

/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************** */
OSA_TASK_DEFINE(startup_task, gMainThreadPriority_c, 1, gMainThreadStackSize_c, 0);
int mcux_kw41z_main(void)
{
    void ble_hci_ram_init(void);
    ble_hci_ram_init();

    OSA_TaskCreate(OSA_TASK(startup_task), NULL);
    thread_yield();

    return 0;
}

/*! *********************************************************************************
* \brief     Allocates a osObjectStruct_t block in the osObjectHeap array.
* \param[in] pointer to the object info struct.
* Object can be semaphore, mutex, message Queue, event
* \return Pointer to the allocated osObjectStruct_t, NULL if failed.
*
* \pre
*
* \post
*
* \remarks Function is unprotected from interrupts.
*
********************************************************************************** */
#if osObjectAlloc_c
static void* osObjectAlloc(const osObjectInfo_t* pOsObjectInfo)
{
    uint32_t i;
    uint8_t* pObj = (uint8_t*)pOsObjectInfo->pHeap;
    for( i=0 ; i < pOsObjectInfo->objNo ; i++, pObj += pOsObjectInfo->objectStructSize)
    {
        if(((osObjStruct_t*)pObj)->inUse == 0)
        {
            ((osObjStruct_t*)pObj)->inUse = 1;
            return (void*)pObj;
        }
    }
    return NULL;
}
#endif

/*! *********************************************************************************
* \brief     Verifies the object is valid and allocated in the osObjectHeap array.
* \param[in] the pointer to the object info struct.
* \param[in] the pointer to the object struct.
* Object can be semaphore, mutex,  message Queue, event
* \return TRUE if the object is valid and allocated, FALSE otherwise
*
* \pre
*
* \post
*
* \remarks Function is unprotected from interrupts.
*
********************************************************************************** */
#if osObjectAlloc_c
static bool_t osObjectIsAllocated(const osObjectInfo_t* pOsObjectInfo, void* pObjectStruct)
{
    uint32_t i;
    uint8_t* pObj = (uint8_t*)pOsObjectInfo->pHeap;
    for( i=0 ; i < pOsObjectInfo->objNo ; i++ , pObj += pOsObjectInfo->objectStructSize)
    {
        if(pObj == pObjectStruct)
        {
            if(((osObjStruct_t*)pObj)->inUse)
            {
                return TRUE;
            }
            break;
        }
    }
    return FALSE;
}
#endif

/*! *********************************************************************************
* \brief     Frees an osObjectStruct_t block from the osObjectHeap array.
* \param[in] pointer to the object info struct.
* \param[in] Pointer to the allocated osObjectStruct_t to free.
* Object can be semaphore, mutex, message Queue, event
* \return none.
*
* \pre
*
* \post
*
* \remarks Function is unprotected from interrupts.
*
********************************************************************************** */
// #if osObjectAlloc_c
// static void osObjectFree(const osObjectInfo_t* pOsObjectInfo, void* pObjectStruct)
// {
//     uint32_t i;
//     uint8_t* pObj = (uint8_t*)pOsObjectInfo->pHeap;
//     for( i=0; i < pOsObjectInfo->objNo; i++, pObj += pOsObjectInfo->objectStructSize )
//     {
//         if(pObj == pObjectStruct)
//         {
//             ((osObjStruct_t*)pObj)->inUse = 0;
//             break;
//         }
//     }
// }
// #endif


uint32_t BOARD_GetTpmClock(uint32_t instance)
{
    uint32_t clock;
    uint32_t clockSource = (SIM->SOPT2 & SIM_SOPT2_TPMSRC_MASK) >> SIM_SOPT2_TPMSRC_SHIFT;

    instance = instance; /* Remove compiler warnings */

    switch(clockSource)
    {
        case 1: /* MCGFLLCLK */
            clock = CLOCK_GetFllFreq();
            break;
        case 2: /* OSCERCLK */
            clock = CLOCK_GetOsc0ErClkFreq();
            break;
        case 3: /* MCGIRCLK */
            clock = CLOCK_GetInternalRefClkFreq();
            break;
        default: /* Clock disabled */
            clock = 0;
            break;
    }

    return clock;
}

void BOARD_GetMCUUid(uint8_t* aOutUid16B, uint8_t* pOutLen)
{
    uint32_t uid[4] = {0};

    uid[0] = SIM->SDID;
    uid[1] = SIM->UIDMH;
    uid[2] = SIM->UIDML;
    uid[3] = SIM->UIDL;

    FLib_MemCpy(aOutUid16B, (uint8_t*)uid, sizeof(uid));
    *pOutLen = sizeof(uid);

    return;
}

uint32_t BOARD_GetLpuartClock(uint32_t instance)
{
    uint32_t clock;
    uint32_t clockSource = (SIM->SOPT2 & SIM_SOPT2_LPUART0SRC_MASK) >> SIM_SOPT2_LPUART0SRC_SHIFT;

    instance = instance; /* Remove compiler warnings */

    switch(clockSource)
    {
        case 1: /* MCGFLLCLK */
            clock = CLOCK_GetFllFreq();
            break;
        case 2: /* OSCERCLK */
            clock = CLOCK_OSCERCLK;
            break;
        case 3: /* MCGIRCLK */
            clock = CLOCK_GetInternalRefClkFreq();
            break;
        default: /* Clock disabled */
            clock = 0;
            break;
    }

    return clock;
}

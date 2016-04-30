/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup        cpu_efm32
 * @brief           EFM32 specific code
 * @ingroup         cpu
 * @{
 *
 * @file
 * @brief           Implementation specific CPU configuration options
 *                  Based on STM32F0 example
 *
 * @author          Hauke Petersen <hauke.peterse@fu-berlin.de>
 */

#ifndef __CPU_CONF_H
#define __CPU_CONF_H

#define __CM3_REV               0x0200U
#define __MPU_PRESENT             0U
#define __NVIC_PRIO_BITS          4U
#define __Vendor_SysTickConfig    0U

#include "cpu_conf_common.h"
#include "cc2650.h"
#include "core_cm3.h"

/**
 * @name Threading configuration
 * 
 */

#define SVC_ISR			isr_svc
#define PEND_SV_ISR 	isr_pendsv


/**
 * @brief   ARM Cortex-M specific CPU configuration
 * @{
 */
#define CPU_DEFAULT_IRQ_PRIO            (1U)
#define CPU_IRQ_NUMOF                   (60U)
#define CPU_FLASH_BASE                  (0U)
/** @} */



#define CPUID_ID_LEN (8)

/* #define KERNEL_CONF_STACKSIZE_IDLE      (192) */
#define KERNEL_CONF_STACKSIZE_IDLE      (512)
/** @} */


#define HAVE_NO_BUILTIN_BSWAP16


#endif /* __CPU_CONF_H */
/** @} */

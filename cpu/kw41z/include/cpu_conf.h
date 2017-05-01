/*
 * Copyright (C) 2017 SKF AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup        cpu_kw41z NXP Kinetis KW41Z, KW31Z, KW21Z
 * @ingroup         cpu
 * @brief           CPU specific definitions for the NXP Kinetis KW41Z series SoC
 *
 *                  The SoC incorporates a low power 2.4 GHz transceiver and a
 *                  Kinetis Cortex-M0+ MCU.
 * @{
 *
 * @file
 * @brief           Implementation specific CPU configuration options
 *
 * @author          Joakim Nohlgård <joakim.nohlgard@eistec.se>
 */

#ifndef CPU_CONF_H
#define CPU_CONF_H

#if defined(CPU_MODEL_MKW41Z512VHT4) || defined(CPU_MODEL_MKW41Z256VHT4)
#include "vendor/MKW41Z4.h"
#elif defined(CPU_MODEL_MKW31Z512VHT4) || defined(CPU_MODEL_MKW31Z256VHT4)
#include "vendor/MKW31Z4.h"
#elif defined(CPU_MODEL_MKW21Z512VHT4) || defined(CPU_MODEL_MKW21Z256VHT4)
#include "vendor/MKW21Z4.h"
#else
#error "unknown or undefined CPU_MODEL"
#endif

#include "cpu_conf_kinetis.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif /* CPU_CONF_H */
/** @} */

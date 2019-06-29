/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup   boards_openlabs-6lowpan-led-driver
 * @{
 *
 * @file
 * @brief     Board specific configuration of direct mapped ADC
 *
 * @author    Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author    Thomas Stilwell <stilwellt@openlabs.co>
 */

#ifndef ADC_PARAMS_H
#define ADC_PARAMS_H

#include "board.h"
#include "saul/periph.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief    ADC configuration
 */
static const  saul_adc_params_t saul_adc_params[] =
{
    {
        .name = "coretemp",
        .line = ADC_LINE(0),
        .res  = ADC_RES_16BIT,
    },
    {
        .name = "corebandgap",
        .line = ADC_LINE(1),
        .res  = ADC_RES_16BIT,
    },
    {
        .name = "dcdcvbat",
        .line = ADC_LINE(2),
        .res  = ADC_RES_16BIT,
    },
    {
        .name = "supply",
        .line = ADC_LINE(3),
        .res  = ADC_RES_16BIT,
    },
    {
        .name = "current channel W",
        .line = ADC_LINE(4),
        .res  = ADC_RES_16BIT,
    },
    {
        .name = "not connected",
        .line = ADC_LINE(5),
        .res  = ADC_RES_16BIT,
    },
};

#ifdef __cplusplus
}
#endif

#endif /* ADC_PARAMS_H */
/** @} */

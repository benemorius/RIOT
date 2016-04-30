/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32f4
 * @{
 *
 * @file
 * @brief       Low-level ADC driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "cpu.h"
#include "periph/adc.h"
#include "periph_conf.h"

/* guard in case that no ADC device is defined */
#if ADC_NUMOF

typedef struct {
    int max_value;
} adc_config_t;

adc_config_t adc_config[ADC_NUMOF];

int adc_init(adc_t line)
{
    return 0;
}

int adc_sample(adc_t line, adc_res_t res)
{
    return 0;
}

void adc_poweron(adc_t dev)
{
}

void adc_poweroff(adc_t dev)
{
}

int adc_map(adc_t dev, int value, int min, int max)
{
    return 0;
}

float adc_mapf(adc_t dev, int value, float min, float max)
{
    return 0;
}

#endif /* ADC_NUMOF */

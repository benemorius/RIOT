/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup   boards_openlabs-kw41z-slip
 * @{
 *
 * @file
 * @brief     Board specific configuration of direct mapped GPIOs
 *
 * @author    Thomas Stilwell <stilwellt@openlabs.co>
 */

#ifndef GPIO_PARAMS_H
#define GPIO_PARAMS_H

#include "board.h"
#include "saul/periph.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief    SAUL configuration
 */
static const  saul_gpio_params_t saul_gpio_params[] =
{
    {
        .name = "LEDRX",
        .pin = LED0_PIN,
        .mode = GPIO_OUT,
    },
    {
        .name = "LEDTX",
        .pin = LED1_PIN,
        .mode = GPIO_OUT,
    },
};

#ifdef __cplusplus
}
#endif

#endif /* GPIO_PARAMS_H */
/** @} */

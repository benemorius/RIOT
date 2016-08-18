/*
 * Copyright (C) 2016 Leon George
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_cc26x0
 * @{
 *
 * @file
 * @brief       implementation of the kernels power management interface
 *
 * @}
 */

#include "arch/lpm_arch.h"
#include "cpu.h"

static enum lpm_mode current_mode = LPM_UNKNOWN;

void lpm_arch_init(void)
{
}

enum lpm_mode lpm_arch_set(enum lpm_mode target)
{
    enum lpm_mode last_mode = current_mode;

//     target = LPM_ON;

    switch (target) {
        case LPM_ON:
        case LPM_UNKNOWN:
            current_mode = LPM_ON;
            break;

        case LPM_IDLE:
        case LPM_SLEEP:
        case LPM_POWERDOWN:
        case LPM_OFF:
            current_mode = LPM_IDLE;
            SCB->SCR &= ~(SCB_SCR_SLEEPDEEP_Msk);

            //wait for uart tx to complete
            while (UART->FR & UART_FR_BUSY) {}

            __WFI();
            break;
    }
    return last_mode;
}

enum lpm_mode lpm_arch_get(void)
{
    return current_mode;
}

void lpm_arch_awake(void)
{
}

void lpm_arch_begin_awake(void)
{
}

void lpm_arch_end_awake(void)
{
}

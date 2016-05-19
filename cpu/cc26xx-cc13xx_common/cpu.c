/*
 *    Copyright (c) 2016 Thomas Stilwell <stilwellt@openlabs.co>
 *
 *    Permission is hereby granted, free of charge, to any person
 *    obtaining a copy of this software and associated documentation
 *    files (the "Software"), to deal in the Software without
 *    restriction, including without limitation the rights to use,
 *    copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the
 *    Software is furnished to do so, subject to the following
 *    conditions:
 *
 *    The above copyright notice and this permission notice shall be
 *    included in all copies or substantial portions of the Software.
 *
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *    OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @ingroup     cpu_cc26xx-cc13xx
 * @{
 *
 * @file
 * @brief       Implementation of the CPU initialization
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 * @}
 */

#include "cpu.h"
#include "periph_conf.h"
#include "periph/cpuid.h"

#include <string.h>

extern void board_init(void);
extern void __libc_init_array(void);
extern void kernel_init(void);
extern void isr_svc(void);
extern void isr_pendsv(void);

static void cpu_clock_init(void);

/**
 * @brief Initialize the CPU, set IRQ priorities
 */
void cpu_init(void)
{
    /* initialize the Cortex-M core */
    cortexm_init();
    /* initialize the clock system */
    cpu_clock_init();
}

/**
 * @brief Configure the controllers clock system
 *
 */
static void cpu_clock_init(void)
{
//     /*  Enable high speed crystal */
//     CMU->OSCENCMD |= CMU_OSCENCMD_HFXOEN;
//
//     /*  Wait for high speed crystal to stabilise */
//     while (!(CMU->STATUS & CMU_STATUS_HFXORDY)) {
//         ;
//     }
//
//     /*  Set high speed crystal as core clock with divisor of 1 */
//     CMU->CMD |= CMU_CMD_HFCLKSEL_HFXO;
// //     CMU->CTRL |= (1 << 14);
//
//     /*  Wait for clock switch */
//     while ((CMU->STATUS & CMU_STATUS_HFRCOSEL)) {
//         ;
//     }
//
//     /*  Disable high speed oscillator */
//     CMU->OSCENCMD |= CMU_OSCENCMD_HFRCODIS;
//
//     /*  Enable low energy interface (for watchdog) */
//     CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_LE;
}

/* Replacement start function */
/* STM port has this in the startup_ */
void _start(void)
{
    /* initialize the board and startup the kernel */
    board_init();
    /* initialize std-c library (this should be done after board_init) */
    __libc_init_array();
    /* startup the kernel */
    kernel_init();
}

void cpuid_get(void* id)
{
// 	void* cpuid = 0xFE081F0;
// 	memcpy(id, cpuid, CPUID_ID_LEN);
}

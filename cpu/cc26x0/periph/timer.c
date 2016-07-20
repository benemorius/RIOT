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
 * @brief       Low-level timer driver implementation for the CC26x0
 *
 * @author      Leon M. George <leon@georgemail.eu>
 *
 * @}
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "cpu.h"
#include "sched.h"
#include "thread.h"
#include "periph_conf.h"
#include "periph/timer.h"

/**
 * @brief   Allocate memory for the interrupt context
 */
static timer_isr_ctx_t ctx[TIMER_NUMOF];

/**
 * @brief           Get the GPT register base for a timer
 *
 * @param[in] tim   index of the timer
 *
 * @return          base address
 */
static inline gpt_reg_t *dev(tim_t tim)
{
    return timer_config[tim].dev;
}

int timer_init(tim_t tim, unsigned long freq, timer_cb_t cb, void *arg)
{
	printf("\n===> %s <===\n",__FUNCTION__);
    /* make sure given timer is valid */
    if (tim >= TIMER_NUMOF) {
        return -1;
    }

	/* enable the periph power domain */
	if(PRCM->PDSTAT0PERIPH != 1) {
		PRCM->PDCTL0 |= PDSTAT0_PERIPH_ON;
		while(PRCM->PDSTAT0PERIPH != 1);
	};

    /* enable the times clock */
    PRCM->GPTCLKGR |= (1 << timer_config[tim].num);
    PRCM->CLKLOADCTL = CLKLOADCTL_LOAD;
    while (!(PRCM->CLKLOADCTL & CLKLOADCTL_LOADDONE)) {}

    /* disable (and reset) timer */
	memset(dev(tim),0x0,sizeof(dev(tim)));
    dev(tim)->CTL &= ~(GPT_CTL_TAEN | GPT_CTL_TBEN);
	dev(tim)->CFG &= 0x00000000;

    /* save context */
    ctx[tim].cb = cb;
    ctx[tim].arg = arg;

    /* configure timer to 16-bit, periodic, up-counting. snapshot mode enabled (the actual free-running value of Timer A is loaded at the time-out event into the GPT Timer A (TAR) register.)*/
    dev(tim)->CFG  |= GPT_CFG_16T;
    dev(tim)->TAMR |= (GPT_TXMR_TXMR_PERIODIC | GPT_TXMR_TXCDIR_DOWN | GPT_TXMR_TXSNAPS);

    /* set the prescaler. clock runs at 48MhZ, with 48 prescaler the timer speed = 1 tick per usec */
    dev(tim)->TAPR = 47;

    /* enable global timer interrupt and set interrupt mask to transmit timeout event for timer A */
    timer_irq_enable(tim);
	dev(tim)->IMR |= GPT_IMR_TATOIM;
    return 0;
}

int timer_set(tim_t tim, int channel, unsigned int timeout)
{	printf("\n===> %s <===\n",__FUNCTION__);
    return timer_set_absolute(tim, channel, timer_read(tim) + timeout);
}

int timer_set_absolute(tim_t tim, int channel, unsigned int value)
{	//printf("\n===> %s <===\n",__FUNCTION__);
	/* disable timer */
    dev(tim)->CTL &= ~(GPT_CTL_TAEN | GPT_CTL_TBEN);

    if (channel != 0) {
        return -1;
    }

    dev(tim)->TAILR = value;
	dev(tim)->CTL |= GPT_CTL_TAEN;
    return 0;
}

int timer_clear(tim_t tim, int channel)
{
	printf("\n===> %s <===\n",__FUNCTION__);
    if (channel != 0) {
        return -1;
    }

    dev(tim)->IMR &= ~(GPT_IMR_TATOIM);

    return 0;
}

unsigned int timer_read(tim_t tim)
{
    return dev(tim)->TAV & 0xFFFF;
}

void timer_stop(tim_t tim)
{
	printf("\n===> %s <===\n",__FUNCTION__);
    dev(tim)->CTL &= ~(GPT_CTL_TAEN | GPT_CTL_TBEN);
}

void timer_start(tim_t tim)
{
	printf("\n===> %s <===\n",__FUNCTION__);
    dev(tim)->CTL |= GPT_CTL_TAEN;
}

void timer_irq_enable(tim_t tim)
{
    NVIC_EnableIRQ(GPTIMER_0A_IRQN + (2 * timer_config[tim].num));
}

void timer_irq_disable(tim_t tim)
{
    NVIC_DisableIRQ(GPTIMER_0A_IRQN + (2 * timer_config[tim].num));
}

/**
 * @brief           handle interrupt for a timer
 *
 * @param[in] tim   index of the timer
 */
static inline void isr_handler(tim_t tim)
{
    uint32_t mis = dev(tim)->MIS;
    dev(tim)->ICLR = mis;
    if (mis & GPT_IMR_TATOIM) {
        ctx[tim].cb(ctx[tim].arg, 0);
    }

    if (sched_context_switch_request) {
        thread_yield();
    }
}

#ifdef TIMER_0_ISR
void TIMER_0_ISR(void)
{
    isr_handler(0);
}
#endif

#ifdef TIMER_1_ISR
void TIMER_1_ISR(void)
{
    isr_handler(1);
}
#endif

#ifdef TIMER_2_ISR
void TIMER_2_ISR(void)
{
    isr_handler(2);
}
#endif

#ifdef TIMER_3_ISR
void TIMER_3_ISR(void)
{
    isr_handler(3);
}
#endif

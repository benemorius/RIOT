/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_efm32
 * @{
 *
 * @file
 * @brief       Low-level timer driver implementation
 *
 * @author      Ryan Kurte
 *
 * @}
 */

#include <stdlib.h>

#include "cpu.h"
#include "board.h"
#include "periph_conf.h"
#include "thread.h"
#include "periph/timer.h"
#include "lpm.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

// static inline void timer_irq_handler(tim_t dev, TIMER_TypeDef *timer);

int timer_init(tim_t dev, unsigned long freq, timer_cb_t cb, void *arg)
{
    DEBUG("timer init: %i (0:%i 1:%i 2:%i)\n", (int)dev, TIMER_0, TIMER_1, TIMER_2);
    return 0;
}

//Set an interrupt on the specified channel n ticks from now
int timer_set(tim_t dev, int channel, unsigned int timeout)
{
    DEBUG("timer_set(): timer %d channel %d timeout %u\n", dev, channel, timeout);
    int now = timer_read(dev);
    return timer_set_absolute(dev, channel, now + timeout - 1);
}

int timer_set_absolute(tim_t dev, int channel, unsigned int value)
{
    return 0;
}

// Clear the event from a timer channel
int timer_clear(tim_t dev, int channel)
{
    return 0;
}

//Fetch timer count in ticks
unsigned int timer_read(tim_t dev)
{
    return 0;
}

//Start the timer
void timer_start(tim_t dev)
{

}


//Stop the timer
void timer_stop(tim_t dev)
{
}

//Enable timer interrupts
void timer_irq_enable(tim_t dev)
{
    DEBUG("timer_irq_enable(): timer %d\n", dev);
}

//Disable timer interrupts
void timer_irq_disable(tim_t dev)
{
    DEBUG("timer_irq_disable(): timer %d\n", dev);
}

//Reset the timer
void timer_reset(tim_t dev)
{
    DEBUG("timer_reset(): timer %d\n", dev);
}

#ifdef TIMER_0_EN
__attribute__((interrupt("IRQ"))) void TIMER_0_ISR(void)
{
//     ISR_ENTER();
//     timer_irq_handler(TIMER_0, TIMER_0_DEV);
    if (sched_context_switch_request) {
       thread_yield();
    }
//     ISR_EXIT();
}
#endif

#ifdef TIMER_1_EN
__attribute__((interrupt("IRQ"))) void TIMER_1_ISR(void)
{
//     ISR_ENTER();
//     timer_irq_handler(TIMER_1, TIMER_1_DEV);
    if (sched_context_switch_request) {
       thread_yield();
    }
//     ISR_EXIT();
}
#endif

#ifdef TIMER_2_EN
__attribute__((interrupt("IRQ"))) void TIMER_2_ISR(void)
{
// 	ISR_ENTER();

// 	if(lpm_get() >= LPM_SLEEP)
// 		lpm_awake();

	DEBUG("timer irq_handler(): timer %d\n", 2);


	if (sched_context_switch_request) {
        thread_yield();
    }

// 	ISR_EXIT();
}
#endif

// static inline void timer_irq_handler(tim_t dev, TIMER_TypeDef *timer)
// {
//     DEBUG("timer irq_handler(): timer %d\n", dev);
//
// }

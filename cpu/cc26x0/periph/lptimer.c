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
 * @ingroup     cpu_cc26x0
 * @{
 *
 * @file
 * @brief       Low-level low-power timer driver implementation
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include <stdlib.h>

#include "board.h"
#include "periph/timer.h"
#include "lpm.h"
#include "hw_aon_rtc.h"
#include "hw_aon_event.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#ifdef CC26X0_LOW_POWER_TIMER

/*---------------------------------------------------------------------------*/
/*
 * Used to test timer wraparounds.
 *
 * Set to 0xFFFFFFFA to test AON RTC second counter wraparound
 * Set to 0xFFFA to test AON RTC 16.16 format wraparound
 */
#ifdef SOC_RTC_CONF_START_TICK_COUNT
#define SOC_RTC_START_TICK_COUNT SOC_RTC_CONF_START_TICK_COUNT
#else
#define SOC_RTC_START_TICK_COUNT 0
#endif
/*---------------------------------------------------------------------------*/

/** Unified IRQ handler for all timers */
static inline void irq_handler(tim_t timer);

/** Timer state memory */
static timer_isr_ctx_t config[TIMER_NUMOF];

int timer_init(tim_t dev, unsigned long freq, timer_cb_t cb, void *arg)
{
    DEBUG("timer init: %i\n", dev);

    /* set callback function */
    config[dev].cb = cb;
    config[dev].arg = arg;

    timer_stop(dev);

    /* clear event flag */
    *(uint32_t*)(AON_RTC_BASE + AON_RTC_O_EVFLAGS) = AON_RTC_EVFLAGS_CH0;

    /* configure wakeup event */
    *(uint32_t*)(AON_EVENT_BASE + AON_EVENT_O_MCUWUSEL) = AON_EVENT_RTC_CH0 << AON_EVENT_MCUWUSEL_WU0_EV_S;
    *(uint32_t*)(AON_RTC_BASE + AON_RTC_O_CTL) = AON_RTC_CH0 << AON_RTC_CTL_COMB_EV_MASK_S;

    /* set timer value to 0 */
    *(uint32_t*)(AON_RTC_BASE + AON_RTC_O_SEC) = SOC_RTC_START_TICK_COUNT;

    timer_irq_enable(dev);

    timer_start(dev);

    return 0;
}

int timer_set(tim_t dev, int channel, unsigned int timeout)
{
    DEBUG("timer_set(): timer %d channel %d timeout %u\n", dev, channel, timeout);
    int now = timer_read(dev);
    return timer_set_absolute(dev, channel, now + timeout - 1);
}

uint32_t _timer_read(void)
{
    uint32_t seconds;
    uint32_t subseconds;
    uint32_t seconds_reread;

    //
    // Reading SEC both before and after SUBSEC in order to detect if SEC incremented while reading SUBSEC
    // If SEC incremented, we can't be sure which SEC the SUBSEC belongs to, so repeating the sequence then.
    //
    do {
        seconds = *(volatile uint32_t*)(AON_RTC_BASE + AON_RTC_O_SEC);
        subseconds = *(volatile uint32_t*)(AON_RTC_BASE + AON_RTC_O_SUBSEC);
        seconds_reread = *(volatile uint32_t*)(AON_RTC_BASE + AON_RTC_O_SEC);
    } while (seconds != seconds_reread);

    return ((seconds << 16) | (subseconds >> 16));
}

int timer_set_absolute(tim_t dev, int channel, unsigned int value)
{
    uint32_t seconds = value / 1000000;
    uint32_t microseconds = value % 1000000;
    uint32_t subseconds = microseconds * 65536 / 1000000;
    uint32_t ll_value = (seconds << 16) + subseconds;

    DEBUG("timer_set_absolute(): timer %d channel %d value %u ll_value %lu now %lu\n", dev, channel, value, ll_value, _timer_read());

    /* set compare value */
    *(uint32_t*)(AON_RTC_BASE + AON_RTC_O_CH0CMP) = ll_value;

    /* enable compare channel */
    *(uint32_t*)(AON_RTC_BASE + AON_RTC_O_CHCTL) |= (1 << AON_RTC_CHCTL_CH0_EN_BITN);

    /* clear compare channel event */
    *(uint32_t*)(AON_RTC_BASE + AON_RTC_O_EVFLAGS) = AON_RTC_EVFLAGS_CH0;

//     if (seconds < 5) {
//         printf("wait\n");
//         while((*(volatile uint32_t*)(AON_RTC_BASE + AON_RTC_O_EVFLAGS) & AON_RTC_EVFLAGS_CH0) == 0) {}
//         printf("done\n");
//     }

    return 0;
}

int timer_clear(tim_t dev, int channel)
{
    *(uint32_t*)(AON_RTC_BASE + AON_RTC_O_CHCTL) &= ~(1 << AON_RTC_CHCTL_CH0_EN_BITN);
    return 0;
}

unsigned int timer_read(tim_t dev)
{
    uint32_t seconds;
    uint32_t subseconds;
    uint32_t seconds_reread;

    //
    // Reading SEC both before and after SUBSEC in order to detect if SEC incremented while reading SUBSEC
    // If SEC incremented, we can't be sure which SEC the SUBSEC belongs to, so repeating the sequence then.
    //
    do {
        seconds = *(volatile uint32_t*)(AON_RTC_BASE + AON_RTC_O_SEC);
        subseconds = *(volatile uint32_t*)(AON_RTC_BASE + AON_RTC_O_SUBSEC);
        seconds_reread = *(volatile uint32_t*)(AON_RTC_BASE + AON_RTC_O_SEC);
    } while (seconds != seconds_reread);

//     return (( ui32CurrentSec << 16 ) | ( ui32CurrentSubSec >> 16 ));

    /* output in microseconds */
    return seconds * 1000000 + subseconds / 4295;
}

void timer_start(tim_t dev)
{
    *(uint32_t*)(AON_RTC_BASE + AON_RTC_O_CTL) |= (1 << AON_RTC_CTL_EN_BITN);
}

void timer_stop(tim_t dev)
{
    *(uint32_t*)(AON_RTC_BASE + AON_RTC_O_CTL) &= ~(1 << AON_RTC_CTL_EN_BITN);
}

void timer_irq_enable(tim_t dev)
{
    DEBUG("timer_irq_enable(): timer %d\n", dev);
    NVIC_EnableIRQ(AON_RTC_IRQN);
}

void timer_irq_disable(tim_t dev)
{
    DEBUG("timer_irq_disable(): timer %d\n", dev);
    NVIC_DisableIRQ(AON_RTC_IRQN);
}

void isr_aon_rtc(void)
{
    tim_t timer = TIMER_DEV(0);
    irq_handler(timer);
}

static inline void irq_handler(tim_t timer)
{
    lpm_awake();

    DEBUG("timer irq_handler(): timer %d\n", timer);

    /* check event flag */
    if(*(uint32_t*)(AON_RTC_BASE + AON_RTC_O_EVFLAGS) & (1 << AON_RTC_EVFLAGS_CH0_BITN)) {

        /* clear event flag */
        *(uint32_t*)(AON_RTC_BASE + AON_RTC_O_EVFLAGS) = AON_RTC_EVFLAGS_CH0;

        /* disable compare channel */
        *(uint32_t*)(AON_RTC_BASE + AON_RTC_O_CHCTL) &= ~(1 << AON_RTC_CHCTL_CH0_EN_BITN);

        config[timer].cb(config[timer].arg, 0);
    }

    if (sched_context_switch_request) {
        thread_yield();
    }
}

#endif // CC26X0_LOW_POWER_TIMER

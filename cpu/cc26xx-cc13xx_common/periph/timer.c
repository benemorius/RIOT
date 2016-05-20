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
 * @brief       Low-level timer driver implementation
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include <stdlib.h>

#include "board.h"
#include "periph/timer.h"

#include "driverlib/aon_rtc.h"
#include "driverlib/aon_event.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

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

    bool interrupts_disabled;

    /* Disable and clear interrupts */
    interrupts_disabled = IntMasterDisable();

    timer_stop(dev);

    AONRTCEventClear(AON_RTC_CH0);
    AONRTCEventClear(AON_RTC_CH1);

    /* Setup the wakeup event */
    AONEventMcuWakeUpSet(AON_EVENT_MCU_WU0, AON_EVENT_RTC_CH0);
    AONEventMcuWakeUpSet(AON_EVENT_MCU_WU1, AON_EVENT_RTC_CH1);
    AONRTCCombinedEventConfig(AON_RTC_CH0 | AON_RTC_CH1);

    HWREG(AON_RTC_BASE + AON_RTC_O_SEC) = SOC_RTC_START_TICK_COUNT;

    timer_irq_enable(dev);

    timer_start(dev);

    /* Re-enable interrupts */
    if(!interrupts_disabled) {
        IntMasterEnable();
    }

    return 0;
}

int timer_set(tim_t dev, int channel, unsigned int timeout)
{
    DEBUG("timer_set(): timer %d channel %d timeout %u\n", dev, channel, timeout);
    int now = timer_read(dev);
    return timer_set_absolute(dev, channel, now + timeout - 1);
}

int timer_set_absolute(tim_t dev, int channel, unsigned int value)
{
    DEBUG("timer_set_absolute(): timer %d channel %d value %u\n", dev, channel, value);
    AONRTCCompareValueSet(AON_RTC_CH0, value);
    AONRTCChannelEnable(AON_RTC_CH0);
    return 0;
}

int timer_clear(tim_t dev, int channel)
{
    AONRTCChannelDisable(AON_RTC_CH0);
    return 0;
}

unsigned int timer_read(tim_t dev)
{
    return AONRTCCurrentCompareValueGet();
}

void timer_start(tim_t dev)
{
    AONRTCEnable();
}

void timer_stop(tim_t dev)
{
    AONRTCDisable();
}

void timer_irq_enable(tim_t dev)
{
    DEBUG("timer_irq_enable(): timer %d\n", dev);
    IntEnable(INT_AON_RTC);
}

void timer_irq_disable(tim_t dev)
{
    DEBUG("timer_irq_disable(): timer %d\n", dev);
    IntDisable(INT_AON_RTC);
}

#ifdef TIMER_0_EN
void AONRTCIntHandler(void)
{
    tim_t timer = TIMER_DEV(0);
    irq_handler(timer);
}
#endif

static inline void irq_handler(tim_t timer)
{

    DEBUG("timer irq_handler(): timer %d\n", timer);

    if(AONRTCEventGet(AON_RTC_CH0)) {
        HWREG(AON_RTC_BASE + AON_RTC_O_EVFLAGS) = AON_RTC_EVFLAGS_CH0;
        config[timer].cb(config[timer].arg, 0);
    }

    if (sched_context_switch_request) {
        thread_yield();
    }
}

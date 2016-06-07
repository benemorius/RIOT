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
 * @brief       Low-level GPIO driver implementation
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include "cpu.h"
#include "sched.h"
#include "thread.h"
#include "periph/gpio.h"
#include "periph_conf.h"

#include "driverlib/gpio.h"
#include "driverlib/prcm.h"
#include "driverlib/ioc.h"

#define IOID_TO_GPIO(x)      (1 << x)

/**
 * @brief   16 EXTI channels
 */
#define EXTI_NUMOF      (16U)

/**
 * @brief   Hold one callback function pointer for each interrupt line
 */
static gpio_isr_ctx_t exti_chan[EXTI_NUMOF];

int gpio_init(gpio_t pin, gpio_mode_t mode)
{
    /* Enable peripheral power domain */
    if(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON) {
        PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
        while(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON);
    }

    /* Enable GPIO peripheral */
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);

    /* Apply settings and wait for them to take effect */
    PRCMLoadSet();
    while(!PRCMLoadGet());

    /* Decode GPIO mode */
    switch(mode) {
        case GPIO_OUT:
            mode = GPIO_DIR_MODE_OUT;
            break;
        case GPIO_IN:
            mode = GPIO_DIR_MODE_IN;
            IOCIOInputSet(pin, IOC_INPUT_ENABLE);
            break;
        default:
            return 1;
    }

    GPIODirModeSet(1 << pin, mode);

    return 0;
}

int gpio_init_int(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank,
                  gpio_cb_t cb, void *arg)
{
    return 0;
}

void gpio_irq_enable(gpio_t pin)
{
}

void gpio_irq_disable(gpio_t pin)
{
}

int gpio_read(gpio_t pin)
{
    return GPIOPinRead(IOID_TO_GPIO(pin)) > 0;
}

BOOT_FUNC void gpio_set(gpio_t pin)
{
    GPIOPinWrite(IOID_TO_GPIO(pin), 1);
}

BOOT_FUNC void gpio_clear(gpio_t pin)
{
    GPIOPinWrite(IOID_TO_GPIO(pin), 0);
}

void gpio_toggle(gpio_t pin)
{
    if (gpio_read(pin)) {
        gpio_clear(pin);
    } else {
        gpio_set(pin);
    }
}

void gpio_write(gpio_t pin, int value)
{
    if (value) {
        gpio_set(pin);
    } else {
        gpio_clear(pin);
    }
}

void isr_exti(void)
{
    exti_chan[0] = exti_chan[0];

    if (sched_context_switch_request) {
        thread_yield();
    }
}

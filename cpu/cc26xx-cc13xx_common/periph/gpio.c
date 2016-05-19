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

    uint32_t ui32Reg;

    //
    // Check the arguments.
    //
    ASSERT(ui32Pins & GPIO_PIN_MASK);
    ASSERT((mode == GPIO_DIR_MODE_IN) ||
    (mode == GPIO_DIR_MODE_OUT));

    //
    // Update the output pin enable bit.
    //
    ui32Reg = HWREG(GPIO_BASE + GPIO_O_DOE31_0); //FIXME hangs here
    if(mode == GPIO_DIR_MODE_IN)
    {
        ui32Reg &= ~pin;
    }
    else
    {
        ui32Reg |= pin;
    }
    HWREG(GPIO_BASE + GPIO_O_DOE31_0) = ui32Reg;


//     GPIODirModeSet(pin, GPIO_DIR_MODE_OUT);

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
    return GPIOPinRead(pin);
}

void gpio_set(gpio_t pin)
{
    GPIOPinWrite(pin, 1);
}

void gpio_clear(gpio_t pin)
{
    GPIOPinWrite(pin, 0);
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

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
 * @brief       Low-level UART driver implementation
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include <math.h>

//#include "cpu.h"
#include "board.h"
#include "periph_conf.h"
#include "periph/uart.h"
#include "thread.h"
#include "lpm.h"

#include "driverlib/prcm.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/ioc.h"
#include "driverlib/sys_ctrl.h"

/* All interrupt masks */
#define CC26XX_UART_INTERRUPT_ALL (UART_INT_OE | UART_INT_BE | UART_INT_PE | \
UART_INT_FE | UART_INT_RT | UART_INT_TX | \
UART_INT_RX | UART_INT_CTS)

/**
 * @brief Unified interrupt handler for all UART devices
 *
 * @param uart          the UART device that triggered the ISR
 */
static inline void irq_handler(uart_t uart);

/**
 * @brief   Allocate memory to store the callback functions
 */
static uart_isr_ctx_t uart_ctx[UART_NUMOF];

/**
 * @brief   Get the base register for the given UART device
 */
static inline uart_t _dev(uart_t uart)
{
    return uart_config[uart].dev;
}

int uart_init(uart_t uart, uint32_t baudrate, uart_rx_cb_t rx_cb, void *arg)
{
    /* remember callback addresses and argument */
    uart_ctx[uart].rx_cb = rx_cb;
    uart_ctx[uart].arg = arg;

    /* Enable peripheral power domain */
    if(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON) {
        PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
        while(PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON);
    }

    /* Enable serial power domain */
    if(PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON) {
        PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL);
        while(PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON);
    }

    /* Enable UART0 peripheral */
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralRunEnable(uart_config[uart].prcmp);

    /* Apply settings and wait for them to take effect */
    PRCMLoadSet();
    while(!PRCMLoadGet());

    UARTDisable(_dev(uart));

    /* Acknowledge UART interrupts */
    IntDisable(uart_config[uart].irqn);

    /* Disable all UART module interrupts */
    UARTIntDisable(_dev(uart), CC26XX_UART_INTERRUPT_ALL);

    /* Clear all UART interrupts */
    UARTIntClear(_dev(uart), CC26XX_UART_INTERRUPT_ALL);

    uint32_t ctl_val = UART_CTL_UARTEN | UART_CTL_TXE;
    /*
     * Make sure the TX pin is output / high before assigning it to UART control
     * to avoid falling edge glitches
     */
    IOCPinTypeGpioOutput(uart_config[uart].ioid_tx);
    GPIOPinWrite(uart_config[uart].gpio_tx, 1);

    /*
     * Map UART signals to the correct GPIO pins and configure them as
     * hardware controlled.
     */
    IOCPinTypeUart(_dev(uart),
                   uart_config[uart].ioid_rx,
                   uart_config[uart].ioid_tx,
                   IOID_UNUSED,
                   IOID_UNUSED);

    UARTConfigSetExpClk(_dev(uart), SysCtrlClockGet(),
                                   baudrate,
                                   (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                   UART_CONFIG_PAR_NONE));

    /*
     * Generate an RX interrupt at FIFO 1/2 full.
     * We don't really care about the TX interrupt
     */
//     UARTFIFOLevelSet(_dev(uart), UART_FIFO_TX7_8, UART_FIFO_RX4_8);

    /* Enable FIFOs */
//     HWREG(_dev(uart) + UART_O_LCRH) |= UART_LCRH_FEN;

    if(rx_cb) {
        ctl_val += UART_CTL_RXE;
    }

    /* Enable TX, RX (conditionally), and the UART. */
    HWREG(_dev(uart) + UART_O_CTL) = ctl_val;

    UARTIntEnable(_dev(uart), UART_INT_RX);
    IntEnable(uart_config[uart].irqn);

    return 0;
}

void uart_write(uart_t uart, const uint8_t *data, size_t len)
{
    char *c = (char *)data;
    for (int i = 0; i < len; i++) {
        if (c[i] == '\n') {
            UARTCharPut(_dev(uart), '\r');
        }
        UARTCharPut(_dev(uart), c[i]);
    }
}

#ifdef UART_0_EN
void UART0IntHandler(void)
{
    uart_t uart = UART_DEV(0);
    irq_handler(uart);
}
#endif

static inline void irq_handler(uart_t uart)
{
    lpm_awake();

    uint8_t c = UARTCharGet(_dev(uart));
//     UARTCharPut(_dev(uart), c);
    uart_ctx[uart].rx_cb(uart_ctx[uart].arg, c);

    if (sched_context_switch_request) {
		thread_yield();
	}
}

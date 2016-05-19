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
 * @param uartnum       the number of the UART that triggered the ISR
 * @param uart          the UART device that triggered the ISR
 */
static inline void irq_handler(uart_t uartnum, uint32_t *uart);


/**
 * @brief Allocate memory to store the callback functions.
 */
// static uart_conf_t config[UART_NUMOF];


int uart_init(uart_t uart, uint32_t baudrate, uart_rx_cb_t rx_cb, void *arg)
{
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
    PRCMPeripheralRunEnable(PRCM_PERIPH_UART0);

    /* Apply settings and wait for them to take effect */
    PRCMLoadSet();
    while(!PRCMLoadGet());

    UARTDisable(UART0_BASE);

    /* Acknowledge UART interrupts */
    IntDisable(INT_UART0);

    /* Disable all UART module interrupts */
    UARTIntDisable(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);

    /* Clear all UART interrupts */
    UARTIntClear(UART0_BASE, CC26XX_UART_INTERRUPT_ALL);

    uint32_t ctl_val = UART_CTL_UARTEN | UART_CTL_TXE;
    /*
     * Make sure the TX pin is output / high before assigning it to UART control
     * to avoid falling edge glitches
     */
    IOCPinTypeGpioOutput(GPIO_PIN_3);
    GPIOPinWrite(GPIO_PIN_3, 1);

    /*
     * Map UART signals to the correct GPIO pins and configure them as
     * hardware controlled.
     */
    IOCPinTypeUart(UART0_BASE, IOID_2, IOID_3,
                             IOID_4, IOID_5); //FIXME

    UARTConfigSetExpClk(UART0_BASE, SysCtrlClockGet(),
                                   baudrate,
                                   (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                   UART_CONFIG_PAR_NONE));

    /*
     * Generate an RX interrupt at FIFO 1/2 full.
     * We don't really care about the TX interrupt
     */
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX7_8, UART_FIFO_RX4_8);

    /* Enable FIFOs */
    HWREG(UART0_BASE + UART_O_LCRH) |= UART_LCRH_FEN;

//     if(input_handler) {
//         ctl_val += UART_CTL_RXE;
//     }

    /* Enable TX, RX (conditionally), and the UART. */
    HWREG(UART0_BASE + UART_O_CTL) = ctl_val;

    return 0;
}

void uart_write(uart_t uart, const uint8_t *data, size_t len)
{
    char *c = (char *)data;
    for (int i = 0; i < len; i++) {
        if (c[i] == '\n') {
            UARTCharPut(UART0_BASE, '\r');
        }
        UARTCharPut(UART0_BASE, c[i]);
    }
}

#ifdef UART_0_EN
__attribute__((naked)) void UART_0_RX_ISR(void)
{
    ISR_ENTER();
    irq_handler(UART_0, UART_0_DEV);
    ISR_EXIT();
}

__attribute__((naked)) void UART_0_TX_ISR(void)
{
    ISR_ENTER();
    irq_handler(UART_0, UART_0_DEV);
    ISR_EXIT();
}
#endif

#ifdef UART_1_EN
# if defined UART_1_RX_ISR && defined UART1_TX_ISR
__attribute__((naked)) void UART_1_RX_ISR(void)
{
    ISR_ENTER();
    irq_handler(UART_1, UART_1_DEV);
    ISR_EXIT();
}

__attribute__((naked)) void UART_1_TX_ISR(void)
{
    ISR_ENTER();
    irq_handler(UART_1, UART_1_DEV);
    ISR_EXIT();
}
# elifdef UART_1_ISR
__attribute__((naked)) void UART_1_ISR(void)
{
	ISR_ENTER();
	irq_handler(UART_1, UART_1_DEV);
	ISR_EXIT();
}
# endif
#endif //UART_1_EN

#ifdef UART_2_EN
# if defined UART_2_RX_ISR && defined UART_2_TX_ISR
__attribute__((naked)) void UART_2_RX_ISR(void)
{
	ISR_ENTER();
	irq_handler(UART_2, UART_2_DEV);
	ISR_EXIT();
}

__attribute__((naked)) void UART_2_TX_ISR(void)
{
	ISR_ENTER();
	irq_handler(UART_2, UART_2_DEV);
	ISR_EXIT();
}
# elifdef UART_2_ISR
__attribute__((naked)) void UART_2_ISR(void)
{
	ISR_ENTER();
	irq_handler(UART_2, UART_2_DEV);
	ISR_EXIT();
}
# endif
#endif //UART_2_EN

static inline void irq_handler(uart_t uartnum, uint32_t *dev)
{
// // 	if(lpm_get() >= LPM_SLEEP)
// // 		lpm_awake();
// //
// //     if (dev->IF & USART_IF_RXDATAV) {
// //         char data = (char)USART_RxDataGet(dev);
// //         config[uartnum].rx_cb(config[uartnum].arg, data);
// //
// //     } else if (dev->IF & USART_IF_TXC) {
// //         config[uartnum].tx_cb(config[uartnum].arg);
// //         dev->IFC |= USART_IFC_TXC;
// //     }
// //
// //     if (sched_context_switch_request) {
// // 		thread_yield();
// // 	}
}

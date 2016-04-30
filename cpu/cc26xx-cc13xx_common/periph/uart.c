/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32f0
 * @{
 *
 * @file
 * @brief       Low-level UART driver implementation
 *
 * @author      Ryan Kurte <ryankurte@gmail.com>
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

#define USART_INT_FLAGS (USART_IF_TXC | USART_IF_RXDATAV)

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
    return 0;
}

//Blocking UART initialisation
int uart_init_blocking(uart_t uart, uint32_t baudrate)
{
    return 0;
}

void uart_tx_begin(uart_t uart)
{

}

void uart_tx_end(uart_t uart)
{

}

void uart_write(uart_t uart, const uint8_t *data, size_t len)
{

}

int uart_read_blocking(uart_t uart, char *data)
{
    return 1;
}

int uart_write_blocking(uart_t uart, char data)
{
    return 1;
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

/*
 * Copyright (C) 2017-2018 Eistec AB
 * Copyright (C) 2014 PHYTEC Messtechnik GmbH
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_kinetis
 * @ingroup     drivers_periph_uart
 *
 * @{
 *
 * @file
 * @brief       Low-level UART driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include "log.h"
#include "cpu.h"
#include "bit.h"
#include "tsrb.h"
#include "mutex.h"
#include "periph_conf.h"
#include "periph/uart.h"
#include "periph/gpio.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifndef KINETIS_HAVE_LPUART
#ifdef LPUART0
#define KINETIS_HAVE_LPUART     1
#else
#define KINETIS_HAVE_LPUART     0
#endif
#endif /* KINETIS_HAVE_LPUART */

#ifndef KINETIS_HAVE_UART
#ifdef UART0
#define KINETIS_HAVE_UART       1
#else
#define KINETIS_HAVE_UART       0
#endif
#endif /* KINETIS_HAVE_LPUART */

#ifndef KINETIS_UART_ADVANCED
/**
 * Attempts to determine the type of the UART,
 * using the BRFA field in the UART C4 register.
 */
#ifdef UART_C4_BRFA
#define KINETIS_UART_ADVANCED   1
#endif
#endif

#ifndef LPUART_OVERSAMPLING_RATE_MIN
/* Use 10x oversampling at minimum, this will be iterated to achieve a better
 * baud rate match than otherwise possible with a fixed oversampling rate */
/* Hardware reset default value is an oversampling rate of 16 */
#define LPUART_OVERSAMPLING_RATE_MIN (10)
#endif
#ifndef LPUART_OVERSAMPLING_RATE_MAX
#define LPUART_OVERSAMPLING_RATE_MAX (32)
#endif

/* Default LPUART clock setting to avoid compilation failures, define this in
 * periph_conf.h to set board specific configuration if using the LPUART. */
#ifndef LPUART_0_SRC
#define LPUART_0_SRC            0
#endif
#ifndef LPUART_1_SRC
#define LPUART_1_SRC            0
#endif

#ifndef LPUART_IDLECFG
/* See IDLECFG in the reference manual. Longer idle configurations give more
 * robust LPUART RX from low power modes, but will also keep the CPU awake for
 * longer periods. */
#define LPUART_IDLECFG      (0b001)
#endif

/* This power mode is blocked while the LP/UART clock needs to stay running.
 * This happens if an RX callback is configured and the UART cannot wake on a
 * start bit without a clock running. This also happens if an RX callback
 * is configured and the baudrate is higher than
 * (UART_MAX_UNCLOCKED_BAUDRATE * 1.2). It happens also while a frame is being
 * received.
 * This default value is safe for all configurations and should be lowered
 * by the board configuration to the highest power mode which disables the
 * configured UART clock */
#ifndef UART_CLOCK_PM_BLOCKER
#define UART_CLOCK_PM_BLOCKER           KINETIS_PM_STOP
#endif

/* In power modes <= UART_CLOCK_PM_BLOCKER an LP/UART clock is not running.
 * Waking for UART RX can occur from LLWU or other sources but it takes time
 * to start a UART sampling clock after waking on a start bit. Therefore the
 * maximum usable baudrate depends on the UART clock source and the wakeup
 * source, but 9600 should be a safe default with most configurations. Using a
 * baudrate faster than (UART_MAX_UNCLOCKED_BAUDRATE * 1.2) will block
 * UART_CLOCK_PM_BLOCKER while an RX callback is configured.
 * This should be increased by the board configuration in most cases. */
#ifndef UART_MAX_UNCLOCKED_BAUDRATE
#define UART_MAX_UNCLOCKED_BAUDRATE     9600ul
#endif

typedef struct {
    uart_rx_cb_t rx_cb;     /**< data received interrupt callback */
    void *arg;              /**< argument to both callback routines */
    unsigned active;        /**< set to 1 while the receiver is active, to avoid mismatched PM_BLOCK/PM_UNBLOCK */
    unsigned enabled;       /**< set to 1 while the receiver is enabled, to avoid mismatched PM_BLOCK/PM_UNBLOCK */
    unsigned char tx_buf_data[1024 * 16];  /**< transmit DMA buffer */
    mutex_t tx_mutex;
    mutex_t tx_return_mutex;
    tsrb_t tx_rb;
} uart_isr_ctx_t;

volatile bool dma_is_in;
volatile bool dma_memcpy_is_in;
volatile bool dma_memcpy_done = true;
bool uart_no_dma;
bool pm_blocked;

/**
 * @brief Runtime configuration space, holds pointers to callback functions for RX
 */
static uart_isr_ctx_t config[UART_NUMOF];

static inline void uart_init_pins(uart_t uart);

#if KINETIS_HAVE_UART
static inline void uart_init_uart(uart_t uart, uint32_t baudrate);
static inline void uart_poweron_uart(uart_t uart);
static inline void uart_poweroff_uart(uart_t uart);
#endif
#if KINETIS_HAVE_LPUART
static inline void uart_init_lpuart(uart_t uart, uint32_t baudrate);
static inline void uart_poweron_lpuart(uart_t uart);
static inline void uart_poweroff_lpuart(uart_t uart);
#endif

/* Only use the dispatch function for uart_write if both UART and LPUART are
 * available at the same time */
#if KINETIS_HAVE_UART && KINETIS_HAVE_LPUART
#define KINETIS_UART_WRITE_INLINE static inline
KINETIS_UART_WRITE_INLINE void uart_write_uart(uart_t uart, const uint8_t *data, size_t len);
KINETIS_UART_WRITE_INLINE void uart_write_lpuart(uart_t uart, const uint8_t *data, size_t len);
#else
#define KINETIS_UART_WRITE_INLINE
#if KINETIS_HAVE_UART
#define uart_write_uart uart_write
#elif KINETIS_HAVE_LPUART
#define uart_write_lpuart uart_write
#endif
#endif

#ifdef MODULE_PERIPH_GPIO_IRQ
/**
 * @brief   pin interrupt callback for UART RX pin
 *
 * This function is called only when we're waiting for a start bit to cause
 * a pin interrupt so we can start the UART clock.
 */
static void uart_pin_int_cb(void *arg)
{
    uart_t uart = (uart_t)arg;
    if (!config[uart].active) {
        config[uart].active = 1;

        /* Keep UART clock on until we are finished with RX */
        PM_BLOCK(UART_CLOCK_PM_BLOCKER);

        /* We don't need the pin interrupt until the bus goes idle again */
        gpio_irq_disable(uart_config[uart].pin_rx);

        DEBUG("UART pin IRQ\n");
    }
}
#endif

#ifdef MODULE_PERIPH_GPIO_IRQ
static uint32_t _get_baudrate(uart_t uart)
{
    uint32_t baudrate = 0;

    switch (uart_config[uart].type) {
#if KINETIS_HAVE_UART
        case KINETIS_UART: {
            UART_Type *dev = uart_config[uart].dev;
            uint16_t sbr = 0;
            sbr += (dev->BDL & UART_BDL_SBR_MASK) >> UART_BDL_SBR_SHIFT;
            sbr += (dev->BDH & UART_BDH_SBR_MASK) >> UART_BDH_SBR_SHIFT << 8;
            baudrate = uart_config[uart].freq / sbr / 16;
            break;
        }
#endif
#if KINETIS_HAVE_LPUART
        case KINETIS_LPUART: {
            LPUART_Type *dev = uart_config[uart].dev;
            uint8_t osr = (dev->BAUD & LPUART_BAUD_OSR_MASK)
                           >> LPUART_BAUD_OSR_SHIFT;
            uint16_t sbr = (dev->BAUD & LPUART_BAUD_SBR_MASK)
                            >> LPUART_BAUD_SBR_SHIFT;
            baudrate = uart_config[uart].freq / ((osr + 1) * sbr);
            break;
        }
#endif
        default:
            break;
    }
    return baudrate;
}
#endif /* MODULE_PERIPH_GPIO_IRQ */

int uart_init(uart_t uart, uint32_t baudrate, uart_rx_cb_t rx_cb, void *arg)
{
    assert(uart < UART_NUMOF);

    tsrb_init(&config[uart].tx_rb, config[uart].tx_buf_data,
              sizeof(config[uart].tx_buf_data));

    /* disable interrupts from UART module */
    NVIC_DisableIRQ(uart_config[uart].irqn);

    /* Turn on module clock gate */
    bit_set32(uart_config[uart].scgc_addr, uart_config[uart].scgc_bit);

    /* Power off before messing with settings, this will ensure a consistent
     * state if we are re-initializing an already initialized module */
    uart_poweroff(uart);

    /* remember callback addresses */
    config[uart].rx_cb = rx_cb;
    config[uart].arg = arg;
    config[uart].active = 0;
    config[uart].enabled = 0;

    uart_init_pins(uart);

    switch (uart_config[uart].type) {
#if KINETIS_HAVE_UART
        case KINETIS_UART:
            uart_init_uart(uart, baudrate);
            break;
#endif
#if KINETIS_HAVE_LPUART
        case KINETIS_LPUART:
            uart_init_lpuart(uart, baudrate);
            break;
#endif
        default:
            return UART_NODEV;
    }

    /* Turn on module */
    uart_poweron(uart);

    /* enable interrupts from UART module */
    NVIC_EnableIRQ(uart_config[uart].irqn);

    return UART_OK;
}

void uart_poweron(uart_t uart)
{
    assert(uart < UART_NUMOF);
    unsigned state = irq_disable();

    switch (uart_config[uart].type) {
#if KINETIS_HAVE_UART
        case KINETIS_UART:
            uart_poweron_uart(uart);
            break;
#endif
#if KINETIS_HAVE_LPUART
        case KINETIS_LPUART:
            uart_poweron_lpuart(uart);
            break;
#endif
        default:
            break;
    }

    irq_restore(state);

    if (!config[uart].enabled) {
        config[uart].enabled = 1;

        if (config[uart].rx_cb) {

            /* As long as the baudrate isn't too fast, we can avoid needing
             * to keep the UART clocked continuously by detecting the falling
             * edge of a start bit using a pin interrupt. */
#ifdef MODULE_PERIPH_GPIO_IRQ
            if (_get_baudrate(uart) < UART_MAX_UNCLOCKED_BAUDRATE * 1.2) {

                /* This will block power modes appropriate for maintaining a
                 * working interrupt as a wakeup source. */
                gpio_init_int(uart_config[uart].pin_rx, GPIO_IN, GPIO_FALLING,
                              uart_pin_int_cb, (void*)uart);

                /* gpio_init_int() has reconfigured pin_rx to GPIO mode */
                uart_init_pins(uart);
            }
            else
            {
                /* We can't wake fast enough from a pin interrupt to start
                 * the UART clock in time to catch the incoming frame, so we
                 * need to keep the UART clock running */
                LOG_INFO("[uart] Blocking power mode %u because baudrate > %"
                         PRIu32 "\n",
                         UART_CLOCK_PM_BLOCKER, UART_MAX_UNCLOCKED_BAUDRATE);
                PM_BLOCK(UART_CLOCK_PM_BLOCKER);
            }
#else
            LOG_INFO("[uart] Blocking power mode %u because module "
                     "periph_gpio_irq is needed to sleep with RX enabled\n",
                     UART_CLOCK_PM_BLOCKER);
            PM_BLOCK(UART_CLOCK_PM_BLOCKER);
#endif /* MODULE_PERIPH_GPIO_IRQ */
        }
    }

}

void uart_poweroff(uart_t uart)
{
    assert(uart < UART_NUMOF);

    while (!dma_memcpy_done) {}
    while (dma_is_in) {}

    unsigned state = irq_disable();
    if (config[uart].rx_cb) {
        if (config[uart].enabled) {
            config[uart].enabled = 0;

            /* We may have been using a pin interrupt to wake the UART clock. */
#ifdef MODULE_PERIPH_GPIO_IRQ
            if (_get_baudrate(uart) < UART_MAX_UNCLOCKED_BAUDRATE * 1.2) {
                /* IRQ was already disabled if active is set */
                if (!config[uart].active) {
                    gpio_irq_disable(uart_config[uart].pin_rx);
                }
            }
            else
            {
#else
            {
#endif
                LOG_INFO("[uart] Unblocking power mode %u\n",
                      UART_CLOCK_PM_BLOCKER);
                PM_UNBLOCK(UART_CLOCK_PM_BLOCKER);
            }

            if (config[uart].active) {
                config[uart].active = 0;
                /* We were in the middle of a RX sequence, need to release that
                 * clock blocker as well */
                PM_UNBLOCK(UART_CLOCK_PM_BLOCKER);
            }
        }
    }
    switch (uart_config[uart].type) {
#if KINETIS_HAVE_UART
        case KINETIS_UART:
            uart_poweroff_uart(uart);
            break;
#endif
#if KINETIS_HAVE_LPUART
        case KINETIS_LPUART:
            uart_poweroff_lpuart(uart);
            break;
#endif
        default:
            break;
    }
    irq_restore(state);
}

#if KINETIS_HAVE_UART && KINETIS_HAVE_LPUART
/* Dispatch function to pass to the proper write function depending on UART type
 * This function is only used when the CPU supports both UART and LPUART. */
void uart_write(uart_t uart, const uint8_t *data, size_t len)
{
    switch (uart_config[uart].type) {
        case KINETIS_UART:
            uart_write_uart(uart, data, len);
            break;
        case KINETIS_LPUART:
            uart_write_lpuart(uart, data, len);
            break;
        default:
            return;
    }
}
#endif

static inline void uart_init_pins(uart_t uart)
{
    /* initialize pins */
    if (uart_config[uart].pin_rx != GPIO_UNDEF) {
        gpio_init_port(uart_config[uart].pin_rx, uart_config[uart].pcr_rx);
    }
    if (uart_config[uart].pin_tx != GPIO_UNDEF) {
        gpio_init_port(uart_config[uart].pin_tx, uart_config[uart].pcr_tx);
    }
}

#if KINETIS_HAVE_UART
static inline void uart_init_uart(uart_t uart, uint32_t baudrate)
{
    /* do basic initialization */
    UART_Type *dev = uart_config[uart].dev;

    uint32_t clk;
    uint16_t ubd;

    clk = uart_config[uart].freq;

    /* disable transmitter and receiver */
    dev->C2 = 0;

    /* Select mode */
    dev->C1 = uart_config[uart].mode;

    /* calculate baudrate */
    ubd = (uint16_t)(clk / (baudrate * 16));

    /* set baudrate */
    dev->BDH = (uint8_t)UART_BDH_SBR(ubd >> 8);
    dev->BDL = (uint8_t)UART_BDL_SBR(ubd);

#if KINETIS_UART_ADVANCED
    /* set baudrate fine adjust (brfa) */
    uint8_t brfa = ((((4 * clk) / baudrate) + 1) / 2) % 32;
    dev->C4 = UART_C4_BRFA(brfa);

    /* Enable FIFO buffers */
    dev->PFIFO |= UART_PFIFO_RXFE_MASK | UART_PFIFO_TXFE_MASK;
    /* Set level to trigger TDRE flag whenever there is space in the TXFIFO */
    /* FIFO size is 2^(PFIFO_TXFIFOSIZE + 1) (4, 8, 16 ...) for values != 0.
     * TXFIFOSIZE == 0 means size = 1 (i.e. only one byte, no hardware FIFO) */
    if ((dev->PFIFO & UART_PFIFO_TXFIFOSIZE_MASK) != 0) {
        uint8_t txfifo_size =
            (2 << ((dev->PFIFO & UART_PFIFO_TXFIFOSIZE_MASK) >>
            UART_PFIFO_TXFIFOSIZE_SHIFT));
        dev->TWFIFO = UART_TWFIFO_TXWATER(txfifo_size - 1);
    }
    else {
        /* Missing hardware support */
        dev->TWFIFO = 0;
    }
    /* Trigger RX interrupt when there is 1 byte or more in the RXFIFO */
    dev->RWFIFO = 1;
    /* Clear all hardware buffers now, this must be done whenever the FIFO
     * enable flags are modified. */
    dev->CFIFO = UART_CFIFO_RXFLUSH_MASK | UART_CFIFO_TXFLUSH_MASK;
#endif /* KINETIS_UART_ADVANCED */

    if (config[uart].rx_cb) {
        /* enable receiver + RX interrupt + IDLE interrupt */
        dev->C2 = UART_C2_RIE_MASK | UART_C2_ILIE_MASK;
        /* enable interrupts on failure flags */
        dev->C3 |= UART_C3_ORIE_MASK | UART_C3_PEIE_MASK | UART_C3_FEIE_MASK | UART_C3_NEIE_MASK;
    }
    /* clear interrupt flags */
    uint8_t s = dev->S2;
    dev->S2 = s;
}

KINETIS_UART_WRITE_INLINE void uart_write_uart(uart_t uart, const uint8_t *data, size_t len)
{
#if DEVELHELP
    if (!config[uart].enabled) {
        return;
    }
#endif

    UART_Type *dev = uart_config[uart].dev;

    for (size_t i = 0; i < len; i++) {
        while (!(dev->S1 & UART_S1_TDRE_MASK)) {}
        dev->D = data[i];
    }
    /* Wait for transmission complete */
    while ((dev->S1 & UART_S1_TC_MASK) == 0) {}
}

static inline void uart_poweron_uart(uart_t uart)
{
    UART_Type *dev = uart_config[uart].dev;

    /* Enable transmitter */
    bit_set8(&dev->C2, UART_C2_TE_SHIFT);
    if (config[uart].rx_cb) {
        /* Enable receiver */
        bit_set8(&dev->C2, UART_C2_RE_SHIFT);
    }
}

static inline void uart_poweroff_uart(uart_t uart)
{
    UART_Type *dev = uart_config[uart].dev;

    /* Disable receiver */
    bit_clear8(&dev->C2, UART_C2_RE_SHIFT);
    /* Disable transmitter */
    bit_clear8(&dev->C2, UART_C2_TE_SHIFT);
}

#if defined(UART_0_ISR) || defined(UART_1_ISR) || defined(UART_2_ISR) || \
    defined(UART_3_ISR) || defined(UART_4_ISR)
static inline void irq_handler_uart(uart_t uart)
{
    UART_Type *dev = uart_config[uart].dev;

    uint8_t s1 = dev->S1;
    uint8_t s2 = dev->S2;
    /* Clear IRQ flags */
    dev->S2 = s2;
    /* The IRQ flags in S1 are cleared by reading the D register */
    uint8_t data = dev->D;
    (void) data;
    if (dev->SFIFO & UART_SFIFO_RXUF_MASK) {
        /* RX FIFO underrun occurred, flush the RX FIFO to get the internal
         * pointer back in sync */
        dev->CFIFO |= UART_CFIFO_RXFLUSH_MASK;
        /* Clear SFIFO flags */
        dev->SFIFO = dev->SFIFO;
        DEBUG("UART RXUF\n");
    }
    DEBUG("U: %c\n", data);
    if (s1 & UART_S1_OR_MASK) {
        /* UART overrun, some data has been lost */
        DEBUG("UART OR\n");
    }
    if (s1 & UART_S1_RDRF_MASK) {
        if (s1 & (UART_S1_FE_MASK | UART_S1_PF_MASK | UART_S1_NF_MASK)) {
            if (s1 & UART_S1_FE_MASK) {
                DEBUG("UART framing error %02x\n", (unsigned) s1);
            }
            if (s1 & UART_S1_PF_MASK) {
                DEBUG("UART parity error %02x\n", (unsigned) s1);
            }
            if (s1 & UART_S1_NF_MASK) {
                DEBUG("UART noise %02x\n", (unsigned) s1);
            }
            /* FE is set when a logic 0 is accepted as the stop bit. */
            /* PF is set when PE is set, S2[LBKDE] is disabled, and the parity
             * of the received data does not match its parity bit. */
            /* NF is set when the UART detects noise on the receiver input. */
        }
        /* Only run callback if no error occurred */
        else if (config[uart].rx_cb != NULL) {
            config[uart].rx_cb(config[uart].arg, data);
        }
    }
    if (s1 & UART_S1_IDLE_MASK) {
        if (config[uart].active) {
            config[uart].active = 0;
            /* UART clock can stop now that RX is complete */
            PM_UNBLOCK(UART_CLOCK_PM_BLOCKER);
#ifdef MODULE_PERIPH_GPIO_IRQ
            /* Need an interrupt source to wake for the next start bit */
            gpio_irq_enable(uart_config[uart].pin_rx);
#endif
            DEBUG("UART IDLE\n");
        }
    }
    DEBUG("UART: s1 %x C1 %x C2 %x S1 %x S2 %x D %x SF %x\n", s1, dev->C1, dev->C2, dev->S1, dev->S2, data, dev->SFIFO);
    cortexm_isr_end();
}
#endif

#ifdef UART_0_ISR
void UART_0_ISR(void)
{
    irq_handler_uart(UART_DEV(0));
}
#endif

#ifdef UART_1_ISR
void UART_1_ISR(void)
{
    irq_handler_uart(UART_DEV(1));
}
#endif

#ifdef UART_2_ISR
void UART_2_ISR(void)
{
    irq_handler_uart(UART_DEV(2));
}
#endif

#ifdef UART_3_ISR
void UART_3_ISR(void)
{
    irq_handler_uart(UART_DEV(3));
}
#endif

#ifdef UART_4_ISR
void UART_4_ISR(void)
{
    irq_handler_uart(UART_DEV(4));
}
#endif

#endif /* KINETIS_HAVE_UART */

#if KINETIS_HAVE_LPUART
static inline void uart_init_lpuart(uart_t uart, uint32_t baudrate)
{
    LPUART_Type *dev = uart_config[uart].dev;

    /* Set LPUART clock source */
#ifdef SIM_SOPT2_LPUART0SRC
    if (dev == LPUART0) {
        SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_LPUART0SRC_MASK) |
            SIM_SOPT2_LPUART0SRC(LPUART_0_SRC);
    }
#endif
#ifdef SIM_SOPT2_LPUART1SRC
    if (dev == LPUART1) {
        SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_LPUART1SRC_MASK) |
            SIM_SOPT2_LPUART1SRC(LPUART_1_SRC);
    }
#endif

    /* Select mode */
    /* transmitter and receiver disabled */
    dev->CTRL = uart_config[uart].mode;

    /* calculate baud rate divisor */
    uint32_t clk = uart_config[uart].freq;
    uint32_t best_err = baudrate;
    uint32_t best_osr = LPUART_OVERSAMPLING_RATE_MIN;
    /* Use the oversampling rate as a baud rate fine adjust tool */
    for (uint32_t osr = LPUART_OVERSAMPLING_RATE_MIN; osr <= LPUART_OVERSAMPLING_RATE_MAX; ++osr) {
        uint32_t div = clk / (osr * baudrate);
        uint32_t actual_baud = clk / (osr * div);
        uint32_t err = ((actual_baud > baudrate) ? actual_baud - baudrate : baudrate - actual_baud);
        if (err < best_err) {
            best_err = err;
            best_osr = osr;
        }
    }

    uint32_t sbr = clk / (best_osr * baudrate);
    /* set baud rate, enable RX active edge interrupt */
    dev->BAUD = LPUART_BAUD_OSR(best_osr - 1) | LPUART_BAUD_SBR(sbr);

    mutex_init(&config[uart].tx_mutex);
    mutex_init(&config[uart].tx_return_mutex);

    /* enable clock for DMAMUX peripheral */
    SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK;

    /* disable DMA channel in case it was configured */
    DMAMUX0->CHCFG[0] = 0;
    /* total bytes to transfer per major loop iteration */
    DMA0->TCD[0].NBYTES_MLNO = 1;
    /* source data chunk size per minor iteration */
    DMA0->TCD[0].SOFF = 1;
    /* 8 bit source data transfer size */
    DMA0->TCD[0].ATTR &= ~DMA_ATTR_SSIZE_MASK;
    /* destination address */
    DMA0->TCD[0].DADDR = (uint32_t)&dev->DATA;
    /* destination data chunk size */
    DMA0->TCD[0].DOFF = 0;
    /* 8 bit destination data transfer size */
    DMA0->TCD[0].ATTR &= ~DMA_ATTR_DSIZE_MASK;
    /* destination address adjustment on major iteration count completion */
    DMA0->TCD[0].DLAST_SGA = 0;
    /* enable ERQ disabled on major loop iteration count completion */
    DMA0->TCD[0].CSR |= DMA_CSR_DREQ_MASK;
    /* enable interrupt on DMA error */
    DMA0->EEI |= DMA_EEI_EEI0_MASK;
//     NVIC_EnableIRQ(DMA_Error_IRQn);
    /* enable interrupt on major iteration count completion */
    DMA0->TCD[0].CSR |= DMA_CSR_INTMAJOR_MASK;
    /* TODO set this priority higher than any isr which might call uart_write() */
    NVIC_EnableIRQ(DMA0_IRQn);
    NVIC_SetPriority(DMA0_IRQn, 0);
    /* set DMA channel 0 source to uart tx */
    DMAMUX0->CHCFG[0] = DMAMUX_CHCFG_SOURCE(3) | DMAMUX_CHCFG_ENBL_MASK;

    /* disable DMA channel in case it was configured */
    DMAMUX0->CHCFG[1] = 0;
    /* starting major iteration count */
    DMA0->TCD[1].BITER_ELINKNO = 1;
    /* current major iteration count */
    DMA0->TCD[1].CITER_ELINKNO = DMA0->TCD[1].BITER_ELINKNO;
    /* source data chunk size per minor iteration */
    DMA0->TCD[1].SOFF = 1;
    /* source address adjustment on major iteration count completion */
    DMA0->TCD[0].SLAST = 0;
    /* 8 bit data transfer size */
    DMA0->TCD[1].ATTR = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0);
    /* destination data chunk size per minor iteration */
    DMA0->TCD[1].DOFF = 1;
    /* destination address adjustment on major iteration count completion */
    DMA0->TCD[1].DLAST_SGA = 0;
    /* enable interrupt on DMA error */
    DMA0->EEI |= DMA_EEI_EEI1_MASK;
    /* enable interrupt on major iteration count completion */
    DMA0->TCD[1].CSR = DMA_CSR_INTMAJOR_MASK;
    /* TODO set this priority higher than any isr which might call uart_write() */
    NVIC_SetPriority(DMA1_IRQn, 1);
    NVIC_EnableIRQ(DMA1_IRQn);
    NVIC_SetPriority(RTC_IRQn, 2);
    NVIC_SetPriority(LPUART0_IRQn, 2);

//     NVIC_DisableIRQ(Radio_0_IRQn);
//     NVIC_DisableIRQ(Radio_1_IRQn);
    NVIC_SetPriority(Radio_0_IRQn, 2);
    NVIC_SetPriority(Radio_1_IRQn, 2);

    for (int i = 2; i < CPU_IRQ_NUMOF; i++) {
        NVIC_SetPriority(i, 2);
    }

    /* set DMA channel 1 source to always enabled */
//     DMAMUX0->CHCFG[1] = DMAMUX_CHCFG_SOURCE(60) | DMAMUX_CHCFG_ENBL_MASK;


    if (config[uart].rx_cb) {
        /* enable RX interrupt + error interrupts */
        dev->CTRL |= LPUART_CTRL_RIE_MASK |
            LPUART_CTRL_ILIE_MASK | LPUART_CTRL_IDLECFG(LPUART_IDLECFG) |
            LPUART_CTRL_ORIE_MASK | LPUART_CTRL_PEIE_MASK |
            LPUART_CTRL_FEIE_MASK | LPUART_CTRL_NEIE_MASK;
    }
    /* clear interrupt flags */
    uint32_t s = dev->STAT;
    dev->STAT = s;
}

size_t buf_len;
size_t in_len;
const uint8_t *in_data;
#include "board.h"
#include "xtimer.h"

static void dma_start(uart_t uart)
{
    LPUART_Type *dev = uart_config[uart].dev;

    if (!pm_blocked) {
        PM_BLOCK(KINETIS_PM_WAIT);
//         LED0_ON;
        /* enable TX complete interrupt */
        dev->CTRL |= LPUART_CTRL_TCIE_MASK;
        pm_blocked = true;
    }

    dma_is_in = true;

    /* set up DMA transfer from tx_rb to UART->DATA */

    buf_len = tsrb_avail(&config[uart].tx_rb);
    volatile size_t bytes_available = tsrb_avail(&config[uart].tx_rb);
    volatile uint8_t empty = tsrb_empty(&config[uart].tx_rb);
    (void)bytes_available;
    (void)empty;

    if (buf_len == 0) {
        dma_is_in = false;
        return;
    }

    if (buf_len > sizeof(config[uart].tx_buf_data) / 8) {
        buf_len = sizeof(config[uart].tx_buf_data) / 8;
    }

    unsigned char *read_ptr = config[uart].tx_buf_data
                   + (config[uart].tx_rb.reads & (config[uart].tx_rb.size - 1));

    /* We mustn't let the DMA engine read past the end of the ring buffer, so
     * check for that and break the transfer in two if necessary. The second
     * transfer for the remaining data will be started for us by the ISR.
     */
    if (read_ptr + buf_len >
        &config[uart].tx_buf_data[sizeof(config[uart].tx_buf_data)]
    ) {
        buf_len = &config[uart].tx_buf_data[sizeof(config[uart].tx_buf_data)]
                  - read_ptr;
    }

    /* starting major iteration count */
    DMA0->TCD[0].BITER_ELINKNO = buf_len;
    /* current major iteration count */
    DMA0->TCD[0].CITER_ELINKNO = DMA0->TCD[0].BITER_ELINKNO;
    /* source data address */
    DMA0->TCD[0].SADDR = (uint32_t)read_ptr;
    /* enable DMA request channel. This gets disabled automatically */
    DMA0->ERQ |= DMA_ERQ_ERQ0_MASK;

    /* enable DMA request on transmit data register empty */
    LPUART0->BAUD |= LPUART_BAUD_TDMAE_MASK;

    assert(DMA0->ERR == 0);
    if (DMA0->ERR != 0) {
        while (1) {}
    }

    /* release uart_write() to return now as we've copied the data */
//     mutex_unlock(&config[uart].tx_return_mutex);
}

static void memcpy_start(uart_t uart)
{
    uint32_t irq_state;
    if (in_len) {
//         LED0_ON;
        PM_BLOCK(KINETIS_PM_STOP);
        dma_memcpy_done = false;

        /* block until the ring buffer has enough free memory */
        size_t bytes_pushed;
//         irq_restore(irq_state);
        do {
            bytes_pushed = tsrb_free(&config[uart].tx_rb);
        } while (bytes_pushed == 0);
        irq_state = irq_disable();

//         irq_state = irq_disable();

        if (in_len < bytes_pushed) {
            bytes_pushed = in_len;
        }

        unsigned char *write_ptr = config[uart].tx_buf_data
        + (config[uart].tx_rb.writes & (config[uart].tx_rb.size - 1));

        /* wrap at the end of the ring buffer */
        if (write_ptr + bytes_pushed >
            &config[uart].tx_buf_data[sizeof(config[uart].tx_buf_data)]
        ) {
            bytes_pushed = &config[uart].tx_buf_data[sizeof(config[uart].tx_buf_data)]
            - write_ptr;
        }

        /* set up DMA transfer from *data to ring buffer */
        /* source data address */
        DMA0->TCD[1].SADDR = (uint32_t)in_data;
        /* destination address */
        DMA0->TCD[1].DADDR = (uint32_t)write_ptr;
        /* total bytes to transfer per major loop iteration */
        DMA0->TCD[1].NBYTES_MLNO = bytes_pushed;
        /* begin the DMA transfer now */
        DMA0->TCD[1].CSR |= DMA_CSR_START_MASK;

        config[uart].tx_rb.writes += bytes_pushed;
        in_data += bytes_pushed;
        in_len -= bytes_pushed;

        irq_restore(irq_state);
    }
}

KINETIS_UART_WRITE_INLINE void uart_write_lpuart(uart_t uart, const uint8_t *data, size_t len)
{
    /* TODO what happens when this is called with IRQs disabled? */

//     uint32_t irq_state;
#if DEVELHELP
    if (!config[uart].enabled) {
        return;
    }
#endif

    static bool only_once = false;
    if (only_once) {
        return;
    }
//     only_once = true;

//     if (irq_is_in()) {
//         return;
//     }

//     assert (len <= sizeof(config[uart].tx_buf_data));

//     unsigned irq_state = irq_disable();

    /* we only support one reentrant call at a time, so this needs to spin */

//     if (mutex_trylock(&config[uart].tx_mutex) == 0) {
//         irq_restore(irq_state);
//         while (mutex_trylock(&config[uart].tx_mutex) == 0) {}
//         irq_state = irq_disable();
//     }

//     assert (dma_memcpy_done);
    if (!dma_memcpy_done) {
//         irq_restore(irq_state);
        while (!dma_memcpy_done) {}
        while (!dma_memcpy_done) {}
        while (!dma_memcpy_done) {}
        while (!dma_memcpy_done) {}
//         irq_state = irq_disable();
    }
//     dma_memcpy_done = false;


//     dma_memcpy_done = false;


//     irq_state = irq_disable();

    in_len = len;
    in_data = data;
    memcpy_start(uart);

//     irq_restore(irq_state);

    while (!dma_memcpy_done) {}

    /* to simulate the old behavior of blocking until TX complete */
    if (uart_no_dma) {
        while (dma_is_in) {}
    }

    return;




//     assert (len <= sizeof(config[uart].tx_buf_data));
//     size_t bytes_added = 0;
//     while (bytes_added < len) {
//         bytes_added += tsrb_add(&config[uart].tx_rb, data, len);
//         if (bytes_added < len) {
//             LOG_WARNING("DMA buffer full. Blocking\n");
//             while (dma_is_in) {}
// //             continue;
//         }
//     }
//
// //     irq_restore(irq_state);
//
//     irq_state = irq_disable();
//     if (!dma_is_in) {
//         dma_start(uart);
//     }
//     irq_restore(irq_state);
//
//     mutex_unlock(&config[uart].tx_mutex);
//
//     return;




    /* double lock this to wait until isr releases it */
//     while (mutex_trylock(&config[uart].tx_return_mutex) == 0) {}
//     mutex_lock(&config[uart].tx_return_mutex);
//     mutex_unlock(&config[uart].tx_return_mutex);



//     while (mutex_trylock(&config[uart].tx_mutex) == 0) {}
//     mutex_lock(&config[uart].tx_mutex);
//     mutex_unlock(&config[uart].tx_mutex);

//     for (size_t i = 0; i < len; i++) {
// //         ringbuffer_add_one(config[uart].tx_buf, data[i]);
//
//
    //         /* begin the DMA transfer now */
// //         DMA0->TCD[0].CSR |= DMA_CSR_START_MASK;
//
// //         while (!(DMA0->TCD[0].CSR & DMA_CSR_DONE_MASK)) {}
//
// //         dev->BAUD |= LPUART_BAUD_TDMAE_MASK;
//
//         while (!(DMA0->INT & DMA_INT_INT0_MASK)) {}
//         DMA0->INT |= DMA_INT_INT0_MASK;
//         /* disable dma channel */
//         DMAMUX0->CHCFG[0] = 0;
//
//         /* Wait for transmission complete */
//         while ((dev->STAT & LPUART_STAT_TDRE_MASK) == 0) {}
// //         while (!(dev->STAT & LPUART_STAT_TDRE_MASK)) {}
//     }
//
//     dev->BAUD &= ~LPUART_BAUD_TDMAE_MASK;
//
//     return;

//     LPUART_Type *dev = uart_config[uart].dev;
//
//     for (size_t i = 0; i < len; i++) {
//         while ((dev->STAT & LPUART_STAT_TDRE_MASK) == 0) {}
//         dev->DATA = data[i];
//     }
//     /* Wait for transmission complete */
//     while ((dev->STAT & LPUART_STAT_TC_MASK) == 0) {}
}

static inline void uart_poweron_lpuart(uart_t uart)
{
    LPUART_Type *dev = uart_config[uart].dev;

    /* Enable transmitter */
    bit_set32(&dev->CTRL, LPUART_CTRL_TE_SHIFT);
    if (config[uart].rx_cb) {
        /* Enable receiver */
        bit_set32(&dev->CTRL, LPUART_CTRL_RE_SHIFT);
    }
}

static inline void uart_poweroff_lpuart(uart_t uart)
{
    LPUART_Type *dev = uart_config[uart].dev;

    /* Disable receiver */
    bit_clear32(&dev->CTRL, LPUART_CTRL_RE_SHIFT);
    /* Disable transmitter */
    bit_clear32(&dev->CTRL, LPUART_CTRL_TE_SHIFT);
}

#if defined(LPUART_0_ISR) || defined(LPUART_1_ISR) || defined(LPUART_2_ISR) || \
    defined(LPUART_3_ISR) || defined(LPUART_4_ISR)
static inline void irq_handler_lpuart(uart_t uart)
{
    LPUART_Type *dev = uart_config[uart].dev;
    uint32_t stat = dev->STAT;
    /* Clear all IRQ flags */
    dev->STAT = stat;

    if (pm_blocked && (stat & LPUART_STAT_TC_MASK)) {
        PM_UNBLOCK(KINETIS_PM_WAIT);
        pm_blocked = false;
//         LED0_OFF;

        /* disable TX complete interrupt */
        dev->CTRL &= ~LPUART_CTRL_TCIE_MASK;
    }

    if (stat & LPUART_STAT_RDRF_MASK) {
        /* RDRF flag will be cleared when LPUART_DATA is read */
        uint8_t data = dev->DATA;
        if (stat & (LPUART_STAT_FE_MASK | LPUART_STAT_PF_MASK)) {
            if (stat & LPUART_STAT_FE_MASK) {
                DEBUG("LPUART framing error %08" PRIx32 "\n", stat);
            }
            if (stat & LPUART_STAT_PF_MASK) {
                DEBUG("LPUART parity error %08" PRIx32 "\n", stat);
            }
            /* FE is set whenever the next character to be read from LPUART_DATA
             * was received with logic 0 detected where a stop bit was expected. */
            /* PF is set whenever the next character to be read from LPUART_DATA
             * was received when parity is enabled (PE = 1) and the parity bit in
             * the received character does not agree with the expected parity value. */
        }
        /* Only run callback if no error occurred */
        else if (config[uart].rx_cb != NULL) {
            config[uart].rx_cb(config[uart].arg, data);
        }
    }
    if (stat & LPUART_STAT_OR_MASK) {
        /* Input buffer overflow, means that the software was too slow to
         * receive the data */
        DEBUG("LPUART overrun %08" PRIx32 "\n", stat);
    }
    if (stat & LPUART_STAT_IDLE_MASK) {
        if (config[uart].active) {
            config[uart].active = 0;
            /* UART clock can stop now that RX is complete */
            PM_UNBLOCK(UART_CLOCK_PM_BLOCKER);
#ifdef MODULE_PERIPH_GPIO_IRQ
            /* Need an interrupt source to wake for the next start bit */
            gpio_irq_enable(uart_config[uart].pin_rx);
#endif
            DEBUG("LPUART IDLE\n");
        }
    }

    cortexm_isr_end();
}
#endif

void isr_dma0(void)
{
    uint32_t irq_flags = DMA0->INT;
    /* clear flags */
//     DMA0->INT = irq_flags;

    /* DMA from internal buffer to UART->DATA transmit register */
    if (irq_flags & DMA_INT_INT0_MASK) {
        DMA0->INT = DMA_INT_INT0_MASK;

        assert(DMA0->ERR == 0);
        if (DMA0->ERR != 0) {
            while (1) {}
        }

        /* disable DMA request on transmit data register empty */
        LPUART0->BAUD &= ~LPUART_BAUD_TDMAE_MASK;

        /* allow uart_write() to be entered again as we're done with buf */
//         mutex_unlock(&config[UART_DEV(0)].tx_mutex);

        tsrb_drop(&config[UART_DEV(0)].tx_rb, buf_len);

        volatile size_t bytes_available = tsrb_avail(&config[UART_DEV(0)].tx_rb);
        volatile uint8_t empty = tsrb_empty(&config[UART_DEV(0)].tx_rb);
        (void)empty;
        if (bytes_available) {
            dma_start(UART_DEV(0));
        }
        else {
            dma_is_in = false;
        }

        assert(DMA0->ERR == 0);
        if (DMA0->ERR != 0) {
            while (1) {}
        }
    }
}

void isr_dma1(void)
{
    uint32_t irq_flags = DMA0->INT;
    /* clear flags */
//     DMA0->INT = irq_flags;

    /* DMA from uart_write() data to internal buffer */
    if (irq_flags & DMA_INT_INT1_MASK) {
        DMA0->INT = DMA_INT_INT1_MASK;

        if (!dma_is_in && tsrb_avail(&config[UART_DEV(0)].tx_rb)) {
            dma_start(UART_DEV(0));
        }

        dma_memcpy_done = true;
        PM_UNBLOCK(KINETIS_PM_STOP);
//         LED0_OFF;
        memcpy_start(UART_DEV(0));

//         mutex_unlock(&config[UART_DEV(0)].tx_mutex);
    }
}

void isr_dma_error(void)
{
    assert(DMA0->ERR == 0);
    if (DMA0->ERR != 0) {
        while (1) {}
    }
}

#ifdef LPUART_0_ISR
void LPUART_0_ISR(void)
{
    irq_handler_lpuart(UART_DEV(0));
}
#endif

#ifdef LPUART_1_ISR
void LPUART_1_ISR(void)
{
    irq_handler_lpuart(UART_DEV(1));
}
#endif

#ifdef LPUART_2_ISR
void LPUART_2_ISR(void)
{
    irq_handler_lpuart(UART_DEV(2));
}
#endif

#ifdef LPUART_3_ISR
void LPUART_3_ISR(void)
{
    irq_handler_lpuart(UART_DEV(3));
}
#endif

#ifdef LPUART_4_ISR
void LPUART_4_ISR(void)
{
    irq_handler_lpuart(UART_DEV(4));
}
#endif

#endif /* KINETIS_HAVE_LPUART */

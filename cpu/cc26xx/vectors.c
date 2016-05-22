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
 * @ingroup     cpu_cc26xx
 * @{
 *
 * @file
 * @brief       Interrupt vector definitions
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include <stdint.h>
#include "vectors_cortexm.h"

/* get the start of the ISR stack as defined in the linkerscript */
extern uint32_t _estack;

/* define a local dummy handler as it needs to be in the same compilation unit
 * as the alias definition */
void dummy_handler(void) {
    dummy_handler_default();
}

/* Cortex-M common interrupt vectors */
WEAK_DEFAULT void isr_svc(void);
WEAK_DEFAULT void isr_pendsv(void);
WEAK_DEFAULT void isr_systick(void);
/* CC26xx specific interrupt vectors */
WEAK_DEFAULT void GPIOIntHandler(void);
WEAK_DEFAULT void I2CIntHandler(void);
WEAK_DEFAULT void RFCCPE1IntHandler(void);
WEAK_DEFAULT void AONIntHandler(void);
WEAK_DEFAULT void AONRTCIntHandler(void);
WEAK_DEFAULT void UART0IntHandler(void);
WEAK_DEFAULT void AUXSWEvent0IntHandler(void);
WEAK_DEFAULT void SSI0IntHandler(void);
WEAK_DEFAULT void SSI1IntHandler(void);
WEAK_DEFAULT void RFCCPE0IntHandler(void);
WEAK_DEFAULT void RFCHardwareIntHandler(void);
WEAK_DEFAULT void RFCCmdAckIntHandler(void);
WEAK_DEFAULT void I2SIntHandler(void);
WEAK_DEFAULT void AUXSWEvent1IntHandler(void);
WEAK_DEFAULT void WatchdogIntHandler(void);
WEAK_DEFAULT void Timer0AIntHandler(void);
WEAK_DEFAULT void Timer0BIntHandler(void);
WEAK_DEFAULT void Timer1AIntHandler(void);
WEAK_DEFAULT void Timer1BIntHandler(void);
WEAK_DEFAULT void Timer2AIntHandler(void);
WEAK_DEFAULT void Timer2BIntHandler(void);
WEAK_DEFAULT void Timer3AIntHandler(void);
WEAK_DEFAULT void Timer3BIntHandler(void);
WEAK_DEFAULT void CryptoIntHandler(void);
WEAK_DEFAULT void uDMAIntHandler(void);
WEAK_DEFAULT void uDMAErrIntHandler(void);
WEAK_DEFAULT void FlashIntHandler(void);
WEAK_DEFAULT void SWEvent0IntHandler(void);
WEAK_DEFAULT void AUXCombEventIntHandler(void);
WEAK_DEFAULT void AONProgIntHandler(void);
WEAK_DEFAULT void DynProgIntHandler(void);
WEAK_DEFAULT void AUXCompAIntHandler(void);
WEAK_DEFAULT void AUXADCIntHandler(void);
WEAK_DEFAULT void TRNGIntHandler(void);

__attribute__((used,section(".boot_vectors"))) const void *interrupt_vector[] = {
    (void*) (&_estack),                     /* pointer to the top of the stack */
    /* Cortex-M3 handlers */
    (void*) reset_handler_default,  /* entry point of the program */
    (void*) nmi_default,            /* non maskable interrupt handler */
    (void*) hard_fault_default,     /* hard fault exception */
    (void*) mem_manage_default,     /* memory manage exception */
    (void*) bus_fault_default,      /* bus fault exception */
    (void*) usage_fault_default,    /* usage fault exception */
    (void*) (0UL),                  /* Reserved */
    (void*) (0UL),                  /* Reserved */
    (void*) (0UL),                  /* Reserved */
    (void*) (0UL),                  /* Reserved */
    (void*) isr_svc,                /* system call interrupt, in RIOT used for
                                     * switching into thread context on boot */
    (void*) debug_mon_default,      /* debug monitor exception */
    (void*) (0UL),                  /* Reserved */
    (void*) isr_pendsv,             /* pendSV interrupt, in RIOT the actual
                                     * context switching is happening here */
    (void*) isr_systick,            /* SysTick interrupt, not used in RIOT */
    /* CC26xx specific peripheral handlers */
    (void*) GPIOIntHandler,                         // AON edge detect
    (void*) I2CIntHandler,                          // I2C
    (void*) RFCCPE1IntHandler,                      // RF Core Command & Packet Engine 1
    (void*) AONIntHandler,                          // AON SpiSplave Rx, Tx and CS
    (void*) AONRTCIntHandler,                       // AON RTC
    (void*) UART0IntHandler,                        // UART0 Rx and Tx
    (void*) AUXSWEvent0IntHandler,                  // AUX software event 0
    (void*) SSI0IntHandler,                         // SSI0 Rx and Tx
    (void*) SSI1IntHandler,                         // SSI1 Rx and Tx
    (void*) RFCCPE0IntHandler,                      // RF Core Command & Packet Engine 0
    (void*) RFCHardwareIntHandler,                  // RF Core Hardware
    (void*) RFCCmdAckIntHandler,                    // RF Core Command Acknowledge
    (void*) I2SIntHandler,                          // I2S
    (void*) AUXSWEvent1IntHandler,                  // AUX software event 1
    (void*) WatchdogIntHandler,                     // Watchdog timer
    (void*) Timer0AIntHandler,                      // Timer 0 subtimer A
    (void*) Timer0BIntHandler,                      // Timer 0 subtimer B
    (void*) Timer1AIntHandler,                      // Timer 1 subtimer A
    (void*) Timer1BIntHandler,                      // Timer 1 subtimer B
    (void*) Timer2AIntHandler,                      // Timer 2 subtimer A
    (void*) Timer2BIntHandler,                      // Timer 2 subtimer B
    (void*) Timer3AIntHandler,                      // Timer 3 subtimer A
    (void*) Timer3BIntHandler,                      // Timer 3 subtimer B
    (void*) CryptoIntHandler,                       // Crypto Core Result available
    (void*) uDMAIntHandler,                         // uDMA Software
    (void*) uDMAErrIntHandler,                      // uDMA Error
    (void*) FlashIntHandler,                        // Flash controller
    (void*) SWEvent0IntHandler,                     // Software Event 0
    (void*) AUXCombEventIntHandler,                 // AUX combined event
    (void*) AONProgIntHandler,                      // AON programmable 0
    (void*) DynProgIntHandler,                      // Dynamic Programmable interrupt
                                                    // source (Default: PRCM)
    (void*) AUXCompAIntHandler,                     // AUX Comparator A
    (void*) AUXADCIntHandler,                       // AUX ADC new sample or ADC DMA
                                                    // done, ADC underflow, ADC overflow
    (void*) TRNGIntHandler                          // TRNG event
};

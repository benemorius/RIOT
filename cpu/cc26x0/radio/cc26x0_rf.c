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
 * @brief       Low-level radio driver for cc26x0_rf
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 * @}
 */

#include "periph_conf.h"

#include "cc26x0_rf.h"
#include "cc26x0_rf_netdev.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

typedef struct {
    reg32_t *reg_addr;
    uint32_t value;
} init_pair_t;

static const init_pair_t init_table[] = {
//     {&SYS_CTRL_RCGCRFC,      0x01                     },
//     {&SYS_CTRL_SCGCRFC,      0x01                     },
//     {&SYS_CTRL_DCGCRFC,      0x01                     },
//     {&RFCORE_XREG_CCACTRL0,  0xf8                     },
//     {&RFCORE_XREG_TXFILTCFG, 0x09                     },
//     {&RFCORE_XREG_AGCCTRL1,  0x15                     },
//     {&ANA_REGS_IVCTRL,       0x0b                     },
//     {&RFCORE_XREG_MDMTEST1,  0x08                     },
//     {&RFCORE_XREG_FSCAL1,    0x01                     },
//     {&RFCORE_XREG_RXCTRL,    0x3f                     },
//     {&RFCORE_XREG_MDMCTRL1,  0x14                     },
//     {&RFCORE_XREG_ADCTEST0,  0x10                     },
//     {&RFCORE_XREG_ADCTEST1,  0x0e                     },
//     {&RFCORE_XREG_ADCTEST2,  0x03                     },
//     {&RFCORE_XREG_CSPT,      0xff                     },
//     {&RFCORE_XREG_MDMCTRL0,  0x85                     },
//     {&RFCORE_XREG_FSCTRL,    0x55                     },
//     {&RFCORE_XREG_FRMCTRL0,  AUTOCRC | AUTOACK        },
//     {&RFCORE_XREG_FRMCTRL1,  0x00                     },
//     {&RFCORE_XREG_SRCMATCH,  0x00                     },
//     {&RFCORE_XREG_FIFOPCTRL, CC2538_RF_MAX_DATA_LEN   },
//     {&RFCORE_XREG_RFIRQM0,   FIFOP | RXPKTDONE        },
//     {&RFCORE_XREG_RFERRM,    STROBE_ERR | TXUNDERF | TXOVERF | RXUNDERF | RXOVERF | NLOCK},
//     {NULL, 0},
};

bool cc26x0_rf_channel_clear(void)
{
//     if (RFCORE->XREG_FSMSTAT0bits.FSM_FFCTRL_STATE == FSM_STATE_IDLE) {
//         bool result;
//         cc26x0_rf_on();
//         RFCORE_WAIT_UNTIL(RFCORE->XREG_RSSISTATbits.RSSI_VALID);
//         result = BOOLEAN(RFCORE->XREG_FSMSTAT1bits.CCA);
//         cc26x0_rf_off();
//         return result;
//     }
//     else {
//         RFCORE_WAIT_UNTIL(RFCORE->XREG_RSSISTATbits.RSSI_VALID);
//         return BOOLEAN(RFCORE->XREG_FSMSTAT1bits.CCA);
//     }
    return 0;
}

void cc26x0_rf_init(void)
{
//     const init_pair_t *pair;
//
//     for (pair = init_table; pair->reg_addr != NULL; pair++) {
//         *pair->reg_addr = pair->value;
//     }
//
//     cc26x0_rf_set_tx_power(CC2538_RF_POWER_DEFAULT);
//     cc26x0_rf_set_chan(CC2538_RF_CHANNEL_DEFAULT);
//     cc26x0_rf_set_pan(CC2538_RF_PANID_DEFAULT);
//     cc26x0_rf_set_addr_long(cc26x0_rf_get_eui64_primary());
//
//     /* Select the observable signals (maximum of three) */
//     RFCORE_XREG_RFC_OBS_CTRL0 = tx_active;
//     RFCORE_XREG_RFC_OBS_CTRL1 = rx_active;
//     RFCORE_XREG_RFC_OBS_CTRL2 = ffctrl_fifo;
//
//     /* Select output pins for the three observable signals */
// #ifdef BOARD_OPENMOTE_CC2538
//     CCTEST_OBSSEL0 = 0;                        /* PC0 = USB_SEL        */
//     CCTEST_OBSSEL1 = 0;                        /* PC1 = N/C            */
//     CCTEST_OBSSEL2 = 0;                        /* PC2 = N/C            */
//     CCTEST_OBSSEL3 = 0;                        /* PC3 = USER_BUTTON    */
//     CCTEST_OBSSEL4 = OBSSEL_EN | rfc_obs_sig0; /* PC4 = RED_LED        */
//     CCTEST_OBSSEL5 = 0;                        /* PC5 = ORANGE_LED     */
//     CCTEST_OBSSEL6 = OBSSEL_EN | rfc_obs_sig1; /* PC6 = YELLOW_LED     */
//     CCTEST_OBSSEL7 = OBSSEL_EN | rfc_obs_sig2; /* PC7 = GREEN_LED      */
// #else
//     /* Assume BOARD_CC2538DK (or similar). */
//     CCTEST_OBSSEL0 = OBSSEL_EN | rfc_obs_sig0; /* PC0 = LED_1 (red)    */
//     CCTEST_OBSSEL1 = OBSSEL_EN | rfc_obs_sig1; /* PC1 = LED_2 (yellow) */
//     CCTEST_OBSSEL2 = OBSSEL_EN | rfc_obs_sig2; /* PC2 = LED_3 (green)  */
//     CCTEST_OBSSEL3 = 0;                        /* PC3 = LED_4 (red)    */
//     CCTEST_OBSSEL4 = 0;                        /* PC4 = BTN_L          */
//     CCTEST_OBSSEL5 = 0;                        /* PC5 = BTN_R          */
//     CCTEST_OBSSEL6 = 0;                        /* PC6 = BTN_UP         */
//     CCTEST_OBSSEL7 = 0;                        /* PC7 = BTN_DN         */
// #endif /* BOARD_OPENMOTE_CC2538 */
//
//     if (SYS_CTRL->I_MAP) {
//         NVIC_SetPriority(RF_RXTX_ALT_IRQn, RADIO_IRQ_PRIO);
//         NVIC_EnableIRQ(RF_RXTX_ALT_IRQn);
//
//         NVIC_SetPriority(RF_ERR_ALT_IRQn, RADIO_IRQ_PRIO);
//         NVIC_EnableIRQ(RF_ERR_ALT_IRQn);
//     }
//     else {
//         NVIC_SetPriority(RF_RXTX_IRQn, RADIO_IRQ_PRIO);
//         NVIC_EnableIRQ(RF_RXTX_IRQn);
//
//         NVIC_SetPriority(RF_ERR_IRQn, RADIO_IRQ_PRIO);
//         NVIC_EnableIRQ(RF_ERR_IRQn);
//     }
//
//     /* Flush the receive and transmit FIFOs */
//     RFCORE_SFR_RFST = ISFLUSHTX;
//     RFCORE_SFR_RFST = ISFLUSHRX;

    cc26x0_rf_on();
}

bool cc26x0_rf_is_on(void)
{
//     return RFCORE->XREG_FSMSTAT1bits.RX_ACTIVE || RFCORE->XREG_FSMSTAT1bits.TX_ACTIVE;
    return 0;
}

void cc26x0_rf_off(void)
{
//     /* Wait for ongoing TX to complete (e.g. this could be an outgoing ACK) */
//     RFCORE_WAIT_UNTIL(RFCORE->XREG_FSMSTAT1bits.TX_ACTIVE == 0);
//
//     /* Flush RX FIFO */
//     RFCORE_SFR_RFST = ISFLUSHRX;
//
//     /* Don't turn off if we are off as this will trigger a Strobe Error */
//     if (RFCORE_XREG_RXENABLE != 0) {
//         RFCORE_SFR_RFST = ISRFOFF;
//     }
}

bool cc26x0_rf_on(void)
{
//     /* Flush RX FIFO */
//     RFCORE_SFR_RFST = ISFLUSHRX;
//
//     /* Enable/calibrate RX */
//     RFCORE_SFR_RFST = ISRXON;

    return true;
}

void cc26x0_rf_setup(cc26x0_rf_t *dev)
{
    netdev2_t *netdev = (netdev2_t *)dev;

    printf("setting driver to 0x%lx\n", (uint32_t)&cc26x0_rf_driver);
    netdev->driver = &cc26x0_rf_driver;

    mutex_init(&dev->mutex);

    cc26x0_rf_init();
}

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

#include <string.h>

#include "periph_conf.h"
#include "cc26x0_rf.h"
#include "cc26x0_rf_netdev.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

uint32_t rfc_send_cmd(void *ropCmd);
uint16_t rfc_wait_cmd_done(radio_op_command_t *command);
uint32_t rfc_cmd_and_wait(void *ropCmd, uint16_t *status);
void rfc_irq_enable(void);
void rfc_irq_disable(void);
bool rfc_start_rat(void);
void cpu_clock_init(void);
int rfc_setup_154(void);
int recv_154(int channel);

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


//     cc26x0_rf_on();
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

    printf("cc26x0_rf_off()\n");
    return;

    /* ??? (not required I think) */
    *(reg32_t*)(0x60041000 + 0x00000010) = 0x0;
    *(reg32_t*)(0x60041000 + 0x00000014) = 0x0;

    /* need to send FS_POWERDOWN or analog components will use power */
    rfc_CMD_FS_POWERDOWN_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.commandNo = CMDR_CMDID_FS_POWERDOWN;
    cmd.condition.rule = R_OP_CONDITION_RULE_NEVER;

    /* FS_POWERDOWN command doesn't cause RFCPEIFG.COMMAND_DONE interrupt */
    uint32_t cmd_status = rfc_send_cmd(&cmd);
    if (cmd_status != 1) {
        printf("bad CMDSTA: 0x%lx", cmd_status);
        while(1);
    }
    uint16_t status = rfc_wait_cmd_done((radio_op_command_t*)&cmd);
    if (status != 0x400) {
        printf("bad status: 0x%x", status);
        while(1);
    }

    rfc_irq_disable();

    /* Shut down the RFCORE clock domain in the MCU VD */
    PRCM->RFCCLKG = 0;
    PRCM->CLKLOADCTL = CLKLOADCTL_LOAD;
    while (!(PRCM->CLKLOADCTL & CLKLOADCTL_LOADDONE)) {}

    /* Turn off RFCORE PD */
    PRCM->PDCTL0 &= ~PDCTL0_RFC_ON;
    while (PRCM->PDSTAT0 & PDSTAT0_RFC_ON) {}

    /* Enable the Osc interface and remember the state of the SMPH clock */
    // Force power on AUX to ensure CPU has access
    *(reg32_t*)(AON_WUC_BASE + AON_WUC_O_AUXCTL) |= AON_WUC_AUXCTL_AUX_FORCE_ON;
    while(!(*(reg32_t*)(AON_WUC_BASE + AON_WUC_O_PWRSTAT) & AONWUC_AUX_POWER_ON)) { }

    // Enable the AUX domain OSC clock and wait for it to be ready
    *(reg32_t*)(AUX_WUC_BASE + AUX_WUC_O_MODCLKEN0) |= AUX_WUC_OSCCTRL_CLOCK;
    while(!(*(reg32_t*)(AUX_WUC_BASE + AUX_WUC_O_MODCLKEN0) & AUX_WUC_MODCLKEN0_AUX_DDI0_OSC)) { }

    /* Set HF and MF clock sources to the HF RC Osc */
    DDI_0_OSC->CTL0 &= ~(DDI_0_OSC_CTL0_SCLK_HF_SRC_SEL_M | DDI_0_OSC_CTL0_SCLK_MF_SRC_SEL_M);

    /* Check to not enable HF RC oscillator if already enabled */
    if ((DDI_0_OSC->STAT0 & DDI_0_OSC_STAT0_SCLK_HF_SRC_M) != DDI_0_OSC_STAT0_SCLK_HF_SRC_RCOSC) {
        /* Switch the HF clock source (cc26xxware executes this from ROM) */
        ROM_HAPI_HFSOURCESAFESWITCH();
    }
}

bool cc26x0_rf_on(void)
{
    printf("cc26x0_rf_on()\n");

    /* switch to xosc */
    cpu_clock_init();

    /* rfc mode must be set before powering up radio (undocumented) */
    printf("RFCMODESEL: 0x%lx\n", PRCM->RFCMODESEL);
    PRCM->RFCMODESEL = 0x5;
    printf("RFCMODESEL: 0x%lx\n", PRCM->RFCMODESEL);

    /* enable RFC clock */
    PRCM->RFCCLKG = 1;
    PRCM->CLKLOADCTL = CLKLOADCTL_LOAD;
    while (!(PRCM->CLKLOADCTL & CLKLOADCTL_LOADDONE)) ;

    /* enable RFC power domain */
    PRCM->PDCTL0 |= PDCTL0_RFC_ON;
    while (!(PRCM->PDSTAT0 & PDSTAT0_RFC_ON)) {}

    //RFC_PWR->PWMCLKEN |= RFC_PWR_PWMCLKEN_CPE;
    //RFC_PWR->PWMCLKEN |= RFC_PWR_PWMCLKEN_CPERAM;
    RFC_PWR->PWMCLKEN |= 0x7FF;
//     printf("RFC_PWR->PWMCLKEN %lx\n", RFC_PWR->PWMCLKEN);

    /* disable all except last command done interrupt */
//     RFC_DBELL->RFCPEIEN = 0x2;
    rfc_irq_enable();

    /*RFC TIMER */
    rfc_start_rat();

    rfc_setup_154();

    recv_154(25);

    return true;
}

void cc26x0_rf_setup(cc26x0_rf_t *dev)
{
    netdev2_t *netdev = (netdev2_t *)dev;

    netdev->driver = &cc26x0_rf_driver;

    mutex_init(&dev->mutex);

    cc26x0_rf_init();
}

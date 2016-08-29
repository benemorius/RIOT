/*
 * Copyright (C) 2016 Leon George, Florent-Valéry Coen
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "cc26x0_prcm.h"
#include "cc26x0_rfc.h"
#include "cc26x0_rfc_ieee.h"
#include "mutex.h"
#include "lpm.h"
#include "thread.h"
#include "xtimer.h"

#define BLE_ADV_STR "this is not a riot\n"

/* BLE Advertisement-related macros */
#define BLE_ADV_TYPE_DEVINFO      0x01
#define BLE_ADV_TYPE_NAME         0x09
#define BLE_ADV_TYPE_MANUFACTURER 0xFF
#define BLE_ADV_NAME_BUF_LEN        29
#define BLE_ADV_PAYLOAD_BUF_LEN     64
#define BLE_UUID_SIZE               16

void cpu_clock_init(void);
static void init_rf_params(rfc_CMD_IEEE_RX_t* cmd);

static rfc_bleAdvPar_t ble_params_buf __attribute__((__aligned__(4)));
uint16_t ble_mac_address[3] __attribute__((__aligned__(4))) = {0xeeff, 0xccdd, 0xaabb};
char adv_name[BLE_ADV_NAME_BUF_LEN] = {"riot-test"};

mutex_t cmd_wait = MUTEX_INIT;
mutex_t done_wait = MUTEX_INIT;

#define RX_BUF_SIZE 512
static uint8_t rx_buf_0[RX_BUF_SIZE] __attribute__((__aligned__(4)));
static uint8_t rx_buf_1[RX_BUF_SIZE] __attribute__((__aligned__(4)));
static uint8_t rx_buf_2[RX_BUF_SIZE] __attribute__((__aligned__(4)));
static uint8_t rx_buf_3[RX_BUF_SIZE] __attribute__((__aligned__(4)));
static __attribute__((__aligned__(4))) dataQueue_t rx_data_queue = { 0 };
static __attribute__((__aligned__(4))) uint8_t rf_stats[16] = { 0 };

typedef struct output_config {
    uint8_t dbm;
    uint8_t register_ib;
    uint8_t register_gc;
    uint8_t temp_coeff;
} output_config_t;

static const output_config_t output_power[] = {
    {  5, 0x30, 0x00, 0x93 },
    {  4, 0x24, 0x00, 0x93 },
    {  3, 0x1c, 0x00, 0x5a },
    {  2, 0x18, 0x00, 0x4e },
    {  1, 0x14, 0x00, 0x42 },
    {  0, 0x21, 0x01, 0x31 },
    { -3, 0x18, 0x01, 0x25 },
    { -6, 0x11, 0x01, 0x1d },
    { -9, 0x0e, 0x01, 0x19 },
    {-12, 0x0b, 0x01, 0x14 },
    {-15, 0x0b, 0x03, 0x0c },
    {-18, 0x09, 0x03, 0x0c },
    {-21, 0x07, 0x03, 0x0c },
};

const output_config_t *tx_power_current = &output_power[0];

void isr_rfc_cmd_ack(void)
{
    /*ROP ack = op submitted, DIR or IMM ack = op executed*/
//     printf("Command acknowledged. CMDSTA: 0x%" PRIx32 " \n", RFC_DBELL->CMDSTA);
    RFC_DBELL->RFACKIFG = 0;
    mutex_unlock(&cmd_wait);
//     printf("ack\n");

    if (sched_context_switch_request) {
        thread_yield();
    }
}

void isr_rfc_hw(void)
{
    uint32_t flags = RFC_DBELL->RFHWIFG;
    printf("hw 0x%" PRIx32 "\n", flags);
    RFC_DBELL->RFHWIFG = ~flags;

    if (sched_context_switch_request) {
        thread_yield();
    }
}

void isr_rfc_cpe0(void)
{
    uint32_t flags = RFC_DBELL->RFCPEIFG & (~RFC_DBELL->RFCPEISL);
    RFC_DBELL->RFCPEIFG = ~flags;
    printf("cpe0 0x%" PRIx32 "\n", flags);

    if (flags & (0x2 | 0x8)) {
        mutex_unlock(&done_wait);
    }
//     printf("done\n");

    if (sched_context_switch_request) {
        thread_yield();
    }
}

void isr_rfc_cpe1(void)
{
    uint32_t flags = RFC_DBELL->RFCPEIFG & RFC_DBELL->RFCPEISL;
    printf("cpe1 0x%" PRIx32 "\n", flags);
    RFC_DBELL->RFCPEIFG = ~flags;

    if (sched_context_switch_request) {
        thread_yield();
    }
}

void rfc_irq_enable(void)
{
    NVIC_EnableIRQ(RF_CMD_ACK_IRQN);
    NVIC_EnableIRQ(RF_CPE0_IRQN);
    NVIC_EnableIRQ(RF_CPE1_IRQN);
    NVIC_EnableIRQ(RF_HW_IRQN);
}

void rfc_irq_disable(void)
{
    NVIC_DisableIRQ(RF_CMD_ACK_IRQN);
    NVIC_DisableIRQ(RF_CPE0_IRQN);
    NVIC_DisableIRQ(RF_CPE1_IRQN);
    NVIC_DisableIRQ(RF_HW_IRQN);
}

uint32_t rfc_send_cmd(void *ropCmd)
{
//     printf("rfc_send_cmd()\n");

    ++lpm_prevent_sleep;

    mutex_lock(&cmd_wait);
//     printf("cmd\n");
    RFC_DBELL->CMDR = (uint32_t) ropCmd;

    /* wait for cmd ack (rop cmd was submitted successfully) */
    mutex_lock(&cmd_wait);
    mutex_unlock(&cmd_wait);

//     printf("ret\n");

    --lpm_prevent_sleep;

    return RFC_DBELL->CMDSTA;
}

uint16_t rfc_wait_cmd_done(volatile radio_op_command_t *command)
{
//     printf("rfc_wait_cmd_done()\n");

    uint32_t timeout_cnt = 0;
    /* wait for cmd execution. condition on rop status doesn't work by itself (too fast?). */
    do {
        if (++timeout_cnt > 500000) {
            break;
        }
    } while (command->status < R_OP_STATUS_SKIPPED);

    return command->status;
}

uint32_t rfc_cmd_and_wait(void *ropCmd, uint16_t *status)
{
    radio_op_command_t *cmd = ropCmd;

    ++lpm_prevent_sleep;

    mutex_lock(&done_wait);

//     printf("cmd\n");
    uint32_t send_status = rfc_send_cmd(ropCmd);

    /* TODO probably need to check if send_status is good and skip waiting
     * for done_wait if not */

    mutex_lock(&done_wait);
    mutex_unlock(&done_wait);

    printf("fin\n");

    *status = cmd->status;

    --lpm_prevent_sleep;

    return send_status;
}

bool rfc_setup_ble(void)
{
//     printf("rfc_setup_ble()\n");

    radio_setup_cmd_t rs;
    memset(&rs, 0, sizeof(rs));

    rs.ropCmd.commandNo = CMDR_CMDID_SETUP;
    rs.ropCmd.status = R_OP_STATUS_IDLE;
    rs.ropCmd.condition.rule = R_OP_CONDITION_RULE_NEVER;
    rs.mode |= RADIO_SETUP_MODE_BLE;
    rs.txPower.IB = 0x29;
    rs.txPower.GC = 0x00;
    rs.txPower.tempCoeff = 0x00;
    rs.txPower.boost = 0x00;
    static uint32_t ble_overrides[] = {
        0x00364038, /* Synth: Set RTRIM (POTAILRESTRIM) to 6 */
        0x000784A3, /* Synth: Set FREF = 3.43 MHz (24 MHz / 7) */
        0xA47E0583, /* Synth: Set loop bandwidth after lock to 80 kHz (K2) */
        0xEAE00603, /* Synth: Set loop bandwidth after lock to 80 kHz (K3, LSB) */
        0x00010623, /* Synth: Set loop bandwidth after lock to 80 kHz (K3, MSB) */
        0x00456088, /* Adjust AGC reference level */
        0xFFFFFFFF, /* End of override list */
    };
    rs.pRegOverride = ble_overrides;

    uint16_t status;
    rfc_cmd_and_wait(&rs, &status);

    return status == R_OP_STATUS_DONE_OK;
}

int send_ble_adv_nc(uint8_t iterations, uint8_t *adv_payload, uint8_t adv_payload_len)
{
//     printf("send_ble_adv_nc()\n");

    /* maximum 31 byte payload */
    if (adv_payload_len > 31) {
        return -1;
    }

    rfc_CMD_BLE_ADV_NC_t cmds[iterations] __attribute__((__aligned__(4)));
    rfc_CMD_BLE_ADV_NC_t *cmd = cmds;
    rfc_bleAdvPar_t *params = (rfc_bleAdvPar_t *)&ble_params_buf;

    /* Clear both buffers */
    memset(cmds, 0x00, sizeof(cmds));
    memset(params, 0x00, sizeof(*params));

    /* Set up BLE Advertisement parameters */
    params->pDeviceAddress = ble_mac_address;
    params->endTrigger.triggerType = R_OP_STARTTRIG_TYPE_TRIG_NEVER;
    params->endTime = R_OP_STARTTRIG_TYPE_TRIG_NEVER;
    params->advLen = adv_payload_len;
    params->pAdvData = adv_payload;

    /* Adv NC */
    cmd->commandNo = 0x1805;
    cmd->condition.rule = R_OP_CONDITION_RULE_ALWAYS;
    cmd->whitening.bOverride = 0;
    cmd->whitening.init = 0;
    cmd->pParams = params;
    cmd->channel = 37;
//     cmd->startTrigger.triggerType = R_OP_STARTTRIG_TYPE_REL_PREVSTART;
//     cmd->startTime = 4000*20;

    uint8_t p;
    for (p = 1; p < iterations; ++p) {
        memcpy(&cmds[p], cmd, sizeof(*cmd));
        cmds[p].channel = (p % 3) + 37;
        cmds[p-1].pNextOp = (radio_op_command_t*)&cmds[p];
    }

    /* last command has no next command */
    cmds[p-1].pNextOp = 0;
    cmds[p-1].condition.rule = R_OP_CONDITION_RULE_NEVER;

    /* first command doesn't have delayed start */
    cmds[0].startTrigger.triggerType = R_OP_STARTTRIG_TYPE_TRIG_NOW;

    for (int repeat = 0; repeat < 1; ++repeat) {
        uint16_t status;
        uint32_t cmd_status = rfc_cmd_and_wait((uint32_t*)cmd, &status);

        if (cmd_status != 1) {
            printf("bad CMDSTA: 0x%lx\n", cmd_status);
            while(1);
        }

        if (status != 0x1400) {
            printf("bad status: 0x%x\n", status);
            while(1);
        }
    }
    return 0;
}

void rfc_ble_beacon(void)
{
//     printf("rfc_ble_beacon()\n");

    uint8_t adv_name_len = strlen(adv_name);
    if (adv_name_len > 29) {
        adv_name_len = 29;
    }

    uint16_t p = 0;
    static uint8_t payload[BLE_ADV_PAYLOAD_BUF_LEN] __attribute__((__aligned__(4)));
    memset(payload, 0, BLE_ADV_PAYLOAD_BUF_LEN);

    /* device info */
//     payload[p++] = 0x02;          /* 2 bytes */
//     payload[p++] = BLE_ADV_TYPE_DEVINFO;
//     payload[p++] = 0x1a;          /* LE general discoverable + BR/EDR */
    payload[p++] = 1 + adv_name_len;
    payload[p++] = BLE_ADV_TYPE_NAME;
    memcpy(&payload[p], adv_name, adv_name_len);
    p += adv_name_len;

    send_ble_adv_nc(15, payload, p);
}

bool rfc_ping_test(void)
{
    direct_command_t pingCommand;
    pingCommand.commandID = CMDR_CMDID_PING;
    RFC_DBELL->CMDR = (uint32_t) (&pingCommand);
    while (!RFC_DBELL->CMDSTA); /* wait for cmd execution */
//     printf("rfc_ping fails without a printf reading cmdsta %lu\n", RFC_DBELL->CMDSTA);
    return RFC_DBELL->CMDSTA == CMDSTA_RESULT_DONE;
}

bool rfc_nop_test(void)
{
    nop_cmd_t nopCommand;
    memset(&nopCommand, 0, sizeof(nopCommand));
    nopCommand.ropCmd.commandNo = CMDR_CMDID_NOP;
    nopCommand.ropCmd.status = R_OP_STATUS_IDLE;
    nopCommand.ropCmd.condition.rule = 1; /* never run next cmd. need to implement definition */

    uint16_t status;
    rfc_cmd_and_wait(&nopCommand, &status);
    return status == R_OP_STATUS_DONE_OK;
}

bool rfc_start_rat(void)
{
    direct_command_t ratCommand;
    ratCommand.commandID = CMDR_CMDID_START_RAT;
    uint32_t status = rfc_send_cmd(&ratCommand);
    /* 0x1 if successful; 0x85 if already running */
    if (status != 1 && status != 0x85) {
        printf("bad CMDSTA: 0x%lx\n", status);
        while(1);
    }
    printf("start_rat status: 0x%lx\n", status);
    return status == CMDSTA_RESULT_DONE;
}

void rfc_prepare(void)
{
    /* switch to xosc */
    cpu_clock_init();

//     printf("rfc_prepare()\n");
    /* rfc mode must be set before powering up radio (undocumented) */
//     uint32_t *rfc_mode_hwopt = (uint32_t*)0x400821D4;
//     printf("modeopt: 0x%lx\n", ((*rfc_mode_hwopt) >> 1) & 0x7);
    PRCM->RFCMODESEL = 0x5;

    /* RFC POWER DOMAIN CLOCK GATE */
    PRCM->RFCCLKG = 1;
    PRCM->CLKLOADCTL = CLKLOADCTL_LOAD;
    while (!(PRCM->CLKLOADCTL & CLKLOADCTL_LOADDONE)) ;

    /* enable RFC power domain */
    PRCM->PDCTL0 |= PDCTL0_RFC_ON;
    while (!(PRCM->PDSTAT0 & PDSTAT0_RFC_ON)) {}

    /* RFC CLOCK */
    //RFC_PWR->PWMCLKEN |= RFC_PWR_PWMCLKEN_CPE;
    //RFC_PWR->PWMCLKEN |= RFC_PWR_PWMCLKEN_CPERAM;
    RFC_PWR->PWMCLKEN |= 0x7FF;
//     printf("RFC_PWR->PWMCLKEN %lx\n", RFC_PWR->PWMCLKEN);

    /* disable all except last command done interrupt */
    RFC_DBELL->RFCPEIEN = 0x2;
    rfc_irq_enable();

    /*RFC TIMER */
    rfc_start_rat();
}

#include "hw_aon_wuc.h"
#include "hw_aux_wuc.h"
#include "cpu.h"
#include "hw_ddi_0_osc.h"

void rfc_powerdown(void)
{
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
        printf("bad CMDSTA: 0x%lx\n", cmd_status);
        while(1);
    }
    uint16_t status = rfc_wait_cmd_done((radio_op_command_t*)&cmd);
    if (status != 0x400) {
        printf("bad status: 0x%x\n", status);
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

int rfc_setup_154(void)
{
//     printf("rfc_setup_154()\n");

    radio_setup_cmd_t rs;
    memset(&rs, 0, sizeof(rs));

    rs.ropCmd.commandNo = CMDR_CMDID_SETUP;
    rs.ropCmd.condition.rule = R_OP_CONDITION_RULE_NEVER;
    rs.mode = RADIO_SETUP_MODE_IEEE8021504;
    rs.txPower.IB = tx_power_current->register_ib;
    rs.txPower.GC = tx_power_current->register_gc;
    rs.txPower.tempCoeff = tx_power_current->temp_coeff;

    /* Overrides for IEEE 802.15.4, differential mode */
    static uint32_t ieee_overrides[] = {
        0x00354038, /* Synth: Set RTRIM (POTAILRESTRIM) to 5 */
        0x4001402D, /* Synth: Correct CKVD latency setting (address) */
        0x00608402, /* Synth: Correct CKVD latency setting (value) */
        //  0x4001405D, /* Synth: Set ANADIV DIV_BIAS_MODE to PG1 (address) */
        //  0x1801F800, /* Synth: Set ANADIV DIV_BIAS_MODE to PG1 (value) */
        0x000784A3, /* Synth: Set FREF = 3.43 MHz (24 MHz / 7) */
        0xA47E0583, /* Synth: Set loop bandwidth after lock to 80 kHz (K2) */
        0xEAE00603, /* Synth: Set loop bandwidth after lock to 80 kHz (K3, LSB) */
        0x00010623, /* Synth: Set loop bandwidth after lock to 80 kHz (K3, MSB) */
        0x002B50DC, /* Adjust AGC DC filter */
        0x05000243, /* Increase synth programming timeout */
        0x002082C3, /* Increase synth programming timeout */
        0xFFFFFFFF, /* End of override list */
    };
    rs.pRegOverride = ieee_overrides;

    uint16_t status;
    rfc_cmd_and_wait(&rs, &status);

    if (status != R_OP_STATUS_DONE_OK) {
        return status;
    }

    printf("setup_154 got 0x%x\n", status);

    memset(rx_buf_0, 0, RX_BUF_SIZE);
    memset(rx_buf_1, 0, RX_BUF_SIZE);
    memset(rx_buf_2, 0, RX_BUF_SIZE);
    memset(rx_buf_3, 0, RX_BUF_SIZE);

    rx_data_queue.pCurrEntry = rx_buf_0;
    rx_data_queue.pLastEntry = NULL;

    rfc_dataEntry_t *entry;
    entry = (rfc_dataEntry_t *)rx_buf_0;
    entry->pNextEntry = rx_buf_1;
    entry->config.lenSz = 1;
    entry->length = sizeof(rx_buf_0) - 8;

    entry = (rfc_dataEntry_t *)rx_buf_1;
    entry->pNextEntry = rx_buf_2;
    entry->config.lenSz = 1;
    entry->length = sizeof(rx_buf_0) - 8;

    entry = (rfc_dataEntry_t *)rx_buf_2;
    entry->pNextEntry = rx_buf_3;
    entry->config.lenSz = 1;
    entry->length = sizeof(rx_buf_0) - 8;

    entry = (rfc_dataEntry_t *)rx_buf_3;
    entry->pNextEntry = rx_buf_0;
    entry->config.lenSz = 1;
    entry->length = sizeof(rx_buf_0) - 8;

    return 0;
}

void hex_dump(void *start_address, uint32_t bytes)
{
    uint8_t *address = start_address;
    uint8_t *stopAddress = address + bytes;
    printf("printing 0x%lx to 0x%lx\r\n", (uint32_t)address, (uint32_t)stopAddress);
    for( ; address < stopAddress; )
    {
        printf("0x%08lx  %02x%02x %02x%02x  %02x%02x %02x%02x", (uint32_t)address, address[0], address[1], address[2], address[3], address[4], address[5], address[6], address[7]);
        address += 8;
        printf("  %02x%02x %02x%02x  %02x%02x %02x%02x", address[0], address[1], address[2], address[3], address[4], address[5], address[6], address[7]);
        address += 8;

//         printf("  %02x%02x %02x%02x  %02x%02x %02x%02x", address[0], address[1], address[2], address[3], address[4], address[5], address[6], address[7]);
//         address += 8;
//         printf("  %02x%02x %02x%02x  %02x%02x %02x%02x", address[0], address[1], address[2], address[3], address[4], address[5], address[6], address[7]);
//         address += 8;

        printf("\r\n");
    }
}

int send_154(uint8_t *payload, uint8_t payload_len)
{
    printf("send_154() length %u\n", payload_len);

    /* maximum 125 byte payload */
    if (payload_len > 125) {
        return -1;
    }

    hex_dump(payload, payload_len);

    volatile rfc_CMD_IEEE_TX_t cmd __attribute__((__aligned__(4)));

    memset((radio_op_command_t*)&cmd, 0x00, sizeof(cmd));

    cmd.commandNo = CMDR_CMDID_IEEE_TX;
    cmd.condition.rule = R_OP_CONDITION_RULE_NEVER;
    cmd.startTrigger.triggerType = R_OP_STARTTRIG_TYPE_TRIG_NOW;

    cmd.payloadLen = payload_len;
    cmd.pPayload = payload;

    uint32_t cmd_status = rfc_send_cmd((radio_op_command_t*)&cmd);
    uint16_t status = rfc_wait_cmd_done((radio_op_command_t*)&cmd);

    printf("send_154 cmd_status 0x%lx status 0x%x\n", cmd_status, cmd.status);

    if (cmd_status != 1) {
        printf("bad CMDSTA: 0x%lx\n", cmd_status);
        while(1);
    }

    if (status != 0x2400) {
        printf("bad status: 0x%x\n", status);
        while(1);
    }

    return 0;
}

void fs_on(void)
{
    rfc_CMD_FS_t cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.condition.rule = R_OP_CONDITION_RULE_NEVER;
    cmd.commandNo = CMD_FS;
    cmd.frequency = 2475;
    cmd.fractFreq = 0;
    cmd.synthConf.bTxMode = 1;
    uint32_t cmd_status = rfc_send_cmd(&cmd);
    uint16_t status = rfc_wait_cmd_done((radio_op_command_t*)&cmd);

    printf("fs_on cmd_status 0x%lx status 0x%x\n", cmd_status, status);

    if (cmd_status != 1) {
        printf("bad CMDSTA: 0x%lx\n", cmd_status);
        while(1);
    }

    if (status != 0x0400) {
        printf("bad status: 0x%x\n", status);
        while(1);
    }

}

void test_tx(void)
{
    rfc_CMD_TX_TEST_t cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.condition.rule = R_OP_CONDITION_RULE_NEVER;
    cmd.commandNo = CMD_TX_TEST;
    cmd.endTrigger.triggerType = R_OP_STARTTRIG_TYPE_TRIG_NEVER;

    uint32_t cmd_status = rfc_send_cmd(&cmd);
    uint16_t status = rfc_wait_cmd_done((radio_op_command_t*)&cmd);

    printf("test_tx cmd_status 0x%lx status 0x%x\n", cmd_status, status);

    if (cmd_status != 1) {
        printf("bad CMDSTA: 0x%lx\n", cmd_status);
        while(1);
    }

    if (status != 0x0002) {
        printf("bad status: 0x%x\n", status);
        while(1);
    }
}

int recv_154(int channel)
{
    printf("recv_154() channel %i\n", channel);
//     fs_on();
//     test_tx();

//     while (1) {
//         send_154(0, 0);
//         xtimer_usleep(1000*100);
//     }
//     return 0;

    static rfc_CMD_IEEE_RX_t cmd __attribute__((__aligned__(4)));
    init_rf_params(&cmd);
    cmd.channel = channel;

    uint32_t cmd_status = rfc_send_cmd(&cmd);
    uint16_t status = rfc_wait_cmd_done((radio_op_command_t*)&cmd);

    printf("recv_154 cmd_status 0x%lx status 0x%x\n", cmd_status, status);

    if (cmd_status != 1) {
        printf("bad CMDSTA: 0x%lx\n", cmd_status);
        while(1);
    }

    if (status != 0x0002) {
        printf("bad status: 0x%x\n", status);
        while(1);
    }

    send_154(0, 0);
    send_154(0, 0);

//     while (1) {
//         send_154(0, 0);
//         xtimer_usleep(1000*100);
//     }

    return 0;
}

static void init_rf_params(rfc_CMD_IEEE_RX_t *cmd)
{
    memset(cmd, 0x00, sizeof(*cmd));

    cmd->commandNo = CMDR_CMDID_IEEE_RX;
    cmd->pNextOp = NULL;
    cmd->startTime = 0x00000000;
    cmd->condition.rule = R_OP_CONDITION_RULE_NEVER;

    cmd->rxConfig.bAutoFlushCrc = 1;
    cmd->rxConfig.bAutoFlushIgn = 0;
    cmd->rxConfig.bIncludePhyHdr = 0;
    cmd->rxConfig.bIncludeCrc = 1;
    cmd->rxConfig.bAppendRssi = 1;
    cmd->rxConfig.bAppendCorrCrc = 1;
    cmd->rxConfig.bAppendSrcInd = 0;
    cmd->rxConfig.bAppendTimestamp = 0;

    cmd->pRxQ = &rx_data_queue;
    cmd->pOutput = (rfc_ieeeRxOutput_t *)rf_stats;

    cmd->frameFiltOpt.frameFiltEn = 0;

    cmd->frameFiltOpt.frameFiltStop = 0;

    cmd->frameFiltOpt.autoAckEn = 0;

    cmd->frameFiltOpt.slottedAckEn = 0;
    cmd->frameFiltOpt.autoPendEn = 0;
    cmd->frameFiltOpt.defaultPend = 0;
    cmd->frameFiltOpt.bPendDataReqOnly = 0;
    cmd->frameFiltOpt.bPanCoord = 0;
    cmd->frameFiltOpt.maxFrameVersion = 1;
    cmd->frameFiltOpt.bStrictLenFilter = 0;

    /* Receive all frame types */
    cmd->frameTypes.bAcceptFt0Beacon = 1;
    cmd->frameTypes.bAcceptFt1Data = 1;
    cmd->frameTypes.bAcceptFt2Ack = 1;
    cmd->frameTypes.bAcceptFt3MacCmd = 1;
    cmd->frameTypes.bAcceptFt4Reserved = 1;
    cmd->frameTypes.bAcceptFt5Reserved = 1;
    cmd->frameTypes.bAcceptFt6Reserved = 1;
    cmd->frameTypes.bAcceptFt7Reserved = 1;

    /* Configure CCA settings */
    cmd->ccaOpt.ccaEnEnergy = 1;
    cmd->ccaOpt.ccaEnCorr = 1;
    cmd->ccaOpt.ccaEnSync = 0;
    cmd->ccaOpt.ccaCorrOp = 1;
    cmd->ccaOpt.ccaSyncOp = 1;
    cmd->ccaOpt.ccaCorrThr = 3;

    cmd->ccaRssiThr = 0xA6;

    cmd->numExtEntries = 0x00;
    cmd->numShortEntries = 0x00;
    cmd->pExtEntryList = 0;
    cmd->pShortEntryList = 0;

    cmd->endTrigger.triggerType = R_OP_STARTTRIG_TYPE_TRIG_NEVER;
    cmd->endTime = 0x00000000;

    cmd->localPanID = 7;
    cmd->localShortAddr = 8;
    cmd->localExtAddr = 9;
}

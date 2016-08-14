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

#define BLE_ADV_STR "this is not a riot\n"

/* BLE Advertisement-related macros */
#define BLE_ADV_TYPE_DEVINFO      0x01
#define BLE_ADV_TYPE_NAME         0x09
#define BLE_ADV_TYPE_MANUFACTURER 0xFF
#define BLE_ADV_NAME_BUF_LEN        32
#define BLE_ADV_PAYLOAD_BUF_LEN     64
#define BLE_UUID_SIZE               16

static rfc_bleAdvPar_t ble_params_buf __attribute__((__aligned__(4)));
uint16_t ble_mac_address[3] __attribute__((__aligned__(4))) = {0xeeff, 0xccdd, 0xaabb};
char adv_name[BLE_ADV_NAME_BUF_LEN] = {"riot-test"};

void isr_rfc_cmd_ack(void)
{
    /*ROP ack = op submitted, DIR or IMM ack = op executed*/
    printf("Command acknowledged. CMDSTA: 0x%" PRIx32 " \n", RFC_DBELL->CMDSTA);
    RFC_DBELL->RFACKIFG = 0;
}

void isr_rfc_hw(void)
{
    uint32_t flags = RFC_DBELL->RFHWIFG;
    printf("hw 0x%" PRIx32 "\n", flags);
    RFC_DBELL->RFHWIFG = ~flags;
}

void isr_rfc_cpe0(void)
{
    uint32_t flags = RFC_DBELL->RFCPEIFG & (~RFC_DBELL->RFCPEISL);
    printf("cpe0 0x%" PRIx32 "\n", flags);
    RFC_DBELL->RFCPEIFG = ~flags;
}

void isr_rfc_cpe1(void)
{
    uint32_t flags = RFC_DBELL->RFCPEIFG & RFC_DBELL->RFCPEISL;
    printf("cpe1 0x%" PRIx32 "\n", flags);
    RFC_DBELL->RFCPEIFG = ~flags;
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
    RFC_DBELL->CMDR = (uint32_t) ropCmd;

    /* wait for cmd ack (rop cmd was submitted successfully) */
    while (RFC_DBELL->RFACKIFG << 31);
    while (!RFC_DBELL->CMDSTA);

    return RFC_DBELL->CMDSTA;
}

uint16_t rfc_wait_cmd_done(radio_op_command_t *command)
{
    uint32_t timeout_cnt = 0;
    /* wait for cmd execution. condition on rop status doesn't work by itself (too fast?). */
    do {
        if (++timeout_cnt > 500000) {
            command->status = R_OP_STATUS_DONE_TIMEOUT;
            break;
        }
    } while (command->status < R_OP_STATUS_SKIPPED);

    return command->status;
}

bool rfc_setup_ble(void)
{
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

    rfc_send_cmd(&rs);

    uint16_t status = rfc_wait_cmd_done(&rs.ropCmd);
    return status == R_OP_STATUS_DONE_OK;
}

int send_ble_adv_nc(int channel, uint8_t *adv_payload, int adv_payload_len)
{
    rfc_CMD_BLE_ADV_NC_t cmd;
    rfc_bleAdvPar_t *params;

    params = (rfc_bleAdvPar_t *)&ble_params_buf;

    /* Clear both buffers */
    memset(&cmd, 0x00, sizeof(cmd));
    memset(params, 0x00, sizeof(*params));

    /* Adv NC */
    cmd.commandNo = 0x1805;
    cmd.condition.rule = R_OP_CONDITION_RULE_NEVER;
    cmd.whitening.bOverride = 0;
    cmd.whitening.init = 0;
    cmd.pParams = params;
    cmd.channel = channel;

    /* Set up BLE Advertisement parameters */
    params->pDeviceAddress = ble_mac_address;
    params->endTrigger.triggerType = R_OP_STARTTRIG_TYPE_TRIG_NEVER;
    params->endTime = R_OP_STARTTRIG_TYPE_TRIG_NEVER;

    params->advLen = adv_payload_len;
    params->pAdvData = adv_payload;

    uint32_t status = rfc_send_cmd((uint32_t*)&cmd);

    if (status != 1) {
        printf("bad CMDSTA: 0x%lx", status);
        while(1);
    }

    radio_op_command_t *rop_cmd = (radio_op_command_t *)&cmd;

    status = rfc_wait_cmd_done(rop_cmd);
    if (status != 0x1400) {
        printf("bad CMDSTA: 0x%lx", status);
        while(1);
    }

    return 0;
}

void rfc_ble_beacon(void)
{
    uint16_t p = 0;
    static uint8_t payload[BLE_ADV_PAYLOAD_BUF_LEN] __attribute__((__aligned__(4)));

    /* device info */
    memset(payload, 0, BLE_ADV_PAYLOAD_BUF_LEN);
    payload[p++] = 0x02;          /* 2 bytes */
    payload[p++] = BLE_ADV_TYPE_DEVINFO;
    payload[p++] = 0x1a;          /* LE general discoverable + BR/EDR */
    payload[p++] = 1 + strlen(adv_name);
    payload[p++] = BLE_ADV_TYPE_NAME;
    memcpy(&payload[p], adv_name,
           strlen(adv_name));
    p += strlen(adv_name);

    send_ble_adv_nc(37, payload, p);
    send_ble_adv_nc(38, payload, p);
    send_ble_adv_nc(39, payload, p);
}

bool rfc_ping_test(void)
{
    direct_command_t pingCommand;
    pingCommand.commandID = CMDR_CMDID_PING;
    RFC_DBELL->CMDR = (uint32_t) (&pingCommand);
    while (!RFC_DBELL->CMDSTA); /* wait for cmd execution */
    printf("rfc_ping fails without a printf reading cmdsta %lu\n", RFC_DBELL->CMDSTA);
    return RFC_DBELL->CMDSTA == CMDSTA_RESULT_DONE;
}

bool rfc_nop_test(void)
{
    nop_cmd_t nopCommand;
    memset(&nopCommand, 0, sizeof(nopCommand));
    nopCommand.ropCmd.commandNo = CMDR_CMDID_NOP;
    nopCommand.ropCmd.status = R_OP_STATUS_IDLE;
    nopCommand.ropCmd.condition.rule = 1; /* never run next cmd. need to implement definition */
    rfc_send_cmd(&nopCommand);

    uint16_t status = rfc_wait_cmd_done(&nopCommand.ropCmd);
    return status == R_OP_STATUS_DONE_OK;
}

static bool rfc_start_rat(void)
{
    direct_command_t ratCommand;
    ratCommand.commandID = CMDR_CMDID_START_RAT;
    RFC_DBELL->CMDR = (uint32_t) (&ratCommand);
    while (!RFC_DBELL->CMDSTA); /* wait for cmd ack */
    printf("START_RAT finished with 0x%lx\n", RFC_DBELL->CMDSTA); //FIXME future beacon send fails without this
    return RFC_DBELL->CMDSTA == CMDSTA_RESULT_DONE;
}

void rfc_prepare(void)
{
    /* rfc mode must be set before powering up radio (undocumented) */
    uint32_t *rfc_mode_hwopt = (uint32_t*)0x400821D4;
    printf("modeopt: 0x%lx\n", ((*rfc_mode_hwopt) >> 1) & 0x7);
    PRCM->RFCMODESEL = 0x5;

    /* RFC POWER DOMAIN CLOCK GATE */
    PRCM->RFCCLKG = 1;

    PRCM->CLKLOADCTL = CLKLOADCTL_LOAD;
    while (!(PRCM->CLKLOADCTL & CLKLOADCTL_LOADDONE)) ;

    /* RFC POWER DOMAIN */
    PRCM->PDCTL0 |= PDCTL0_RFC_ON;
    PRCM->PDCTL1 |= PDCTL1_RFC_ON;
    while (!(PRCM->PDSTAT0 & PDSTAT0_RFC_ON)) ;
    while (!(PRCM->PDSTAT1 & PDSTAT1_RFC_ON)) ;

    /* RFC CLOCK */
    //RFC_PWR->PWMCLKEN |= RFC_PWR_PWMCLKEN_CPE;
    //RFC_PWR->PWMCLKEN |= RFC_PWR_PWMCLKEN_CPERAM;
    RFC_PWR->PWMCLKEN |= 0x7FF;
    printf("RFC_PWR->PWMCLKEN %lx\n", RFC_PWR->PWMCLKEN);

    /* RFC IRQ */
    rfc_irq_enable();

    /*RFC TIMER */
    rfc_start_rat();
}

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
#include "mutex.h"
#include "lpm.h"
#include "thread.h"

#define BLE_ADV_STR "this is not a riot\n"

/* BLE Advertisement-related macros */
#define BLE_ADV_TYPE_DEVINFO      0x01
#define BLE_ADV_TYPE_NAME         0x09
#define BLE_ADV_TYPE_MANUFACTURER 0xFF
#define BLE_ADV_NAME_BUF_LEN        32
#define BLE_ADV_PAYLOAD_BUF_LEN     64
#define BLE_UUID_SIZE               16

void cpu_clock_init(void);

static rfc_bleAdvPar_t ble_params_buf __attribute__((__aligned__(4)));
uint16_t ble_mac_address[3] __attribute__((__aligned__(4))) = {0xeeff, 0xccdd, 0xaabb};
char adv_name[BLE_ADV_NAME_BUF_LEN] = {"riot-test"};

mutex_t cmd_wait = MUTEX_INIT;
mutex_t done_wait = MUTEX_INIT;

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
//     printf("hw 0x%" PRIx32 "\n", flags);
    RFC_DBELL->RFHWIFG = ~flags;

    if (sched_context_switch_request) {
        thread_yield();
    }
}

void isr_rfc_cpe0(void)
{
    uint32_t flags = RFC_DBELL->RFCPEIFG & (~RFC_DBELL->RFCPEISL);
    RFC_DBELL->RFCPEIFG = ~flags;
//     printf("cpe0 0x%" PRIx32 "\n", flags);

    if (flags & 0x2) {
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
//     printf("cpe1 0x%" PRIx32 "\n", flags);
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

uint16_t rfc_wait_cmd_done(radio_op_command_t *command)
{
//     printf("rfc_wait_cmd_done()\n");

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

//     printf("fin\n");

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

int send_ble_adv_nc(uint8_t iterations, uint8_t *adv_payload, int adv_payload_len)
{
//     printf("send_ble_adv_nc()\n");

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
            printf("bad CMDSTA: 0x%lx", cmd_status);
            while(1);
        }

        if (status != 0x1400) {
            printf("bad status: 0x%x", status);
            while(1);
        }
    }
    return 0;
}

void rfc_ble_beacon(void)
{
//     printf("rfc_ble_beacon()\n");

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

static bool rfc_start_rat(void)
{
    direct_command_t ratCommand;
    ratCommand.commandID = CMDR_CMDID_START_RAT;
    uint32_t status = rfc_send_cmd(&ratCommand);
    if (status != 1) {
        printf("bad CMDSTA: 0x%lx", status);
        while(1);
    }
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

/* ROM HAPI HFSourceSafeSwitch function */
#define ROM_HAPI_HFSOURCESAFESWITCH_ADDR_P (0x10000048 + (14*4))
#define ROM_HAPI_HFSOURCESAFESWITCH_ADDR (*(reg32_t*)ROM_HAPI_HFSOURCESAFESWITCH_ADDR_P)
#define ROM_HAPI_HFSOURCESAFESWITCH() (((void(*)(void))ROM_HAPI_HFSOURCESAFESWITCH_ADDR)())

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

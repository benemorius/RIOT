/*
 * Copyright (c) 2015, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup rf-core
 * @{
 *
 * \file
 * Implementation of the CC13xx/CC26xx RF core driver
 */
/*---------------------------------------------------------------------------*/
#include "rf-core.h"
#include "driverlib/prcm.h"
#include "driverlib/chipinfo.h"

#include "thread.h"
/*---------------------------------------------------------------------------*/
/* RF core and RF HAL API */
#include "hw_rfc_dbell.h"
#include "hw_rfc_pwr.h"
/*---------------------------------------------------------------------------*/
/* RF Core Mailbox API */
#include "api/mailbox.h"
#include "api/common_cmd.h"
#include "api/ble_cmd.h"
#include "api/ieee_cmd.h"
#include "api/data_entry.h"
#include "api/ble_mailbox.h"
#include "api/ieee_mailbox.h"
#include "api/prop_mailbox.h"
#include "api/prop_cmd.h"
/*---------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
#ifdef RF_CORE_CONF_DEBUG_CRC
#define RF_CORE_DEBUG_CRC RF_CORE_CONF_DEBUG_CRC
#else
#define RF_CORE_DEBUG_CRC DEBUG
#endif
/*---------------------------------------------------------------------------*/
/* RF interrupts */
#define RX_FRAME_IRQ IRQ_RX_ENTRY_DONE
#define ERROR_IRQ    IRQ_INTERNAL_ERROR
#define RX_NOK_IRQ   IRQ_RX_NOK

/* Those IRQs are enabled all the time */
#if RF_CORE_DEBUG_CRC
#define ENABLED_IRQS (RX_FRAME_IRQ | ERROR_IRQ | RX_NOK_IRQ)
#else
#define ENABLED_IRQS (RX_FRAME_IRQ | ERROR_IRQ)
#endif

#define cc26xx_rf_cpe0_isr RFCCPE0IntHandler
#define cc26xx_rf_cpe1_isr RFCCPE1IntHandler
/*---------------------------------------------------------------------------*/
/* Remember the last Radio Op issued to the radio */
static rfc_radioOp_t *last_radio_op = NULL;
/*---------------------------------------------------------------------------*/
/* A struct holding pointers to the primary mode's abort() and restore() */
static const rf_core_primary_mode_t *primary_mode = NULL;
/*---------------------------------------------------------------------------*/
#define RF_CORE_CLOCKS_MASK (RFC_PWR_PWMCLKEN_RFC_M | RFC_PWR_PWMCLKEN_CPE_M \
                             | RFC_PWR_PWMCLKEN_CPERAM_M)
/*---------------------------------------------------------------------------*/
uint8_t
rf_core_is_accessible(void)
{
  if(PRCMRfReady()) {
    return RF_CORE_ACCESSIBLE;
  }
  return RF_CORE_NOT_ACCESSIBLE;
}
/*---------------------------------------------------------------------------*/
uint_fast8_t
rf_core_send_cmd(uint32_t cmd, uint32_t *status)
{
  uint32_t timeout_count = 0;
  bool interrupts_disabled;
  bool is_radio_op = false;

  /*
   * If cmd is 4-byte aligned, then it's either a radio OP or an immediate
   * command. Clear the status field if it's a radio OP
   */
  if((cmd & 0x03) == 0) {
    uint32_t cmd_type;
    cmd_type = ((rfc_command_t *)cmd)->commandNo & RF_CORE_COMMAND_TYPE_MASK;
    if(cmd_type == RF_CORE_COMMAND_TYPE_IEEE_FG_RADIO_OP ||
       cmd_type == RF_CORE_COMMAND_TYPE_RADIO_OP) {
      is_radio_op = true;
      ((rfc_radioOp_t *)cmd)->status = RF_CORE_RADIO_OP_STATUS_IDLE;
    }
  }

  /*
   * Make sure ContikiMAC doesn't turn us off from within an interrupt while
   * we are accessing RF Core registers
   */
  interrupts_disabled = IntMasterDisable();

  if(!rf_core_is_accessible()) {
    PRINTF("rf_core_send_cmd: RF was off\n");
    if(!interrupts_disabled) {
      IntMasterEnable();
    }
    return RF_CORE_CMD_ERROR;
  }

  if(is_radio_op) {
    uint16_t command_no = ((rfc_radioOp_t *)cmd)->commandNo;
    if((command_no & RF_CORE_COMMAND_PROTOCOL_MASK) != RF_CORE_COMMAND_PROTOCOL_COMMON &&
       (command_no & RF_CORE_COMMAND_TYPE_MASK) == RF_CORE_COMMAND_TYPE_RADIO_OP) {
      last_radio_op = (rfc_radioOp_t *)cmd;
    }
  }

  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = cmd;
  do {
    *status = HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA);
    if(++timeout_count > 50000) {
      PRINTF("rf_core_send_cmd: 0x%08lx Timeout\n", cmd);
      if(!interrupts_disabled) {
        IntMasterEnable();
      }
      return RF_CORE_CMD_ERROR;
    }
  } while((*status & RF_CORE_CMDSTA_RESULT_MASK) == RF_CORE_CMDSTA_PENDING);

  if(!interrupts_disabled) {
    IntMasterEnable();
  }

  /*
   * If we reach here the command is no longer pending. It is either completed
   * successfully or with error
   */
  return (*status & RF_CORE_CMDSTA_RESULT_MASK) == RF_CORE_CMDSTA_DONE;
}
/*---------------------------------------------------------------------------*/
uint_fast8_t
rf_core_wait_cmd_done(void *cmd)
{
  volatile rfc_radioOp_t *command = (rfc_radioOp_t *)cmd;
  uint32_t timeout_cnt = 0;

  /*
   * 0xn4nn=DONE, 0x0400=DONE_OK while all other "DONE" values means done
   * but with some kind of error (ref. "Common radio operation status codes")
   */
  do {
    if(++timeout_cnt > 500000) {
      return RF_CORE_CMD_ERROR;
    }
  } while((command->status & RF_CORE_RADIO_OP_MASKED_STATUS)
          != RF_CORE_RADIO_OP_MASKED_STATUS_DONE);

  return (command->status & RF_CORE_RADIO_OP_MASKED_STATUS)
         == RF_CORE_RADIO_OP_STATUS_DONE_OK;
}
/*---------------------------------------------------------------------------*/
static int
fs_powerdown(void)
{
  rfc_CMD_FS_POWERDOWN_t cmd;
  uint32_t cmd_status;

  rf_core_init_radio_op((rfc_radioOp_t *)&cmd, sizeof(cmd), CMD_FS_POWERDOWN);

  if(rf_core_send_cmd((uint32_t)&cmd, &cmd_status) != RF_CORE_CMD_OK) {
    PRINTF("fs_powerdown: CMDSTA=0x%08lx\n", cmd_status);
    return RF_CORE_CMD_ERROR;
  }

  if(rf_core_wait_cmd_done(&cmd) != RF_CORE_CMD_OK) {
    PRINTF("fs_powerdown: CMDSTA=0x%08lx, status=0x%04x\n",
           cmd_status, cmd.status);
    return RF_CORE_CMD_ERROR;
  }

  return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
int
rf_core_power_up(void)
{
  uint32_t cmd_status;
  bool interrupts_disabled = IntMasterDisable();

  IntPendClear(INT_RF_CPE0);
  IntPendClear(INT_RF_CPE1);
  IntDisable(INT_RF_CPE0);
  IntDisable(INT_RF_CPE1);

  /* Enable RF Core power domain */
  PRCMPowerDomainOn(PRCM_DOMAIN_RFCORE);
  while(PRCMPowerDomainStatus(PRCM_DOMAIN_RFCORE)
        != PRCM_DOMAIN_POWER_ON);

  PRCMDomainEnable(PRCM_DOMAIN_RFCORE);
  PRCMLoadSet();
  while(!PRCMLoadGet());

  HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIFG) = 0x0;
  HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIEN) = 0x0;
  IntEnable(INT_RF_CPE0);
  IntEnable(INT_RF_CPE1);

  if(!interrupts_disabled) {
    IntMasterEnable();
  }

  /* Let CPE boot */
  HWREG(RFC_PWR_NONBUF_BASE + RFC_PWR_O_PWMCLKEN) = RF_CORE_CLOCKS_MASK;

  /* Send ping (to verify RFCore is ready and alive) */
  if(rf_core_send_cmd(CMDR_DIR_CMD(CMD_PING), &cmd_status) != RF_CORE_CMD_OK) {
    PRINTF("rf_core_power_up: CMD_PING fail, CMDSTA=0x%08lx\n", cmd_status);
    return RF_CORE_CMD_ERROR;
  }

  return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
void
rf_core_power_down(void)
{
  bool interrupts_disabled = IntMasterDisable();
  IntDisable(INT_RF_CPE0);
  IntDisable(INT_RF_CPE1);

  if(rf_core_is_accessible()) {
    HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIFG) = 0x0;
    HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIEN) = 0x0;

    /* need to send FS_POWERDOWN or analog components will use power */
    fs_powerdown();
  }

  /* Shut down the RFCORE clock domain in the MCU VD */
  PRCMDomainDisable(PRCM_DOMAIN_RFCORE);
  PRCMLoadSet();
  while(!PRCMLoadGet());

  /* Turn off RFCORE PD */
  PRCMPowerDomainOff(PRCM_DOMAIN_RFCORE);
  while(PRCMPowerDomainStatus(PRCM_DOMAIN_RFCORE)
        != PRCM_DOMAIN_POWER_OFF);

  IntPendClear(INT_RF_CPE0);
  IntPendClear(INT_RF_CPE1);
  IntEnable(INT_RF_CPE0);
  IntEnable(INT_RF_CPE1);
  if(!interrupts_disabled) {
    IntMasterEnable();
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
rf_core_set_modesel(void)
{
  uint8_t rv = RF_CORE_CMD_ERROR;

  if(ChipInfo_ChipFamilyIsCC26xx()) {
    if(ChipInfo_SupportsBLE() == true &&
       ChipInfo_SupportsIEEE_802_15_4() == true) {
      /* CC2650 */
      HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) = PRCM_RFCMODESEL_CURR_MODE5;
      rv = RF_CORE_CMD_OK;
    } else if(ChipInfo_SupportsBLE() == false &&
              ChipInfo_SupportsIEEE_802_15_4() == true) {
      /* CC2630 */
      HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) = PRCM_RFCMODESEL_CURR_MODE2;
      rv = RF_CORE_CMD_OK;
    }
  } else if(ChipInfo_ChipFamilyIsCC13xx()) {
    if(ChipInfo_SupportsBLE() == false &&
       ChipInfo_SupportsIEEE_802_15_4() == false) {
      /* CC1310 */
      HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) = PRCM_RFCMODESEL_CURR_MODE3;
      rv = RF_CORE_CMD_OK;
    }
  }

  return rv;
}
/*---------------------------------------------------------------------------*/
uint8_t
rf_core_start_rat(void)
{
  uint32_t cmd_status;

  /* Start radio timer (RAT) */
  if(rf_core_send_cmd(CMDR_DIR_CMD(CMD_START_RAT), &cmd_status)
     != RF_CORE_CMD_OK) {
    PRINTF("rf_core_apply_patches: START_RAT fail, CMDSTA=0x%08lx\n",
           cmd_status);
    return RF_CORE_CMD_ERROR;
  }

  return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
uint8_t
rf_core_boot(void)
{
  if(rf_core_power_up() != RF_CORE_CMD_OK) {
    PRINTF("rf_core_boot: rf_core_power_up() failed\n");

    rf_core_power_down();

    return RF_CORE_CMD_ERROR;
  }

  if(rf_core_start_rat() != RF_CORE_CMD_OK) {
    PRINTF("rf_core_boot: rf_core_start_rat() failed\n");

    rf_core_power_down();

    return RF_CORE_CMD_ERROR;
  }

  return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
void
rf_core_setup_interrupts(void)
{
  bool interrupts_disabled;

  /* We are already turned on by the caller, so this should not happen */
  if(!rf_core_is_accessible()) {
    PRINTF("setup_interrupts: No access\n");
    return;
  }

  /* Disable interrupts */
  interrupts_disabled = IntMasterDisable();

  /* Set all interrupt channels to CPE0 channel, error to CPE1 */
  HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEISL) = ERROR_IRQ;

  /* Acknowledge configured interrupts */
  HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIEN) = ENABLED_IRQS;

  /* Clear interrupt flags, active low clear(?) */
  HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIFG) = 0x0;

  IntPendClear(INT_RF_CPE0);
  IntPendClear(INT_RF_CPE1);
  IntEnable(INT_RF_CPE0);
  IntEnable(INT_RF_CPE1);

  if(!interrupts_disabled) {
    IntMasterEnable();
  }
}
/*---------------------------------------------------------------------------*/
void
rf_core_cmd_done_en(bool fg)
{
  uint32_t irq = fg ? IRQ_LAST_FG_COMMAND_DONE : IRQ_LAST_COMMAND_DONE;

  HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIFG) = ENABLED_IRQS;
  HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIEN) = ENABLED_IRQS | irq;
}
/*---------------------------------------------------------------------------*/
void
rf_core_cmd_done_dis(void)
{
  HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIEN) = ENABLED_IRQS;
}
/*---------------------------------------------------------------------------*/
rfc_radioOp_t *
rf_core_get_last_radio_op(void)
{
  return last_radio_op;
}
/*---------------------------------------------------------------------------*/
void
rf_core_init_radio_op(rfc_radioOp_t *op, uint16_t len, uint16_t command)
{
  memset(op, 0, len);

  op->commandNo = command;
  op->condition.rule = COND_NEVER;
}
/*---------------------------------------------------------------------------*/
void
rf_core_primary_mode_register(const rf_core_primary_mode_t *mode)
{
  primary_mode = mode;
}
/*---------------------------------------------------------------------------*/
void
rf_core_primary_mode_abort(void)
{
  if(primary_mode) {
    if(primary_mode->abort) {
      primary_mode->abort();
    }
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
rf_core_primary_mode_restore(void)
{
  if(primary_mode) {
    if(primary_mode->restore) {
      return primary_mode->restore();
    }
  }

  return RF_CORE_CMD_ERROR;
}
/*---------------------------------------------------------------------------*/
void* rf_core_thread(void* arg)
{
//   int len;
//
//   while(1) {
//     thread_sleep();
//     do {
//       packetbuf_clear();
//       len = NETSTACK_RADIO.read(packetbuf_dataptr(), PACKETBUF_SIZE);
//
//       if(len > 0) {
//         packetbuf_set_datalen(len);
//
//         NETSTACK_RDC.input();
//       }
//     } while(len > 0);
//   }
    return NULL;
}
/*---------------------------------------------------------------------------*/
static void
rx_nok_isr(void)
{
  PRINTF("RF: Bad CRC\n");
}
/*---------------------------------------------------------------------------*/
void
cc26xx_rf_cpe1_isr(void)
{
  PRINTF("RF Error\n");

  if(!rf_core_is_accessible()) {
    if(rf_core_power_up() != RF_CORE_CMD_OK) {
      return;
    }
  }

  /* Clear INTERNAL_ERROR interrupt flag */
  HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIFG) = 0x7FFFFFFF;
}
/*---------------------------------------------------------------------------*/
void
cc26xx_rf_cpe0_isr(void)
{
  if(!rf_core_is_accessible()) {
    printf("RF ISR called but RF not ready... PANIC!!\n");
    if(rf_core_power_up() != RF_CORE_CMD_OK) {
      PRINTF("rf_core_power_up() failed\n");
      return;
    }
  }

  IntMasterDisable();

  if(HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIFG) & RX_FRAME_IRQ) {
    /* Clear the RX_ENTRY_DONE interrupt flag */
    HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIFG) = 0xFF7FFFFF;
    thread_wakeup(rf_core_pid);
  }

  if(RF_CORE_DEBUG_CRC) {
    if(HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIFG) & RX_NOK_IRQ) {
      /* Clear the RX_NOK interrupt flag */
      HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIFG) = 0xFFFDFFFF;
      rx_nok_isr();
    }
  }

  if(HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIFG) &
     (IRQ_LAST_FG_COMMAND_DONE | IRQ_LAST_COMMAND_DONE)) {
    /* Clear the two TX-related interrupt flags */
    HWREG(RFC_DBELL_NONBUF_BASE + RFC_DBELL_O_RFCPEIFG) = 0xFFFFFFF5;
  }

  IntMasterEnable();
}
/*---------------------------------------------------------------------------*/
/** @} */

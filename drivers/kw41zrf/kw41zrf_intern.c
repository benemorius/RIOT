/*
 * Copyright (C) 2017 SKF AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_kw41zrf
 * @{
 * @file
 * @brief       Internal function of kw41zrf driver
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @}
 */

#include "irq.h"
#include "panic.h"
#include "kw41zrf.h"
#include "kw41zrf_getset.h"
#include "kw41zrf_intern.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

struct {
    void (*cb)(void *arg); /**< Callback function called from radio ISR */
    void *arg;             /**< Argument to callback */
} isr_config;

void kw41zrf_set_irq_callback(void (*cb)(void *arg), void *arg)
{
    unsigned int mask = irq_disable();
    isr_config.cb = cb;
    isr_config.arg = arg;
    irq_restore(mask);
}

void kw41zrf_disable_interrupts(kw41zrf_t *dev)
{
    DEBUG("[kw41zrf] disable interrupts\n");
    /* Clear and disable all interrupts */
    ZLL->PHY_CTRL |=
        ZLL_PHY_CTRL_TSM_MSK_MASK |
        ZLL_PHY_CTRL_WAKE_MSK_MASK |
        ZLL_PHY_CTRL_CRC_MSK_MASK |
        ZLL_PHY_CTRL_PLL_UNLOCK_MSK_MASK |
        ZLL_PHY_CTRL_FILTERFAIL_MSK_MASK |
        ZLL_PHY_CTRL_RX_WMRK_MSK_MASK |
        ZLL_PHY_CTRL_CCAMSK_MASK |
        ZLL_PHY_CTRL_RXMSK_MASK |
        ZLL_PHY_CTRL_TXMSK_MASK |
        ZLL_PHY_CTRL_SEQMSK_MASK;

    /* Mask all timer interrupts and clear all interrupt flags */
    ZLL->IRQSTS =
        ZLL_IRQSTS_TMR1MSK_MASK |
        ZLL_IRQSTS_TMR2MSK_MASK |
        ZLL_IRQSTS_TMR3MSK_MASK |
        ZLL_IRQSTS_TMR4MSK_MASK |
        ZLL_IRQSTS_TMR1IRQ_MASK |
        ZLL_IRQSTS_TMR2IRQ_MASK |
        ZLL_IRQSTS_TMR3IRQ_MASK |
        ZLL_IRQSTS_TMR4IRQ_MASK |
        ZLL_IRQSTS_WAKE_IRQ_MASK |
        ZLL_IRQSTS_PLL_UNLOCK_IRQ_MASK |
        ZLL_IRQSTS_FILTERFAIL_IRQ_MASK |
        ZLL_IRQSTS_RXWTRMRKIRQ_MASK |
        ZLL_IRQSTS_CCAIRQ_MASK |
        ZLL_IRQSTS_RXIRQ_MASK |
        ZLL_IRQSTS_TXIRQ_MASK |
        ZLL_IRQSTS_SEQIRQ_MASK;
}

void kw41zrf_set_power_mode(kw41zrf_t *dev, kw41zrf_powermode_t pm)
{
    DEBUG("[kw41zrf] set power mode to %u\n", pm);
    /* TODO handle event timer */
    switch (pm) {
        case KW41ZRF_POWER_IDLE:
            bit_clear32(&ZLL->DSM_CTRL, ZLL_DSM_CTRL_ZIGBEE_SLEEP_EN_SHIFT);
            break;
        case KW41ZRF_POWER_DSM:
            bit_set32(&ZLL->DSM_CTRL, ZLL_DSM_CTRL_ZIGBEE_SLEEP_EN_SHIFT);
            break;
        default:
            DEBUG("[kw41zrf] Unknown power mode %u\n", pm);
            return;
    }
}

void kw41zrf_set_sequence(kw41zrf_t *dev, uint32_t seq)
{
    DEBUG("[kw41zrf] set sequence to %u\n", (unsigned int)seq);
    /* Clear interrupt flags, sometimes the sequence complete flag is immediately set */
    ZLL->IRQSTS = ZLL->IRQSTS;
    ZLL->PHY_CTRL = (ZLL->PHY_CTRL & ~(ZLL_PHY_CTRL_XCVSEQ_MASK | ZLL_PHY_CTRL_SEQMSK_MASK)) | seq;
    while (((ZLL->SEQ_CTRL_STS & ZLL_SEQ_CTRL_STS_XCVSEQ_ACTUAL_MASK) >>
        ZLL_SEQ_CTRL_STS_XCVSEQ_ACTUAL_SHIFT) != (ZLL_PHY_CTRL_XCVSEQ_MASK & seq)) {}
}

void kw41zrf_set_idle_sequence(kw41zrf_t *dev, uint32_t seq)
{
    dev->idle_seq = seq;

    if (kw41zrf_can_switch_to_idle(dev)) {
        kw41zrf_abort_sequence(dev);
        kw41zrf_set_sequence(dev, seq);
    }
}

int kw41zrf_can_switch_to_idle(kw41zrf_t *dev)
{
    uint8_t seq = (ZLL->PHY_CTRL & ZLL_PHY_CTRL_XCVSEQ_MASK) >> ZLL_PHY_CTRL_XCVSEQ_SHIFT;

    DEBUG("[kw41zrf] XCVSEQ=0x%x, SEQ_STATE=0x%" PRIx32 ", SEQ_CTRL_STS=0x%" PRIx32 "\n", seq,
        ZLL->SEQ_STATE, ZLL->SEQ_CTRL_STS);

    switch (seq)
    {
        case XCVSEQ_TRANSMIT:
        case XCVSEQ_TX_RX:
        case XCVSEQ_CCA:
            /* We should wait until TX or CCA has finished before moving to
             * another mode */
            return 0;

        case XCVSEQ_RECEIVE:
            {
                uint32_t seq_state = ZLL->SEQ_STATE;
                if (seq_state & ZLL_SEQ_STATE_SFD_DET_MASK) {
                    if (seq_state & ZLL_SEQ_STATE_RX_BYTE_COUNT_MASK) {
                        /* A packet is currently being received, we won't be able to
                        * transmit until the channel is free anyway, so we might as well just
                        * wait until the RX has finished */
                        DEBUG("[kw41zrf] RX in p\n");
                        return 0;
                    }
                }
            }
        default:
            break;
    }

    return 1;
}

void isr_radio_int1(void)
{
    DEBUG("[kw41zrf] INT1\n");
    if (isr_config.cb != NULL) {
        isr_config.cb(isr_config.arg);
    }
    cortexm_isr_end();
}

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

int kw41zrf_can_switch_to_idle(kw41zrf_t *dev)
{
    uint8_t seq = (ZLL->PHY_CTRL & ZLL_PHY_CTRL_XCVSEQ_MASK) >> ZLL_PHY_CTRL_XCVSEQ_SHIFT;
    uint8_t actual = (ZLL->SEQ_CTRL_STS & ZLL_SEQ_CTRL_STS_XCVSEQ_ACTUAL_MASK) >> ZLL_SEQ_CTRL_STS_XCVSEQ_ACTUAL_SHIFT;

    DEBUG("[kw41zrf] XCVSEQ_ACTUAL=0x%x, XCVSEQ=0x%x, SEQ_STATE=0x%" PRIx32 "\n", actual, seq,
        (ZLL->SEQ_STATE & ZLL_SEQ_STATE_SEQ_STATE_MASK) >> ZLL_SEQ_STATE_SEQ_STATE_SHIFT);

    switch (seq)
    {
        case XCVSEQ_TRANSMIT:
        case XCVSEQ_TX_RX:
            return 0;

        default:
            break;
    }
    switch (actual)
    {
        case XCVSEQ_TRANSMIT:
        case XCVSEQ_TX_RX:
            return 0;

        default:
            break;
    }

    return 1;
}

/** Load the timer value (Setting Current Time) */
static inline void kw41zrf_timer_load(kw41zrf_t *dev, uint32_t value)
{
    (void) dev;
    ZLL->EVENT_TMR = ZLL_EVENT_TMR_EVENT_TMR(value) | ZLL_EVENT_TMR_EVENT_TMR_LD_MASK;
}

static inline uint32_t kw41zrf_timer_get(kw41zrf_t *dev)
{
    (void) dev;
    return (ZLL->EVENT_TMR & ZLL_EVENT_TMR_EVENT_TMR_MASK) >> ZLL_EVENT_TMR_EVENT_TMR_SHIFT;
}

/** Set a timeout value for the given compare register of the Event Timer */
static inline void kw41zrf_timer_set(kw41zrf_t *dev, volatile uint32_t *cmp_reg, uint32_t timeout)
{
    uint32_t now = kw41zrf_timer_get(dev);

    DEBUG("[kw41zrf] timer now: %" PRIx32 ", set %" PRIx32 "\n", now, now + timeout);
    *cmp_reg = now + timeout;
}

void kw41zrf_timer_init(kw41zrf_t *dev, kw41zrf_timer_timebase_t tb)
{
    ZLL->TMR_PRESCALE = (ZLL->TMR_PRESCALE & ~ZLL_TMR_PRESCALE_TMR_PRESCALE_MASK) |
        ZLL_TMR_PRESCALE_TMR_PRESCALE(tb);
    kw41zrf_timer_load(dev, 0);
}

void kw41zrf_timer2_seq_start_on(kw41zrf_t *dev)
{
    bit_set32(&ZLL->PHY_CTRL, ZLL_PHY_CTRL_TMRTRIGEN_SHIFT);
}

void kw41zrf_timer2_seq_start_off(kw41zrf_t *dev)
{
    bit_clear32(&ZLL->PHY_CTRL, ZLL_PHY_CTRL_TMRTRIGEN_SHIFT);
}

void kw41zrf_timer3_seq_abort_on(kw41zrf_t *dev)
{
    bit_set32(&ZLL->PHY_CTRL, ZLL_PHY_CTRL_TC3TMOUT_SHIFT);
}

void kw41zrf_timer3_seq_abort_off(kw41zrf_t *dev)
{
    bit_clear32(&ZLL->PHY_CTRL, ZLL_PHY_CTRL_TC3TMOUT_SHIFT);
}

void kw41zrf_trigger_tx_ops_enable(kw41zrf_t *dev, uint32_t timeout)
{
    kw41zrf_timer_set(dev, &ZLL->T2CMP, timeout);
    bit_set32(&ZLL->PHY_CTRL, ZLL_PHY_CTRL_TMR2CMP_EN_SHIFT);
}

void kw41zrf_trigger_tx_ops_disable(kw41zrf_t *dev)
{
    bit_clear32(&ZLL->PHY_CTRL, ZLL_PHY_CTRL_TMR2CMP_EN_SHIFT);
    DEBUG("[kw41zrf] trigger_tx_ops_disable, now: %" PRIx32 "\n", kw41zrf_timer_get(dev));
}

void kw41zrf_abort_rx_ops_enable(kw41zrf_t *dev, uint32_t timeout)
{
    kw41zrf_timer_set(dev, &ZLL->T3CMP, timeout);
    bit_set32(&ZLL->PHY_CTRL, ZLL_PHY_CTRL_TMR3CMP_EN_SHIFT);
}

void kw41zrf_abort_rx_ops_disable(kw41zrf_t *dev)
{
    bit_clear32(&ZLL->PHY_CTRL, ZLL_PHY_CTRL_TMR3CMP_EN_SHIFT);
    DEBUG("[kw41zrf] abort_rx_ops_disable, now: %" PRIx32 "\n", kw41zrf_timer_get(dev));
}

uint32_t kw41zrf_get_timestamp(kw41zrf_t *dev)
{
    return ZLL->TIMESTAMP;
}

void isr_radio_int1(void)
{
    DEBUG("[kw41zrf] INT1\n");
    if (isr_config.cb != NULL) {
        isr_config.cb(isr_config.arg);
    }
    cortexm_isr_end();
}

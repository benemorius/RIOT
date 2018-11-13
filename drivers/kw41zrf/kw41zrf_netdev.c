/*
 * Copyright (C) 2017 SKF AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_kw41zrf
 * @{
 *
 * @file
 * @brief       Netdev interface for kw41zrf drivers
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include "log.h"
#include "random.h"
#include "thread_flags.h"
#include "net/eui64.h"
#include "net/ieee802154.h"
#include "net/netdev.h"
#include "net/netdev/ieee802154.h"
#include "pm_layered.h"

#include "kw41zrf.h"
#include "kw41zrf_netdev.h"
#include "kw41zrf_intern.h"
#include "kw41zrf_getset.h"

#if MODULE_OD
#include "od.h"
#endif

#define ENABLE_DEBUG (0)
#include "debug.h"

#define _MAX_MHR_OVERHEAD           (25)

/* Timing */
#define KW41ZRF_CCA_TIME               8
#define KW41ZRF_SHR_PHY_TIME          12
#define KW41ZRF_PER_BYTE_TIME          2
#define KW41ZRF_ACK_WAIT_TIME         54
#define KW41ZRF_CSMA_UNIT_TIME        20

static void kw41zrf_netdev_isr(netdev_t *netdev);

static volatile unsigned int num_irqs_queued = 0;
static volatile unsigned int num_irqs_handled = 0;
static unsigned int spinning_for_irq = 0;

/* Set this to a flag bit that is not used by the MAC implementation */
#define KW41ZRF_THREAD_FLAG_ISR (1u << 8)

static void kw41zrf_irq_handler(void *arg)
{
    netdev_t *netdev = arg;
    kw41zrf_t *dev = (kw41zrf_t *)netdev;
    LED_IRQ_ON;
    kw41zrf_mask_irqs();
    /* Signal to the thread that an IRQ has arrived, if it is waiting */
    thread_flags_set(dev->thread, KW41ZRF_THREAD_FLAG_ISR);

    /* We use this counter to avoid filling the message queue with redundant ISR events */
    if (num_irqs_queued == num_irqs_handled) {
        ++num_irqs_queued;
        if (netdev->event_callback) {
            netdev->event_callback(netdev, NETDEV_EVENT_ISR);
        }
    }
}

static int kw41zrf_netdev_init(netdev_t *netdev)
{
    kw41zrf_t *dev = (kw41zrf_t *)netdev;
    dev->thread = (thread_t *)thread_get(thread_getpid());

    /* initialize hardware */
    if (kw41zrf_init(dev, kw41zrf_irq_handler)) {
        LOG_ERROR("[kw41zrf] unable to initialize device\n");
        return -1;
    }

#ifdef MODULE_NETSTATS_L2
    memset(&netdev->stats, 0, sizeof(netstats_t));
#endif

    return 0;
}

/**
 * @brief Generate a random number for using as a CSMA delay value
 */
static inline uint32_t kw41zrf_csma_random_delay(kw41zrf_t *dev)
{
    /* Use topmost csma_be bits of the random number */
    uint32_t rnd = random_uint32() >> (32 - dev->csma_be);
    return (rnd * KW41ZRF_CSMA_UNIT_TIME);
}

static inline size_t kw41zrf_tx_load(const void *buf, size_t len, size_t offset)
{
    /* Array bounds are checked in the kw41zrf_netdev_send loop */
    /* offset + 1 is used because buf[0] contains the frame length byte */
    memcpy(((uint8_t *)&ZLL->PKT_BUFFER_TX[0]) + offset + 1, buf, len);
    return offset + len;
}

static void kw41zrf_tx_exec(kw41zrf_t *dev)
{
    kw41zrf_abort_sequence(dev);
    uint16_t len_fcf = ZLL->PKT_BUFFER_TX[0];
    DEBUG("[kw41zrf] len_fcf=0x%04x\n", len_fcf);
    /* Check FCF field in the TX buffer to see if the ACK_REQ flag was set in
     * the packet that is queued for transmission */
    uint8_t fcf = (len_fcf >> 8) & 0xff;
    uint32_t backoff_delay = 0;
    if (dev->flags & KW41ZRF_OPT_CSMA) {
        /* Use CSMA/CA random delay in the interval [0, 2**dev->csma_be) */
        backoff_delay = kw41zrf_csma_random_delay(dev);
    }
    uint32_t tx_timeout = 0;
    if ((fcf & IEEE802154_FCF_ACK_REQ) &&
        (dev->netdev.flags & NETDEV_IEEE802154_ACK_REQ)) {
        uint8_t payload_len = len_fcf & 0xff;
        tx_timeout = backoff_delay + dev->tx_warmup_time +
            KW41ZRF_SHR_PHY_TIME + payload_len * KW41ZRF_PER_BYTE_TIME +
            KW41ZRF_ACK_WAIT_TIME;
    }
    if (backoff_delay > 0) {
        /* Avoid risk of setting a timer in the past */
        kw41zrf_timer_set(dev, &ZLL->T2CMP, ~0ul);
        bit_set32(&ZLL->PHY_CTRL, ZLL_PHY_CTRL_TMRTRIGEN_SHIFT);
    }
    else {
        bit_clear32(&ZLL->PHY_CTRL, ZLL_PHY_CTRL_TMRTRIGEN_SHIFT);
    }

    /* This is quite timing sensitive, as interrupts may lead to setting a timer
     * target which has already passed */
    unsigned mask = irq_disable();
    LED_TX_ON;
    if (tx_timeout > 0) {
        /* Set a long timeout to avoid timer races while setting up the TX sequence.
         * By setting the timeout to now - 1 we get 267 seconds to set up everything
         * before the timer rolls over */
        kw41zrf_timer_set(dev, &ZLL->T3CMP, ~0ul);
        DEBUG("[kw41zrf] Start TR\n");
        /* Initiate transmission, with timeout */
        kw41zrf_set_sequence(dev, XCVSEQ_TX_RX | ZLL_PHY_CTRL_TC3TMOUT_MASK);
    }
    else {
        DEBUG("[kw41zrf] Start T\n");
        /* Initiate transmission */
        kw41zrf_set_sequence(dev, XCVSEQ_TRANSMIT);
    }
    if (backoff_delay > 0) {
        /* Set real trigger time for CSMA */
        kw41zrf_timer_set(dev, &ZLL->T2CMP, backoff_delay);
    }
    if (tx_timeout > 0) {
        /* Set real timeout for RX ACK */
        kw41zrf_timer_set(dev, &ZLL->T3CMP, tx_timeout);
    }
    irq_restore(mask);
}

/**
 * @brief Block the current thread until the radio is idle
 *
 * Any ongoing TX or CCA sequence will have finished when this function returns.
 *
 * @param[in] dev       kw41zrf device descriptor
 */
static void kw41zrf_wait_idle(kw41zrf_t *dev)
{
    /* make sure any ongoing T or TR sequence is finished */
    if (kw41zrf_can_switch_to_idle(dev) == 0) {
        DEBUG("[kw41zrf] waiting for idle\n");
        num_irqs_handled = num_irqs_queued;
        spinning_for_irq = 1;
        pm_block(KINETIS_PM_LLS);
        while (1) {
            /* TX or CCA in progress */
            /* Block until we get an IRQ */
            thread_flags_wait_any(KW41ZRF_THREAD_FLAG_ISR);
            /* Handle the IRQ */
            kw41zrf_netdev_isr((netdev_t *)dev);
            /* kw41zrf_netdev_isr() will switch the transceiver back to idle after
             * handling the TX complete IRQ */
            if (kw41zrf_can_switch_to_idle(dev)) {
                break;
            }
            DEBUG("[kw41zrf] waited ISR\n");
        }
        pm_unblock(KINETIS_PM_LLS);
        spinning_for_irq = 0;
        DEBUG("[kw41zrf] previous TX done\n");
    }
}

int kw41zrf_cca(kw41zrf_t *dev)
{
    kw41zrf_wait_idle(dev);
    if (kw41zrf_is_dsm()) {
        /* bring the device out of DSM */
        kw41zrf_set_power_mode(dev, KW41ZRF_POWER_IDLE);
    }
    kw41zrf_abort_sequence(dev);
    kw41zrf_unmask_irqs();
    LED_RX_ON;
    kw41zrf_set_sequence(dev, XCVSEQ_CCA);
    /* Wait for the CCA to finish, it will take exactly RX warmup time + 128 µs */
    kw41zrf_wait_idle(dev);
    LED_RX_OFF;
    DEBUG("[kw41zrf] CCA: %u RSSI: %d\n", (unsigned)dev->cca_result,
          kw41zrf_get_ed_level(dev));
    return dev->cca_result;
}

static int kw41zrf_netdev_send(netdev_t *netdev, const iolist_t *iolist)
{
    kw41zrf_t *dev = (kw41zrf_t *)netdev;
    size_t len = 0;

    kw41zrf_wait_idle(dev);
    if (kw41zrf_is_dsm()) {
        /* bring the device out of DSM */
        kw41zrf_set_power_mode(dev, KW41ZRF_POWER_IDLE);
    }

    /* load packet data into buffer */
    for (const iolist_t *iol = iolist; iol; iol = iol->iol_next) {
        /* current packet data + FCS too long */
        if ((len + iol->iol_len) > (KW41ZRF_MAX_PKT_LENGTH - IEEE802154_FCS_LEN)) {
            LOG_ERROR("[kw41zrf] packet too large (%u byte) to fit\n",
                  (unsigned)len + IEEE802154_FCS_LEN);
            return -EOVERFLOW;
        }
        len = kw41zrf_tx_load(iol->iol_base, iol->iol_len, len);
    }

    DEBUG("[kw41zrf] TX %u bytes\n", len);

    /*
     * First octet in the TX buffer contains the frame length.
     * Nbytes = FRAME_LEN - 2 -> FRAME_LEN = Nbytes + 2
     * MKW41Z ref. man. 44.6.2.6.3.1.3 Sequence T (Transmit), p. 2147
     */
    *((volatile uint8_t *)&ZLL->PKT_BUFFER_TX[0]) = len + IEEE802154_FCS_LEN;
#if defined(MODULE_OD) && ENABLE_DEBUG
    DEBUG("[kw41zrf] send:\n");
    od_hex_dump((const uint8_t *)ZLL->PKT_BUFFER_TX, len, OD_WIDTH_DEFAULT);
#endif

#ifdef MODULE_NETSTATS_L2
    netdev->stats.tx_bytes += len;
#endif

    /* send data out directly if pre-loading is disabled */
    if (!(dev->flags & KW41ZRF_OPT_PRELOADING)) {
        dev->csma_be = dev->csma_min_be;
        dev->csma_num_backoffs = 0;
        dev->num_retrans = 0;
        kw41zrf_tx_exec(dev);
    }

    return (int)len;
}

static int kw41zrf_netdev_recv(netdev_t *netdev, void *buf, size_t len, void *info)
{
    kw41zrf_t *dev = (kw41zrf_t *)netdev;
    if (kw41zrf_is_dsm()) {
        /* bring the device out of DSM, sleep will be restored before returning */
        kw41zrf_set_power_mode(dev, KW41ZRF_POWER_IDLE);
    }
    /* get size of the received packet */
    uint8_t pkt_len = (ZLL->IRQSTS & ZLL_IRQSTS_RX_FRAME_LENGTH_MASK) >> ZLL_IRQSTS_RX_FRAME_LENGTH_SHIFT;
    if (pkt_len < IEEE802154_FCS_LEN) {
        if (kw41zrf_can_switch_to_idle(dev)) {
            kw41zrf_abort_sequence(dev);
            kw41zrf_set_sequence(dev, dev->idle_seq);
        }
        return -EAGAIN;
    }
    /* skip FCS */
    pkt_len -= IEEE802154_FCS_LEN;
    DEBUG("[kw41zrf] RX %u bytes\n", pkt_len);

    /* just return length when buf == NULL */
    if (buf == NULL) {
        if (len > 0) {
            /* discard what we have stored in the buffer, unblock RX */
            dev->recv_blocked = false;
            if (kw41zrf_can_switch_to_idle(dev)) {
                kw41zrf_abort_sequence(dev);
                kw41zrf_set_sequence(dev, dev->idle_seq);
            }
        }
        /* No set_sequence(idle_seq) here, keep transceiver turned on if the
         * buffer was not discarded, we expect the higher layer to call again
         * shortly with a proper buffer */
        return pkt_len;
    }

#if defined(MODULE_OD) && ENABLE_DEBUG
    DEBUG("[kw41zrf] recv:\n");
    od_hex_dump((const uint8_t *)ZLL->PKT_BUFFER_RX, pkt_len, OD_WIDTH_DEFAULT);
#endif

#ifdef MODULE_NETSTATS_L2
    netdev->stats.rx_count++;
    netdev->stats.rx_bytes += pkt_len;
#else
    (void)netdev;
#endif

    if (pkt_len > len) {
        /* not enough space in buf */
        if (kw41zrf_can_switch_to_idle(dev)) {
            kw41zrf_abort_sequence(dev);
            kw41zrf_set_sequence(dev, dev->idle_seq);
        }
        return -ENOBUFS;
    }
    memcpy(buf, (const void *)&ZLL->PKT_BUFFER_RX[0], pkt_len);

    if (info != NULL) {
        netdev_ieee802154_rx_info_t *radio_info = info;
        uint8_t hw_lqi = (ZLL->LQI_AND_RSSI & ZLL_LQI_AND_RSSI_LQI_VALUE_MASK) >>
            ZLL_LQI_AND_RSSI_LQI_VALUE_SHIFT;
        /* TODO Validate, verify or adjust this LQI calculation */
        if (hw_lqi >= 220) {
            radio_info->lqi = 255;
        } else {
            radio_info->lqi = (51 * hw_lqi) / 44;
        }
        radio_info->rssi = (ZLL->LQI_AND_RSSI & ZLL_LQI_AND_RSSI_RSSI_MASK) >> ZLL_LQI_AND_RSSI_RSSI_SHIFT;
    }

    /* Go back to RX mode */
    dev->recv_blocked = false;
    if (kw41zrf_can_switch_to_idle(dev)) {
        kw41zrf_abort_sequence(dev);
        kw41zrf_set_sequence(dev, dev->idle_seq);
    }

    return pkt_len;
}

static int kw41zrf_netdev_set_state(kw41zrf_t *dev, netopt_state_t state)
{
    switch (state) {
        case NETOPT_STATE_OFF:
            /* There is no deeper 'off' mode than deep sleep mode */
            /* fall through */
        case NETOPT_STATE_SLEEP:
            if (kw41zrf_is_dsm()) {
                /* Transceiver is already in deep sleep mode */
                break;
            }
            kw41zrf_abort_sequence(dev);
            kw41zrf_set_power_mode(dev, KW41ZRF_POWER_DSM);
            dev->idle_seq = XCVSEQ_DSM_IDLE;
            break;
        case NETOPT_STATE_STANDBY:
            kw41zrf_set_power_mode(dev, KW41ZRF_POWER_IDLE);
            kw41zrf_abort_sequence(dev);
            dev->idle_seq = XCVSEQ_IDLE;
            kw41zrf_set_sequence(dev, dev->idle_seq);
            break;
        case NETOPT_STATE_IDLE:
            kw41zrf_set_power_mode(dev, KW41ZRF_POWER_IDLE);
            kw41zrf_abort_sequence(dev);
            dev->idle_seq = XCVSEQ_RECEIVE;
            kw41zrf_set_sequence(dev, dev->idle_seq);
            break;
        case NETOPT_STATE_TX:
            if (dev->flags & KW41ZRF_OPT_PRELOADING) {
                kw41zrf_wait_idle(dev);
                if (kw41zrf_is_dsm()) {
                    /* bring the device out of DSM */
                    kw41zrf_set_power_mode(dev, KW41ZRF_POWER_IDLE);
                }
                dev->csma_be = dev->csma_min_be;
                dev->csma_num_backoffs = 0;
                dev->num_retrans = 0;
                kw41zrf_tx_exec(dev);
            }
            break;
        case NETOPT_STATE_RESET:
            kw41zrf_reset(dev);
            break;
        default:
            return -ENOTSUP;
    }
    return sizeof(netopt_state_t);
}

static netopt_state_t kw41zrf_netdev_get_state(kw41zrf_t *dev)
{
    (void) dev;
    /* ZLL register access require that the transceiver is powered on and not in
     * deep sleep mode */
    if (kw41zrf_is_dsm()) {
        /* Transceiver is in deep sleep mode */
        return NETOPT_STATE_SLEEP;
    }
    uint32_t seq = (ZLL->PHY_CTRL & ZLL_PHY_CTRL_XCVSEQ_MASK) >> ZLL_PHY_CTRL_XCVSEQ_SHIFT;

    switch (seq) {
        case XCVSEQ_TRANSMIT:
        case XCVSEQ_TX_RX:
            return NETOPT_STATE_TX;

        case XCVSEQ_CCA:
        case XCVSEQ_CONTINUOUS_CCA:
            return NETOPT_STATE_RX;

        case XCVSEQ_RECEIVE:
        {
            uint32_t seq_state = ZLL->SEQ_STATE;
            if (seq_state & ZLL_SEQ_STATE_SFD_DET_MASK) {
                /* SFD detection has been triggered */
                if (seq_state & ZLL_SEQ_STATE_RX_BYTE_COUNT_MASK) {
                    /* packet reception is in progress */
                    return NETOPT_STATE_RX;
                }
            }
            /* NETOPT_STATE_IDLE means on, and listening for incoming packets */
            return NETOPT_STATE_IDLE;
        }

        case XCVSEQ_IDLE:
            /* SEQ_IDLE in kw41z means on, but not listening for incoming traffic */
            return NETOPT_STATE_STANDBY;

        default:
            /* Unknown state */
            LOG_ERROR("[kw41z] in unknown sequence: 0x%02" PRIx32 "\n", seq);
            return NETOPT_STATE_OFF;
    }
}

int kw41zrf_netdev_get(netdev_t *netdev, netopt_t opt, void *value, size_t len)
{
    kw41zrf_t *dev = (kw41zrf_t *)netdev;

    if (dev == NULL) {
        return -ENODEV;
    }

    /* These settings do not require the transceiver to be powered on */
    switch (opt) {
        case NETOPT_STATE:
            if (len != sizeof(netopt_state_t)) {
                return -EOVERFLOW;
            }
            *((netopt_state_t *)value) = kw41zrf_netdev_get_state(dev);
            return len;

        case NETOPT_MAX_PACKET_SIZE:
            if (len != sizeof(int16_t)) {
                return -EOVERFLOW;
            }
            *((uint16_t *)value) = KW41ZRF_MAX_PKT_LENGTH - _MAX_MHR_OVERHEAD;
            return len;

        case NETOPT_PRELOADING:
            if (len != sizeof(netopt_enable_t)) {
                return -EOVERFLOW;
            }
            *((netopt_enable_t *)value) =
                !!(dev->flags & KW41ZRF_OPT_PRELOADING);
            return len;

        case NETOPT_PROMISCUOUSMODE:
            if (len != sizeof(netopt_enable_t)) {
                return -EOVERFLOW;
            }
            *((netopt_enable_t *)value) =
                !!(dev->flags & KW41ZRF_OPT_PROMISCUOUS);
            return len;

        case NETOPT_RX_START_IRQ:
            if (len != sizeof(netopt_enable_t)) {
                return -EOVERFLOW;
            }
            *((netopt_enable_t *)value) =
                !!(dev->flags & KW41ZRF_OPT_TELL_RX_START);
            return len;

        case NETOPT_RX_END_IRQ:
            if (len != sizeof(netopt_enable_t)) {
                return -EOVERFLOW;
            }
            *((netopt_enable_t *)value) =
                !!(dev->flags & KW41ZRF_OPT_TELL_RX_END);
            return len;

        case NETOPT_TX_START_IRQ:
            if (len != sizeof(netopt_enable_t)) {
                return -EOVERFLOW;
            }
            *((netopt_enable_t *)value) =
                !!(dev->flags & KW41ZRF_OPT_TELL_TX_START);
            return len;

        case NETOPT_TX_END_IRQ:
            if (len != sizeof(netopt_enable_t)) {
                return -EOVERFLOW;
            }
            *((netopt_enable_t *)value) =
                !!(dev->flags & KW41ZRF_OPT_TELL_TX_END);
            return len;

        case NETOPT_CSMA:
            if (len != sizeof(netopt_enable_t)) {
                return -EOVERFLOW;
            }
            *((netopt_enable_t *)value) =
                !!(dev->flags & KW41ZRF_OPT_CSMA);
            return len;

        case NETOPT_CSMA_RETRIES:
            if (len != sizeof(uint8_t)) {
                return -EOVERFLOW;
            }
            *((uint8_t *)value) = dev->csma_max_backoffs;
            return len;

        case NETOPT_CSMA_MAXBE:
            if (len != sizeof(uint8_t)) {
                return -EOVERFLOW;
            }
            *((uint8_t *)value) = dev->csma_max_be;
            return len;

        case NETOPT_CSMA_MINBE:
            if (len != sizeof(uint8_t)) {
                return -EOVERFLOW;
            }
            *((uint8_t *)value) = dev->csma_min_be;
            return len;

        case NETOPT_RETRANS:
            if (len != sizeof(uint8_t)) {
                return -EOVERFLOW;
            }
            *((uint8_t *)value) = dev->max_retrans;
            return len;

        case NETOPT_TX_RETRIES_NEEDED:
            if (len != sizeof(uint8_t)) {
                return -EOVERFLOW;
            }
            *((uint8_t *)value) = dev->num_retrans;
            return len;

        case NETOPT_CHANNEL_PAGE:
            if (len != sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            *((uint16_t *)value) = 0;
            return len;

        default:
            break;
    }

    /* The below settings require the transceiver to be powered on */
    bool put_to_sleep_when_done = false;
    if (kw41zrf_is_dsm()) {
        /* Transceiver is in deep sleep mode */
        switch (opt) {
            case NETOPT_TX_POWER:
            case NETOPT_IS_CHANNEL_CLR:
            case NETOPT_CCA_THRESHOLD:
            case NETOPT_CCA_MODE:
            case NETOPT_LAST_ED_LEVEL:
                kw41zrf_set_power_mode(dev, KW41ZRF_POWER_IDLE);
#ifdef MODULE_NETOPT
                DEBUG("[kw41zrf] Wake to get opt %s\n", netopt2str(opt));
#else
                DEBUG("[kw41zrf] Wake to get opt %d\n", (int)opt);
#endif
                put_to_sleep_when_done = true;
                break;

            default:
                break;
        }
    }
    else {
        /* Wait for oscillator ready signal if the CPU is coming out of low
         * power mode */
        while((RSIM->CONTROL & RSIM_CONTROL_RF_OSC_READY_MASK) == 0) {}
    }

    int res = -ENOTSUP;

    switch (opt) {
        case NETOPT_TX_POWER:
            if (len != sizeof(int16_t)) {
                res = -EOVERFLOW;
                break;
            }
            *((uint16_t *)value) = kw41zrf_get_txpower(dev);
            res = len;
            break;

        case NETOPT_IS_CHANNEL_CLR:
            if (len != sizeof(netopt_enable_t)) {
                res = -EOVERFLOW;
                break;
            }
            *((netopt_enable_t *)value) = !(kw41zrf_cca(dev));
            res = len;
            break;

        case NETOPT_CCA_THRESHOLD:
            if (len != sizeof(int8_t)) {
                res = -EOVERFLOW;
                break;
            }
            *((int8_t *)value) = kw41zrf_get_cca_threshold(dev);
            res = len;
            break;

        case NETOPT_CCA_MODE:
            if (len != sizeof(uint8_t)) {
                res = -EOVERFLOW;
                break;
            }
            uint8_t mode = kw41zrf_get_cca_mode(dev);
            switch (mode) {
                case NETDEV_IEEE802154_CCA_MODE_1:
                case NETDEV_IEEE802154_CCA_MODE_2:
                case NETDEV_IEEE802154_CCA_MODE_3:
                    *((uint8_t *)value) = mode;
                    res = len;
                    break;
                default:
                    res = -EINVAL;
                    break;
            }
            break;

        case NETOPT_LAST_ED_LEVEL:
            if (len != sizeof(int8_t)) {
                res = -EOVERFLOW;
                break;
            }
            *((int8_t *)value) = kw41zrf_get_ed_level(dev);
            res = len;
            break;

        default:
            break;
    }

    if (put_to_sleep_when_done) {
        DEBUG("[kw41zrf] Go back to sleep\n");
        kw41zrf_set_power_mode(dev, KW41ZRF_POWER_DSM);
    }

    if (res == -ENOTSUP) {
        res = netdev_ieee802154_get((netdev_ieee802154_t *)netdev, opt, value, len);
    }
    return res;
}

static int kw41zrf_netdev_set(netdev_t *netdev, netopt_t opt, const void *value, size_t len)
{
    kw41zrf_t *dev = (kw41zrf_t *)netdev;
    int res = -ENOTSUP;

    if (dev == NULL) {
        return -ENODEV;
    }

    /* These settings do not require the transceiver to be awake */
    switch (opt) {
        case NETOPT_CHANNEL_PAGE:
            res = -EINVAL;
            break;

        case NETOPT_STATE:
            if (len != sizeof(const netopt_state_t)) {
                res = -EOVERFLOW;
                break;
            }
            res = kw41zrf_netdev_set_state(dev, *((const netopt_state_t *)value));
            break;

        case NETOPT_PRELOADING:
            if (len != sizeof(const netopt_enable_t)) {
                res = -EOVERFLOW;
                break;
            }
            kw41zrf_set_option(dev, KW41ZRF_OPT_PRELOADING,
                               *((const netopt_enable_t *)value));
            res = len;
            break;

        case NETOPT_RX_END_IRQ:
            if (len != sizeof(const netopt_enable_t)) {
                res = -EOVERFLOW;
                break;
            }
            kw41zrf_set_option(dev, KW41ZRF_OPT_TELL_RX_END,
                               *((const netopt_enable_t *)value));
            res = len;
            break;

        case NETOPT_TX_START_IRQ:
            if (len != sizeof(const netopt_enable_t)) {
                res = -EOVERFLOW;
                break;
            }
            kw41zrf_set_option(dev, KW41ZRF_OPT_TELL_TX_START,
                               *((const netopt_enable_t *)value));
            res = len;
            break;

        case NETOPT_TX_END_IRQ:
            if (len != sizeof(const netopt_enable_t)) {
                res = -EOVERFLOW;
                break;
            }
            kw41zrf_set_option(dev, KW41ZRF_OPT_TELL_TX_END,
                               *((const netopt_enable_t *)value));
            res = len;
            break;

        case NETOPT_CSMA_RETRIES:
            if (len != sizeof(uint8_t)) {
                res = -EOVERFLOW;
                break;
            }
            dev->csma_max_backoffs = *((const uint8_t*)value);
            res = len;
            break;

        case NETOPT_CSMA_MAXBE:
            if (len != sizeof(uint8_t)) {
                res = -EOVERFLOW;
                break;
            }
            dev->csma_max_be = *((const uint8_t*)value);
            res = len;
            break;

        case NETOPT_CSMA_MINBE:
            if (len != sizeof(uint8_t)) {
                res = -EOVERFLOW;
                break;
            }
            dev->csma_min_be = *((const uint8_t*)value);
            res = len;
            break;

        case NETOPT_RETRANS:
            if (len != sizeof(uint8_t)) {
                return -EOVERFLOW;
            }
            dev->max_retrans = *((const uint8_t *)value);
            res = len;
            break;

        default:
            break;
    }

    bool put_to_sleep_when_done = false;

    if (kw41zrf_is_dsm()) {
        /* Transceiver is in deep sleep mode, check if setting the option
         * requires the radio powered on */
        switch (opt) {
            case NETOPT_AUTOACK:
            case NETOPT_PROMISCUOUSMODE:
            case NETOPT_RX_START_IRQ:
            case NETOPT_CSMA:
            case NETOPT_ADDRESS:
            case NETOPT_ADDRESS_LONG:
            case NETOPT_NID:
            case NETOPT_CHANNEL:
            case NETOPT_TX_POWER:
            case NETOPT_CCA_THRESHOLD:
            case NETOPT_CCA_MODE:
                kw41zrf_set_power_mode(dev, KW41ZRF_POWER_IDLE);
#ifdef MODULE_NETOPT
                DEBUG("[kw41zrf] Wake to set opt %s\n", netopt2str(opt));
#else
                DEBUG("[kw41zrf] Wake to set opt %d\n", (int)opt);
#endif
                put_to_sleep_when_done = true;
                break;

            default:
                break;
        }
    }

    switch (opt) {
        case NETOPT_AUTOACK:
            /* Set up HW generated automatic ACK after Receive */
            if (len != sizeof(const netopt_enable_t)) {
                res = -EOVERFLOW;
                break;
            }
            kw41zrf_set_option(dev, KW41ZRF_OPT_AUTOACK,
                               *((const netopt_enable_t *)value));
            res = len;
            break;

        case NETOPT_PROMISCUOUSMODE:
            if (len != sizeof(const netopt_enable_t)) {
                res = -EOVERFLOW;
                break;
            }
            kw41zrf_set_option(dev, KW41ZRF_OPT_PROMISCUOUS,
                               *((const netopt_enable_t *)value));
            res = len;
            break;

        case NETOPT_RX_START_IRQ:
            if (len != sizeof(const netopt_enable_t)) {
                res = -EOVERFLOW;
                break;
            }
            kw41zrf_set_option(dev, KW41ZRF_OPT_TELL_RX_START,
                               *((const netopt_enable_t *)value));
            res = len;
            break;

        case NETOPT_CSMA:
            if (len != sizeof(const netopt_enable_t)) {
                res = -EOVERFLOW;
                break;
            }
            kw41zrf_set_option(dev, KW41ZRF_OPT_CSMA,
                               ((const netopt_enable_t *)value)[0]);
            res = len;
            break;

        case NETOPT_ADDRESS:
            if (len != sizeof(const uint16_t)) {
                res = -EOVERFLOW;
                break;
            }
            kw41zrf_set_addr_short(dev, *((const uint16_t *)value));
            /* don't set res to set netdev_ieee802154_t::short_addr */
            break;

        case NETOPT_ADDRESS_LONG:
            if (len != sizeof(const uint64_t)) {
                res = -EOVERFLOW;
                break;
            }
            kw41zrf_set_addr_long(dev, *((const uint64_t *)value));
            /* don't set res to set netdev_ieee802154_t::short_addr */
            break;

        case NETOPT_NID:
            if (len != sizeof(const uint16_t)) {
                res = -EOVERFLOW;
                break;
            }
            kw41zrf_set_pan(dev, *((const uint16_t *)value));
            /* don't set res to set netdev_ieee802154_t::pan */
            break;

        case NETOPT_CHANNEL:
            if (len != sizeof(const uint16_t)) {
                res = -EOVERFLOW;
                break;
            }
            if (kw41zrf_set_channel(dev, *((const uint16_t *)value))) {
                res = -EINVAL;
                break;
            }
            /* don't set res to set netdev_ieee802154_t::chan */
            break;

        case NETOPT_TX_POWER:
            if (len != sizeof(const uint16_t)) {
                res = -EOVERFLOW;
                break;
            }
            kw41zrf_set_tx_power(dev, *(const int16_t *)value);
            res = len;
            break;

        case NETOPT_CCA_THRESHOLD:
            if (len != sizeof(const uint8_t)) {
                res = -EOVERFLOW;
                break;
            }
            kw41zrf_set_cca_threshold(dev, *((const uint8_t*)value));
            res = len;
            break;

        case NETOPT_CCA_MODE:
            if (len != sizeof(const uint8_t)) {
                res = -EOVERFLOW;
                break;
            }
            uint8_t mode = *((const uint8_t*)value);
            switch (mode) {
                case NETDEV_IEEE802154_CCA_MODE_1:
                case NETDEV_IEEE802154_CCA_MODE_2:
                case NETDEV_IEEE802154_CCA_MODE_3:
                    kw41zrf_set_cca_mode(dev, mode);
                    res = len;
                    break;
                case NETDEV_IEEE802154_CCA_MODE_4:
                case NETDEV_IEEE802154_CCA_MODE_5:
                case NETDEV_IEEE802154_CCA_MODE_6:
                default:
                    res = -EINVAL;
                    break;
            }
            break;

        default:
            break;
    }

    if (put_to_sleep_when_done) {
        DEBUG("[kw41zrf] Go back to sleep\n");
        kw41zrf_set_power_mode(dev, KW41ZRF_POWER_DSM);
    }

    if (res == -ENOTSUP) {
        res = netdev_ieee802154_set((netdev_ieee802154_t *)netdev, opt, value, len);
    }

    return res;
}

/* Common CCA check handler code for sequences T and TR */
static uint32_t _isr_event_seq_t_ccairq(kw41zrf_t *dev, uint32_t irqsts)
{
    uint32_t handled_irqs = 0;
    if (irqsts & ZLL_IRQSTS_CCAIRQ_MASK) {
        /* CCA before TX has completed */
        handled_irqs |= ZLL_IRQSTS_CCAIRQ_MASK;
        if (irqsts & ZLL_IRQSTS_CCA_MASK) {
            /* Channel was determined busy */
            DEBUG("[kw41zrf] CCA ch busy (RSSI: %d)\n",
                  (int8_t)((ZLL->LQI_AND_RSSI & ZLL_LQI_AND_RSSI_CCA1_ED_FNL_MASK) >>
                ZLL_LQI_AND_RSSI_CCA1_ED_FNL_SHIFT));
            if (dev->csma_num_backoffs < dev->csma_max_backoffs) {
                /* Perform CSMA/CA backoff algorithm */
                ++dev->csma_num_backoffs;
                if (dev->csma_be < dev->csma_max_be) {
                    /* Increase delay exponent */
                    ++dev->csma_be;
                }
                /* Resubmit the frame for transmission */
                kw41zrf_tx_exec(dev);
                return handled_irqs;
            }
            if (dev->flags & KW41ZRF_OPT_TELL_TX_END) {
                dev->netdev.netdev.event_callback(&dev->netdev.netdev, NETDEV_EVENT_TX_MEDIUM_BUSY);
            }
        }
        else {
            /* Channel is idle */
            DEBUG("[kw41zrf] CCA ch idle (RSSI: %d)\n",
                  (int8_t)((ZLL->LQI_AND_RSSI & ZLL_LQI_AND_RSSI_CCA1_ED_FNL_MASK) >>
                  ZLL_LQI_AND_RSSI_CCA1_ED_FNL_SHIFT));
            if (dev->flags & KW41ZRF_OPT_TELL_TX_START) {
                /* TX will start automatically after CCA check succeeded */
                dev->netdev.netdev.event_callback(&dev->netdev.netdev, NETDEV_EVENT_TX_STARTED);
            }
        }
    }
    return handled_irqs;
}

static uint32_t _isr_event_seq_r(kw41zrf_t *dev, uint32_t irqsts)
{
    uint32_t handled_irqs = 0;

    if (irqsts & ZLL_IRQSTS_RXWTRMRKIRQ_MASK) {
        LED_RX_ON;
        DEBUG("[kw41zrf] RXWTRMRKIRQ (R)\n");
        handled_irqs |= ZLL_IRQSTS_RXWTRMRKIRQ_MASK;
        if (dev->flags & KW41ZRF_OPT_TELL_RX_START) {
            dev->netdev.netdev.event_callback(&dev->netdev.netdev, NETDEV_EVENT_RX_STARTED);
        }
    }

    if (irqsts & ZLL_IRQSTS_FILTERFAIL_IRQ_MASK) {
        DEBUG("[kw41zrf] FILTERFAILIRQ: %04"PRIx32"\n", ZLL->FILTERFAIL_CODE);
        handled_irqs |= ZLL_IRQSTS_FILTERFAIL_IRQ_MASK;
    }

    if (irqsts & ZLL_IRQSTS_RXIRQ_MASK) {
        DEBUG("[kw41zrf] finished RX\n");
        handled_irqs |= ZLL_IRQSTS_RXIRQ_MASK;
        DEBUG("[kw41zrf] RX len: %3u\n",
            (unsigned int)((ZLL->IRQSTS & ZLL_IRQSTS_RX_FRAME_LENGTH_MASK) >>
            ZLL_IRQSTS_RX_FRAME_LENGTH_SHIFT));
        if (ZLL->PHY_CTRL & ZLL_PHY_CTRL_AUTOACK_MASK) {
            LED_TX_ON;
            DEBUG("[kw41zrf] perform TXACK\n");
        }
    }

    if (irqsts & ZLL_IRQSTS_TXIRQ_MASK) {
        LED_TX_OFF;
        DEBUG("[kw41zrf] finished TXACK\n");
        handled_irqs |= ZLL_IRQSTS_TXIRQ_MASK;
    }

    if (irqsts & ZLL_IRQSTS_SEQIRQ_MASK) {
        uint32_t seq_ctrl_sts = ZLL->SEQ_CTRL_STS;
        kw41zrf_abort_sequence(dev);
        DEBUG("[kw41zrf] SEQIRQ (R)\n");
        handled_irqs |= ZLL_IRQSTS_SEQIRQ_MASK;
        LED_RX_OFF;
        if ((irqsts & ZLL_IRQSTS_CRCVALID_MASK) == 0) {
            LOG_ERROR("[kw41zrf] CRC failure (R)\n");
        }
        else if (seq_ctrl_sts & ZLL_SEQ_CTRL_STS_TC3_ABORTED_MASK) {
            LOG_ERROR("[kw41zrf] RX timeout (R)\n");
        }
        else if (seq_ctrl_sts & ZLL_SEQ_CTRL_STS_PLL_ABORTED_MASK) {
            LOG_ERROR("[kw41zrf] PLL unlock (R)\n");
        }
        else if (seq_ctrl_sts & ZLL_SEQ_CTRL_STS_SW_ABORTED_MASK) {
            LOG_ERROR("[kw41zrf] SW abort (R)\n");
        }
        else {
            /* No error reported */
            DEBUG("[kw41zrf] success (R)\n");
            /* Block XCVSEQ_RECEIVE until netdev->recv has been called */
            dev->recv_blocked = true;
            kw41zrf_set_sequence(dev, dev->idle_seq);
            if (dev->flags & KW41ZRF_OPT_TELL_RX_END) {
                dev->netdev.netdev.event_callback(&dev->netdev.netdev, NETDEV_EVENT_RX_COMPLETE);
            }
            return handled_irqs;
        }
        kw41zrf_set_sequence(dev, dev->idle_seq);
    }

    return handled_irqs;
}

static uint32_t _isr_event_seq_t(kw41zrf_t *dev, uint32_t irqsts)
{
    uint32_t handled_irqs = 0;
    if (irqsts & ZLL_IRQSTS_TXIRQ_MASK) {
        DEBUG("[kw41zrf] finished TX (T)\n");
        handled_irqs |= ZLL_IRQSTS_TXIRQ_MASK;
    }
    if (irqsts & ZLL_IRQSTS_SEQIRQ_MASK) {
        /* Finished T sequence */
        kw41zrf_abort_sequence(dev);
        DEBUG("[kw41zrf] SEQIRQ (T)\n");
        handled_irqs |= ZLL_IRQSTS_SEQIRQ_MASK;
        LED_TX_OFF;
        if (dev->flags & KW41ZRF_OPT_TELL_TX_END) {
            dev->netdev.netdev.event_callback(&dev->netdev.netdev, NETDEV_EVENT_TX_COMPLETE);
        }
        /* Go back to being idle */
        kw41zrf_set_sequence(dev, dev->idle_seq);
    }

    return handled_irqs;
}

/* Standalone CCA */
static uint32_t _isr_event_seq_cca(kw41zrf_t *dev, uint32_t irqsts)
{
    uint32_t handled_irqs = 0;

    if (irqsts & ZLL_IRQSTS_SEQIRQ_MASK) {
        /* Finished CCA sequence */
        kw41zrf_abort_sequence(dev);
        DEBUG("[kw41zrf] SEQIRQ (C)\n");
        handled_irqs |= ZLL_IRQSTS_SEQIRQ_MASK;
        if (irqsts & ZLL_IRQSTS_CCA_MASK) {
            DEBUG("[kw41zrf] CCA ch busy\n");
            dev->cca_result = 1;
        }
        else {
            DEBUG("[kw41zrf] CCA ch idle\n");
            dev->cca_result = 0;
        }
        kw41zrf_set_sequence(dev, dev->idle_seq);
    }

    return handled_irqs;
}

static uint32_t _isr_event_seq_tr(kw41zrf_t *dev, uint32_t irqsts)
{
    uint32_t handled_irqs = 0;
    if (irqsts & ZLL_IRQSTS_TXIRQ_MASK) {
        LED_RX_ON;
        DEBUG("[kw41zrf] finished TX (TR)\n");
        handled_irqs |= ZLL_IRQSTS_TXIRQ_MASK;
        DEBUG("[kw41zrf] wait for RX ACK\n");
    }

    if (irqsts & ZLL_IRQSTS_RXIRQ_MASK) {
        LED_RX_OFF;
        DEBUG("[kw41zrf] got RX ACK\n");
        handled_irqs |= ZLL_IRQSTS_RXIRQ_MASK;
    }

    if (irqsts & ZLL_IRQSTS_FILTERFAIL_IRQ_MASK) {
        DEBUG("[kw41zrf] FILTERFAILIRQ (TR): %04"PRIx32"\n", ZLL->FILTERFAIL_CODE);
        handled_irqs |= ZLL_IRQSTS_FILTERFAIL_IRQ_MASK;
    }

    if (irqsts & ZLL_IRQSTS_SEQIRQ_MASK) {
        uint32_t seq_ctrl_sts = ZLL->SEQ_CTRL_STS;
        kw41zrf_abort_sequence(dev);
        DEBUG("[kw41zrf] SEQIRQ (TR)\n");

        handled_irqs |= ZLL_IRQSTS_SEQIRQ_MASK;
        LED_TX_OFF;
        LED_RX_OFF;
        if (seq_ctrl_sts & ZLL_SEQ_CTRL_STS_TC3_ABORTED_MASK) {
            if (dev->num_retrans < dev->max_retrans) {
                /* Perform frame retransmission */
                ++dev->num_retrans;
                DEBUG("[kw41zrf] TX retry %u\n", (unsigned)dev->num_retrans);
                /* Reset CSMA counters for backoff handling */
                dev->csma_be = dev->csma_min_be;
                dev->csma_num_backoffs = 0;
                /* Resubmit the frame for transmission */
                kw41zrf_tx_exec(dev);
                return handled_irqs;
            }
        }
        if (dev->flags & KW41ZRF_OPT_TELL_TX_END) {
            if (seq_ctrl_sts & ZLL_SEQ_CTRL_STS_TC3_ABORTED_MASK) {
                DEBUG("[kw41zrf] RXACK timeout (TR)\n");
                dev->netdev.netdev.event_callback(&dev->netdev.netdev, NETDEV_EVENT_TX_NOACK);
            }
            else if (seq_ctrl_sts & ZLL_SEQ_CTRL_STS_PLL_ABORTED_MASK) {
                DEBUG("[kw41zrf] PLL unlock (TR)\n");
                /* TODO: there is no other error event for TX failures */
                dev->netdev.netdev.event_callback(&dev->netdev.netdev, NETDEV_EVENT_TX_MEDIUM_BUSY);
            }
            else if (seq_ctrl_sts & ZLL_SEQ_CTRL_STS_SW_ABORTED_MASK) {
                DEBUG("[kw41zrf] SW abort (TR)\n");
                /* TODO: there is no other error event for TX failures */
                dev->netdev.netdev.event_callback(&dev->netdev.netdev, NETDEV_EVENT_TX_MEDIUM_BUSY);
            }
            else {
                /* No error reported */
                DEBUG("[kw41zrf] TX success (TR)\n");
                dev->netdev.netdev.event_callback(&dev->netdev.netdev, NETDEV_EVENT_TX_COMPLETE);
            }
        }
        kw41zrf_set_sequence(dev, dev->idle_seq);
    }

    return handled_irqs;
}

static uint32_t _isr_event_seq_ccca(kw41zrf_t *dev, uint32_t irqsts)
{
    uint32_t handled_irqs = 0;
    if (irqsts & ZLL_IRQSTS_SEQIRQ_MASK) {
        DEBUG("[kw41zrf] SEQIRQ (CCCA)\n");
        handled_irqs |= ZLL_IRQSTS_SEQIRQ_MASK;
        if (irqsts & ZLL_IRQSTS_CCA_MASK) {
            DEBUG("[kw41zrf] CCCA ch busy\n");
        }
        else {
            DEBUG("[kw41zrf] CCCA ch idle\n");
        }
        kw41zrf_abort_sequence(dev);
        kw41zrf_set_sequence(dev, dev->idle_seq);
    }

    return handled_irqs;
}

static void kw41zrf_netdev_isr(netdev_t *netdev)
{
    kw41zrf_t *dev = (kw41zrf_t *)netdev;
    if (!spinning_for_irq) {
        num_irqs_handled = num_irqs_queued;
        thread_flags_clear(KW41ZRF_THREAD_FLAG_ISR);
    }

    /* ZLL register access requires that the transceiver is not in deep sleep mode */
    if (kw41zrf_is_dsm()) {
        /* Transceiver is sleeping, the IRQ must have occurred before entering
         * sleep, discard the call */
        DEBUG("kw41zrf: unexpected IRQ while sleeping\n");
        kw41zrf_unmask_irqs();
        return;
    }
    uint32_t irqsts = ZLL->IRQSTS;

    /* Clear all IRQ flags now */
    ZLL->IRQSTS = irqsts;

    uint32_t handled_irqs = 0;
    DEBUG("[kw41zrf] CTRL %08" PRIx32 ", IRQSTS %08" PRIx32 ", FILTERFAIL %08" PRIx32 "\n",
          ZLL->PHY_CTRL, irqsts, ZLL->FILTERFAIL_CODE);

    uint8_t seq = (ZLL->PHY_CTRL & ZLL_PHY_CTRL_XCVSEQ_MASK) >> ZLL_PHY_CTRL_XCVSEQ_SHIFT;

    switch (seq) {
        case XCVSEQ_RECEIVE:
            handled_irqs |= _isr_event_seq_r(dev, irqsts);
            break;

        case XCVSEQ_TRANSMIT:
            /* First check CCA flags */
            handled_irqs |= _isr_event_seq_t_ccairq(dev, irqsts);
            /* Then TX flags */
            handled_irqs |= _isr_event_seq_t(dev, irqsts);
            break;

        case XCVSEQ_CCA:
            handled_irqs |= _isr_event_seq_cca(dev, irqsts);
            break;

        case XCVSEQ_TX_RX:
            /* First check CCA flags */
            handled_irqs |= _isr_event_seq_t_ccairq(dev, irqsts);
            /* Then TX/RX flags */
            handled_irqs |= _isr_event_seq_tr(dev, irqsts);
            break;

        case XCVSEQ_CONTINUOUS_CCA:
            handled_irqs |= _isr_event_seq_ccca(dev, irqsts);
            break;

        case XCVSEQ_IDLE:
            DEBUG("[kw41zrf] IRQ while IDLE\n");
            break;

        default:
            DEBUG("[kw41zrf] undefined seq state in isr\n");
            break;
    }

    irqsts &= ~handled_irqs;

    if (irqsts & 0x000f017ful) {
        DEBUG("[kw41zrf] Unhandled IRQs: 0x%08"PRIx32"\n",
              (irqsts & 0x000f017ful));
    }

    kw41zrf_unmask_irqs();
}

const netdev_driver_t kw41zrf_driver = {
    .init = kw41zrf_netdev_init,
    .send = kw41zrf_netdev_send,
    .recv = kw41zrf_netdev_recv,
    .get  = kw41zrf_netdev_get,
    .set  = kw41zrf_netdev_set,
    .isr  = kw41zrf_netdev_isr,
};

/** @} */

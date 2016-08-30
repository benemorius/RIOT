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
 * @ingroup     cpu_cc2538
 * @{
 *
 * @file
 * @brief       Netdev adaption for the cc26x0_rf driver
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include <errno.h>

#include "net/gnrc.h"
#include "net/netdev2.h"

#include "cc26x0_rf_netdev.h"
#include "cc26x0_rf.h"
#include "cc26x0_rf_internal.h"
#include "xtimer.h"

#define ENABLE_DEBUG        (1)
#include "debug.h"

#define _MAX_MHR_OVERHEAD   (25)

static int  _get(netdev2_t *dev, netopt_t opt, void *value, size_t max_len);
static int  _set(netdev2_t *dev, netopt_t opt, void *value, size_t value_len);
static int  _send(netdev2_t *netdev, const struct iovec *vector, unsigned count);
static int  _recv(netdev2_t *netdev, void *buf, size_t len, void *info);
static void _isr(netdev2_t *netdev);
static int  _init(netdev2_t *dev);

int send_154(uint8_t *payload, uint8_t payload_len);
int rfc_read_frame(void *buf, unsigned short buf_len);
uint8_t rfc_rx_length(void);
void release_data_entry(void);
int8_t rfc_rx_rssi(void);

const netdev2_driver_t cc26x0_rf_driver = {
    .get  = _get,
    .set  = _set,
    .send = _send,
    .recv = _recv,
    .isr  = _isr,
    .init = _init,
};

/* Reference pointer for the IRQ handler */
static netdev2_t *_dev;

/* radio transmit buffer */
uint8_t tx_buf[CC2538_RF_MAX_DATA_LEN] __attribute__((__aligned__(4)));
/* radio transmit buffer write pointer */
uint8_t tx_buf_w_p;
/* radio transmit buffer read pointer */
uint8_t tx_buf_r_p;


void _irq_handler(void)
{
    if (_dev->event_callback) {
        _dev->event_callback(_dev, NETDEV2_EVENT_ISR);
    }
}

static int _get(netdev2_t *netdev, netopt_t opt, void *value, size_t max_len)
{
    cc26x0_rf_t *dev = (cc26x0_rf_t *)netdev;

    if (dev == NULL) {
        return -ENODEV;
    }

    switch (opt) {
        case NETOPT_AUTOACK:
//             if (RFCORE->XREG_FRMCTRL0bits.AUTOACK) {
            if (1) {
                *((netopt_enable_t *)value) = NETOPT_ENABLE;
            }
            else {
                *((netopt_enable_t *)value) = NETOPT_DISABLE;
            }
            return sizeof(netopt_enable_t);

        case NETOPT_CHANNEL_PAGE:
            if (max_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            /* This tranceiver only supports page 0 */
            *((uint16_t *)value) = 0;
            return sizeof(uint16_t);

        case NETOPT_DEVICE_TYPE:
            if (max_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            *((uint16_t *) value) = NETDEV2_TYPE_IEEE802154;
            return sizeof(uint16_t);

        case NETOPT_IS_CHANNEL_CLR:
            if (cc26x0_rf_channel_clear()) {
                *((netopt_enable_t *)value) = NETOPT_ENABLE;
            }
            else {
                *((netopt_enable_t *)value) = NETOPT_DISABLE;
            }
            return sizeof(netopt_enable_t);

        case NETOPT_IS_WIRED:
            return -ENOTSUP;

        case NETOPT_MAX_PACKET_SIZE:
            if (max_len < sizeof(int16_t)) {
                return -EOVERFLOW;
            }
            *((uint16_t *)value) = CC2538_RF_MAX_DATA_LEN - _MAX_MHR_OVERHEAD;
            return sizeof(uint16_t);

        case NETOPT_PROMISCUOUSMODE:
            if (cc26x0_rf_get_monitor()) {
                *((netopt_enable_t *)value) = NETOPT_ENABLE;
            }
            else {
                *((netopt_enable_t *)value) = NETOPT_DISABLE;
            }
            return sizeof(netopt_enable_t);

        case NETOPT_STATE:
            if (max_len < sizeof(netopt_state_t)) {
                return -EOVERFLOW;
            }
            *((netopt_state_t *)value) = dev->state;
            return sizeof(netopt_state_t);

        case NETOPT_TX_POWER:
            if (max_len < sizeof(int16_t)) {
                return -EOVERFLOW;
            }
            *((uint16_t *)value) = cc26x0_rf_get_tx_power();
            return sizeof(uint16_t);

        default:
            break;
    }

    int res;

    if (((res = netdev2_ieee802154_get((netdev2_ieee802154_t *)netdev, opt, value,
                                       max_len)) >= 0) || (res != -ENOTSUP)) {
        return res;
    }

    return -ENOTSUP;
}

static int _set(netdev2_t *netdev, netopt_t opt, void *value, size_t value_len)
{
    cc26x0_rf_t *dev = (cc26x0_rf_t *)netdev;
    int res = -ENOTSUP;

    if (dev == NULL) {
        return -ENODEV;
    }

    switch (opt) {
        case NETOPT_ADDRESS:
            if (value_len > sizeof(uint16_t)) {
                res = -EOVERFLOW;
            }
            else {
                cc26x0_rf_set_addr_short(*((uint16_t*)value));
            }
            break;

        case NETOPT_ADDRESS_LONG:
            if (value_len > sizeof(uint64_t)) {
                res = -EOVERFLOW;
            }
            else {
                cc26x0_rf_set_addr_long(*((uint64_t*)value));
            }
            break;

        case NETOPT_AUTOACK:
//             RFCORE->XREG_FRMCTRL0bits.AUTOACK = ((bool *)value)[0];
            res = sizeof(netopt_enable_t);
            break;

        case NETOPT_CHANNEL:
            if (value_len != sizeof(uint16_t)) {
                res = -EINVAL;
            }
            else {
                uint8_t chan = ((uint8_t *)value)[0];
                if (chan < IEEE802154_MIN_CHANNEL ||
                    chan > IEEE802154_MAX_CHANNEL) {
                    res = -EINVAL;
                }
                else {
                    cc26x0_rf_set_chan(chan);
                }
            }
            break;

        case NETOPT_CHANNEL_PAGE:
            /* This tranceiver only supports page 0 */
            if (value_len != sizeof(uint16_t) ||
                *((uint16_t *)value) != 0 ) {
                res = -EINVAL;
            }
            else {
                res = sizeof(uint16_t);
            }
            break;

        case NETOPT_IS_WIRED:
            return -ENOTSUP;

        case NETOPT_NID:
            if (value_len > sizeof(uint16_t)) {
                res = -EOVERFLOW;
            }
            else {
                cc26x0_rf_set_pan(*((uint16_t *)value));
            }
            break;

        case NETOPT_PROMISCUOUSMODE:
            cc26x0_rf_set_monitor(((bool *)value)[0]);
            res = sizeof(netopt_enable_t);
            break;

        case NETOPT_STATE:
            if (value_len > sizeof(netopt_state_t)) {
                return -EOVERFLOW;
            }
            cc26x0_rf_set_state(dev, *((netopt_state_t *)value));
            res = sizeof(netopt_state_t);
            break;

        case NETOPT_TX_POWER:
            if (value_len > sizeof(int16_t)) {
                return -EOVERFLOW;
            }
            cc26x0_rf_set_tx_power(*((int16_t *)value));
            res = sizeof(uint16_t);
            break;

        default:
            break;
    }

    if (res == -ENOTSUP) {
        res = netdev2_ieee802154_set((netdev2_ieee802154_t *)netdev, opt,
                                     value, value_len);
    }

    return res;
}

static int _send(netdev2_t *netdev, const struct iovec *vector, unsigned count)
{
    int pkt_len = 0;

//     printf("_send() %u chunks\n", count);

    cc26x0_rf_t *dev = (cc26x0_rf_t *) netdev;
    mutex_lock(&dev->mutex);

    /* reset tx buffer pointer */
    tx_buf_w_p = 0;

    /* The first byte of the TX FIFO must be the packet length,
       but we don't know what it is yet. Write a null byte to the
       start of the FIFO, so we can come back and update it later */
//     tx_buf[tx_buf_w_p++] = 0;

    for (unsigned i = 0; i < count; i++) {
        pkt_len += vector[i].iov_len;

        if (pkt_len > CC2538_RF_MAX_DATA_LEN) {
            printf("cc26x0_rf: packet too large (%u > %u)\n",
                  pkt_len, CC2538_RF_MAX_DATA_LEN);
            return -EOVERFLOW;
        }

        memcpy(&tx_buf[tx_buf_w_p], vector[i].iov_base, vector[i].iov_len);
        tx_buf_w_p += vector[i].iov_len;
    }

    /* Set first byte of TX FIFO to the packet length */
    /* Set first byte of tx buffer to the packet length */
//     tx_buf[0] = pkt_len + CC2538_AUTOCRC_LEN;

    /* FIXME force panid to 0x777 */
//     tx_buf[3] = 0x77;
//     tx_buf[4] = 0x07;

    /* packet length will be prepended and crc appended by radio hardware */
    send_154(tx_buf, pkt_len);

    mutex_unlock(&dev->mutex);

    return pkt_len;
}

static int _recv(netdev2_t *netdev, void *buf, size_t len, void *info)
{
//     printf("_recv() buf 0x%lx len %u info 0x%lx\n", (uint32_t)buf, len, (uint32_t)info);

    cc26x0_rf_t *dev = (cc26x0_rf_t *) netdev;
    size_t pkt_len;

    mutex_lock(&dev->mutex);

    if (buf == NULL) {
        /* GNRC wants to know how much data we've got for it */
        pkt_len = rfc_rx_length() - 2;

        /* Make sure pkt_len is sane */
        if (pkt_len > CC2538_RF_MAX_DATA_LEN) {
            release_data_entry();
            mutex_unlock(&dev->mutex);
            return -EOVERFLOW;
        }

//         /* CRC check */
//         if (!(rfcore_peek_rx_fifo(pkt_len) & 0x80)) {
//             /* CRC failed; discard packet */
//             release_data_entry();
//             mutex_unlock(&dev->mutex);
//             return -ENODATA;
//         }

        if (len > 0) {
            /* GNRC wants us to drop the packet */
            release_data_entry();
        }

        mutex_unlock(&dev->mutex);
        return pkt_len - IEEE802154_FCS_LEN;
    }
    else {
        /* GNRC is expecting len bytes of data */
        pkt_len = len;
    }

    rfc_read_frame(buf, pkt_len);

    if (info != NULL) {
        netdev2_ieee802154_rx_info_t *radio_info = info;
        radio_info->rssi = rfc_rx_rssi();

        /* This is not strictly 802.15.4 compliant, since according to
           the CC2538 documentation, this value will tend between ~50
           for the lowest quality link detectable by the receiver, and
           ~110 for the highest. The IEEE 802.15.4 spec mandates that
           this value be between 0-255, with 0 as lowest quality and
           255 as the highest. FIXME. */
        radio_info->lqi = 7;

        printf("got packet with rssi %i lqi %u\n", (int8_t)radio_info->rssi, radio_info->lqi);
    }

    release_data_entry();
    mutex_unlock(&dev->mutex);

    return pkt_len;
}

static void _isr(netdev2_t *netdev)
{
    netdev->event_callback(netdev, NETDEV2_EVENT_RX_COMPLETE);
}

static int _init(netdev2_t *netdev)
{
    cc26x0_rf_t *dev = (cc26x0_rf_t *) netdev;
    _dev = netdev;

    mutex_lock(&dev->mutex);

    uint16_t pan = cc26x0_rf_get_pan();
    uint16_t chan = cc26x0_rf_get_chan();
    uint16_t addr_short = cc26x0_rf_get_addr_short();
    uint64_t addr_long = cc26x0_rf_get_addr_long();

    /* Initialise netdev2_ieee802154_t struct */
    netdev2_ieee802154_set((netdev2_ieee802154_t *)netdev, NETOPT_NID,
                                     &pan, sizeof(pan));
    netdev2_ieee802154_set((netdev2_ieee802154_t *)netdev, NETOPT_CHANNEL,
                                     &chan, sizeof(chan));
    netdev2_ieee802154_set((netdev2_ieee802154_t *)netdev, NETOPT_ADDRESS,
                                     &addr_short, sizeof(addr_short));
    netdev2_ieee802154_set((netdev2_ieee802154_t *)netdev, NETOPT_ADDRESS_LONG,
                                     &addr_long, sizeof(addr_long));

    cc26x0_rf_set_state(dev, NETOPT_STATE_IDLE);

    /* set default protocol */
#ifdef MODULE_GNRC_SIXLOWPAN
    dev->netdev.proto = GNRC_NETTYPE_SIXLOWPAN;
#elif MODULE_GNRC
    dev->netdev.proto = GNRC_NETTYPE_UNDEF;
#endif

    mutex_unlock(&dev->mutex);

    return 0;
}

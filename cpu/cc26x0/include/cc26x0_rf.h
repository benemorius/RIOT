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
 */

#ifndef CC26X0_RF_H
#define CC26X0_RF_H

#include <stdbool.h>

#include "mutex.h"
#include "net/netdev2.h"
#include "net/netdev2/ieee802154.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CC2538_AUTOCRC_LEN          (2)
#define CC2538_RF_FIFO_SIZE         (128)
#define CC2538_PACKET_LENGTH_SIZE   (1)

#define CC2538_RF_MAX_DATA_LEN (CC2538_RF_FIFO_SIZE - CC2538_PACKET_LENGTH_SIZE)

#define CC2538_EUI64_LOCATION_PRI   (0x00280028) /**< Primary EUI-64 address location */
#define CC2538_EUI64_LOCATION_SEC   (0x0027FFCC) /**< Secondary EUI-64 address location */

/* TODO: Move these to sys/include/net/ieee802154.h somehow */
/* IEEE 802.15.4 defined constants (2.4 GHz logical channels) */
#define IEEE802154_MIN_FREQ         (2405) /**< Min. frequency (2405 MHz) */
#define IEEE802154_MAX_FREQ         (2480) /**< Max. frequency (2480 MHz) */

#define IEEE802154_MIN_CHANNEL      (11)   /**< Min. channel (2405 MHz) */
#define IEEE802154_MAX_CHANNEL      (26)   /**< Max. channel (2480 MHz) */
#define IEEE802154_CHANNEL_SPACING  (5)    /**< Channel spacing in MHz  */

#define IEEE802154_CHAN2FREQ(chan)  ( IEEE802154_MIN_FREQ + ((chan) - IEEE802154_MIN_CHANNEL) * IEEE802154_CHANNEL_SPACING )
#define IEEE802154_FREQ2CHAN(freq)  ( IEEE802154_MIN_CHANNEL + ((freq) - IEEE802154_MIN_FREQ) / IEEE802154_CHANNEL_SPACING )
/* /TODO */

#define CC2538_MIN_FREQ             (2394)
#define CC2538_MAX_FREQ             (2507)

#define CC2538_RF_POWER_DEFAULT     (3)    /**< Default output power in dBm */
#ifdef DEFAULT_CHANNEL
#define CC2538_RF_CHANNEL_DEFAULT   (DEFAULT_CHANNEL)
#endif
#ifndef CC2538_RF_CHANNEL_DEFAULT
#define CC2538_RF_CHANNEL_DEFAULT   (26U)
#endif
#define CC2538_RF_PANID_DEFAULT     (0x0023)

#define OUTPUT_POWER_MIN            (-24)  /**< Min output power in dBm */
#define OUTPUT_POWER_MAX            (7)    /**< Max output power in dBm */
#define NUM_POWER_LEVELS            ( OUTPUT_POWER_MAX - OUTPUT_POWER_MIN + 1 )

#define RFCORE_ASSERT(expr) (void)( (expr) || RFCORE_ASSERT_failure(#expr, __FUNCTION__, __LINE__) )

#if DEVELHELP
#define RFCORE_WAIT_UNTIL(expr) while (!(expr)) {                                        \
    DEBUG("RFCORE_WAIT_UNTIL(%s) at line %u in %s()\n", #expr, __LINE__, __FUNCTION__);  \
    thread_yield();                                                                      \
}
#else
#define RFCORE_WAIT_UNTIL(expr) while (!(expr)) thread_yield()
#endif

#define RFCORE_FLUSH_RECEIVE_FIFO() rfcore_strobe(ISFLUSHRX)

#define ABS_DIFF(x, y)          ( ((x) < (y))? ((y) - (x)) : ((x) - (y)) )
#define BOOLEAN(x)              ( (x) != 0 )
#define NOT(x)                  ( (x) == 0 )
#define GET_BYTE(buffer, index) ( (unsigned char*)(buffer) )[index]

#define BIT(n) ( 1 << (n) )

enum {
    FSM_STATE_IDLE           =  0,
    FSM_STATE_RX_CALIBRATION =  2,
    FSM_STATE_TX_CALIBRATION = 32,
};

/*
 * @brief RFCORE_XREG_RFERRM bits
 */
enum {
    STROBE_ERR = BIT(6),
    TXUNDERF   = BIT(5),
    TXOVERF    = BIT(4),
    RXUNDERF   = BIT(3),
    RXOVERF    = BIT(2),
    RXABO      = BIT(1),
    NLOCK      = BIT(0),
};

 /*
  * @brief RFCORE_XREG_FRMCTRL0 bits
  */
enum {
    ENERGY_SCAN      = BIT(4),
    AUTOACK          = BIT(5),
    AUTOCRC          = BIT(6),
    APPEND_DATA_MODE = BIT(7),
};

/*
 * @brief RFCORE_XREG_RFIRQM0 / RFCORE_XREG_RFIRQF0 bits
 */
enum {
    ACT_UNUSED       = BIT(0),
    SFD              = BIT(1), /**< Start of frame event */
    FIFOP            = BIT(2),
    SRC_MATCH_DONE   = BIT(3),
    SRC_MATCH_FOUND  = BIT(4),
    FRAME_ACCEPTED   = BIT(5),
    RXPKTDONE        = BIT(6), /**< End of frame event */
    RXMASKZERO       = BIT(7),
};

/* Values for use with CCTEST_OBSSELx registers: */
#define OBSSEL_EN BIT(7)
enum {
    rfc_obs_sig0 = 0,
    rfc_obs_sig1 = 1,
    rfc_obs_sig2 = 2,
};

/* Values for RFCORE_XREG_RFC_OBS_CTRLx registers: */
enum {
    constant_value_0 = 0x00,
    constant_value_1 = 0x01,
    rfc_sniff_data   = 0x08,
    rfc_sniff_clk    = 0x09,
    rssi_valid       = 0x0c,
    demod_cca        = 0x0d,
    sampled_cca      = 0x0e,
    sfd_sync         = 0x0f,
    tx_active        = 0x10,
    rx_active        = 0x11,
    ffctrl_fifo      = 0x12,
    ffctrl_fifop     = 0x13,
    packet_done      = 0x14,
    rfc_xor_rand_i_q = 0x16,
    rfc_rand_q       = 0x17,
    rfc_rand_i       = 0x18,
    lock_status      = 0x19,
    pa_pd            = 0x20,
    lna_pd           = 0x2a,
};

/**
 * @brief   Device descriptor for CC2538 transceiver
 *
 * @extends netdev2_ieee802154_t
 */
typedef struct {
    netdev2_ieee802154_t netdev;  /**< netdev2 parent struct */
    uint8_t state;                /**< current state of the radio */
    mutex_t mutex;                /**< device access lock */
} cc26x0_rf_t;

/**
 * @brief   IRQ handler for RF events
 *
 */
void _irq_handler(void);

/**
 * @brief   Trigger a clear channel assessment
 *
 * @return  True if channel is clear
 * @return  False if channel is busy
 */
bool cc26x0_rf_channel_clear(void);

/**
 * @brief   Get the configured long address of the device
 *
 * @return  The currently set (8-byte) long address
 */
uint64_t cc26x0_rf_get_addr_long(void);

/**
 * @brief   Get the configured short address of the device
 *
 * @return  The currently set (2-byte) short address
 */
uint16_t cc26x0_rf_get_addr_short(void);

/**
 * @brief   Get the primary (burned-in) EUI-64 of the device
 *
 * @return  The primary EUI-64 of the device
 */
uint64_t cc26x0_rf_get_eui64_primary(void);

/**
 * @brief   Get the configured channel number of the device
 *
 * @return  The currently set channel number
 */
unsigned int cc26x0_rf_get_chan(void);

/**
 * @brief   Check if device is in monitor (promiscuous) mode
 *
 * @return  True if device is in monitor mode
 * @return  False if device is not in monitor mode
 */
bool cc26x0_rf_get_monitor(void);

/**
 * @brief   Get the configured PAN ID of the device
 *
 * @return  The currently set PAN ID
 */
uint16_t cc26x0_rf_get_pan(void);

/**
 * @brief   Get the configured transmission power of the device
 *
 * @return  The currently configured transmission power in dBm
 */
int cc26x0_rf_get_tx_power(void);

/**
 * @brief   Initialise the CC2538 radio hardware
 *
 */
void cc26x0_rf_init(void);

/**
 * @brief   Check if device is active
 *
 * @return  True if device is active
 * @return  False if device is not active
 */
bool cc26x0_rf_is_on(void);

/**
 * @brief   Deactivate the CC2538 radio device
 *
 */
void cc26x0_rf_off(void);

/**
 * @brief   Activate the CC2538 radio device
 *
 */
bool cc26x0_rf_on(void);

/**
 * @brief   Setup a CC2538 radio device for use with netdev
 *
 * @param[out] dev          Device descriptor
 */
void cc26x0_rf_setup(cc26x0_rf_t *dev);

/**
 * @brief   Set the short address of the device
 *
 * @param[in] addr          (2-byte) short address to set
 */
void cc26x0_rf_set_addr_short(uint16_t addr);

/**
 * @brief   Set the long address of the device
 *
 * @param[in] addr          (8-byte) short address to set
 */
void cc26x0_rf_set_addr_long(uint64_t addr);

/**
 * @brief   Set the channel number of the device
 *
 * @param[in] chan          Channel number to set
 */
void cc26x0_rf_set_chan(unsigned int chan);

/**
 * @brief   Set the frequency of the device
 *
 * @param[in] MHz          Frequency to set in MHz
 */
void cc26x0_rf_set_freq(unsigned int MHz);

/**
 * @brief   Enable/disable monitor (promiscuous) mode for the device
 *
 * @param[in] mode          True for enable, false for disable
 */
void cc26x0_rf_set_monitor(bool mode);

/**
 * @brief   Set the PAN ID of the device
 *
 * @param[in] pan           PAN ID to set
 */
void cc26x0_rf_set_pan(uint16_t pan);

/**
 * @brief   Set the state of the device
 *
 * @param[out] dev          Device descriptor
 * @param[in]  state        State to set device to
 */
void cc26x0_rf_set_state(cc26x0_rf_t *dev, netopt_state_t state);

/**
 * @brief   Set the transmission power for the device
 *
 * @param[in] dBm          Transmission power to set in dBm
 */
void cc26x0_rf_set_tx_power(int dBm);

#ifdef __cplusplus
}
#endif

#endif /* CC26X0_RF_H */
/** @} */

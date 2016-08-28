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
 * @brief       Getter and setter functions for the cc26x0_rf driver
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 *
 * @}
 */

#include "cc26x0_rf.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/* static const __flash uint8_t? */
static const uint8_t power_lut[NUM_POWER_LEVELS] = {
    0,   /**< -24 dBm */
    7,   /**< -23 dBm */
    15,  /**< -22 dBm */
    22,  /**< -21 dBm */
    29,  /**< -20 dBm */
    37,  /**< -19 dBm */
    44,  /**< -18 dBm */
    51,  /**< -17 dBm */
    59,  /**< -16 dBm */
    66,  /**< -15 dBm */
    77,  /**< -14 dBm */
    88,  /**< -13 dBm */
    93,  /**< -12 dBm */
    98,  /**< -11 dBm */
    106, /**< -10 dBm */
    114, /**<  -9 dBm */
    125, /**<  -8 dBm */
    136, /**<  -7 dBm */
    141, /**<  -6 dBm */
    145, /**<  -5 dBm */
    153, /**<  -4 dBm */
    161, /**<  -3 dBm */
    169, /**<  -2 dBm */
    176, /**<  -1 dBm */
    182, /**<   0 dBm */
    197, /**<   1 dBm */
    205, /**<   2 dBm */
    213, /**<   3 dBm */
    225, /**<   4 dBm */
    237, /**<   5 dBm */
    246, /**<   6 dBm */
    255, /**<   7 dBm */
};

uint64_t cc26x0_rf_get_addr_long(void)
{
//     uint64_t addr = RFCORE_FFSM_EXT_ADDR0;
//     addr <<= 8;
//     addr |= RFCORE_FFSM_EXT_ADDR1;
//     addr <<= 8;
//     addr |= RFCORE_FFSM_EXT_ADDR2;
//     addr <<= 8;
//     addr |= RFCORE_FFSM_EXT_ADDR3;
//     addr <<= 8;
//     addr |= RFCORE_FFSM_EXT_ADDR4;
//     addr <<= 8;
//     addr |= RFCORE_FFSM_EXT_ADDR5;
//     addr <<= 8;
//     addr |= RFCORE_FFSM_EXT_ADDR6;
//     addr <<= 8;
//     addr |= RFCORE_FFSM_EXT_ADDR7;
//     return addr;
    return 0;
}

uint16_t cc26x0_rf_get_addr_short(void)
{
//     return (RFCORE_FFSM_SHORT_ADDR0 << 8) | RFCORE_FFSM_SHORT_ADDR1;
    return 0;
}

unsigned int cc26x0_rf_get_chan(void)
{
//     return IEEE802154_FREQ2CHAN(CC2538_MIN_FREQ + RFCORE_XREG_FREQCTRL);
    return 0;
}

uint64_t cc26x0_rf_get_eui64_primary(void)
{
//     /*
//      * The primary EUI-64 seems to be written to memory in a non-sequential
//      * byte order, with both 4-byte halves of the address flipped.
//      */
//     uint64_t eui64 = ((uint8_t*)CC2538_EUI64_LOCATION_PRI)[4];
//     eui64 <<= 8;
//     eui64 |= ((uint8_t*)CC2538_EUI64_LOCATION_PRI)[5];
//     eui64 <<= 8;
//     eui64 |= ((uint8_t*)CC2538_EUI64_LOCATION_PRI)[6];
//     eui64 <<= 8;
//     eui64 |= ((uint8_t*)CC2538_EUI64_LOCATION_PRI)[7];
//     eui64 <<= 8;
//     eui64 |= ((uint8_t*)CC2538_EUI64_LOCATION_PRI)[0];
//     eui64 <<= 8;
//     eui64 |= ((uint8_t*)CC2538_EUI64_LOCATION_PRI)[1];
//     eui64 <<= 8;
//     eui64 |= ((uint8_t*)CC2538_EUI64_LOCATION_PRI)[2];
//     eui64 <<= 8;
//     eui64 |= ((uint8_t*)CC2538_EUI64_LOCATION_PRI)[3];
//     return eui64;
    return 0;
}

bool cc26x0_rf_get_monitor(void)
{
//     return NOT(RFCORE->XREG_FRMFILT0bits.FRAME_FILTER_EN);
    return 0;
}

uint16_t cc26x0_rf_get_pan(void)
{
//     return (RFCORE_FFSM_PAN_ID1 << 8) | RFCORE_FFSM_PAN_ID0;
    return 0;
}

int cc26x0_rf_get_tx_power(void)
{
    int index;
    int best_index = 0;
    int best_delta = INT_MAX;
    int txpower = 0;

//     txpower = RFCORE_XREG_TXPOWER & 0xff;

    for (index = 0; index < NUM_POWER_LEVELS; index++) {
        int delta = ABS_DIFF(power_lut[index], txpower);

        if (delta < best_delta) {
            best_delta = delta;
            best_index = index;
        }
    }

    return OUTPUT_POWER_MIN + best_index;
}

void cc26x0_rf_set_addr_long(uint64_t addr)
{
//     RFCORE_FFSM_EXT_ADDR0 = addr >> (7 * 8);
//     RFCORE_FFSM_EXT_ADDR1 = addr >> (6 * 8);
//     RFCORE_FFSM_EXT_ADDR2 = addr >> (5 * 8);
//     RFCORE_FFSM_EXT_ADDR3 = addr >> (4 * 8);
//     RFCORE_FFSM_EXT_ADDR4 = addr >> (3 * 8);
//     RFCORE_FFSM_EXT_ADDR5 = addr >> (2 * 8);
//     RFCORE_FFSM_EXT_ADDR6 = addr >> (1 * 8);
//     RFCORE_FFSM_EXT_ADDR7 = addr >> (0 * 8);
}

void cc26x0_rf_set_addr_short(uint16_t addr)
{
//     RFCORE_FFSM_SHORT_ADDR1 = addr;
//     RFCORE_FFSM_SHORT_ADDR0 = addr >> 8;
}

void cc26x0_rf_set_chan(unsigned int chan)
{
    DEBUG("%s(%u): Setting channel to ", __FUNCTION__, chan);

    if (chan < IEEE802154_MIN_CHANNEL) {
        chan = IEEE802154_MIN_CHANNEL;
    }
    else if (chan > IEEE802154_MAX_CHANNEL) {
        chan = IEEE802154_MAX_CHANNEL;
    }

    DEBUG("%i (range %i-%i)\n", chan, IEEE802154_MIN_CHANNEL,
                                      IEEE802154_MAX_CHANNEL);

    cc26x0_rf_set_freq(IEEE802154_CHAN2FREQ(chan));
}

void cc26x0_rf_set_freq(unsigned int MHz)
{
    DEBUG("%s(%u): Setting frequency to ", __FUNCTION__, MHz);

    if (MHz < IEEE802154_MIN_FREQ) {
        MHz = IEEE802154_MIN_FREQ;
    }
    else if (MHz > IEEE802154_MAX_FREQ) {
        MHz = IEEE802154_MAX_FREQ;
    }

    DEBUG("%i (range %i-%i)\n", MHz, IEEE802154_MIN_FREQ, IEEE802154_MAX_FREQ);
//     RFCORE_XREG_FREQCTRL = MHz - CC2538_MIN_FREQ;
}

void cc26x0_rf_set_monitor(bool mode)
{
//     RFCORE->XREG_FRMFILT0bits.FRAME_FILTER_EN = NOT(mode);
}

void cc26x0_rf_set_state(cc26x0_rf_t *dev, netopt_state_t state)
{
    switch (state) {
        case NETOPT_STATE_OFF:
        case NETOPT_STATE_SLEEP:
            cc26x0_rf_off();
            dev->state = state;
            break;

        case NETOPT_STATE_RX:
        case NETOPT_STATE_IDLE:
            if (!cc26x0_rf_is_on()) {
                cc26x0_rf_on();
//                 RFCORE_WAIT_UNTIL(RFCORE->XREG_FSMSTAT0bits.FSM_FFCTRL_STATE > FSM_STATE_RX_CALIBRATION);
            }
            dev->state = state;
            break;

        case NETOPT_STATE_TX:
            dev->state = NETOPT_STATE_IDLE;
            break;

        case NETOPT_STATE_RESET:
            cc26x0_rf_off();
            cc26x0_rf_on();
//             RFCORE_WAIT_UNTIL(RFCORE->XREG_FSMSTAT0bits.FSM_FFCTRL_STATE > FSM_STATE_RX_CALIBRATION);
            dev->state = NETOPT_STATE_IDLE;
            break;
    }
}

void cc26x0_rf_set_pan(uint16_t pan)
{
//     RFCORE_FFSM_PAN_ID0 = pan;
//     RFCORE_FFSM_PAN_ID1 = pan >> 8;
}

void cc26x0_rf_set_tx_power(int dBm)
{
    DEBUG("%s(%i): Setting TX power to ", __FUNCTION__, dBm);

    if (dBm < OUTPUT_POWER_MIN) {
        dBm = OUTPUT_POWER_MIN;
    }
    else if (dBm > OUTPUT_POWER_MAX) {
        dBm = OUTPUT_POWER_MAX;
    }

    DEBUG("%idBm (range %i-%i dBm)\n", dBm, OUTPUT_POWER_MIN, OUTPUT_POWER_MAX);
//     RFCORE_XREG_TXPOWER = power_lut[dBm - OUTPUT_POWER_MIN];
}

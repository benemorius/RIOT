#include "controller_interface.h"


void hci_send_packet_to_controller(uint8_t type, char *data, uint8_t datalen)
{
    Hci_SendPacketToController(type, data, datalen);
}

/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include "od.h"

#ifdef MODULE_NIMBLE_HOST
#include "syscfg/syscfg.h"
#include "sysinit/sysinit.h"
#include "os/os.h"
#include "mem/mem.h"
#include "nimble/ble.h"
#include "nimble/ble_hci_trans.h"
#include "nimble/hci_common.h"
#endif

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifndef MODULE_OD
#define od_hex_dump(...)
#endif

#ifdef MODULE_NIMBLE_HOST

ble_hci_trans_rx_cmd_fn *ble_hci_ram_rx_cmd_hs_cb;
void *ble_hci_ram_rx_cmd_hs_arg;

static ble_hci_trans_rx_cmd_fn *ble_hci_ram_rx_cmd_ll_cb;
static void *ble_hci_ram_rx_cmd_ll_arg;

ble_hci_trans_rx_acl_fn *ble_hci_ram_rx_acl_hs_cb;
void *ble_hci_ram_rx_acl_hs_arg;

static ble_hci_trans_rx_acl_fn *ble_hci_ram_rx_acl_ll_cb;
static void *ble_hci_ram_rx_acl_ll_arg;

static struct os_mempool ble_hci_ram_cmd_pool;
static os_membuf_t ble_hci_ram_cmd_buf[
        OS_MEMPOOL_SIZE(1, BLE_HCI_TRANS_CMD_SZ)
];

static struct os_mempool ble_hci_ram_evt_hi_pool;
static os_membuf_t ble_hci_ram_evt_hi_buf[
    OS_MEMPOOL_SIZE(MYNEWT_VAL(BLE_HCI_EVT_HI_BUF_COUNT),
                    MYNEWT_VAL(BLE_HCI_EVT_BUF_SIZE))
];

static struct os_mempool ble_hci_ram_evt_lo_pool;
static os_membuf_t ble_hci_ram_evt_lo_buf[
        OS_MEMPOOL_SIZE(MYNEWT_VAL(BLE_HCI_EVT_LO_BUF_COUNT),
                        MYNEWT_VAL(BLE_HCI_EVT_BUF_SIZE))
];

static struct os_mbuf_pool ble_hci_ram_acl_mbuf_pool;
static struct os_mempool_ext ble_hci_ram_acl_pool;

/*
 * The MBUF payload size must accommodate the HCI data header size plus the
 * maximum ACL data packet length. The ACL block size is the size of the
 * mbufs we will allocate.
 */
#define ACL_BLOCK_SIZE  OS_ALIGN(MYNEWT_VAL(BLE_ACL_BUF_SIZE) \
+ BLE_MBUF_MEMBLOCK_OVERHEAD \
+ BLE_HCI_DATA_HDR_SZ, OS_ALIGNMENT)

static os_membuf_t ble_hci_ram_acl_buf[
    OS_MEMPOOL_SIZE(MYNEWT_VAL(BLE_ACL_BUF_COUNT), ACL_BLOCK_SIZE)
];

void
ble_hci_trans_cfg_hs(ble_hci_trans_rx_cmd_fn *cmd_cb,
                     void *cmd_arg,
                     ble_hci_trans_rx_acl_fn *acl_cb,
                     void *acl_arg)
{
    ble_hci_ram_rx_cmd_hs_cb = cmd_cb;
    ble_hci_ram_rx_cmd_hs_arg = cmd_arg;
    ble_hci_ram_rx_acl_hs_cb = acl_cb;
    ble_hci_ram_rx_acl_hs_arg = acl_arg;
}

void
ble_hci_trans_cfg_ll(ble_hci_trans_rx_cmd_fn *cmd_cb,
                     void *cmd_arg,
                     ble_hci_trans_rx_acl_fn *acl_cb,
                     void *acl_arg)
{
    ble_hci_ram_rx_cmd_ll_cb = cmd_cb;
    ble_hci_ram_rx_cmd_ll_arg = cmd_arg;
    ble_hci_ram_rx_acl_ll_cb = acl_cb;
    ble_hci_ram_rx_acl_ll_arg = acl_arg;
}

int _ble_hci_ram_rx_cmd_hs_cb(uint8_t* data, uint16_t len)
{
    uint8_t *buf = ble_hci_trans_buf_alloc(BLE_HCI_TRANS_BUF_EVT_LO);
    assert(buf);
    memcpy(buf, data, len);

    od_hex_dump(buf, len, 16);

    return ble_hci_ram_rx_cmd_hs_cb(buf, ble_hci_ram_rx_cmd_hs_arg);
}

int _ble_hci_ram_rx_acl_hs_cb(uint8_t* data, uint16_t len)
{
    struct os_mbuf *mbuf = os_mbuf_get_pkthdr(&ble_hci_ram_acl_mbuf_pool, 0);
    assert(mbuf);
    int rc = os_mbuf_append(mbuf, data, len);
    assert(rc == 0);

    od_hex_dump(mbuf->om_data, len, 16);

    return ble_hci_ram_rx_acl_hs_cb(mbuf, ble_hci_ram_rx_acl_hs_arg);
}

int
ble_hci_trans_hs_cmd_tx(uint8_t *cmd)
{
    int rc;

    struct ble_hci_cmd *c = (struct ble_hci_cmd *)cmd;

    DEBUG("send 0x%04x len %u\n", c->opcode, c->length);

    od_hex_dump(cmd, sizeof(struct ble_hci_cmd) + c->length, 16);

#ifdef BOARD_NRF52840DONGLE
    assert(ble_hci_ram_rx_cmd_ll_cb != NULL);
    rc = ble_hci_ram_rx_cmd_ll_cb(cmd, ble_hci_ram_rx_cmd_ll_arg);
#else
    hci_send_packet_to_controller(0x01, (char *)cmd, c->length + 3);
    ble_hci_trans_buf_free(cmd);
#endif

    return rc;
}

int
ble_hci_trans_ll_evt_tx(uint8_t *hci_ev)
{
    int rc;

    DEBUG("recv 0x%x len %u\n", ((struct ble_hci_ev *)hci_ev)->opcode,
                                        ((struct ble_hci_ev *)hci_ev)->length);

    od_hex_dump(hci_ev,
                sizeof(struct ble_hci_ev) + ((struct ble_hci_ev *)hci_ev)->length,
                16);

#ifdef BOARD_NRF52840DONGLE
    assert(ble_hci_ram_rx_cmd_hs_cb != NULL);
    rc = ble_hci_ram_rx_cmd_hs_cb(hci_ev, ble_hci_ram_rx_cmd_hs_arg);
#endif
    return rc;
}

int
ble_hci_trans_hs_acl_tx(struct os_mbuf *om)
{
    int rc;

    DEBUG("send acl bytes: %u\n", om->om_len);

    od_hex_dump(om->om_data, om->om_len, 16);

#ifdef BOARD_NRF52840DONGLE
    assert(ble_hci_ram_rx_acl_ll_cb != NULL);
    rc = ble_hci_ram_rx_acl_ll_cb(om, ble_hci_ram_rx_acl_ll_arg);
#else
    hci_send_packet_to_controller(0x02, (char *)om->om_data, om->om_len);
    os_mbuf_free_chain(om);
#endif
    return rc;
}

int
ble_hci_trans_ll_acl_tx(struct os_mbuf *om)
{
    int rc;

    DEBUG("receive acl bytes: %u\n", om->om_len);

    od_hex_dump(om->om_data, om->om_len, 16);

#ifdef BOARD_NRF52840DONGLE
    assert(ble_hci_ram_rx_acl_hs_cb != NULL);
    rc = ble_hci_ram_rx_acl_hs_cb(om, ble_hci_ram_rx_acl_hs_arg);
#endif
    return rc;
}

uint8_t *
ble_hci_trans_buf_alloc(int type)
{
    uint8_t *buf;

    switch (type) {
    case BLE_HCI_TRANS_BUF_CMD:
        buf = os_memblock_get(&ble_hci_ram_cmd_pool);
        break;

    case BLE_HCI_TRANS_BUF_EVT_HI:
        buf = os_memblock_get(&ble_hci_ram_evt_hi_pool);
        if (buf == NULL) {
            /* If no high-priority event buffers remain, try to grab a
             * low-priority one.
             */
            buf = ble_hci_trans_buf_alloc(BLE_HCI_TRANS_BUF_EVT_LO);
        }
        break;

    case BLE_HCI_TRANS_BUF_EVT_LO:
        buf = os_memblock_get(&ble_hci_ram_evt_lo_pool);
        break;

    default:
        assert(0);
        buf = NULL;
    }

    return buf;
}

void
ble_hci_trans_buf_free(uint8_t *buf)
{
    int rc;

    /* XXX: this may look a bit odd, but the controller uses the command
    * buffer to send back the command complete/status as an immediate
    * response to the command. This was done to insure that the controller
    * could always send back one of these events when a command was received.
    * Thus, we check to see which pool the buffer came from so we can free
    * it to the appropriate pool
    */
    if (os_memblock_from(&ble_hci_ram_evt_hi_pool, buf)) {
        rc = os_memblock_put(&ble_hci_ram_evt_hi_pool, buf);
        assert(rc == 0);
    } else if (os_memblock_from(&ble_hci_ram_evt_lo_pool, buf)) {
        rc = os_memblock_put(&ble_hci_ram_evt_lo_pool, buf);
        assert(rc == 0);
    } else {
        assert(os_memblock_from(&ble_hci_ram_cmd_pool, buf));
        rc = os_memblock_put(&ble_hci_ram_cmd_pool, buf);
        assert(rc == 0);
    }
}

/**
 * Unsupported; the RAM transport does not have a dedicated ACL data packet
 * pool.
 */
int
ble_hci_trans_set_acl_free_cb(os_mempool_put_fn *cb, void *arg)
{
    return BLE_ERR_UNSUPPORTED;
}

int
ble_hci_trans_reset(void)
{
    /* No work to do.  All allocated buffers are owned by the host or
     * controller, and they will get freed by their owners.
     */
    return 0;
}

#endif /* MODULE_NIMBLE_HOST */

void ble_hci_ram_init(void)
{
#ifdef MODULE_NIMBLE_HOST

    int rc;

    /* Ensure this function only gets called by sysinit. */
//     SYSINIT_ASSERT_ACTIVE();

    rc = os_mempool_ext_init(&ble_hci_ram_acl_pool,
                             MYNEWT_VAL(BLE_ACL_BUF_COUNT),
                             ACL_BLOCK_SIZE,
                             ble_hci_ram_acl_buf,
                             "ble_hci_ram_acl_pool");
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = os_mbuf_pool_init(&ble_hci_ram_acl_mbuf_pool,
                           &ble_hci_ram_acl_pool.mpe_mp,
                           ACL_BLOCK_SIZE,
                           MYNEWT_VAL(BLE_ACL_BUF_COUNT));
    SYSINIT_PANIC_ASSERT(rc == 0);

    /*
     * Create memory pool of HCI command buffers. NOTE: we currently dont
     * allow this to be configured. The controller will only allow one
     * outstanding command. We decided to keep this a pool in case we allow
     * allow the controller to handle more than one outstanding command.
     */
    rc = os_mempool_init(&ble_hci_ram_cmd_pool,
                         1,
                         BLE_HCI_TRANS_CMD_SZ,
                         ble_hci_ram_cmd_buf,
                         "ble_hci_ram_cmd_pool");
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = os_mempool_init(&ble_hci_ram_evt_hi_pool,
                         MYNEWT_VAL(BLE_HCI_EVT_HI_BUF_COUNT),
                         MYNEWT_VAL(BLE_HCI_EVT_BUF_SIZE),
                         ble_hci_ram_evt_hi_buf,
                         "ble_hci_ram_evt_hi_pool");
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = os_mempool_init(&ble_hci_ram_evt_lo_pool,
                         MYNEWT_VAL(BLE_HCI_EVT_LO_BUF_COUNT),
                         MYNEWT_VAL(BLE_HCI_EVT_BUF_SIZE),
                         ble_hci_ram_evt_lo_buf,
                         "ble_hci_ram_evt_lo_pool");
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif /* MODULE_NIMBLE_HOST */
}

bleResult_t Hcit_SendPacket
(
    hciPacketType_t packetType,
 void*           pPacket,
 uint16_t        packetSize
)
{
    bleResult_t result = gBleSuccess_c;

    if (packetType == 0x4) { /* CMD */
        _ble_hci_ram_rx_cmd_hs_cb(pPacket, packetSize);

    } else if (packetType == 0x2) { /* ACL */
        _ble_hci_ram_rx_acl_hs_cb(pPacket, packetSize);
    }

    return result;
}

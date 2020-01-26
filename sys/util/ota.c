#ifdef MODULE_UTIL_OTA_SERVER

#include <stdio.h>

#define ENABLE_DEBUG (0)
#include "debug.h"

#include <sys/types.h>

#include "net/nanocoap_sock.h"
#include "riotboot/slot.h"
#include "periph/pm.h"
#include "thread.h"


#include <stdlib.h>
#include <stdio.h>
#include <string.h>



#include "net/nanocoap.h"
#include "riotboot/flashwrite.h"

riotboot_flashwrite_t _writer;

#define OTA_PORT COAP_PORT

#define COAP_INBUF_SIZE (256U)

#define QUEUE_SIZE (8)
static msg_t _ota_msg_queue[QUEUE_SIZE];
static char _ota_thread_stack [THREAD_STACKSIZE_DEFAULT + 1024];

ssize_t _flashwrite_handler(coap_pkt_t* pkt, uint8_t *buf, size_t len, void *context);

/* WARNING must be sorted by path (ASCII order) */
const coap_resource_t coap_resources[] = {
    COAP_WELL_KNOWN_CORE_DEFAULT_HANDLER,
    { "/flashwrite", COAP_POST, _flashwrite_handler, &_writer },
};
const unsigned coap_resources_numof = sizeof(coap_resources) / sizeof(coap_resources[0]);

static void *_ota_thread(void *arg)
{
    (void)arg;

    /* nanocoap_server uses gnrc sock which uses gnrc which needs a msg queue */
    msg_init_queue(_ota_msg_queue, QUEUE_SIZE);

    /* initialize nanocoap server instance */
    uint8_t buf[COAP_INBUF_SIZE];
    sock_udp_ep_t local = { .port=OTA_PORT, .family=AF_INET6 };
    nanocoap_server(&local, buf, sizeof(buf));

    return NULL;
}

int ota_start_server(void)
{
    static char name[32];
    snprintf(name, sizeof(name), "ota server %u", OTA_PORT);
    thread_create(_ota_thread_stack, sizeof(_ota_thread_stack), 7,
                  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD,
                  _ota_thread, NULL, name);
    return 0;
}

// static xtimer_t reboot_timer;
static bool _is_flashing;

static void _reboot(void *arg)
{
    (void)arg;
    pm_reboot();
//     riotboot_slot_jump(riotboot_slot_other());
}

ssize_t _flashwrite_handler(coap_pkt_t* pkt, uint8_t *buf, size_t len, void *context)
{
    riotboot_flashwrite_t *writer = context;

    uint32_t result = COAP_CODE_204;

    coap_block1_t block1;
    int blockwise = coap_get_block1(pkt, &block1);


    DEBUG("_flashwrite_handler(): received data: offset=%u len=%u blockwise=%i more=%i\n", \
    (unsigned)block1.offset, pkt->payload_len, blockwise, block1.more);


    uint8_t *payload_start = pkt->payload;
    size_t payload_len = pkt->payload_len;
    size_t offset;
    if (block1.offset == 0) {
        DEBUG("_flashwrite_handler(): init len=%u\n", pkt->payload_len);

        riotboot_hdr_t hdr;
        memcpy(&hdr, pkt->payload, sizeof(hdr));

        const riotboot_hdr_t *cur_hdr = riotboot_slot_get_hdr(
                                                    riotboot_slot_current());

        /* make sure the incoming firmware has a valid header */
        if (riotboot_hdr_validate(&hdr)) {
            LOG_ERROR("[OTA] firmware header is not valid\n");
            char payload[] = "riotboot firmware header is not valid";
            return coap_reply_simple(pkt, COAP_CODE_CONFLICT, buf, len,
                                     COAP_FORMAT_TEXT, (uint8_t*)payload, strlen(payload));
        }

        /* make sure the firmware version is newer than the current slot */
        if (hdr.version <= cur_hdr->version) {
            LOG_ERROR("[OTA] the firmware is not newer than the current version %lu\n",
                      cur_hdr->version);
            char payload[] = "riotboot firmware version is not an upgrade";
            return coap_reply_simple(pkt, COAP_CODE_CONFLICT, buf, len,
                                     COAP_FORMAT_TEXT, (uint8_t*)payload, strlen(payload));

        }

        /* make sure the image we're flashing has the right address for the slot */
        if (hdr.start_addr == cur_hdr->start_addr){
            LOG_ERROR("[OTA] the firmware would overwrite the currently running slot at 0x%lx\n",
                   hdr.start_addr);

            char payload[] = "riotboot start address collision - try other slot";
            return coap_reply_simple(pkt, COAP_CODE_CONFLICT, buf, len,
                        COAP_FORMAT_TEXT, (uint8_t*)payload, strlen(payload));
        }

        riotboot_flashwrite_init(writer, riotboot_slot_other());
        _is_flashing = true;
        LOG_WARNING("[OTA] firmware update started for slot %u start_address 0x%lx version %lu\n",
                    riotboot_slot_other(), hdr.start_addr, hdr.version);
    }

    /* skip first RIOTBOOT_FLASHWRITE_SKIPLEN bytes, but handle the case where
     * payload_len is smaller than RIOTBOOT_FLASHWRITE_SKIPLEN
     */
    if (block1.offset <= RIOTBOOT_FLASHWRITE_SKIPLEN) {
        size_t skip = RIOTBOOT_FLASHWRITE_SKIPLEN - block1.offset;
        skip = (payload_len > skip) ? skip : payload_len;
        payload_start += skip;
        payload_len -= skip;
        offset = block1.offset + skip;
    }
    else {
        offset = block1.offset;
    }

    if (offset == writer->offset) {
        riotboot_flashwrite_putbytes(writer, payload_start, payload_len, block1.more);
    }
    else {
        DEBUG("_flashwrite_handler(): skipping invalid offset (data=%u, writer=%u)\n", (unsigned)offset, (unsigned)writer->offset);
    }

    if (block1.more == 1) {
        result = COAP_CODE_CONTINUE;
    }

    if (_is_flashing && (!blockwise || !block1.more)) {
        DEBUG("_flashwrite_handler(): finish");
        int res = riotboot_flashwrite_finish(writer);
        if (res == 0) {
            _is_flashing = false;

            LOG_WARNING("riotboot firmware flashed successfully; rebooting in 3 seconds\n");

            char payload[] = "riotboot firmware flashed successfully; rebooting in 3 seconds";

            coap_reply_simple(pkt, COAP_CODE_CHANGED, buf, len,
                                     COAP_FORMAT_TEXT, (uint8_t*)payload, strlen(payload));

            _reboot(NULL);
//             reboot_timer.callback = _reboot;
//             xtimer_set(&reboot_timer, xtimer_ticks_from_usec(US_PER_MS * 100).ticks32);
        }
    }

    ssize_t reply_len = coap_build_reply(pkt, result, buf, len, 0);
    uint8_t *pkt_pos = (uint8_t*)pkt->hdr + reply_len;
    pkt_pos += coap_put_block1_ok(pkt_pos, &block1, 0);

    return pkt_pos - (uint8_t*)pkt->hdr;
}

#endif /* MODULE_UTIL_OTA_SERVER */

#if (defined MODULE_NANOCOAP_SOCK && defined MODULE_RIOTBOOT_FLASHWRITE) || defined DOXYGEN

#define ENABLE_DEBUG (0)
#include "debug.h"

#include <stdio.h>
#include <sys/types.h>

#include "net/nanocoap_sock.h"
#include "riotboot/slot.h"
#include "thread.h"


#include <stdlib.h>
#include <stdio.h>
#include <string.h>



#include "net/nanocoap.h"
#include "riotboot/flashwrite.h"

riotboot_flashwrite_t _writer;

#if defined UTIL_OTA_SERVER || defined DOXYGEN

#define COAP_INBUF_SIZE (256U)

#define QUEUE_SIZE (8)
static msg_t _ota_msg_queue[QUEUE_SIZE];
static char _ota_thread_stack [THREAD_STACKSIZE_DEFAULT];

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
    sock_udp_ep_t local = { .port=COAP_PORT, .family=AF_INET6 };
    nanocoap_server(&local, buf, sizeof(buf));

    return NULL;
}

int ota_start_server(void)
{
    thread_create(_ota_thread_stack, sizeof(_ota_thread_stack), 7,
                  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD,
                  _ota_thread, NULL, "ota server 5683");
    return 0;
}

#endif /* UTIL_OTA_SERVER */

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
        riotboot_flashwrite_init(writer, riotboot_slot_other());
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

    if (!blockwise || !block1.more) {
        DEBUG("_flashwrite_handler(): finish");
        int res = riotboot_flashwrite_finish(writer);
        if (res == 0) {
            /* reboot into new firmware, but maybe finish the coap session first */
        }
    }

    ssize_t reply_len = coap_build_reply(pkt, result, buf, len, 0);
    uint8_t *pkt_pos = (uint8_t*)pkt->hdr + reply_len;
    pkt_pos += coap_put_block1_ok(pkt_pos, &block1, 0);

    return pkt_pos - (uint8_t*)pkt->hdr;
}

#endif /* MODULE_NANOCOAP_SOCK */

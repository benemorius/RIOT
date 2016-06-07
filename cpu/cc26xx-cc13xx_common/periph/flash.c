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

#include "flash.h"
#include <string.h>
#include <stdio.h>
#include "driverlib/flash.h"
#include "cpu.h"

static const uint32_t sector_size = 4096;

BOOT_FUNC void _memcpy(void *dest, const void *src, size_t size) {
    uint word_size = 4;
    for (int i = 0; i < size; ) {
        *(uint32_t*)dest = *(uint32_t*)src;
        i += word_size;
        dest += word_size;
        src += word_size;
    }
}

BOOT_FUNC int flash_read(uint32_t read_address, uint8_t *buffer, uint32_t length) {
    memcpy(buffer, (uint32_t*)read_address, length);
    return 0;
}

BOOT_FUNC int flash_write(uint32_t write_address, uint8_t *buffer, uint32_t length) {
    return FlashProgram(buffer, write_address, length);
}

BOOT_FUNC int flash_erase(uint32_t start_address, uint32_t end_address) {
    uint32_t ret = 0;
    uint32_t first_sector = start_address - start_address % sector_size;
    uint32_t last_sector = end_address - end_address % sector_size;

//     __disable_irq();

    for(uint32_t erase_address = first_sector; erase_address <= last_sector; erase_address += sector_size) {
        /* check whether it's ok to erase this sector */
        if (erase_address + sector_size == 0x20000) {
            /* never erase the last flash sector, as the default ccfg values
             * subsequently applied by ti-lib apparently disallow jtag access */
            ret = -1;
            goto out;
        } else if (erase_address < 0x1000) {
            /* never erase the bootloader sector */
            ret = -1;
            goto out;
        }

//         printf("erasing sector 0x%lx\n", erase_address);

        /* erase sector */
        ret = FlashSectorErase(erase_address);
        if (ret) {
            goto out;
        }
    }

    out:
//     __enable_irq();
    return ret;
}

BOOT_FUNC int flash_copy_sectors(uint32_t read_address, uint32_t write_address, uint32_t bytes)
{
    /* write n chunks of size sizeof(buf), erasing sectors as necessary */
    uint8_t buf[128];
    uint32_t ret = 0;
    uint32_t last_erased_address = 0xffffffff;

    __disable_irq();

    while(bytes) {
        /* check whether it's ok to erase this sector' */
        uint32_t erase_address = write_address - write_address % sector_size;
        if (erase_address + sector_size == 0x20000) {
            /* never erase the last flash sector, as the default ccfg values
             * subsequently applied by ti-lib apparently disallow jtag access */
            ret = -1;
            goto out;
        } else if (erase_address < 0x1000) {
            /* never erase the bootloader sector */
            ret = -1;
            goto out;
        }

        /* erase sector if not yet erased */
        if (erase_address != last_erased_address) {
            last_erased_address = erase_address;
            ret = FlashSectorErase(erase_address);
            if (ret) {
                goto out;
            }
        }

        /* write chunk */
        uint32_t chunk_length = sizeof(buf);
        if (chunk_length > bytes)
            chunk_length = bytes;
        _memcpy(buf, (uint32_t*)read_address, chunk_length);

        ret = FlashProgram(buf, write_address, chunk_length);
        if (ret) {
            goto out;
        }
        bytes -= chunk_length;
        read_address += chunk_length;
        write_address += chunk_length;
    }

    out:
    __enable_irq();
    return ret;
}

BOOT_FUNC int flash_copy_test(void)
{
    uint32_t low_begin = 0x1000;
    uint32_t high_begin = 0x10000;
    uint32_t size = 0xf000;
    int ret = 0;

    ret = flash_copy_sectors(low_begin, high_begin, size);
    if (ret) return ret;

    ret = flash_copy_sectors(high_begin, low_begin, size);

    return ret;
}

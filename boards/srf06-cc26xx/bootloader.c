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

#include "bootloader.h"
#include "flash.h"
#include "at45.h"
#include "board.h"
#include "string.h"
#include "hashes/md5.h"

#include "xtimer.h"

static void generate_external_hash(at45_t *dev, uint8_t *hash);
static void copy_internal_to_external(at45_t *dev);
static void store_external_hash(at45_t *dev, uint8_t *hash);

static const uint32_t firmware_size = 0x1f000;
static const uint32_t external_hash_address = 0x0;
static const uint32_t external_firmware_address = 0x100;

BOOT_FUNC void bootloader_board(void) {

//     flash_copy_test() works from Sensortag.cpp but it doesn't work here yet

//     uint32_t now = xtimer_now();
//     int ret = flash_copy_test();
//     ret = ret;
//     uint32_t later = xtimer_now();
//     printf("flash_copy_test returned %i after %lu us\n", ret, later - now);





//     gpio_init(GPIO_MEM_PWR, GPIO_OUT);
//     gpio_clear(GPIO_MEM_PWR); //on
//     at45_t flash;
//     at45_init(&flash, SPI_DEV(0), GPIO_MEM_CS, GPIO_MEM_RST, GPIO_MEM_WP);
//
//     uint8_t id[4];
//     id[0] = 0x55;
//     id[1] = 0x55;
//     id[2] = 0x55;
//     id[3] = 0x55;
//     at45_Read_DF_ID(&flash, id);
// //     printf("flash id: %02x %02x %02x %02x\n", id[0], id[1], id[2], id[3]);
//
//
//
//     uint8_t buf[128];
//     at45_read(&flash, 0, buf, 128);
//     for(int i = 0; i < 128; i++) {
// //         printf("%02x ", buf[i]);
// //         if ((i+1) % 16 == 0)
// //             printf("\n");
//     }
//
//     buf[0] = 0xde;
//     buf[1] = 0xad;
//     buf[2] = 0xbe;
//     buf[3] = 0xef;
//
// //     buf[0] = 0xfe;
// //     buf[1] = 0xed;
// //     buf[2] = 0xbe;
// //     buf[3] = 0xef;
//
//     at45_write(&flash, 0x00, buf, 4);
//
// //     printf("after:\n");
//     at45_read(&flash, 0, buf, 128);
//     for(int i = 0; i < 128; i++) {
// //         printf("%02x ", buf[i]);
// //         if ((i+1) % 16 == 0)
// //             printf("\n");
//     }




//     return;



    //FIXME non-BOOT_FUNC functions can't be called here (memcmp, gpio_init, etc.)

    gpio_init(GPIO_MEM_PWR, GPIO_OUT);
    gpio_clear(GPIO_MEM_PWR); //on
    at45_t flash;
    at45_init(&flash, SPI_DEV(0), GPIO_MEM_CS, GPIO_MEM_RST, GPIO_MEM_WP);


    // read hash stored in external flash
    uint8_t external_hash_stored[16];
    at45_read(&flash, 0x0, external_hash_stored, 16);
    printf("external stored hash is ");
    for(uint8_t i = 0; i < 16; i++) {
        printf("%02x", external_hash_stored[i]);
    }
    printf("\n");

    // generate hash of internal flash except boot sector
    uint32_t internal_address = 0x1000;
    uint32_t internal_bytes = firmware_size;
    uint8_t internal_hash[16];

    uint32_t now = xtimer_now();
    md5(internal_hash, (uint8_t*)internal_address, internal_bytes);
    uint32_t later = xtimer_now();

    printf("md5 of 0x%lx - 0x%lx is ", internal_address, internal_address + internal_bytes);
    for(uint8_t i = 0; i < 16; i++) {
        printf("%02x", internal_hash[i]);
    }
    printf("\n");
    printf("hashing took %lu us\n", later - now);


    // compare externally stored hash with internal hash
    if(memcmp(external_hash_stored, internal_hash, 16) == 0) {
        // hashes match, so just run program as normal
        printf("hash of internal flash matches externally stored hash; booting normally\n");
//         return;
    }
    // else hashes didn't match; need to flash new firmware
    printf("hash of internal flash doesn't match externally stored hash\n");

    // generate hash from external firmware image
    printf("hashing external firmware image\n");
    uint8_t external_hash[16];
    now = xtimer_now();
    generate_external_hash(&flash, external_hash);
    later = xtimer_now();
    printf("hashing took %lu ms\n", (later - now) / 1000);
    printf("generated external hash is ");
    for(uint8_t i = 0; i < 16; i++) {
        printf("%02x", external_hash[i]);
    }
    printf("\n");

    // compare generated hash with stored hash
    if(memcmp(external_hash, external_hash_stored, 16) != 0) {
        // hashes don't match; external firmware image is invalid
        printf("external firmware is invalid - stored hash doesn't match generated hash\n");
        // some error handling should perhaps go here
        printf("copying internal firmware to external flash\n");
        copy_internal_to_external(&flash);
        printf("storing external hash\n");
        store_external_hash(&flash, internal_hash);

        generate_external_hash(&flash, external_hash);
        printf("generated external hash is ");
        for(uint8_t i = 0; i < 16; i++) {
            printf("%02x", external_hash[i]);
        }
        printf("\n");

        at45_read(&flash, 0x0, external_hash_stored, 16);
        printf("external stored hash is ");
        for(uint8_t i = 0; i < 16; i++) {
            printf("%02x", external_hash_stored[i]);
        }
        printf("\n");

        // then run program as normal
        return;
    }
    // else hashes matched; flash external firmware to microcontroller
    printf("external firmware is valid; copying new firmware to microcontroller\n");

    return;
    // copy from external flash to microcontroller flash, one sector at a time
    uint8_t buf[4096];
    uint32_t bytes = 128 * 1024 * 1024;
    for(uint32_t i = 0; i < bytes; i += 4096) {
        at45_read(&flash, i, buf, 4096);
        flash_copy_sectors((uint32_t)buf, i, 4096);
    }

    // firmware flashing will be verified by bootloader after reboot

    // reboot into new firmware
    //...
}

void generate_external_hash(at45_t *dev, uint8_t *hash) {
    const uint32_t start_address = 256;
    const uint32_t bytes = firmware_size;
    uint16_t chunk_size = 256;
    uint8_t buf[chunk_size];
    md5_ctx_t md5_c;

    md5_init(&md5_c);
    for(uint32_t ptr = 0; ptr < bytes; ptr += chunk_size) {
        if(ptr + chunk_size > bytes)
            chunk_size = bytes - ptr;
//         printf("hashing from 0x%lx to 0x%lx\n", ptr + start_address, ptr + start_address + chunk_size);
        at45_read(dev, ptr + start_address, buf, chunk_size);
        md5_update(&md5_c, buf, chunk_size);
    }
    md5_final(&md5_c, hash);
}

void copy_internal_to_external(at45_t *dev) {
    uint32_t internal_start_address = 0x1000;
    uint32_t bytes = firmware_size;

    uint32_t chunk_size = bytes;
//     uint8_t buf[chunk_size];

    printf("copying from internal 0x%lx to external 0x%lx size 0x%lx\n", internal_start_address, external_firmware_address, chunk_size);
    at45_write(dev, external_firmware_address, (uint8_t*)internal_start_address, chunk_size);

}

void store_external_hash(at45_t *dev, uint8_t *hash) {
    at45_write(dev, 0x0, hash, 16);
}

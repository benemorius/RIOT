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
#include "periph/uart.h"

#define ENABLE_DEBUG (1)

#if ENABLE_DEBUG
#include "xtimer.h"
#else
#define printf(...)
#endif

BOOT_FUNC static void calculate_external_hash(uint8_t *hash);
BOOT_FUNC static void calculate_internal_hash(uint8_t *hash);
BOOT_FUNC static void copy_internal_to_external(void);
BOOT_FUNC static void store_external_hash(uint8_t *hash);
BOOT_FUNC static void read_external_hash(uint8_t *hash);
BOOT_FUNC static void uart_rx(void *arg, uint8_t data);
BOOT_FUNC static void receive_firmware_over_uart(void);
BOOT_FUNC static uint8_t* hash_to_string(uint8_t *hash);

static const uint32_t firmware_size = 0x1f000;
static const uint32_t external_hash_address = 0x0;
static const uint32_t external_firmware_address = 0x100;
static const uint32_t internal_firmware_address = 0x1000;

#define CHUNK_SIZE 256
static volatile uint8_t uart_buf[CHUNK_SIZE];
static volatile uint8_t* volatile uart_buf_p = 0;

static at45_t flash;

BOOT_FUNC void bootloader_board(void) {
    //FIXME non-BOOT_FUNC functions can't be called here (memcmp, gpio_init, etc.)

    uart_init(UART_DEV(0), 115200, NULL, NULL);

    gpio_init(GPIO_MEM_PWR, GPIO_OUT);
    gpio_clear(GPIO_MEM_PWR); //on
    at45_init(&flash, SPI_DEV(0), GPIO_MEM_CS, GPIO_MEM_RST, GPIO_MEM_WP);

    gpio_init(GPIO_BTN_B, GPIO_IN);
    if(gpio_read(GPIO_BTN_B)) { // activate uart firmware upload
        // invalidate external firmware (for temporary debugging purposes)
        uint8_t hash[16];
        memset(hash, 0x55, 16);
        store_external_hash(hash);

        receive_firmware_over_uart();
        return;
    }

    // read hash stored in external flash
    uint8_t external_hash_stored[16];
    read_external_hash(external_hash_stored);
    printf("external stored hash is     %s\n", hash_to_string(external_hash_stored));

    // calculate hash of internal flash except boot sector
    uint8_t internal_hash[16];
    calculate_internal_hash(internal_hash);
    printf("md5 of 0x%lx - 0x%lx is  %s\n", internal_firmware_address, internal_firmware_address + firmware_size, hash_to_string(internal_hash));

    // compare externally stored hash with internal hash
    if(memcmp(external_hash_stored, internal_hash, 16) == 0) {
        // hashes match, so just run program as normal
        printf("hash of internal flash matches externally stored hash; booting normally\n");
//         return;
    }
    // else hashes didn't match; need to flash new firmware
    printf("hash of internal flash doesn't match externally stored hash\n");

    // calculate hash from external firmware image
    printf("hashing external firmware image\n");
    uint8_t external_hash[16];
    calculate_external_hash(external_hash);
    printf("calculated external hash is %s\n", hash_to_string(external_hash));

    // compare calculated hash with stored hash
    if(memcmp(external_hash, external_hash_stored, 16) != 0) {
        // hashes don't match; external firmware image is invalid
        printf("external firmware is invalid - stored hash doesn't match calculated hash\n");
        // some error handling should perhaps go here
        printf("copying internal firmware to external flash\n");
        copy_internal_to_external();
        printf("storing external hash\n");
        store_external_hash(internal_hash);

        calculate_external_hash(external_hash);
        printf("calculated external hash is %s\n", hash_to_string(external_hash));

        read_external_hash(external_hash_stored);
        printf("stored external hash is     %s\n", hash_to_string(external_hash_stored));

        // then run program as normal
        return;
    }
    // else hashes matched; flash external firmware to microcontroller
    printf("external firmware is valid; copying new firmware to microcontroller\n");

//     return;

    __disable_irq();

    // erase internal flash
    flash_erase(internal_firmware_address, internal_firmware_address + firmware_size - 1);

    // copy from external flash to internal flash
    uint8_t buf[256];
    for(uint32_t i = 0; i < firmware_size - 0x1000; i += 256) {
//         printf("writing to internal flash at 0x%lx\n", internal_firmware_address + i);
        at45_read(&flash, external_firmware_address + i, buf, 256);
        int ret = flash_write(internal_firmware_address + i, buf, 256);
        if(ret) {
//             printf("flash_write failed with %i at 0x%lx\n", ret, internal_firmware_address + i);
            break;
        }
    }
    __enable_irq();

    printf("done\n");

    // firmware flashing will be verified by bootloader after reboot

    // reboot into new firmware
    //...
}

void calculate_external_hash(uint8_t *hash) {
    const uint32_t start_address = 256;
    const uint32_t bytes = firmware_size;
    uint16_t chunk_size = 256;
    uint8_t buf[chunk_size];
    md5_ctx_t md5_c;

#if ENABLE_DEBUG
    uint32_t now = xtimer_now();
#endif //ENABLE_DEBUG

    md5_init(&md5_c);
    for(uint32_t ptr = 0; ptr < bytes; ptr += chunk_size) {
        if(ptr + chunk_size > bytes)
            chunk_size = bytes - ptr;
//         printf("hashing from 0x%lx to 0x%lx\n", ptr + start_address, ptr + start_address + chunk_size);
        at45_read(&flash, ptr + start_address, buf, chunk_size);
        md5_update(&md5_c, buf, chunk_size);
    }
    md5_final(&md5_c, hash);

#if ENABLE_DEBUG
    uint32_t later = xtimer_now();
    printf("hashing external firmware took %lu ms\n", (later - now) / 1000);
#endif //ENABLE_DEBUG
}

void calculate_internal_hash(uint8_t *hash) {
#if ENABLE_DEBUG
    uint32_t now = xtimer_now();
#endif //ENABLE_DEBUG
    md5(hash, (uint8_t*)internal_firmware_address, firmware_size);
#if ENABLE_DEBUG
    uint32_t later = xtimer_now();
    printf("hashing internal firmware took %lu ms\n", (later - now) / 1000);
#endif //ENABLE_DEBUG
}

void read_external_hash(uint8_t *hash) {
    at45_read(&flash, 0x0, hash, 16);
}

void copy_internal_to_external(void) {
    uint32_t internal_start_address = 0x1000;
    uint32_t bytes = firmware_size;

    uint32_t chunk_size = bytes;
//     uint8_t buf[chunk_size];

#if ENABLE_DEBUG
    printf("copying from internal 0x%lx to external 0x%lx size 0x%lx\n", internal_start_address, external_firmware_address, chunk_size);
    uint32_t now = xtimer_now();
#endif //ENABLE_DEBUG
    at45_write(&flash, external_firmware_address, (uint8_t*)internal_start_address, chunk_size);
#if ENABLE_DEBUG
    uint32_t later = xtimer_now();
    printf("copy took %lu ms\n", (later - now) / 1000);
#endif //ENABLE_DEBUG

}

void store_external_hash(uint8_t *hash) {
    at45_write(&flash, 0x0, hash, 16);
}

void receive_firmware_over_uart(void)
{
    uart_init(UART_DEV(0), 115200, uart_rx, NULL);

    uart_write(UART_DEV(0), (uint8_t*)"receive", 7);

    uint8_t flash_buf[CHUNK_SIZE];
    uart_buf_p = uart_buf;
    for(uint32_t byte = 0; byte < firmware_size + 256; byte += CHUNK_SIZE) {
        while(uart_buf_p != 0);
        uart_write(UART_DEV(0), (uint8_t*)".", 1);
        memcpy(flash_buf, (uint8_t*)uart_buf, CHUNK_SIZE);
        uart_buf_p = uart_buf;
        at45_write(&flash, external_hash_address + byte, flash_buf, CHUNK_SIZE);
    }
    uart_init(UART_DEV(0), 115200, NULL, NULL);
    printf("done flashing\n");

    uint8_t stored_hash[16];
    uint8_t calculated_hash[16];
    read_external_hash(stored_hash);
    printf("received md5:   %s\n", hash_to_string(stored_hash));
    calculate_external_hash(calculated_hash);
    printf("calculated md5: %s\n", hash_to_string(calculated_hash));
    if(memcmp(stored_hash, calculated_hash, 16) == 0) {
        printf("firmware is valid\n");
    } else {
        printf("firmware not valid; md5 mismatch\n");
    }

    // should reset the microcontroller here
}

uint8_t* hash_to_string(uint8_t *hash) {
    static uint8_t hash_string[33];
    for(uint8_t i = 0; i < 32; i+=2) {
        hash_string[i] = (hash[i/2] >> 4) + '0';
        if(hash_string[i] > '9') hash_string[i] += 'a' - '0' - 0xa;
        hash_string[i+1] = (hash[i/2] & 0xf) + '0';
        if(hash_string[i+1] > '9') hash_string[i+1] += 'a' - '0' - 0xa;
    }
    hash_string[32] = '\0';
    return hash_string;
}

void uart_rx(void *arg, uint8_t data)
{
    if(uart_buf_p == 0) {
//         printf("!");
        return;
    }

    *uart_buf_p++ = data;
    if(uart_buf_p == &uart_buf[CHUNK_SIZE]) {
        uart_buf_p = 0;
    }
}

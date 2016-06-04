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

BOOT_FUNC void bootloader_board(void) {

//     flash_copy_test() works from Sensortag.cpp but it doesn't work here yet

//     uint32_t now = xtimer_now();
//     int ret = flash_copy_test();
//     ret = ret;
//     uint32_t later = xtimer_now();
//     printf("flash_copy_test returned %i after %lu us\n", ret, later - now);





    gpio_init(GPIO_MEM_PWR, GPIO_OUT);
    gpio_clear(GPIO_MEM_PWR); //on
    at45_t flash;
    at45_init(&flash, SPI_DEV(0), GPIO_MEM_CS, GPIO_MEM_RST, GPIO_MEM_WP);

    uint8_t id[4];
    id[0] = 0x55;
    id[1] = 0x55;
    id[2] = 0x55;
    id[3] = 0x55;
    at45_Read_DF_ID(&flash, id);
//     printf("flash id: %02x %02x %02x %02x\n", id[0], id[1], id[2], id[3]);



    uint8_t buf[128];
    at45_read(&flash, 0, buf, 128);
    for(int i = 0; i < 128; i++) {
//         printf("%02x ", buf[i]);
//         if ((i+1) % 16 == 0)
//             printf("\n");
    }

    buf[0] = 0xde;
    buf[1] = 0xad;
    buf[2] = 0xbe;
    buf[3] = 0xef;

//     buf[0] = 0xfe;
//     buf[1] = 0xed;
//     buf[2] = 0xbe;
//     buf[3] = 0xef;

    at45_write(&flash, 0x00, buf, 4);

//     printf("after:\n");
    at45_read(&flash, 0, buf, 128);
    for(int i = 0; i < 128; i++) {
//         printf("%02x ", buf[i]);
//         if ((i+1) % 16 == 0)
//             printf("\n");
    }

}

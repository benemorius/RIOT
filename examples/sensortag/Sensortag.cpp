/*
    Copyright (c) 2016 Thomas Stilwell <stilwellt@openlabs.co>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
*/

#include "Sensortag.h"

#include "board.h"
#include "periph_conf.h"
#include "periph/cpuid.h"
#include "periph/gpio.h"
#include "thread.h"
#include "xtimer.h"
#include "periph/spi.h"
#include "at45.h"
#include "si70xx.h"
#include "hashes/md5.h"
#include <algorithm>

#include <stdio.h>

// #include "flash.h"
// #include "wdt.h"

// #include "rf-core.h"
// #include "rf-ble.h"

#ifndef GIT_VERSION
#define GIT_VERSION "undefined"
#endif

#define GPIO_LED GPIO_PIN(18)

Sensortag* sensortagS;
uint8_t radio_id;

extern uint16_t ble_mac_address[];
extern char adv_name[];

extern "C" void rfc_prepare(void);
extern "C" bool rfc_setup_ble(void);
extern "C" void rfc_ble_beacon(void);
extern "C" void rfc_powerdown(void);

void debug_printf(const char *format, ...) {
//     tm time;
//     wtimer_get_localtime(&time);
//
//     va_list args;
//     va_start(args, format);
//     char new_format[256];
//     snprintf(new_format, sizeof(new_format), "%04i-%02i-%02i %02i:%02i:%02i %s",
//              time.tm_year, time.tm_mon, time.tm_mday, time.tm_hour,
//              time.tm_min, time.tm_sec, format);
// //     snprintf(new_format, 256, "%s", format);
//     vprintf(new_format, args);
//     va_end(args);
}

// #define DEBUG(...) debug_printf(__VA_ARGS__)
#define DEBUG(...) printf(__VA_ARGS__)

void flash_led(gpio_t led, uint flashes) {
    for (uint flash = 0; flash < flashes; flash++) {
        gpio_set(led);
        xtimer_usleep(200*1000);
        gpio_clear(led);
        xtimer_usleep(150*1000);
    }
}

Sensortag::Sensortag() :
main_pid(thread_getpid())
{
    sensortagS = this;

    gpio_init(GPIO_LED, GPIO_OUT);
//     flash_led(GPIO_LED, 3);

	DEBUG("firmware version: %s\r\n", GIT_VERSION);

	cpuid_get(cpuid);
	DEBUG("cpuid: %02x %02x %02x %02x %02x %02x %02x %02x\n",
		   cpuid[0], cpuid[1], cpuid[2], cpuid[3], cpuid[4], cpuid[5], cpuid[6], cpuid[7]
	);
	radio_id = cpuid[0];
    DEBUG("radio_id: %02x\n", radio_id);

//     rfc_prepare();
//     rfc_setup_ble();

// 	setup_network(cpuid[0], false, 1111, &handle_packet, &handle_transmit);

//     gpio_init_int(BUTTON, GPIO_PULLDOWN, GPIO_RISING, button_handler, NULL);
//     gpio_init_int(BTN_1, GPIO_PULLDOWN, GPIO_RISING, button_handler, NULL);
//     gpio_init_int(BTN_2, GPIO_PULLDOWN, GPIO_RISING, button_handler, NULL);


//     gpio_init(GPIO_MEM_PWR, GPIO_OUT);
//     gpio_clear(GPIO_MEM_PWR); //on
//     at45_t flash;
//     at45_init(&flash, SPI_DEV(0), GPIO_MEM_CS, GPIO_MEM_RST, GPIO_MEM_WP);

//     uint8_t id[4];
//     id[0] = 0x55;
//     id[1] = 0x55;
//     id[2] = 0x55;
//     id[3] = 0x55;
//     at45_Read_DF_ID(&flash, id);
//     printf("flash id: %02x %02x %02x %02x\n", id[0], id[1], id[2], id[3]);
//
//     at45_read_uid(&flash, cpuid, 8);
//     DEBUG("cpuid: %02x %02x %02x %02x %02x %02x %02x %02x\n",
//           cpuid[0], cpuid[1], cpuid[2], cpuid[3], cpuid[4], cpuid[5], cpuid[6], cpuid[7]
//     );
//     radio_id = cpuid[4];
//     DEBUG("radio_id: %02x\n", radio_id);


//     uint32_t size = 512;
//     uint8_t buf[size];
//     printf("before:\n");
//     at45_read(&flash, 0, buf, size);
//     for(uint32_t i = 0; i < size; i++) {
//         printf("%02x ", buf[i]);
//         if ((i+1) % 32 == 0)
//             printf("\n");
//     }


//     bootloader_board();


//     printf("after:\n");
//     at45_read(&flash, 0, buf, size);
//     for(uint32_t i = 0; i < size; i++) {
//         printf("%02x ", buf[i]);
//         if ((i+1) % 32 == 0)
//             printf("\n");
//     }

//     printf("internal:\n");
//     uint8_t *p = (uint8_t*)0x1000;
//     for(uint32_t i = 0; i < size; i++) {
//         printf("%02x ", p[i]);
//         if ((i+1) % 32 == 0)
//             printf("\n");
//     }

}

void Sensortag::mainloop()
{
//     thread_yield();
//     thread_sleep();

	DEBUG("starting mainloop...\r\n");

    printf("do si70xx_init\n");
    si70xx_t si;
    si70xx_init(&si, I2C_DEV(0), 0x40);
    printf("do si70xx_test\n");
    int ret = si70xx_test(&si);
    printf("si70xx_test returned %i\n", ret);

    ble_mac_address[2] = 0xaabb;
    ble_mac_address[1] = 0xccdd;
    ble_mac_address[0] = 0xeeff;
    char ble_name[32];
    snprintf(ble_name, sizeof(ble_name), "riot-cc2650");

    // dev       0d 06 12 07 0b 27 1f 23
    // fridge    0d 06 12 06 2c 0e 1f 23
    // red       04 46 0d 46 04 d9 16 23

    if(radio_id == 0x2c) {
        snprintf(ble_name, sizeof(ble_name), "fridge");
        ble_mac_address[0] = 0xee01;
    }

    if(radio_id == 0x04) {
        snprintf(ble_name, sizeof(ble_name), "red");
        ble_mac_address[0] = 0xee04;
    }

    ble_mac_address[0] = 0xee00;
    ble_mac_address[0] &= (radio_id << 8);
    ble_mac_address[0] = radio_id;

    xtimer_t t;
    uint32_t i =0;
    int16_t temperature = 7;
    uint16_t humidity = 8;

    uint8_t interval = 5;
    xtimer_set_wakeup(&t, 100*1000, thread_getpid());
    while(1) {
        thread_sleep();
        xtimer_set_wakeup(&t, interval * 1000*1000, thread_getpid());

//         static uint32_t wdt_last;
//         uint32_t wdt_current = wdt_read();
//         printf("wdt %lu %lu\n", wdt_current, wdt_last - wdt_current);
//         wdt_last = wdt_current;
//         wdt_periodic();

        si70xx_get_both(&si, &humidity, &temperature);
        char temperature_sign = temperature >= 0 ? ' ' : '-';
        temperature = abs(temperature);

//         uint32_t now = xtimer_now();
//         uint32_t nowt = timer_read(TIMER_DEV(0));
//         printf("0x%08"PRIx32" 0x%08"PRIx32"\n",
//             now,
//             nowt
//         );
//         printf("%i %"PRIu32" %"PRIu32" ",
//                i++,
//                now / 1000,
//                nowt
//         );
        printf("%c%i.%02iC %u.%02u%%\n",
               temperature_sign,
               temperature / 100, temperature % 100,
               humidity / 100, humidity % 100
        );

        snprintf(adv_name, 32, "%s %c%i.%02iC %i%% %lu",
                 ble_name,
                 temperature_sign,
                 temperature / 100, temperature % 100,
                 humidity / 100,
                 i * interval
        );

        rfc_prepare();
        rfc_setup_ble();
        rfc_ble_beacon();
        rfc_powerdown();
        printf("powerdown done\n");

//         xtimer_usleep(1000*1000);
//         printf("back\n");

        flash_led(GPIO_PIN(18), 2);

        // reset before xtimer bug (wdt not working right yet)
//         if(i >= 1000) {
//             NVIC_SystemReset();
//         }
        ++i;
    }
}

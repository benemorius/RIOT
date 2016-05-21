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
#include "s25fl.h"
#include "si70xx.h"

#include <stdio.h>

#ifndef GIT_VERSION
#define GIT_VERSION "undefined"
#endif

#define GPIO_MEM_PWR GPIO_PIN(22)
#define GPIO_MEM_RST GPIO_PIN(21)
#define GPIO_MEM_CS GPIO_PIN(20)
#define GPIO_MEM_WP GPIO_PIN(19)
#define GPIO_LED GPIO_PIN(18)

Sensortag* sensortagS;

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
    flash_led(GPIO_LED, 3);

	DEBUG("firmware version: %s\r\n", GIT_VERSION);

// 	cpuid_get(cpuid);
	DEBUG("cpuid: %02x %02x %02x %02x %02x %02x %02x %02x\n",
		   cpuid[0], cpuid[1], cpuid[2], cpuid[3], cpuid[4], cpuid[5], cpuid[6], cpuid[7]
	);
	cpuid[0] ^= cpuid[4]; // first byte is not sufficiently unique
	uint8_t radio_id = cpuid[0];
    DEBUG("radio_id: %02x\n", radio_id);

// 	setup_network(cpuid[0], false, 1111, &handle_packet, &handle_transmit);

//     gpio_init_int(BUTTON, GPIO_PULLDOWN, GPIO_RISING, button_handler, NULL);
//     gpio_init_int(BTN_1, GPIO_PULLDOWN, GPIO_RISING, button_handler, NULL);
//     gpio_init_int(BTN_2, GPIO_PULLDOWN, GPIO_RISING, button_handler, NULL);


//     s25fl_t flash;
//     s25fl_init(&flash, SPI_DEV(0), GPIO_MEM_CS, GPIO_MEM_RST, GPIO_MEM_WP, GPIO_MEM_PWR);
//
//     uint8_t buf[128];
//     s25fl_read(&flash, 0, buf, 128);
//
//     for(int i = 0; i < 128; i++) {
//         printf("%02x ", buf[i]);
//         if ((i+1) % 16 == 0)
//             printf("\n");
//     }
//     s25fl_power(&flash, false);





}

void Sensortag::mainloop()
{
	thread_yield();

	DEBUG("starting mainloop...\r\n");

    printf("do si70xx_test\n");
    si70xx_t si;
    si70xx_init(&si, I2C_DEV(0), 0x40);
    int ret = si70xx_test(&si);
    printf("si70xx_test returned %i\n", ret);

    xtimer_t t;
    int i =0;
    int16_t temperature;
    uint16_t humidity;

    while(1) {
        si70xx_get_both(&si, &humidity, &temperature);

        printf("%i %lu %u ", i++, xtimer_now() / 1000, timer_read(TIMER_DEV(0)));
        printf("%i.%02uC %u.%02u%%\n",
               temperature / 100, temperature % 100,
               humidity / 100, humidity % 100);

//         char receive;
//         spi_transfer_byte(SPI_DEV(0), 0x55, &receive);
//         printf("got byte 0x%02x\n", receive);

        xtimer_set_wakeup(&t, 2000*1000, thread_getpid());
        thread_sleep();

        flash_led(GPIO_PIN(18), 2);
    }
}

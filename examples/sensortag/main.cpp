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

#include "periph_conf.h"
#include "shell.h"
// #include "posix_io.h"
#include "thread.h"
#include "xtimer.h"
#include "hw_aon_rtc.h"

#include <stdio.h>

char shell_thread_stack [THREAD_STACKSIZE_MAIN + 256];
volatile uint32_t SysTickCnt;

void *shell_thread(void *arg);

void SysTick_Handler(void)
{
	SysTickCnt++;
}

uint32_t read(void)
{
    uint32_t   ui32CurrentSec    ;
    uint32_t   ui32CurrentSubSec ;
    uint32_t   ui32SecondSecRead ;

    //
    // Reading SEC both before and after SUBSEC in order to detect if SEC incremented while reading SUBSEC
    // If SEC incremented, we can’t be sure which SEC the SUBSEC belongs to, so repeating the sequence then.
    //
    do {
        ui32CurrentSec    = *(volatile uint32_t*)( AON_RTC_BASE + AON_RTC_O_SEC    );
        ui32CurrentSubSec = *(volatile uint32_t*)( AON_RTC_BASE + AON_RTC_O_SUBSEC );
        ui32SecondSecRead = *(volatile uint32_t*)( AON_RTC_BASE + AON_RTC_O_SEC    );
    } while ( ui32CurrentSec != ui32SecondSecRead );

//     return (( ui32CurrentSec << 16 ) | ( ui32CurrentSubSec >> 16 ));
    return ui32CurrentSubSec / 4295 + ui32CurrentSec * 1000000;
//     return ui32CurrentSubSec / 4295;
}

int main()
{
	//enable bus, usage, and memmanage faults
// 	SCB->SHCSR |= (1<<18) | (1<<17) | (1<<16);

// 	SysTick_Config(SystemCoreClockGet()/1000 - 1); //interrupt period 1ms

    printf("\n************ Sensortag ***********\n");
    printf("\n");

    printf("starting Sensortag\r\n");

//     uint32_t j = 1;
//     while(1) {
//         while (read() <= 1000000*j) { }
//         printf("%lu\n", j++);
//     }

//     uint32_t x = 0;
//     while(1) {
//         printf("%lu\n", x++);
//         uint32_t spin = 1000*1000;
//         printf("spin for: %lu now: %lu seconds: %lu\n", spin, read(), *(volatile uint32_t*)( AON_RTC_BASE + AON_RTC_O_SEC    ));
//         xtimer_spin(spin);
//     }

    printf("sleeping in 2 seconds...\n");
    xtimer_spin(1000*1000);
    printf("sleeping in 1 second...\n");
    xtimer_spin(1000*1000);
    xtimer_usleep(1000*1000*3);
    printf("hi\n");

    uint32_t i = 0;
    xtimer_t sleep;
    xtimer_set_wakeup(&sleep, 1000*1000, thread_getpid());
    while(1) {
        printf("%lu\n", i++);
        thread_sleep();
        xtimer_set_wakeup(&sleep, 1000*1000, thread_getpid());
    }

    Sensortag* sensortag = new Sensortag();

	thread_create(shell_thread_stack, sizeof(shell_thread_stack), 13,
				  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD, shell_thread,
                  NULL, "shell");

    sensortag->mainloop();

    return 0;
}

void *shell_thread(void *arg)
{
    char input_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, input_buf, SHELL_DEFAULT_BUFSIZE);
    return NULL;
}

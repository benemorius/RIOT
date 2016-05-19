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

#include <stdio.h>

char shell_thread_stack [THREAD_STACKSIZE_MAIN + 256];
volatile uint32_t SysTickCnt;

void *shell_thread(void *arg);

void SysTick_Handler(void)
{
	SysTickCnt++;
}

int main()
{
	//enable bus, usage, and memmanage faults
// 	SCB->SHCSR |= (1<<18) | (1<<17) | (1<<16);

// 	SysTick_Config(SystemCoreClockGet()/1000 - 1); //interrupt period 1ms

    printf("\n************ Sensortag ***********\n");
    printf("\n");

    printf("starting Sensortag\r\n");

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

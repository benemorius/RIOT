/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#if defined MODULE_SHELL || defined DOXYGEN

#include "log.h"
#include "util.h"
#include "shell.h"
#include "thread.h"
#include "periph/adc.h"
#include "riotboot/slot.h"

#include <stdlib.h>

#define QUEUE_SIZE (8)
static msg_t _main_msg_queue[QUEUE_SIZE];
static char _shell_thread_stack [THREAD_STACKSIZE_DEFAULT + 1024 + 512];
static char _line_buf[SHELL_DEFAULT_BUFSIZE];

shell_command_t *util_all_shell_commands;

// int _temp(int argc, char **argv)
// {
//     (void)argc;
//     (void)argv;
//     uint32_t sample = adc_sample(0, ADC_RES_16BIT);
//     LOG_DEBUG("sample %lu\n", sample);
//     int32_t temperature_milli_c = sample * 100 / 3108 - 212;
//     printf("temperature: %li.%lu C\n", temperature_milli_c / 10, temperature_milli_c % 10);
//     return 0;
// }

#ifdef MODULE_PERIPH_ADC
/* for KW41Z */
void _get_temperature(void)
{
    uint32_t sample = adc_sample(ADC_TEMPERATURE_CHANNEL, ADC_RES_16BIT);
    // // printf("sample %" PRIu32 "\n", sample);

    float vtemp = (float)sample / 65535 * ADC_REF_VOLTAGE;
//     float vtemp = (float)sample / 65535 * 1.2016;
    float vtemp25 = 0.716f;
    float m = 0.00174f;
    float temperature_c = 25.0f - ((vtemp - vtemp25) / m);
    //     int32_t temperature_centi_c = (int32_t)(temperature_c * 100);
    // printf("temperature: %i\n", temperature_centi_c);


    // float temperature_c = 25.0 - (((float)sample/65535 * 3.3) - 0.716) / 0.00174;
    int32_t temperature_deci_c = temperature_c * 10;
    // printf("sample %" PRIu32 " temperature %" PRIi32 "\n", sample, temperature_deci_c);
    // printf("%" PRIi32 "\n", (int32_t)(temperature_c * 1000));


    printf("sample: %" PRIu32 " temperature: %" PRIi32 ".%" PRIu32 " C\n", sample, (int32_t)temperature_c, temperature_deci_c % 10);
}

int _read_temperature(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    _get_temperature();
    return 0;
}
#endif

#ifdef MODULE_RIOTBOOT

static int cmd_print_slot_nr(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("Current slot: %d\n", riotboot_slot_current());
    return 0;
}

static int cmd_print_slot_hdr(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    int current_slot = riotboot_slot_current();
    if (current_slot < 0) {
        printf("Current slot not found; not running a riotboot firmware?\n");
        return current_slot;
    }
    riotboot_slot_print_hdr(current_slot);
    return 0;
}

static int cmd_print_slot_addr(int argc, char **argv)
{
    (void)argc;

    int reqslot = atoi(argv[1]);
    printf("Slot %d address=0x%08" PRIx32 "\n",
           reqslot, riotboot_slot_get_image_startaddr(reqslot));
    return 0;
}

static int cmd_dumpaddrs(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    riotboot_slot_dump_addrs();
    return 0;
}

static int cmd_boot_slot(int argc, char **argv)
{
    (void)argc;

    int slot = atoi(argv[1]);
    riotboot_slot_jump(slot);
    return 0;
}

#endif

static const shell_command_t _util_shell_commands[] = {
#ifdef MODULE_PERIPH_ADC
    { "temp", "read board temperature", _read_temperature },
#endif
#ifdef MODULE_RIOTBOOT
    { "curslotnr", "Print current slot number", cmd_print_slot_nr },
    { "curslothdr", "Print current slot header", cmd_print_slot_hdr },
    { "getslotaddr", "Print address of requested slot", cmd_print_slot_addr },
    { "dumpaddrs", "Prints all slot data in header", cmd_dumpaddrs },
    { "boot", "Boots firmware in requested slot", cmd_boot_slot },
#endif
    { NULL, NULL, NULL },
};

static void *_shell_thread(void *arg)
{
    const shell_command_t *extra_commands = (const shell_command_t *)arg;

    /* count the extra shell commands provided and append them */
    unsigned num_extra_commands = 0;
    while (extra_commands && (extra_commands[num_extra_commands].handler != NULL)) {
        num_extra_commands++;
    }
    unsigned num_util_commands = sizeof(_util_shell_commands) / sizeof(_util_shell_commands[0]);
    shell_command_t all_commands[num_extra_commands + num_util_commands];

    memcpy(all_commands, extra_commands, num_extra_commands * sizeof(shell_command_t));
    memcpy((uint8_t *)all_commands + num_extra_commands * sizeof(shell_command_t),
           _util_shell_commands,
           sizeof(_util_shell_commands)
    );

    util_all_shell_commands = all_commands;

    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, QUEUE_SIZE);

    while (1) {
        shell_run(all_commands, _line_buf, sizeof(_line_buf));
    }
    return NULL;
}

void shell_start(const shell_command_t *shell_commands)
{
    thread_create(_shell_thread_stack, sizeof(_shell_thread_stack), 7,
                  THREAD_CREATE_STACKTEST,
                  _shell_thread, (void *)shell_commands, "uart shell");
}

#endif /* MODULE_SHELL */

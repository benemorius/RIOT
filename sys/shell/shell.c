/*
 * Copyright (C) 2009, Freie Universitaet Berlin (FUB).
 * Copyright (C) 2013, INRIA.
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_shell
 * @{
 *
 * @file
 * @brief       Implementation of a very simple command interpreter.
 *              For each command (i.e. "echo"), a handler can be specified.
 *              If the first word of a user-entered command line matches the
 *              name of a handler, the handler will be called with the whole
 *              command line as parameter.
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      René Kijewski <rene.kijewski@fu-berlin.de>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "od.h"
#include "log.h"
#include "shell.h"
#include "shell_commands.h"

#define ETX '\x03'  /** ASCII "End-of-Text", or ctrl-C */
#if !defined(SHELL_NO_ECHO) || !defined(SHELL_NO_PROMPT)
#ifdef MODULE_NEWLIB
/* use local copy of putchar, as it seems to be inlined,
 * enlarging code by 50% */
static void _putchar(int c) {
    putchar(c);
}
#else
#define _putchar putchar
#endif
#endif

static void flush_if_needed(void)
{
#ifdef MODULE_NEWLIB
    fflush(stdout);
#endif
}

static inline void print_prompt(void);
static void replace_line(const char *line_buf);
static void refresh_line(const char *line_buf);
static void load_line_history(char *line_history_buf, int size, int *index);
static void push_line_history(char *line_history_buf, int size);
static char *find_line_history(char *line_history_buf, int size, int *index);

static shell_command_handler_t find_handler(const shell_command_t *command_list, char *command)
{
    const shell_command_t *command_lists[] = {
        command_list,
#ifdef MODULE_SHELL_COMMANDS
        _shell_command_list,
#endif
    };

    /* iterating over command_lists */
    for (unsigned int i = 0; i < ARRAY_SIZE(command_lists); i++) {

        const shell_command_t *entry;

        if ((entry = command_lists[i])) {
            /* iterating over commands in command_lists entry */
            while (entry->name != NULL) {
                if (strcmp(entry->name, command) == 0) {
                    return entry->handler;
                }
                else {
                    entry++;
                }
            }
        }
    }

    return NULL;
}

static void print_help(const shell_command_t *command_list)
{
    printf("%-20s %s\n", "Command", "Description");
    puts("---------------------------------------");

    const shell_command_t *command_lists[] = {
        command_list,
#ifdef MODULE_SHELL_COMMANDS
        _shell_command_list,
#endif
    };

    /* iterating over command_lists */
    for (unsigned int i = 0; i < ARRAY_SIZE(command_lists); i++) {

        const shell_command_t *entry;

        if ((entry = command_lists[i])) {
            /* iterating over commands in command_lists entry */
            while (entry->name != NULL) {
                printf("%-20s %s\n", entry->name, entry->desc);
                entry++;
            }
        }
    }
}

static void handle_input_line(const shell_command_t *command_list, char *line)
{
    static const char *INCORRECT_QUOTING = "shell: incorrect quoting";

    /* first we need to calculate the number of arguments */
    unsigned argc = 0;
    char *pos = line;
    int contains_esc_seq = 0;
    while (1) {
        if ((unsigned char) *pos > ' ') {
            /* found an argument */
            if (*pos == '"' || *pos == '\'') {
                /* it's a quoted argument */
                const char quote_char = *pos;
                do {
                    ++pos;
                    if (!*pos) {
                        puts(INCORRECT_QUOTING);
                        return;
                    }
                    else if (*pos == '\\') {
                        /* skip over the next character */
                        ++contains_esc_seq;
                        ++pos;
                        if (!*pos) {
                            puts(INCORRECT_QUOTING);
                            return;
                        }
                        continue;
                    }
                } while (*pos != quote_char);
                if ((unsigned char) pos[1] > ' ') {
                    puts(INCORRECT_QUOTING);
                    return;
                }
            }
            else {
                /* it's an unquoted argument */
                do {
                    if (*pos == '\\') {
                        /* skip over the next character */
                        ++contains_esc_seq;
                        ++pos;
                        if (!*pos) {
                            puts(INCORRECT_QUOTING);
                            return;
                        }
                    }
                    ++pos;
                    if (*pos == '"') {
                        puts(INCORRECT_QUOTING);
                        return;
                    }
                } while ((unsigned char) *pos > ' ');
            }

            /* count the number of arguments we got */
            ++argc;
        }

        /* zero out the current position (space or quotation mark) and advance */
        if (*pos > 0) {
            *pos = 0;
            ++pos;
        }
        else {
            break;
        }
    }
    if (!argc) {
        return;
    }

    /* then we fill the argv array */
    char *argv[argc + 1];
    argv[argc] = NULL;
    pos = line;
    for (unsigned i = 0; i < argc; ++i) {
        while (!*pos) {
            ++pos;
        }
        if (*pos == '"' || *pos == '\'') {
            ++pos;
        }
        argv[i] = pos;
        while (*pos) {
            ++pos;
        }
    }
    for (char **arg = argv; contains_esc_seq && *arg; ++arg) {
        for (char *c = *arg; *c; ++c) {
            if (*c != '\\') {
                continue;
            }
            for (char *d = c; *d; ++d) {
                *d = d[1];
            }
            if (--contains_esc_seq == 0) {
                break;
            }
        }
    }

    /* then we call the appropriate handler */
    shell_command_handler_t handler = find_handler(command_list, argv[0]);
    if (handler != NULL) {
        handler(argc, argv);
    }
    else {
        if (strcmp("help", argv[0]) == 0) {
            print_help(command_list);
        }
        else {
            printf("shell: command not found: %s\n", argv[0]);
        }
    }
}

static int readline(char *buf, size_t size)
{
    char *line_buf_ptr = buf;
    *line_buf_ptr = '\0';
    int line_history_index = -1;

    while (1) {
        int c = getchar();
        if (c < 0) {
            return EOF;
        }

        /* We allow Unix linebreaks (\n), DOS linebreaks (\r\n), and Mac linebreaks (\r). */
        /* QEMU transmits only a single '\r' == 13 on hitting enter ("-serial stdio"). */
        /* DOS newlines are handled like hitting enter twice, but empty lines are ignored. */
        /* Ctrl-C cancels the current line. */
        if (c == '\r' || c == '\n' || c == ETX) {
            *line_buf_ptr = '\0';
#ifndef SHELL_NO_ECHO
            _putchar('\r');
            _putchar('\n');
#endif

            /* return 1 if line is empty, 0 otherwise */
            return c == ETX || line_buf_ptr == buf;
        }
        /* ^C */
        else if (c == 'C' - '@') {
            _putchar('\r');
            _putchar('\n');
#ifdef MODULE_NEWLIB
            fflush(stdout);
#endif
            return 1; // 1 for empty line
        }
        /* QEMU uses 0x7f (DEL) as backspace, while 0x08 (BS) is for most terminals */
        else if (c == 0x08 || c == 0x7f) {
            /* perform an insertion-style delete */

            if (line_buf_ptr == buf) {
                /* The line is empty. */
                continue;
            }

            /* overwrite previous character and keep shifting left until end */
            --line_buf_ptr;
            char *line_buf_ptr_restore = line_buf_ptr;
            while (line_buf_ptr[1] != '\0') {
                *line_buf_ptr = line_buf_ptr[1];
                ++line_buf_ptr;
            }
            *line_buf_ptr = '\0';

            /* we if deleted a character somewhere other than the
             * end of the line then we need to refresh the line
             */
            if (line_buf_ptr != line_buf_ptr_restore) {
                fputs("\033[D", stdout); /* move cursor left one */
                refresh_line(buf);
                line_buf_ptr = line_buf_ptr_restore;
                continue;
            }
            /* otherwise the character was deleted from the end */

            /* white-tape the character */
#ifndef SHELL_NO_ECHO
            _putchar('\b');
            _putchar(' ');
            _putchar('\b');
#endif
        }
        /* escape sequence */
        else if (c == '[' - '@') { /* first character is ^[ */
            if (getchar() != '[') { /* second character is [ */
                continue;
            }
            c = getchar(); /* third/last character is of interest */
            /* up or down */
            if (c == 'A' || c == 'B') {
                /* load commands from history into the current line_buf */

                int delta = c == 'A' ? 1 : -1;
                /* can't have line_history_index < -1 */
                if (line_history_index + delta < -1) {
                    line_history_index = -1;
                    continue;
                }

                /* If line_buf contains a command and it didn't come from
                 * history, then we need to store it before overwriting
                 * line_buf with the requested history. This also means
                 * it will become a part of the command history even though
                 * the user didn't run the command. This is perhaps not
                 * desirable.
                 */
                if (line_history_index == -1
                    && delta > 0 /* should always be true here actually */
                    && line_buf_ptr != buf
                ) {
                    push_line_history(buf, size);
                    ++line_history_index;
                }

                line_history_index += delta;

                /* going back down past the most recent history
                 * always comes back to a blank line
                 */
                if (line_history_index == -1) {
                    line_buf_ptr = buf;
                    *line_buf_ptr = '\0';
                    replace_line(buf);
                    continue;
                }

                /* load the requested history into the current line_buf */
                load_line_history(buf, size, &line_history_index);
                replace_line(buf);
                line_buf_ptr = buf + strlen(buf);
            }
            /* left or right */
            else if (c == 'D' || c == 'C') {
                /* check that it's ok to move the cursor in that direction */
                int delta = c == 'D' ? -1 : 1;
                if (delta > 0 && *line_buf_ptr == '\0') {
                    continue;
                }
                if (delta < 0 && line_buf_ptr == buf) {
                    continue;
                }

                /* now move the cursor */
                line_buf_ptr += delta;
                if (delta > 0) {
                    fputs("\033[C", stdout);
                }
                else {
                    fputs("\033[D", stdout);
                }
            }
            else {
                const char msg[] = "[shell] unhandled escape sequence";
                LOG_INFO("%s ^[[<0x%02x> (\\033[\\0%o)\n", msg, c, c);
            }
        }
        /* received new character to add to line buffer */
        else {
            /* can't add another character if line buffer is full */
            if ((line_buf_ptr - buf) >= ((int) size) - 1) {
                continue;
            }

            /* check if we're inserting somewhere before the end of the string,
             * and make room for the inserted character if so
             */
            if (*line_buf_ptr != '\0') {
                /* shift right one by one until string ends, inserting
                 * the new character in the first opening
                 */
                char tmp;
                char *line_buf_ptr_restore = line_buf_ptr;
                while (*line_buf_ptr != '\0') {
                    tmp = c;
                    c = *line_buf_ptr;
                    *line_buf_ptr++ = tmp;
                }
                *line_buf_ptr++ = c; /* while loop misses the last one */
                *line_buf_ptr = '\0';
                line_buf_ptr = line_buf_ptr_restore + 1;
                /* move cursor right one */
                fputs("\033[C", stdout);
                refresh_line(buf);
                continue;
            }
            else {
                *line_buf_ptr++ = c;
                *line_buf_ptr = '\0';
            }
#ifndef SHELL_NO_ECHO
            _putchar(c);
#endif
        }
        flush_if_needed();
    }
}

static void replace_line(const char *line_buf)
{
    fputs("\033[2K\r", stdout); /* delete line */
    print_prompt();
    fputs(line_buf, stdout);
    fflush(stdout);
}

static void refresh_line(const char *line_buf)
{
    fputs("\033[s\033[2K\r", stdout); /* store cursor location and delete line */
    print_prompt();
    fputs(line_buf, stdout);
    fputs("\033[u", stdout); /* restore cursor location */
    fflush(stdout);
}

static char *_find_history_front(char *line_history_buf)
{
    /* find the beginning of history (the 'e' in "echo 1") */
     char *history_ptr = strchr(line_history_buf, '\0');
     ++history_ptr;
     if (*history_ptr == '\0') {
        /* current line_buf hasn't yet collided with history
         * beginning of history will be the next nonzero character
         */
         while (*history_ptr == '\0') {
            ++history_ptr;
         }
     }
     else {
        /* line_buf has collided with history already
         * history_ptr is currently inside the overwritten command
         * beginning of undamaged history will start after the next \0
         */
         history_ptr = strchr(history_ptr, '\0');
         ++history_ptr;
     }
     /* actually this needs to point to the \0 */
     --history_ptr;
     return history_ptr;
}

static void push_line_history(char *line_history_buf, int size)
{
    /* store the active line_buf in history, but not
     * if it matches the last command already there
     */
    int history_index = 0;
    char *latest_line = find_line_history(line_history_buf, size, &history_index);
    // printf("comparing with index %i\n", history_index);
    if (latest_line[0] != '\0' && strcmp(line_history_buf, latest_line) == 0) {
        return;
    }

    /* shift bytes out the front and into the back of line_history_buf
     * until the front character is \0
     */
    char *buf_ptr;
    do {
        buf_ptr = line_history_buf;
        /* take the first character to place it at the rear */
        char front_c = *buf_ptr;

        /* shift the rest of line_history_buf left one position */
        while (buf_ptr < line_history_buf + size - 1) {
            *buf_ptr = buf_ptr[1];
            ++buf_ptr;
        }
        *buf_ptr = front_c;

    } while (*buf_ptr != '\0');
}

static char *find_line_history(char *line_history_buf, int size, int *index)
{
    /* line_history_buf currently looks like "ifconfig\0...\0echo 1\0echo 2\0" */

    /* find the *index'th command string starting from the rear
     * and copy it to the front of line_history_buf for line_buf
     */
    char *history_front = _find_history_front(line_history_buf);
    char *buf_ptr = line_history_buf + size - 1;
    // --buf_ptr; /* start one character left of the \0 */
    char *line_ptr;
    int count = 0;
    do {
        do {} while (*--buf_ptr != '\0');
        line_ptr = buf_ptr + 1;
    } while (++count <= *index && buf_ptr > history_front);
    *index = count - 1;
    return line_ptr;
}

static void load_line_history(char *line_history_buf, int size, int *index)
{
    char *line_ptr = find_line_history(line_history_buf, size, index);
    // printf("loading from index %i\n", *index);
    strcpy(line_history_buf, line_ptr);

    return;

    /* line_history_buf currently looks like "ifconfig\0\0...\0echo 1\0echo 2\0"
     * the extra \0 after ifconfig marks the true end of line_buf (including its future)
     * and the one before echo 1 marks the beginning of history
     */

}

static inline void print_prompt(void)
{
#ifndef SHELL_NO_PROMPT
    fputs("\033[91m>\033[0m ", stdout);
#endif

    flush_if_needed();
}

void shell_run_once(const shell_command_t *shell_commands,
                    char *line_buf, int len)
{
    /* begin prompt on a new line */
    _putchar('\r');
    _putchar('\n');
    print_prompt();

    /* required by push_line_history() */
    memset(line_buf, '\0', len);

    while (1) {
        int res = readline(line_buf, len);

        if (res == EOF) {
            break;
        }

        if (!res) {
            // _putchar('\n'); od_hex_dump(line_buf, len, 0);

            /* handle_input_line() will turn all ' ' into '\0'
             * so give it a copy on the stack
             * and find a better solution later
             */
            char fix_me[len];
            strcpy(fix_me, line_buf);
            handle_input_line(shell_commands, fix_me);

            push_line_history(line_buf, len);
            // _putchar('\n'); od_hex_dump(line_buf, len, 0);
        }

        print_prompt();
    }
}
